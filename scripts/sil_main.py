#!/usr/bin/env python

import os
import sys
import rospy
from std_msgs.msg import String
from sil_ros.vlm_node import PerceptionModule
from sil_ros.llm_node import SILLLMInterface
from sil_ros.commands_parser import CommandParser
from sil_ros.action_executor import ActionExecutor
from sil_ros.data_logger import DataLogger, LogEntry
from sil_ros.memory_module import EpisodicSemanticMemory
from sil_ros.interaction_manager import SymbioticInteractionManager
from datetime import datetime
from typing import Dict, List
import threading
import signal
import json
import numpy as np
import torch
import pickle
from collections import deque
from sil_ros.config import SILConfig

# ============= CONTINUAL LEARNING SAFEGUARD =============
class ContinualLearningSafeguard:
    def __init__(self, save_dir=None, config: SILConfig = None):
        self.config = config or SILConfig.load()
        cl = self.config.continual_learning
        self.save_dir = save_dir if save_dir is not None else \
            rospy.get_param("sil/model_directory", "/tmp/sil_models")
        os.makedirs(self.save_dir, exist_ok=True)
        self.fisher_matrices = {}
        self.optimal_params = {}
        self.replay_buffer = deque(maxlen=cl.replay_buffer_capacity)
        self.current_task_id = 0
        self.task_performances = []
        self.task_shift_window = cl.task_shift_window
        self.task_shift_drop_threshold = cl.task_shift_drop_threshold
        self.replay_batch_size = cl.replay_batch_size
        self.checkpoint_keep_recent = cl.checkpoint_keep_recent
        self.ewc_lambda = self.config.training.ewc_lambda

    def detect_task_shift(self, current_performance: Dict, history: List) -> bool:
        if len(history) < self.task_shift_window:
            return False

        recent_avg = np.mean(history[-self.task_shift_window:])
        current = current_performance.get('success_rate', 0)
        if recent_avg - current > self.task_shift_drop_threshold:
            rospy.loginfo("[ContinualLearning] Task shift detected")
            return True

        return False
    
    def protect_knowledge(self, model=None, data=None):
        self.task_performances.append({
            'task_id': self.current_task_id,
            'timestamp': datetime.now()
        })
        self.current_task_id += 1
        self.save_checkpoint()
    
    def save_checkpoint(self):
        keep = self.checkpoint_keep_recent
        replay_sample = list(self.replay_buffer)[-keep:] if len(self.replay_buffer) > keep \
            else list(self.replay_buffer)
        checkpoint = {
            'task_id': self.current_task_id,
            'performances': self.task_performances,
            'replay_buffer': replay_sample 
        }
        
        path = os.path.join(self.save_dir, f"checkpoint_task_{self.current_task_id}.pkl")
        with open(path, 'wb') as f:
            pickle.dump(checkpoint, f)

    def compute_fisher_information(self, model_params: Dict, gradients: List):
        fisher = {}
        
        for param_name in model_params:
            if len(gradients) > 0:
                # Fisher = E[gradient^2]
                grad_sum = np.zeros_like(model_params[param_name])
                for grad in gradients:
                    if param_name in grad:
                        grad_sum += grad[param_name] ** 2
                fisher[param_name] = grad_sum / len(gradients)
            else:
                fisher[param_name] = np.zeros_like(model_params[param_name])
        
        return fisher
    
    def ewc_loss(self, current_params: Dict, importance: float = None) -> float:
        if importance is None:
            importance = self.ewc_lambda
        loss = 0.0
        
        for task_id, fisher in self.fisher_matrices.items():
            if task_id in self.optimal_params:
                for param_name in fisher:
                    if param_name in current_params:
                        diff = current_params[param_name] - self.optimal_params[task_id][param_name]
                        loss += importance * np.sum(fisher[param_name] * diff ** 2)
        return loss

    def store_task_knowledge(self, model_params: Dict, gradients: List):
        fisher = self.compute_fisher_information(model_params, gradients)
        self.fisher_matrices[self.current_task_id] = fisher
        self.optimal_params[self.current_task_id] = {
            k: v.copy() if hasattr(v, 'copy') else v 
            for k, v in model_params.items()
        }
        rospy.loginfo("[ContinualLearning] Stored Fisher information for task %d", self.current_task_id)

    def add_to_replay(self, experience: Dict):
        self.replay_buffer.append(experience)
    
    def sample_replay(self, batch_size: int = None) -> List[Dict]:
        if batch_size is None:
            batch_size = self.replay_batch_size
        if len(self.replay_buffer) < batch_size:
            return list(self.replay_buffer)
        
        indices = np.random.choice(len(self.replay_buffer), batch_size, replace=False)
        return [self.replay_buffer[i] for i in indices]

# ============= MUTUAL ADAPTATION TRACKER =============
class MutualAdaptationTracker: 
    def __init__(self):
        self.human_trajectory = []
        self.agent_trajectory = []
        self.interaction_outcomes = []
        
        self.metrics = {
            'human_learning_rate': 0.0,
            'agent_learning_rate': 0.0,
            'mutual_information': 0.0,
            'convergence_rate': 0.0
        }
    
    def update(self, human_belief, agent_belief, outcome):
        if human_belief:
            self.human_trajectory.append({
                'embedding': human_belief.task_embedding.copy(),
                'confidence': human_belief.confidence,
                'timestamp': datetime.now()
            })
        
        if agent_belief:
            self.agent_trajectory.append({
                'embedding': agent_belief.task_embedding.copy(),
                'confidence': agent_belief.confidence,
                'timestamp': datetime.now()
            })
        
        self.interaction_outcomes.append(outcome)
        self._compute_metrics()
    
    def _compute_metrics(self):
        if len(self.human_trajectory) < 2:
            return
        h_curr = self.human_trajectory[-1]['embedding']
        h_prev = self.human_trajectory[-2]['embedding']
        self.metrics['human_learning_rate'] = np.linalg.norm(h_curr - h_prev)
        
        if len(self.agent_trajectory) >= 2:
            a_curr = self.agent_trajectory[-1]['embedding']
            a_prev = self.agent_trajectory[-2]['embedding']
            self.metrics['agent_learning_rate'] = np.linalg.norm(a_curr - a_prev)
        if len(self.human_trajectory) > 0 and len(self.agent_trajectory) > 0:
            dist_curr = np.linalg.norm(h_curr - self.agent_trajectory[-1]['embedding'])
            if len(self.human_trajectory) > 1 and len(self.agent_trajectory) > 1:
                dist_prev = np.linalg.norm(h_prev - self.agent_trajectory[-2]['embedding'])
                self.metrics['convergence_rate'] = dist_prev - dist_curr

# ============= DATA LOGGER =============
class SILDataLogger(DataLogger):
    
    def __init__(self, log_file=None):
        if log_file is None:
            log_file = rospy.get_param("logging/log_file", "sil_robot_log.csv")
        super().__init__(log_file)
        self.sil_metrics = {
            'total_interactions': 0,
            'clarification_requests': 0,
            'proactive_suggestions': 0,
            'adaptation_events': 0,
            'belief_alignments': [],
            'success_rates': []
        }
    
    def log_sil_interaction(self, interaction_data):
        log_entry = LogEntry()
        log_entry.input_command = interaction_data.get('human_input', '')
        log_entry.input_timestamp = datetime.now().isoformat()
        log_entry.response_timestamp = datetime.now().isoformat()
        log_entry.predicted_action_sequence = str(interaction_data.get('parsed_intent', ''))
        log_entry.execution_success = str(interaction_data.get('success', False))
        sil_data = {
            'belief_alignment': interaction_data.get('belief_alignment', 0.0),
            'agent_confidence': interaction_data.get('agent_confidence', 0.0),
            'adaptation_metrics': interaction_data.get('adaptation_metrics', {})
        }
        
        log_entry.true_action_sequence = json.dumps(sil_data)
        self.log_entry(log_entry)
        self.sil_metrics['total_interactions'] += 1
        if interaction_data.get('belief_alignment'):
            self.sil_metrics['belief_alignments'].append(interaction_data['belief_alignment'])
        if interaction_data.get('success'):
            self.sil_metrics['success_rates'].append(1.0)
        else:
            self.sil_metrics['success_rates'].append(0.0)

# ============= MAIN CONTROLLER =============
class EnhancedRobotController:
    def __init__(self):
        rospy.init_node('sil_robot_controller')
        self.config = SILConfig.load()
        self.data_logger = SILDataLogger()
        self._setup_llm_configuration()
        self.perception = PerceptionModule(self.data_logger)
        self.memory_module = EpisodicSemanticMemory(config=self.config)
        if not hasattr(self.memory_module, 'shared_task_space'):
            self.memory_module.shared_task_space = None
        self.llm_interface = SILLLMInterface(
            api_key=self.api_key_value,
            destinations={},
            memory_module=self.memory_module,
            config=self.config
        )
        self.memory_module.llm_interface = self.llm_interface
        self.action_executor = ActionExecutor(
            self.data_logger,
            self.perception,
            self.llm_interface,
        )
        self.action_executor.llm_interface = self.llm_interface
        self.action_executor.response_publisher = rospy.Publisher(
            rospy.get_param("topics/llm_output", "/llm_output"), String, queue_size=10)
        self.action_executor.perception_module = self.perception
        self.llm_interface.destinations = self.action_executor.get_destinations()
        self.command_parser = CommandParser(self.action_executor)
        self.interaction_manager = SymbioticInteractionManager(
            llm_interface=self.llm_interface,
            memory_module=self.memory_module,
            action_executor=self.action_executor,
            perception_module=self.perception,
            config=self.config
        )
        self.interaction_manager.set_controller_reference(self)
        self.interaction_manager.data_logger = self.data_logger
        self.memory_module.shared_task_space = self.interaction_manager.shared_task_space
        self.continual_learning = ContinualLearningSafeguard(config=self.config)
        self.adaptation_tracker = MutualAdaptationTracker()
        self._setup_communication()
        self.enable_sil = rospy.get_param("sil/enable", True)
        self.auto_save_memory = rospy.get_param("sil/auto_save_memory", True)
        self.save_interval = rospy.get_param("sil/memory_save_interval", 300)
        self.recent_window = int(rospy.get_param("logging/metrics/recent_window_size", 20))
        self.performance_history = []
        self._start_monitoring()
        if self.auto_save_memory:
            self._start_memory_autosave()
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        rospy.loginfo("[SIL] Robot Controller initialized with full co-adaptation")
    
    def _setup_communication(self):
        self.response_publisher = rospy.Publisher(
            rospy.get_param("topics/llm_output", "/llm_output"), String, queue_size=10)
        self.tts_publisher = rospy.Publisher(
            rospy.get_param("topics/tts_text", "/tts_text"), String, queue_size=10)
        self.sil_status_publisher = rospy.Publisher(
            rospy.get_param("topics/sil_status", "/sil_status"), String, queue_size=10)
        self.sil_metrics_publisher = rospy.Publisher(
            rospy.get_param("topics/sil_metrics", "/sil_metrics"), String, queue_size=10)
        self.adaptation_publisher = rospy.Publisher(
            rospy.get_param("topics/sil_adaptation", "/sil_adaptation"), String, queue_size=10)
        self.interaction_manager.response_publisher = self.response_publisher
        self.interaction_manager.tts_publisher = self.tts_publisher
        rospy.Subscriber(rospy.get_param("topics/llm_input", "/llm_input"), String, self.handle_input)
        rospy.Subscriber(rospy.get_param("topics/sil_feedback", "/sil_feedback"), String, self.handle_feedback)
        rospy.Subscriber(rospy.get_param("topics/sil_command", "/sil_command"), String, self.handle_command)
    
    def handle_input(self, msg):
        input_text = msg.data.strip()
        rospy.loginfo(f"[SIL] Received: {input_text}")
        try:
            if self.enable_sil:
                context = {
                    'timestamp': datetime.now().isoformat(),
                    'interaction_count': len(self.performance_history),
                    'recent_performance': self._get_recent_performance(),
                    'shared_task_space': self.interaction_manager.shared_task_space
                }
                result = self.interaction_manager.process_bidirectional_interaction(
                    input_text, context
                )
                self._track_performance(input_text, result)
                if self.interaction_manager.shared_task_space.human_belief_state:
                    self.adaptation_tracker.update(
                        self.interaction_manager.shared_task_space.human_belief_state,
                        self.interaction_manager.shared_task_space.agent_belief_state,
                        result.get('result', {}).get('success', False)
                    )
                current_perf = self._get_recent_performance()
                if self.continual_learning.detect_task_shift(current_perf, self.performance_history):
                    rospy.loginfo("[SIL] Task shift detected. Preparing to protect and adapt knowledge.") 
                    anchor_input = input_text
                    positive_input = self.memory_module.find_positive_example(anchor_input)
                    negative_input = self.memory_module.find_negative_example(anchor_input)
                    if positive_input and negative_input:
                        gradients = self.interaction_manager.fine_tune_on_interaction(
                            anchor_input, positive_input, negative_input
                        )
                        model_params = {name: p.detach().numpy() for name, p in 
                                    self.interaction_manager.shared_task_space.task_encoder.named_parameters()}
                        self.continual_learning.store_task_knowledge(model_params, [gradients])
                    self.continual_learning.protect_knowledge()
                current_perf = self._get_recent_performance()
                if self.continual_learning.detect_task_shift(current_perf, self.performance_history):
                    self.continual_learning.protect_knowledge()
                self._publish_metrics()  
            else:
                self._process_basic_command(input_text)  
        except Exception as e:
            rospy.logerr(f"[SIL] Error: {e}")
            self.response_publisher.publish(String(data="I encountered an error. Please try again."))
    
    def handle_feedback(self, msg):
        feedback_text = msg.data.strip()
        rospy.loginfo(f"[SIL] Received feedback: {feedback_text}")
        if hasattr(self.interaction_manager, 'shared_task_space'):
            self.interaction_manager.shared_task_space.update_agent_belief({
                'feedback_received': True,
                'feedback_content': feedback_text
            })
    
    def handle_command(self, msg):
        command = msg.data.strip()
        
        if command == "get_metrics":
            self._publish_metrics()
        elif command == "save_memory":
            self.memory_module.save_memory()
        elif command == "reset_adaptation":
            self.adaptation_tracker = MutualAdaptationTracker()
    
    def _track_performance(self, input_text: str, result: Dict):
        success = result.get('result', {}).get('success', False)
        
        self.performance_history.append(1.0 if success else 0.0)
        if len(self.performance_history) > 1000:
            self.performance_history.pop(0)
        self.data_logger.log_sil_interaction({
            'human_input': input_text,
            'success': success,
            'belief_alignment': self.interaction_manager.shared_task_space.compute_belief_alignment(),
            'agent_confidence': self.interaction_manager.shared_task_space.agent_belief_state.confidence 
                               if self.interaction_manager.shared_task_space.agent_belief_state else 0.5,
            'adaptation_metrics': self.adaptation_tracker.metrics
        })
    
    def _get_recent_performance(self) -> Dict:
        if not self.performance_history:
            return {'success_rate': 0.5, 'sample_size': 0}
        
        recent = self.performance_history[-self.recent_window:]
        return {
            'success_rate': np.mean(recent),
            'sample_size': len(recent)
        }
    
    def _publish_metrics(self):
        metrics = {
            'performance': self._get_recent_performance(),
            'adaptation': self.adaptation_tracker.metrics,
            'belief_alignment': self.interaction_manager.shared_task_space.compute_belief_alignment(),
            'data_logger_stats': {
                'total_interactions': self.data_logger.sil_metrics['total_interactions'],
                'avg_alignment': np.mean(self.data_logger.sil_metrics['belief_alignments'][-self.recent_window:])
                                if self.data_logger.sil_metrics['belief_alignments'] else 0.5
            }
        }
        
        self.sil_metrics_publisher.publish(String(data=json.dumps(metrics)))
        self.adaptation_publisher.publish(String(data=json.dumps(self.adaptation_tracker.metrics)))
    
    def _start_monitoring(self):
        def monitor_worker():
            rate = rospy.Rate(rospy.get_param("logging/status_publish_rate_hz", 0.1))
            while not rospy.is_shutdown():
                try:
                    status = {
                        'active': True,
                        'sil_enabled': self.enable_sil,
                        'interactions': len(self.performance_history),
                        'current_task': self.continual_learning.current_task_id
                    }
                    self.sil_status_publisher.publish(String(data=json.dumps(status)))
                    rate.sleep()
                except Exception as e:
                    rospy.logerr(f"[SIL] Monitoring error: {e}")
        monitor_thread = threading.Thread(target=monitor_worker, daemon=True)
        monitor_thread.start()
    
    def _start_memory_autosave(self):
        def autosave_worker():
            while not rospy.is_shutdown():
                try:
                    rospy.sleep(self.save_interval)
                    if not rospy.is_shutdown():
                        self.memory_module.save_memory()
                        self.continual_learning.save_checkpoint()
                        rospy.loginfo("[SIL] Auto-save completed")
                except Exception as e:
                    rospy.logerr(f"[SIL] Auto-save error: {e}")
        autosave_thread = threading.Thread(target=autosave_worker, daemon=True)
        autosave_thread.start()
    
    def _setup_llm_configuration(self):
        llm_provider_name = rospy.get_param("models/llm_provider", "openai")
        llm_api_key_from_config = rospy.get_param("models/llm_api_key", None)
        api_key_mapping = {
            "openai": "OPENAI_API_KEY",
            "deepseek": "DEEPSEEK_API_KEY",
            "claude": "ANTHROPIC_API_KEY",
            "gemini": "GEMINI_API_KEY",
            "llama.cpp": None
        }
        api_key_env_var = api_key_mapping.get(llm_provider_name)
        self.api_key_value = None
        
        if api_key_env_var:
            if llm_api_key_from_config and str(llm_api_key_from_config).strip():
                self.api_key_value = llm_api_key_from_config
            else:
                self.api_key_value = os.getenv(api_key_env_var)
                if not self.api_key_value:
                    raise ValueError(f"API key for {llm_provider_name} not found")
            os.environ[api_key_env_var] = self.api_key_value
    
    def _signal_handler(self, signum, frame):
        rospy.loginfo(f"[SIL] Shutting down gracefully...")
        if self.auto_save_memory:
            self.memory_module.save_memory()
            self.continual_learning.save_checkpoint()
        rospy.signal_shutdown("SIL Robot Controller shutdown")
    
    def _process_basic_command(self, input_text):
        rospy.logwarn("[SIL] SIL is disabled, processing basic command")
        parsed_intent = self.command_parser.parse_command(input_text)
        if parsed_intent:
            response = self.action_executor.execute_action(parsed_intent)
            self.response_publisher.publish(String(data=response))
        else:
            self.response_publisher.publish(String(data="I didn't understand that command."))

    def run(self):
        rospy.loginfo("[SIL] Robot Controller running with full co-adaptation...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("[SIL] Keyboard interrupt")
        finally:
            self._signal_handler(signal.SIGTERM, None)

def main():
    try:
        controller = EnhancedRobotController()
        controller.run()
    except Exception as e:
        rospy.logerr(f"[SIL] Fatal error: {e}")
        raise

if __name__ == '__main__':
    main()
