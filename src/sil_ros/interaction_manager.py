#!/usr/bin/env python
import rospy
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import re
import os
from typing import Dict, List, Any, Optional, Tuple
from std_msgs.msg import String
from datetime import datetime
import threading
import json
from sentence_transformers import SentenceTransformer
from sklearn.metrics.pairwise import cosine_similarity
from dataclasses import dataclass
from sil_ros.config import SILConfig

# ============= BELIEF STATE =============
@dataclass
class BeliefState:
    task_embedding: np.ndarray
    confidence: float
    uncertainty_map: Dict[str, float]
    intent_distribution: np.ndarray
    temporal_context: List[np.ndarray]

# ============= LATENT TASK ENCODER =============
class LatentTaskEncoder(nn.Module):
    def __init__(self, input_dim=768, latent_dim=256, hidden_dims=None, dropout=0.2):
        super().__init__()
        hidden_dims = list(hidden_dims) if hidden_dims else [512, 384]
        layers = []
        prev_dim = input_dim
        for idx, hidden in enumerate(hidden_dims):
            layers.append(nn.Linear(prev_dim, hidden))
            layers.append(nn.ReLU())
            layers.append(nn.BatchNorm1d(hidden))
            if idx == 0 and dropout and dropout > 0.0:
                layers.append(nn.Dropout(dropout))
            prev_dim = hidden
        layers.append(nn.Linear(prev_dim, latent_dim))
        layers.append(nn.Tanh())
        self.encoder = nn.Sequential(*layers)

    def forward(self, x):
        return self.encoder(x)
    
# ============= CO-ADAPTATION MECHANISM =============
class CoAdaptationDynamics:
    def __init__(self, latent_dim=256, alpha_agent=0.1, alpha_human=0.05,
                 influence_init_std=0.01, influence_matrix_lr=0.01):
        self.W_ha = np.random.randn(latent_dim, latent_dim) * influence_init_std
        self.W_ah = np.random.randn(latent_dim, latent_dim) * influence_init_std
        self.alpha_agent = alpha_agent          
        self.beta_human = alpha_human          
        self.matrix_lr = influence_matrix_lr    
        self.gradient_history = []

    def compute_mutual_influence(self, human_belief: np.ndarray,
                                agent_belief: np.ndarray,
                                interaction_success: float) -> Tuple[np.ndarray, np.ndarray]:
        human_to_agent = np.tanh(self.W_ha @ human_belief)
        agent_to_human = np.tanh(self.W_ah @ agent_belief)
        influence_strength = 0.5 + 0.5 * interaction_success
        delta_agent = self.alpha_agent * influence_strength * human_to_agent
        delta_human = self.beta_human * influence_strength * agent_to_human
        self.gradient_history.append({
            'W_ha_grad': np.outer(human_to_agent, human_belief),
            'W_ah_grad': np.outer(agent_to_human, agent_belief)
        })
        return delta_agent, delta_human

    def update_influence_matrices(self, interaction_success: float):
        if not self.gradient_history:
            return
        avg_W_ha_grad = np.mean([g['W_ha_grad'] for g in self.gradient_history], axis=0)
        avg_W_ah_grad = np.mean([g['W_ah_grad'] for g in self.gradient_history], axis=0)
        self.W_ha += self.matrix_lr * interaction_success * avg_W_ha_grad
        self.W_ah += self.matrix_lr * interaction_success * avg_W_ah_grad
        self.gradient_history = []

    def adapt_beliefs(self, human_state: BeliefState, agent_state: BeliefState,
                      interaction_success: float) -> Tuple[BeliefState, BeliefState]:
        delta_a, delta_h = self.compute_mutual_influence(
            human_state.task_embedding, 
            agent_state.task_embedding,
            interaction_success
        )
        agent_state.task_embedding += delta_a * interaction_success
        human_state.task_embedding += delta_h * (2.0 - interaction_success)
        agent_state.task_embedding = agent_state.task_embedding / np.linalg.norm(agent_state.task_embedding)
        human_state.task_embedding = human_state.task_embedding / np.linalg.norm(human_state.task_embedding)
        alignment = cosine_similarity(
            agent_state.task_embedding.reshape(1, -1),
            human_state.task_embedding.reshape(1, -1)
        )[0, 0]
        agent_state.confidence = 0.7 * agent_state.confidence + 0.3 * alignment
        human_state.confidence = 0.8 * human_state.confidence + 0.2 * alignment
        return human_state, agent_state
    
# ============= SHARED TASK SPACE =============
class SharedTaskSpace:
    def __init__(self, config: SILConfig = None):
        self.config = config or SILConfig.load()
        enc = self.config.encoder
        coad = self.config.co_adaptation
        binit = self.config.belief_init
        self.latent_dim = enc.latent_dim
        self.sentence_encoder = SentenceTransformer(enc.sentence_encoder_model)
        self.task_encoder = LatentTaskEncoder(
            input_dim=enc.input_dim,
            latent_dim=enc.latent_dim,
            hidden_dims=enc.hidden_dims,
            dropout=enc.dropout,
        )
        self.task_encoder.eval()
        self.co_adaptation = CoAdaptationDynamics(
            latent_dim=self.latent_dim,
            alpha_agent=coad.alpha_agent,
            alpha_human=coad.alpha_human,
            influence_init_std=coad.influence_init_std,
            influence_matrix_lr=coad.influence_matrix_lr,
        )
        self.eta = list(coad.belief_update_eta)
        self.confidence_blend_agent = coad.confidence_blend_agent
        self.confidence_blend_human = coad.confidence_blend_human
        self.init_human_confidence = binit.human_confidence  
        self.init_agent_confidence = binit.agent_confidence  
        self.init_noise_std = binit.init_noise_std          
        self.intent_dim = binit.intent_distribution_dim
        self.human_belief_state = None
        self.agent_belief_state = None
        self.current_task_representation = {}
        self.shared_plan = []
        self.task_history = []
        self.misalignment_threshold = self.config.alignment.misalignment_threshold  
        self.uncertainty_source_threshold = self.config.alignment.uncertainty_source_threshold
        rospy.loginfo(
            f"[SharedTaskSpace] Neural latent space initialized "
            f"(d={self.latent_dim}, encoder={enc.sentence_encoder_model})"
        )
    
    def project_to_latent(self, text: str) -> np.ndarray:
        sentence_emb = self.sentence_encoder.encode(text)
        with torch.no_grad():
            tensor_emb = torch.FloatTensor(sentence_emb).unsqueeze(0)
            latent = self.task_encoder(tensor_emb).squeeze().numpy()
        return latent
    
    def initialize_beliefs(self, human_input: str, context: Dict):
        human_latent = self.project_to_latent(human_input)
        self.human_belief_state = BeliefState(
            task_embedding=human_latent,
            confidence=self.init_human_confidence,           
            uncertainty_map={},
            intent_distribution=np.ones(self.intent_dim) / self.intent_dim,
            temporal_context=[]
        )
        agent_latent = human_latent + np.random.randn(self.latent_dim) * self.init_noise_std
        self.agent_belief_state = BeliefState(
            task_embedding=agent_latent,
            confidence=self.init_agent_confidence,            
            uncertainty_map={'intent': 0.3, 'parameters': 0.4},
            intent_distribution=np.ones(self.intent_dim) / self.intent_dim,
            temporal_context=[]
        )

    def evolve_beliefs(self, human_input: str, agent_response: str, success: float):
        if not self.human_belief_state or not self.agent_belief_state:
            self.initialize_beliefs(human_input, {})
            return
        
        human_latent = self.project_to_latent(human_input)
        agent_latent = self.project_to_latent(agent_response)
        delta_agent, delta_human = self.co_adaptation.compute_mutual_influence(
            self.human_belief_state.task_embedding,
            self.agent_belief_state.task_embedding,
            success
        )

        e = self.eta 
        self.agent_belief_state.task_embedding = (
            e[0] * self.agent_belief_state.task_embedding
            + e[1] * agent_latent
            + e[2] * delta_agent
        )
        self.human_belief_state.task_embedding = (
            e[3] * self.human_belief_state.task_embedding
            + e[4] * human_latent
            + e[5] * delta_human
        )
        self.human_belief_state.task_embedding /= np.linalg.norm(self.human_belief_state.task_embedding)
        self.agent_belief_state.task_embedding /= np.linalg.norm(self.agent_belief_state.task_embedding)
        alignment = self.compute_belief_alignment()
        wh = self.confidence_blend_human
        wa = self.confidence_blend_agent
        self.human_belief_state.confidence = float(np.clip(
            wh * self.human_belief_state.confidence + (1.0 - wh) * success, 0.1, 1.0))
        self.agent_belief_state.confidence = float(np.clip(
            wa * self.agent_belief_state.confidence + (1.0 - wa) * success, 0.1, 1.0))
        self.co_adaptation.update_influence_matrices(success)
    
    def update_uncertainty(self, human_uncertainty: Dict[str, float], agent_uncertainty: Dict[str, float]):
        if self.human_belief_state:
            self.human_belief_state.uncertainty_map.update(human_uncertainty)
        if self.agent_belief_state:
            self.agent_belief_state.uncertainty_map.update(agent_uncertainty)

    def update_human_belief(self, belief_update: Dict):
        if self.human_belief_state:
            for key, value in belief_update.items():
                if hasattr(self.human_belief_state, key):
                    setattr(self.human_belief_state, key, value)
    def update_agent_belief(self, belief_update: Dict):
        if self.agent_belief_state:
            for key, value in belief_update.items():
                if hasattr(self.agent_belief_state, key):
                    setattr(self.agent_belief_state, key, value)
    
    def compute_belief_alignment(self) -> float:
        if not self.human_belief_state or not self.agent_belief_state:
            return 0.0
        cos_sim = cosine_similarity(
            self.human_belief_state.task_embedding.reshape(1, -1),
            self.agent_belief_state.task_embedding.reshape(1, -1)
        )[0, 0]
        normalised_sim = (1.0 + cos_sim) / 2.0
        conf = float(np.sqrt(max(self.human_belief_state.confidence, 0.0)
                             * max(self.agent_belief_state.confidence, 0.0)))
        rho = normalised_sim * conf
        return max(0.0, min(1.0, rho))
    
    def belief_alignment(self) -> float:
        return self.compute_belief_alignment()
    
    def needs_alignment(self) -> bool:
        return self.compute_belief_alignment() < self.misalignment_threshold
    
    def detect_misalignment(self) -> Tuple[bool, Dict]:
        alignment = self.compute_belief_alignment()
        is_misaligned = alignment < self.misalignment_threshold
        diagnosis = {
            'alignment_score': alignment,
            'is_misaligned': is_misaligned,
            'uncertainty_sources': []
        }
        
        if is_misaligned and self.agent_belief_state:
            for aspect, uncertainty in self.agent_belief_state.uncertainty_map.items():
                if uncertainty > self.uncertainty_source_threshold:
                    diagnosis['uncertainty_sources'].append(aspect)
        
        return is_misaligned, diagnosis
    
    def generate_alignment_questions(self) -> List[str]:
        questions = [] 
        if self.needs_alignment():
            if self.agent_belief_state:
                for aspect in self.agent_belief_state.uncertainty_map:
                    if self.agent_belief_state.uncertainty_map[aspect] > self.uncertainty_source_threshold:
                        if aspect == 'intent':
                            questions.append("I want to make sure I understand your goal correctly. Could you clarify?")
                        elif aspect == 'parameters':
                            questions.append("Could you provide more specific details?")
        return questions
    
    def update_from_interaction(self, human_input: str, agent_response: str, success: float):
        self.evolve_beliefs(human_input, agent_response, success)
        self.task_history.append({
            'human_input': human_input,
            'agent_response': agent_response,
            'success': success,
            'timestamp': datetime.now()
        })
        self.current_task_representation = {
            'latest_human_input': human_input,
            'latest_agent_response': agent_response,
            'human_confidence': self.human_belief_state.confidence if self.human_belief_state else 0.0,
            'agent_confidence': self.agent_belief_state.confidence if self.agent_belief_state else 0.0,
            'alignment_score': self.compute_belief_alignment()
        }
    
    def update_shared_plan(self, plan_steps: List[Dict]):
        self.shared_plan = plan_steps
        self.current_task_representation['plan'] = plan_steps

# ============= INTERACTION MANAGER =============
class SymbioticInteractionManager: 
    def __init__(self, llm_interface, memory_module, action_executor, perception_module,
                 config: SILConfig = None):
        self.llm_interface = llm_interface
        self.memory_module = memory_module
        self.action_executor = action_executor
        self.perception_module = perception_module
        self.config = config or SILConfig.load()
        self.shared_task_space = SharedTaskSpace(config=self.config)
        self.destinations = self.action_executor.get_destinations()
        self.encoder_optimizer = optim.Adam(
            self.shared_task_space.task_encoder.parameters(),
            lr=self.config.encoder.learning_rate
        )
        self.triplet_loss = nn.TripletMarginLoss(
            margin=self.config.training.triplet_margin, p=2
        )
        self.ewc_lambda = self.config.training.ewc_lambda  
        self.uncertainty_estimator = UncertaintyEstimator(llm_interface, config=self.config)
        self.suggestion_generator = ProactiveSuggestionGenerator(memory_module, llm_interface)
        self.clarification_generator = ClarificationGenerator(llm_interface)
        self.clarification_threshold = self.config.clarification_threshold
        self.proactive_suggestions_enabled = rospy.get_param(
            "sil/proactive_suggestions_enabled", True
        )
        self.plan_step_delay = rospy.get_param("interaction/plan_step_delay", 0.5)
        self.history_context_turns = int(
            rospy.get_param("interaction/conversation_history_context", 3)
        )
        self.default_forward = float(
            rospy.get_param("sil/action_generation/default_distances/forward", 1.0))
        self.default_backward = float(
            rospy.get_param("sil/action_generation/default_distances/backward", 1.0))
        self.default_turn_angle = float(
            rospy.get_param("sil/action_generation/default_angles/turn", 90.0))
        self.default_rotate_angle = float(
            rospy.get_param("sil/action_generation/default_angles/rotate", 360.0))
        self.default_circle_radius = float(
            rospy.get_param("sil/action_generation/default_circle_radius", 0.5))
        self.follow_up_enabled = bool(rospy.get_param("sil/follow_up/enabled", True))
        self.follow_up_num_suggestions = int(rospy.get_param("sil/follow_up/num_suggestions", 2))
        self.follow_up_max_sentences = int(rospy.get_param("sil/follow_up/max_sentences", 2))
        self.follow_up_use_emojis = bool(rospy.get_param("sil/follow_up/use_emojis", False))
        self.follow_up_avoid_self_praise = bool(rospy.get_param("sil/follow_up/avoid_self_praise", True))
        self.follow_up_tone = rospy.get_param("sil/follow_up/tone", "natural")
        self.persistent_context = {
            'pending_action': None,
            'last_suggestion': None,
            'awaiting_response': False,
            'is_executing_plan': False,
            'current_plan': [],
            'plan_step_index': 0,
            'conversation_history': [],
            'last_action': None,
            'last_command': None,
            'session_start': datetime.now(),
            'success_history': []
        }
        self.adaptation_metrics = {
            'alignment_history': [],
            'success_rate': 0.5,
            'adaptation_events': 0
        }

        self.response_publisher = rospy.Publisher(
            rospy.get_param("topics/llm_output", "/llm_output"), String, queue_size=10)
        self.tts_publisher = rospy.Publisher(
            rospy.get_param("topics/tts_text", "/tts_text"), String, queue_size=10)
        self.belief_publisher = rospy.Publisher(
            rospy.get_param("topics/sil_beliefs", "/sil_beliefs"), String, queue_size=10)
        self.interaction_lock = threading.Lock()
        rospy.loginfo("[SIL] Symbiotic Interaction Manager initialized")
    
    def set_controller_reference(self, controller):
        self.controller_ref = controller

    def _task_signature(self, intent: dict) -> str:
        actions = intent.get('actions')
        if actions:
            names = [a.get('action', '') for a in actions if isinstance(a, dict)]
            return names[0] if names else intent.get('action_type', 'UNKNOWN')
        return intent.get('action', intent.get('action_type', 'UNKNOWN'))

    def _notify_interaction_complete(self, human_input: str, intent: dict, success: bool):
        self.last_task_signature = self._task_signature(intent)
        ctrl = getattr(self, 'controller_ref', None)
        if ctrl is not None and hasattr(ctrl, 'on_interaction_complete'):
            try:
                ctrl.on_interaction_complete(human_input, self.last_task_signature, success)
            except Exception as e:
                rospy.logerr(f"[SIL] on_interaction_complete failed: {e}")
    
    def fine_tune_on_interaction(self, anchor_input, positive_input, negative_input):
        rospy.loginfo("[SIL] Starting fine-tuning of LatentTaskEncoder...")
        encoder = self.shared_task_space.task_encoder
        sent_encoder = self.shared_task_space.sentence_encoder
        encoder.train()
        anchor_sent   = torch.FloatTensor(sent_encoder.encode(anchor_input)).unsqueeze(0)
        positive_sent = torch.FloatTensor(sent_encoder.encode(positive_input)).unsqueeze(0)
        negative_sent = torch.FloatTensor(sent_encoder.encode(negative_input)).unsqueeze(0)
        self.encoder_optimizer.zero_grad()
        triplet_batch = torch.cat([anchor_sent, positive_sent, negative_sent], dim=0)  
        triplet_latent = encoder(triplet_batch)                                        
        anchor_latent, positive_latent, negative_latent = triplet_latent.unbind(dim=0)
        anchor_latent   = anchor_latent.unsqueeze(0)   
        positive_latent = positive_latent.unsqueeze(0)
        negative_latent = negative_latent.unsqueeze(0)
        task_loss = self.triplet_loss(anchor_latent, positive_latent, negative_latent)
        ewc_safeguard = self.controller_ref.continual_learning
        ewc_loss = torch.tensor(0.0)
        ewc_lambda = self.ewc_lambda  
        for task_id, fisher in ewc_safeguard.fisher_matrices.items():
            if task_id not in ewc_safeguard.optimal_params:
                continue
            for name, param in encoder.named_parameters():
                if name in fisher and name in ewc_safeguard.optimal_params[task_id]:
                    fisher_diag = torch.FloatTensor(fisher[name])
                    optimal_val = torch.FloatTensor(ewc_safeguard.optimal_params[task_id][name])
                    ewc_loss = ewc_loss + (fisher_diag * (param - optimal_val).pow(2)).sum()

        ewc_loss = (ewc_lambda / 2.0) * ewc_loss
        total_loss = task_loss + ewc_loss
        total_loss.backward()
        gradients = {
            name: p.grad.detach().cpu().numpy().copy()
            for name, p in encoder.named_parameters()
            if p.grad is not None
        }
        self.encoder_optimizer.step()
        encoder.eval()
        rospy.loginfo(
            f"[SIL] Fine-tuning complete. "
            f"L_triplet={task_loss.item():.4f}, L_ewc={ewc_loss.item():.4f}"
        )
        return gradients
    
    def collect_task_gradients(self, texts, max_samples=16):
        import numpy as np
        encoder = self.shared_task_space.task_encoder
        sent_encoder = self.shared_task_space.sentence_encoder
        encoder.eval() 
        grads = []
        for t in [x for x in texts if x][:max_samples]:
            emb = torch.FloatTensor(sent_encoder.encode(t)).unsqueeze(0)
            encoder.zero_grad()
            z = encoder(emb)
            (0.5 * (z ** 2).sum()).backward()
            g = {name: p.grad.detach().cpu().numpy().copy()
                 for name, p in encoder.named_parameters() if p.grad is not None}
            if g:
                grads.append(g)
        encoder.zero_grad()
        return grads
    
    def process_bidirectional_interaction(self, human_input: str, context: Dict = None) -> Dict:
        with self.interaction_lock:
            context = context or {}
            rospy.loginfo(f"[SIL] Processing: {human_input}")
            rospy.loginfo(f"[SIL] Current persistent context: {self.persistent_context}") 
            rospy.loginfo(f"[SIL] Awaiting: {self.persistent_context['awaiting_response']}, "
                         f"Pending: {self.persistent_context['pending_action'] is not None}")
            self.persistent_context['last_command'] = human_input
            self.persistent_context['conversation_history'].append({
                'type': 'human',
                'message': human_input,
                'timestamp': datetime.now()
            })
            if self.persistent_context['is_executing_plan']:
                rospy.logwarn("[SIL] User provided new input while executing a plan. Pausing and refining.")
                return self._refine_plan(human_input, context)
            if self.persistent_context['awaiting_response'] and self.persistent_context['pending_action']:
                result = self._handle_pending_response(human_input, context)
                if result:
                    return result
            if any(k in human_input.lower() for k in
                   ("actually mean", "i meant", "not ", "no,", "that's wrong", "i do not see", "that is not")):
                self.memory_module.penalize_last_episode()
            parsed_intent, confidence = self._parse_with_enhanced_llm(human_input, context)
            self._update_beliefs(human_input, context, parsed_intent)
            rospy.loginfo(f"[SIL] Parsed intent type: {parsed_intent.get('action_type')}") 
            if parsed_intent.get('action_type') == 'ACTIONS_WITH_CONVERSATION':
                conversation = parsed_intent.get('conversation', '')
                actions = parsed_intent.get('actions', [])
                if conversation:
                    self.response_publisher.publish(String(data=conversation))
                if actions:
                    self.persistent_context['is_executing_plan'] = True
                    self.persistent_context['current_plan'] = actions
                    self.persistent_context['plan_step_index'] = 0
                    self.persistent_context['plan_success'] = True
                    self.persistent_context['plan_failed_steps'] = []
                    result = self._execute_next_plan_step(context)
                    return {'type': 'PLAN_STARTED', 'result': result}
            elif parsed_intent.get('action_type') == 'SUGGESTION_WITH_PENDING':
                return self._handle_suggestion_with_pending(parsed_intent, context)    
            else:
                execution_result = self._execute_intent(parsed_intent, context)
            self._update_from_execution(human_input, execution_result,
                                        bool(execution_result.get('success', False)))
            if (execution_result.get('success') and
                self.proactive_suggestions_enabled and
                not hasattr(self.action_executor, '_in_sequence_timing')):
                self._generate_dynamic_follow_up(parsed_intent, human_input, execution_result)
            self._store_interaction(human_input, parsed_intent, execution_result, confidence,
                                    agent_response=execution_result.get('response', ''))
            self._notify_interaction_complete(human_input, parsed_intent,
                                              bool(execution_result.get('success', False)))
            return {'type': 'EXECUTION_RESULT', 'result': execution_result}
    
    def _refine_plan(self, new_human_input: str, context: Dict) -> Dict:
        """Pauses the current plan and asks the LLM to refine it."""
        self.persistent_context['is_executing_plan'] = False
        plan = self.persistent_context['current_plan']
        idx = self.persistent_context['plan_step_index']
        remaining_steps = plan[idx:]
        refinement_prompt = f"""
        You were in the middle of a multi-step plan.
        Original Goal: {self.persistent_context.get('last_command', 'unknown')}
        Completed Steps: {plan[:idx]}
        Remaining Steps: {remaining_steps}
        
        The user has just interrupted with a new instruction: "{new_human_input}"
        
        Based on this new information, please generate a new, revised list of 'Action' steps to achieve the user's updated goal.
        Start with a conversational response acknowledging the change.
        """
        return self.process_bidirectional_interaction(refinement_prompt, context)
    
    def _update_beliefs(self, human_input: str, context: Dict, parsed_intent: Dict = None):
        if self.shared_task_space.human_belief_state is None:
            self.shared_task_space.initialize_beliefs(human_input, context)
        else:
            agent_interpretation = self._generate_agent_interpretation(parsed_intent or {})
            success_score = np.mean(self.persistent_context['success_history'][-10:]) \
                          if self.persistent_context['success_history'] else 0.5
            self.shared_task_space.evolve_beliefs(
                human_input, agent_interpretation, success_score)
        alignment = self.shared_task_space.compute_belief_alignment()
        self.adaptation_metrics['alignment_history'].append(alignment)
        self._publish_beliefs()
    
    def _update_from_execution(self, human_input: str, result: Dict, success: bool):
        self.persistent_context['success_history'].append(1.0 if success else 0.0)
        if len(self.persistent_context['success_history']) > 100:
            self.persistent_context['success_history'].pop(0)
        self.adaptation_metrics['success_rate'] = np.mean(
            self.persistent_context['success_history'][-20:]
        ) if self.persistent_context['success_history'] else 0.5
        self.adaptation_metrics['adaptation_events'] += 1
    
    def _publish_beliefs(self):
        if self.shared_task_space.human_belief_state and self.shared_task_space.agent_belief_state:
            belief_data = {
                'alignment': self.shared_task_space.compute_belief_alignment(),
                'human_confidence': self.shared_task_space.human_belief_state.confidence,
                'agent_confidence': self.shared_task_space.agent_belief_state.confidence,
                'adaptation_metrics': self.adaptation_metrics
            }
            self.belief_publisher.publish(String(data=json.dumps(belief_data)))
    
    def _generate_agent_interpretation(self, parsed_intent: dict) -> str:
        actions = parsed_intent.get('actions') or (
            [parsed_intent] if parsed_intent.get('action') else [])
        parts = []
        for a in actions:
            act = (a.get('action') or '').replace('_', ' ').lower()
            if not act:
                continue
            if a.get('destination_name'):
                act += f" {a['destination_name'].replace('_', ' ')}"
            elif a.get('angle') is not None:
                act += f" {a['angle']} degrees"
            elif a.get('distance') is not None:
                act += f" {a['distance']} meters"
            parts.append(act)
        if parts:
            return "; ".join(parts)
        return parsed_intent.get('conversation') or parsed_intent.get('action_type', 'converse')
    
    def _handle_misalignment(self, diagnosis: Dict, human_input: str, context: Dict) -> Dict:
        questions = self.shared_task_space.generate_alignment_questions()
        if questions:
            question = questions[0]
            self.response_publisher.publish(String(data=question)) 
            return {
                'type': 'CLARIFICATION_REQUEST',
                'message': question,
                'diagnosis': diagnosis
            }
        return {'type': 'CONTINUE'}

    def _parse_with_enhanced_llm(self, human_input: str, context: Dict) -> Tuple[Dict, float]:
        input_lower = human_input.lower()
        if 'coordinates' in input_lower or 'x=' in input_lower or 'y=' in input_lower:
            coords = self._extract_coordinates(input_lower)
            if coords:
                rospy.loginfo(f"[SIL] Direct coordinate command detected: {coords}")
                return {
                    'action_type': 'GO_TO_COORDINATES',
                    'coordinates': coords
                }, 0.9
        prompt = self._build_enhanced_prompt()
        try:
            robot_state = self._get_robot_state()
            history_context = self._get_conversation_context()
            system_msg = f"""{prompt}
{history_context}
Current robot state:
- Position: ({robot_state.get('x', 0)}, {robot_state.get('y', 0)}, {robot_state.get('z', 0)})
- Orientation: {robot_state.get('yaw', 0)}° facing {robot_state.get('direction', 'north')}

User command: {human_input}

IMPORTANT: 
1. First provide a natural, conversational response
2. Then generate Action lines for any tasks
3. For questions, just provide conversation without actions"""
            llm_response = self.llm_interface.call_llm(system_msg, human_input)
            rospy.loginfo(f"[SIL] LLM Response: {llm_response[:100]}...")

            if 'PendingAction:' in llm_response:
                conversation = self._extract_conversation(llm_response)
                pending_action_line = [line for line in llm_response.split('\n') if 'PendingAction:' in line]
                if pending_action_line:
                    action_desc = pending_action_line[0].split('PendingAction:')[1].strip()
                    parsed_action = self._parse_action_description(action_desc, human_input)
                    if parsed_action:
                        rospy.loginfo(f"[SIL] Recognized suggestion with pending action: {parsed_action}")
                        return {
                            'action_type': 'SUGGESTION_WITH_PENDING',
                            'conversation': conversation,
                            'suggestion': conversation,
                            'pending_action': parsed_action
                        }, 0.85
            conversation = self._extract_conversation(llm_response)
            actions = self._extract_actions_from_llm(llm_response, human_input)
            if actions:
                if len(actions) == 1:
                    return {
                        'action_type': 'ACTIONS_WITH_CONVERSATION',
                        'conversation': conversation,
                        'actions': actions,
                        'single_action': actions[0]
                    }, 0.8
                else:
                    return {
                        'action_type': 'ACTIONS_WITH_CONVERSATION',
                        'conversation': conversation,
                        'actions': actions
                    }, 0.8
            return {
                'action_type': 'CONVERSATION',
                'response': llm_response
            }, 0.6
            
        except Exception as e:
            rospy.logerr(f"[SIL] LLM error: {e}")
            return {'action_type': 'UNKNOWN'}, 0.2
    
    def _build_enhanced_prompt(self) -> str:
        registry = getattr(self.action_executor, 'skill_registry', None)
        available_actions = registry.describe_for_prompt() if registry else ""
        return ("""You are a helpful robot assistant. When given commands:

    RULES:
    1. If you make a suggestion that requires user confirmation, you MUST:
    - First, write your conversational question (e.g., \"Would you like me to go to the kitchen?\").
    - On a NEW LINE, add a `PendingAction:` tag with the specific action the robot should perform if the user confirms.
    - Example:
        User: \"Where can I get water?\"
        Response: \"You can get water from the kitchen. Would you like me to navigate there?
        PendingAction: NAVIGATE_TO_DESTINATION(destination_name='kitchen')\"

    2. If a command contains ambiguous references (like \"there\", \"that\", \"it\") without clear context:
    - DO NOT make assumptions about what the user means by those terms.
    - DO NOT generate actions based on ambiguous terms
    - Instead, use the conversation history to suggest the most likely intended action.

    3. For clear commands, provide a friendly response followed by Action lines

    4. When making suggestions, be explicit about what you're suggesting:
    - Use phrases like \"Would you like me to [specific action]?\"
    - Make it clear you're waiting for confirmation
    5. If the user asks a question, respond conversationally without actions
    6. ALWAYS provide a friendly, conversational response first
    7. THEN provide Action lines if applicable
    8. If you are unsure about the user's intent, ask for clarification instead of guessing
    9. If a user asks to go to a place that is NOT in the list of available destinations, you MUST NOT
       guess a location. Instead, ask a clarifying question and suggest the most likely known destinations.

    Compose any task as a sequence of the available action primitives below. You do not need to
    enumerate higher-level tasks — build them from these primitives. Emit each step on its own line as
    `Action N: PRIMITIVE(arg=value, ...)` using the exact primitive names, or in plain language.

    Format your response like:
    [Your friendly conversational response here]
    Action 1: [primitive with parameters]
    Action 2: [next primitive if needed]

    AVAILABLE ACTION PRIMITIVES:
""" + available_actions + """

    Examples:

    User: \"quickly go to the professor office and tell me what you see there. don't forget photos\"
    Response: I'll head to the professor's office and document what I find.
    Action 1: NAVIGATE_TO_DESTINATION(destination_name='professor_office')
    Action 2: DESCRIBE_SURROUNDINGS
    Action 3: SEND_IMAGE

    User: \"dance for me\"
    Response: Here's a little routine.
    Action 1: ROTATE(angle=90)
    Action 2: FORWARD(distance=0.5)
    Action 3: ROTATE(angle=180)
    Action 4: BACKWARD(distance=0.5)

    User: \"what can you do?\"
    Response: I can navigate to locations, take photos, describe what I see, and perform movement patterns. How can I help?
    """)
    def _extract_conversation(self, llm_response: str) -> str:
        match = re.search(r'^\s*(Action|PendingAction)', llm_response, re.MULTILINE)
        if match:
            return llm_response[:match.start()].strip()
        return llm_response.strip()
    
    def _extract_actions_from_llm(self, llm_response: str, original_input: str) -> List[Dict]:
        actions = []
        for line in llm_response.split('\n'):
            if re.match(r'Action\s*\d*:?\s*(.+)', line, re.IGNORECASE):
                match = re.match(r'Action\s*\d*:?\s*(.+)', line, re.IGNORECASE)
                action_desc = match.group(1).strip()
                action = self._parse_action_description(action_desc, original_input)
                if action:
                    actions.append(action)
                    rospy.loginfo(f"[SIL] Parsed action: {action}")
        return actions
    
    def _resolve_destination_slug(self, name: Optional[str]) -> Optional[str]:
        if not name:
            return None
        resolver = getattr(self.action_executor, 'dest_resolver', None)
        if resolver is not None:
            slug, _ = resolver.resolve(str(name).replace('_', ' '))
            if slug:
                return slug
        name_l = str(name).lower()
        for display_name, slug in self.destinations.items():
            if name_l in (display_name.lower(), slug.lower()):
                return slug
        return None

    def _parse_action_description(self, desc: str, original: str) -> Optional[Dict]:
        registry = getattr(self.action_executor, 'skill_registry', None)
        if registry is not None:
            structured = registry.parse_structured(desc)
            if structured is not None:
                if structured.get('action') == 'NAVIGATE_TO_DESTINATION':
                    slug = self._resolve_destination_slug(structured.get('destination_name'))
                    if slug:
                        structured['destination_name'] = slug
                        return structured
                else:
                    return structured

        desc_lower = desc.lower()
        if 'coordinates' in desc_lower or 'coordinate' in desc_lower:
            coords = self._extract_coordinates(desc_lower)
            if not coords:
                coords = self._extract_coordinates(original.lower())
            if coords:
                rospy.loginfo(f"[SIL] Parsed coordinates from action: {coords}")
                return {'action': 'GO_TO_COORDINATES', 'coordinates': coords}
        if 'x=' in desc_lower or 'y=' in desc_lower or 'z=' in desc_lower:
            coords = self._extract_coordinates(desc_lower)
            if not coords:
                coords = self._extract_coordinates(original.lower())
            if coords:
                rospy.loginfo(f"[SIL] Parsed direct coordinates: {coords}")
                return {'action': 'GO_TO_COORDINATES', 'coordinates': coords}
        if 'navigate to' in desc_lower or 'go to' in desc_lower or 'navigate_to_destination' in desc_lower:
            if 'coordinate' not in desc_lower:
                destination_name = None
                structured_match = re.search(r"destination_name=['\"]?([^'\")]+)['\"]?", desc_lower)
                if structured_match:
                    destination_name = structured_match.group(1).strip().replace('_', ' ')
                else:
                    for display_name in self.destinations.keys():
                        if display_name.lower() in desc_lower:
                            destination_name = display_name.lower()
                            break
                        slug = self.destinations[display_name]
                        aliases = self.action_executor.dest_resolver.raw.get(slug, {}).get('aliases', [])
                        for alias in aliases:
                            if alias.lower() in desc_lower:
                                destination_name = display_name.lower()
                                break
                        if destination_name:
                            break
                if destination_name:
                    for display_name, slug in self.destinations.items():
                        aliases = self.action_executor.dest_resolver.raw.get(slug, {}).get('aliases', [])
                        if destination_name == display_name.lower() or destination_name in [a.lower() for a in aliases]:
                            rospy.loginfo(f"[SIL] Validated destination '{destination_name}' and mapped to slug '{slug}'")
                            return {'action': 'NAVIGATE_TO_DESTINATION', 'destination_name': slug}
                rospy.logwarn(f"[SIL] A destination was mentioned but could not be validated against the known list: '{destination_name}'")
        if 'move forward' in desc_lower or 'forward' in desc_lower:
            dist = self._extract_number(desc_lower, self.default_forward)
            return {'action': 'FORWARD', 'distance': dist}
        elif 'move backward' in desc_lower or 'backward' in desc_lower:
            dist = self._extract_number(desc_lower, self.default_backward)
            return {'action': 'BACKWARD', 'distance': dist}
        elif 'turn left' in desc_lower:
            angle = self._extract_number(desc_lower, self.default_turn_angle)
            return {'action': 'TURN_LEFT', 'angle': angle}
        elif 'turn right' in desc_lower:
            angle = self._extract_number(desc_lower, self.default_turn_angle)
            return {'action': 'TURN_RIGHT', 'angle': angle}
        elif 'rotate' in desc_lower:
            angle = self._extract_number(desc_lower, self.default_rotate_angle)
            return {'action': 'ROTATE', 'angle': angle}
        elif 'send image' in desc_lower or 'photo' in desc_lower:
            return {'action': 'SEND_IMAGE'}
        elif 'describe' in desc_lower:
            return {'action': 'DESCRIBE_SURROUNDINGS'}
        elif 'report coordinate' in desc_lower:
            return {'action': 'REPORT_COORDINATES'}
        elif 'report orientation' in desc_lower:
            return {'action': 'REPORT_ORIENTATION'}
        elif 'circle' in desc_lower:
            radius = self._extract_number(desc_lower, self.default_circle_radius)
            return {'action': 'CIRCULAR_MOTION', 'radius': radius, 'angle': 360.0}
        elif 'stop' in desc_lower:
            return {'action': 'STOP'}
        return None
    
    def _execute_next_plan_step(self, context: Dict) -> Dict:
        if not self.persistent_context['is_executing_plan']:
            return {'success': True, 'response': "No plan to execute."}
        plan = self.persistent_context['current_plan']
        idx = self.persistent_context['plan_step_index']

        if idx >= len(plan):
            original_command = self.persistent_context.get('last_command', 'the plan')
            all_success = self.persistent_context.get('plan_success', True)
            failed_steps = self.persistent_context.get('plan_failed_steps', [])
            intent_summary = {'action_type': 'MULTI_STEP_PLAN', 'steps': len(plan)}
            exec_result = {'success': all_success, 'failed_steps': failed_steps}
            dynamic_response = self._generate_dynamic_follow_up(intent_summary, original_command, exec_result)
            self._update_from_execution(original_command, exec_result, all_success)
            self._store_interaction(original_command, intent_summary, exec_result, 1.0,
                                    agent_response=dynamic_response)
            self._notify_interaction_complete(original_command, intent_summary, all_success)
            self.persistent_context['is_executing_plan'] = False
            self.persistent_context['current_plan'] = []
            self.persistent_context['plan_step_index'] = 0
            self.persistent_context['plan_success'] = True
            self.persistent_context['plan_failed_steps'] = []
            self.action_executor.speak(dynamic_response) 
            return {'success': True, 'response': dynamic_response, 'is_plan_ongoing': False}
        action = plan[idx]
        rospy.loginfo(f"[SIL] Executing plan step {idx + 1}/{len(plan)}: {action}")
        step_success = False
        try:
            step_success = bool(self.action_executor.execute_action(action))
        except Exception as e:
            rospy.logerr(f"[SIL] Action step {idx + 1} raised: {e}")
            step_success = False
        if not step_success:
            self.persistent_context['plan_success'] = False
            self.persistent_context.setdefault('plan_failed_steps', []).append(
                action.get('action', f'step {idx + 1}'))
            rospy.logwarn(f"[SIL] Plan step {idx + 1} ({action.get('action')}) did not succeed.")
        self.persistent_context['plan_step_index'] += 1
        is_ongoing = self.persistent_context['plan_step_index'] < len(plan)
        if is_ongoing:
            rospy.Timer(rospy.Duration(self.plan_step_delay),
                        lambda event: self._execute_next_plan_step(context), oneshot=True)
        else:
            self._execute_next_plan_step(context)
        return {
            'success': step_success,
            'response': f"Completed step {idx + 1}",
            'is_plan_ongoing': is_ongoing
        }
    
    def _handle_suggestion_with_pending(self, parsed_intent: Dict, context: Dict) -> Dict:
        conversation = parsed_intent.get('conversation', '')
        suggestion = parsed_intent.get('suggestion', '')
        if conversation:
            self.response_publisher.publish(String(data=conversation))
            rospy.sleep(self.plan_step_delay)
        if suggestion:
            self.response_publisher.publish(String(data=suggestion))
        self.persistent_context['pending_action'] = parsed_intent.get('pending_action')
        self.persistent_context['awaiting_response'] = True
        self.persistent_context['last_suggestion'] = suggestion
        return {
            'type': 'SUGGESTION',
            'response': conversation + "\n" + suggestion,
            'awaiting_confirmation': True
        }
    
    def _handle_pending_response(self, input_text: str, context: Dict) -> Optional[Dict]:
        if not self.persistent_context.get('awaiting_response'):
            return None
        pending_action = self.persistent_context.get('pending_action')
        last_command = self.persistent_context.get('last_command', '')
        last_suggestion = self.persistent_context.get('last_suggestion', '')
        classification_prompt = f"""
        The robot asked: "{last_suggestion}"
        The user was previously asked to: "{last_command}"
        The user responded: "{input_text}"
        
        Classify the user's response as:
        - CONFIRMATION (user agrees/wants the suggested action)
        - CLARIFICATION (user is providing additional details)  
        - REJECTION (user declines/cancels the action)
        - NEW_COMMAND (user is giving a completely different instruction)
        - CONTINUATION (user wants to continue with what was discussed)
        
        Respond with just the classification word.
        """
        try:
            response_type = self.llm_interface.call_llm(classification_prompt, input_text).strip().upper()
            rospy.loginfo(f"[SIL] Classified response as: {response_type}")
            
            if "CONFIRMATION" in response_type:
                if pending_action:
                    rospy.loginfo(f"[SIL] Executing confirmed action: {pending_action}")
                    self.persistent_context['pending_action'] = None
                    self.persistent_context['awaiting_response'] = False
                    self.persistent_context['last_suggestion'] = None
                    return self._execute_intent(pending_action, context)
            
            elif "CLARIFICATION" in response_type:
                if last_command:
                    clarified_command = f"{last_command} {input_text}"
                    self.persistent_context['pending_action'] = None
                    self.persistent_context['awaiting_response'] = False
                    self.persistent_context['last_suggestion'] = None
                    rospy.loginfo(f"[SIL] Processing clarified command: {clarified_command}")
                    return self.process_bidirectional_interaction(clarified_command, context)
            
            elif "CONTINUATION" in response_type or "EXACTLY" in input_text.upper():
                if pending_action:
                    rospy.loginfo(f"[SIL] Continuing with pending action: {pending_action}")
                    self.persistent_context['pending_action'] = None
                    self.persistent_context['awaiting_response'] = False
                    self.persistent_context['last_suggestion'] = None
                    return self._execute_intent(pending_action, context)
            
            elif "REJECTION" in response_type:
                self.persistent_context['pending_action'] = None
                self.persistent_context['awaiting_response'] = False
                self.persistent_context['last_suggestion'] = None
                return {'type': 'CANCELLED', 'result': {'success': True, 'response': 'Understood, cancelled.'}}
            self.persistent_context['pending_action'] = None
            self.persistent_context['awaiting_response'] = False
            self.persistent_context['last_suggestion'] = None
            return None
        except Exception as e:
            rospy.logerr(f"[SIL] Error classifying response: {e}")
            self.persistent_context['pending_action'] = None
            self.persistent_context['awaiting_response'] = False
            self.persistent_context['last_suggestion'] = None
            return None

    def _generate_dynamic_follow_up(self, intent: Dict, original_command: str, execution_result: Dict) -> str:
        try:
            perf_stats = "No performance data available."
            perf_metrics = execution_result.get('performance_metrics')
            if perf_metrics:
                duration = perf_metrics.get('duration', 0.0)
                success_rate = perf_metrics.get('success_rate', 0.0)
                perf_stats = (f"- Duration: {duration:.1f} seconds.\n"
                            f"- Overall Success Rate: {success_rate:.1%}.")
            if hasattr(self.action_executor, 'performance_tracker'):
                perf = self.action_executor.performance_tracker
                if perf['completion_times']:
                    duration = perf['completion_times'][-1]
                    success_rate = perf['success_count'] / max(1, perf['total_actions'])
                    perf_stats = (f"- Duration: {duration:.1f} seconds.\n"
                                f"- Overall Success Rate: {success_rate:.1%}.")
            interaction_data = {
                'human_input': original_command,
                'agent_response': f"Completed a {intent.get('steps', 1)}-step plan.",
                'execution_result': execution_result,
                'context': {'task_type': intent.get('action_type', 'unknown')}
            }
            learning_reflection = self.memory_module.generate_learning_reflection(interaction_data)
            if not learning_reflection:
                learning_reflection = "Every task helps me learn, and this was a good experience."
            suggestion_context = {
                'robot_state': self._get_robot_state(),
                'recent_success': execution_result.get('success', True),
                'completed_task': intent.get('action_type', 'unknown'),
                'user_goal': original_command
            }
            suggestions = self.suggestion_generator.generate_suggestions(intent, suggestion_context)
            if not suggestions:
                suggestions = ["Explore the area", "Return to the starting point"]
            suggestions = suggestions[:max(1, self.follow_up_num_suggestions)]
            suggestions_formatted = "\n".join([f"- {s}" for s in suggestions])

            if not self.follow_up_enabled:
                return ""
            succeeded = execution_result.get('success', True)
            failed_steps = execution_result.get('failed_steps', []) or []
            if succeeded:
                outcome_rule = ("The task completed successfully. Briefly acknowledge "
                                "it is done in a few words.")
            else:
                failed_desc = ", ".join(str(s) for s in failed_steps) or "part of the task"
                outcome_rule = (
                    f"IMPORTANT: the task did NOT fully succeed — these step(s) failed: "
                    f"{failed_desc}. Do NOT claim it was completed. Honestly say what "
                    f"could not be done and, if useful, offer to retry or suggest an "
                    f"alternative. Never state you reached a location or sent something "
                    f"if that step failed.")
            emoji_rule = ("You may use at most one tasteful emoji."
                          if self.follow_up_use_emojis else
                          "Do not use emojis.")
            praise_rule = ("Do not praise yourself, and do not mention success "
                           "rates, percentages, durations, or phrases like "
                           "'record time'. Sound like a helpful colleague, not a "
                           "status report."
                           if self.follow_up_avoid_self_praise else "")
            follow_up_prompt = f"""
            You are SIL-Robo, a helpful robot assistant. You just attempted a task
            for the user. Write a short, natural reply.

            What the user asked for: "{original_command}"
            Outcome: {outcome_rule}
            Candidate next steps you could offer:
            {suggestions_formatted}

            Write at most {self.follow_up_max_sentences} sentences in a {self.follow_up_tone},
            conversational tone. Then naturally offer {self.follow_up_num_suggestions} concrete,
            relevant next action(s) the user might want, phrased as an offer.
            {praise_rule}
            {emoji_rule}
            """
            final_response = self.llm_interface.call_llm(
                "You offer natural, relevant next-step suggestions after finishing a task.",
                follow_up_prompt
            ).strip()
            rospy.loginfo(f"[SIL] Generated follow-up:\n{final_response}")
            return final_response
        except Exception as e:
            rospy.logerr(f"Error generating follow-up: {e}")
            return ""
    
    def _execute_intent(self, intent: Dict, context: Dict) -> Dict:
        action_type = intent.get('action_type') or intent.get('action')
        if action_type == 'CONVERSATION':
            response = intent.get('response', '')
            if response:
                self.response_publisher.publish(String(data=response))
            return {'success': True, 'response': response}
        elif action_type == 'UNKNOWN':
            msg = "I couldn't understand that command. Could you please rephrase?"
            self.response_publisher.publish(String(data=msg))
            return {'success': False, 'response': msg}
        elif action_type == 'GO_TO_COORDINATES':
            coords = intent.get('coordinates')
            if coords is None and any(k in intent for k in ('x', 'y', 'z')):
                coords = {k: intent[k] for k in ('x', 'y', 'z') if k in intent}
            if coords is not None:
                rospy.loginfo(f"[SIL] Executing GO_TO_COORDINATES with {coords}")
                action = {'action': 'GO_TO_COORDINATES', 'coordinates': coords}
                try:
                    ok = bool(self.action_executor.execute_action(action))
                    return {'success': ok,
                            'response': "Navigating to coordinates" if ok
                            else "I couldn't navigate to those coordinates."}
                except Exception as e:
                    return {'success': False, 'error': str(e)}
            else:
                return {'success': False, 'response': 'No coordinates provided'}
        else:
            action_to_execute = intent
            try:
                ok = bool(self.action_executor.execute_action(action_to_execute))
                return {'success': ok, 'response': f"Executed {action_type}"}
            except Exception as e:
                return {'success': False, 'error': str(e)}
    
    def _get_conversation_context(self) -> str:
        """Get recent conversation history for context."""
        if not self.persistent_context['conversation_history']:
            return ""
        recent = self.persistent_context['conversation_history'][-self.history_context_turns:]
        context = "Recent conversation:\n"
        for entry in recent:
            context += f"{entry['type']}: {entry['message'][:100]}...\n"
        return context
    
    def _store_interaction(self, human_input, intent, result, confidence, agent_response=''):
        try:
            interaction_data = {
                'human_input': human_input,
                'agent_response': agent_response, 
                'parsed_intent': intent,
                'execution_result': result,
                'confidence': confidence,
                'timestamp': datetime.now()
            }
            self.memory_module.store_interaction(interaction_data)
        except Exception as e:
            rospy.logerr(f"[SIL] Failed to store interaction: {e}")
    
    def _extract_coordinates(self, text: str) -> Optional[Dict]:
        coords = {}
        patterns = [
            r'x\s*[:=]\s*([-\d.]+)', 
            r'x\s*([-\d.]+)',          
            r'\(\s*([-\d.]+)\s*,',    
        ]
        for axis in ['x', 'y', 'z']:
            match = re.search(rf'{axis}\s*[:=]\s*([-\d.]+)', text, re.IGNORECASE)
            if match:
                coords[axis] = float(match.group(1))
        paren_match = re.search(r'\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)', text)
        if paren_match and not coords:
            coords['x'] = float(paren_match.group(1))
            coords['y'] = float(paren_match.group(2))
            coords['z'] = float(paren_match.group(3))
        if coords:
            coords.setdefault('x', 0.0)
            coords.setdefault('y', 0.0)
            coords.setdefault('z', 0.0)
            rospy.loginfo(f"[SIL] Extracted coordinates: {coords}")
            return coords
        return None
    
    def _extract_number(self, text: str, default: float) -> float:
        match = re.search(r'(\d+(?:\.\d+)?)', text)
        return float(match.group(1)) if match else default
    
    def _get_robot_state(self) -> Dict:
        state = {'x': 0, 'y': 0, 'z': 0, 'yaw': 0, 'direction': 'north'}
        if hasattr(self.action_executor, 'current_position'):
            pos = self.action_executor.current_position
            if pos:
                state['x'] = round(pos.x, 2)
                state['y'] = round(pos.y, 2)
                state['z'] = round(pos.z, 2)
        if hasattr(self.action_executor, 'current_orientation'):
            q = self.action_executor.current_orientation
            if q:
                import tf, math
                euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
                state['yaw'] = round(math.degrees(euler[2]), 1)
                state['direction'] = self.action_executor.get_cardinal_direction(state['yaw'])
        return state

# ============= HELPER CLASSES =============
class UncertaintyEstimator:
    def __init__(self, llm_interface, config: SILConfig = None):
        self.llm_interface = llm_interface
        self.config = config or SILConfig.load()
        uc = self.config.uncertainty
        self.confidence_keywords = {
            'high': uc.high_confidence_keywords,
            'medium': uc.medium_confidence_keywords,
            'low': uc.low_confidence_keywords,
        }
        self.high_w = uc.high_confidence_weight
        self.medium_w = uc.medium_confidence_weight
        self.low_w = uc.low_confidence_weight
        self.baseline = uc.confidence_baseline
        self.clarity_baseline = float(rospy.get_param("sil/uncertainty/clarity_baseline", 0.8))
        self.ambiguous_penalty = float(rospy.get_param("sil/uncertainty/ambiguous_penalty", 0.15))
        self.clear_commands = rospy.get_param("sil/uncertainty/clear_commands", {})
        self.known_locations = rospy.get_param("sil/uncertainty/known_locations", [])
        self.ambiguous_words = rospy.get_param(
            "sil/uncertainty/ambiguous_words",
            ['it', 'that', 'there', 'thing', 'something', 'somewhere', 'stuff'])
    
    def estimate_input_clarity(self, human_input: str) -> float:
        text_lower = human_input.lower().strip()
        clarity = self.clarity_baseline
        for cmd, score in self.clear_commands.items():
            if cmd in text_lower:
                return float(score)
        if any(phrase in text_lower for phrase in ['go to', 'navigate to', 'move to', 'head to']):
            has_known_location = any(loc in text_lower for loc in self.known_locations)
            return 0.9 if has_known_location else 0.75
        words = text_lower.split()
        ambiguous_count = sum(1 for word in self.ambiguous_words if word in words)
        clarity -= ambiguous_count * self.ambiguous_penalty
        has_numbers = bool(re.search(r'\d+', text_lower))
        has_units = any(unit in text_lower for unit in ['meter', 'degree', 'second', 'm', 'deg', 'sec'])
        if has_numbers:
            clarity += 0.1
        if has_units:
            clarity += 0.1
        word_count = len(words)
        if word_count == 1:
            if text_lower in ['stop', 'wait', 'help', 'forward', 'backward', 'left', 'right']:
                clarity = max(clarity, 0.85)
            else:
                clarity *= 0.8
        elif word_count > 50:
            clarity *= 0.85  
        return max(0.0, min(1.0, clarity))
    
    def estimate_response_confidence(self, agent_response: str, context: Dict = None) -> float:
        text_lower = agent_response.lower()
        confidence_score = self.baseline
        for level, keywords in self.confidence_keywords.items():
            count = sum(1 for keyword in keywords if keyword in text_lower)
            if level == 'high':
                confidence_score += count * self.high_w
            elif level == 'medium':
                confidence_score += count * self.medium_w
            elif level == 'low':
                confidence_score -= count * self.low_w
        if 'action' in text_lower and any(word in text_lower for word in ['meter', 'degree', 'coordinate']):
            confidence_score += 0.1
        if context:
            if context.get('similar_past_success', False):
                confidence_score += 0.15
            if context.get('novel_situation', False):
                confidence_score -= 0.1
        return max(0.0, min(1.0, confidence_score))

class ProactiveSuggestionGenerator:
    def __init__(self, memory_module, llm_interface):
        self.memory_module = memory_module
        self.llm_interface = llm_interface
        
    def generate_suggestions(self, parsed_intent: Dict, context: Dict) -> List[str]:
        suggestions_prompt = f"""
        A mobile robot just finished a task. Suggest relevant next actions it could do.

        The user's request was: "{context.get('user_goal', 'unknown')}"
        The action just performed: "{context.get('completed_task', 'unknown')}"
        Current robot state (pose/orientation): {context.get('robot_state', {})}

        The robot can: navigate to known locations, go to coordinates, move/turn/rotate,
        describe its surroundings, take and send a photo, report its position, patrol
        between locations, and return to the starting point.

        Propose 1-2 specific, useful next actions that follow naturally from what was
        just done (e.g. after arriving somewhere: "describe what you see there" or
        "take a photo"). Make them concrete and tied to the robot's capabilities.
        Do not suggest generic maintenance like checking battery or status.

        Return only the suggestions, one short phrase per line, no numbering.
        """
        suggestions_text = self.llm_interface.call_llm(
            "You suggest concrete, relevant next actions a mobile robot can perform.",
            suggestions_prompt
        )
        return [line.strip("-• ").strip()
                for line in suggestions_text.split('\n') if line.strip()]
    
    def _navigation_suggestions(self, intent: Dict, context: Dict) -> List[str]:
        suggestions = []
        destination = intent.get('destination_name', '')
        if destination and not intent.get('speed'):
            suggestions.append(f"Would you like me to navigate to {destination} at a specific speed?")
        if context.get('obstacles_detected'):
            suggestions.append("I notice potential obstacles. Should I plan an alternative route?")
        return suggestions
    
    def _movement_suggestions(self, intent: Dict, context: Dict) -> List[str]:
        suggestions = []
        distance = intent.get('distance', 0)
        if distance > 3:
            suggestions.append("That's quite a distance. Would you like me to provide updates during movement?")
        if not intent.get('speed'):
            suggestions.append("Would you prefer a specific movement speed?")
        return suggestions
    
    def _rotation_suggestions(self, intent: Dict, context: Dict) -> List[str]:
        suggestions = []
        angle = intent.get('angle', 0)
        if angle > 180:
            suggestions.append("That's a large rotation. Should I break it into smaller turns for precision?")
        return suggestions
    
    def _safety_suggestions(self, intent: Dict, context: Dict) -> List[str]:
        suggestions = []
        if intent.get('action_type') == 'FORWARD' and intent.get('distance', 0) > 5:
            suggestions.append("For safety, should I check for obstacles before moving that distance?")
        if context.get('low_battery'):
            suggestions.append("Battery is low. Should I complete this task before returning to charge?")
        return suggestions

class ClarificationGenerator:
    def __init__(self, llm_interface):
        self.llm_interface = llm_interface
        
    def generate_clarification(self, uncertainty_type: str, context: Dict) -> str:
        clarification_prompt = f"""
        Generate a natural clarification question for uncertainty: {uncertainty_type}
        Context: {context}
        
        Create a friendly question that acknowledges what you understood and asks specifically about the unclear part.
        """
        return self.llm_interface.call_llm(
            "You are asking for clarification naturally.", clarification_prompt
        ).strip()
    
    def identify_uncertainty_type(self, parsed_intent: Dict, clarity_score: float) -> Tuple[str, Dict]:
        for key, value in parsed_intent.items():
            if isinstance(value, str) and value.lower() in ['it', 'that', 'there', 'thing']:
                previous_interaction = self.memory.get_last_interaction()
                if previous_interaction and 'LLM Response' in previous_interaction:
                    if 'would you like me to perform any actions related to that task again' in previous_interaction['LLM Response'].lower():
                        return 'clarification_needed_with_context', {'reference': value, 'last_task': 'the previous task'}
                return 'ambiguous_reference', {'reference': value}
        if clarity_score < 0.7:
            if not parsed_intent.get('action_type'):
                return 'unclear_goal', {'goal': 'help you with your task'} 
            action_type = parsed_intent.get('action_type', '')
            missing_params = []  
            if action_type in ['FORWARD', 'BACKWARD'] and 'distance' not in parsed_intent:
                missing_params.append('distance')
            if action_type in ['TURN_LEFT', 'TURN_RIGHT'] and 'angle' not in parsed_intent:
                missing_params.append('angle')
            if action_type == 'NAVIGATE_TO_DESTINATION' and not parsed_intent.get('destination_name'):
                missing_params.append('destination')
            if missing_params:
                return 'missing_parameter', {
                    'action': action_type.lower().replace('_', ' '),
                    'parameter': ', '.join(missing_params)
                }
        return 'unclear_goal', {'goal': 'complete your request'}
