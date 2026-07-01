#!/usr/bin/env python

import rospy
import numpy as np
import json
import pickle
from pickle import UnpicklingError
import os
from datetime import datetime, timedelta
from collections import defaultdict, deque, Counter
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity
from sentence_transformers import SentenceTransformer
import threading
from typing import Dict, List, Any, Optional, Tuple
from sil_ros.config import SILConfig

def convert_numpy_types(obj):
    if isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, np.bool_):
        return bool(obj)
    elif isinstance(obj, dict):
        return {key: convert_numpy_types(value) for key, value in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy_types(item) for item in obj]
    elif isinstance(obj, tuple):
        return tuple(convert_numpy_types(item) for item in obj)
    elif hasattr(obj, 'isoformat'):
        return obj.isoformat()
    return obj

# ============= INTERACTION EPISODE =============
class InteractionEpisode:
    def __init__(self, interaction_id: str, human_input: str, agent_response: str, 
                 context: Dict = None, feedback: Dict = None, timestamp: datetime = None):
        self.interaction_id = interaction_id
        self.human_input = human_input
        self.agent_response = agent_response
        self.context = context or {}
        self.feedback = feedback or {}
        self.timestamp = timestamp or datetime.now()
        self.belief_states = {}  
        self.latent_representation = None
        self.belief_alignment = 0.0
        self.success_score = None
        self.clarification_needed = False
        self.human_satisfaction = None
        self.adaptation_triggers = []
        self.mutual_influence_score = 0.0
        self.convergence_delta = 0.0

# ============= SEMANTIC PATTERN =============
class SemanticPattern:
    def __init__(self, pattern_id: str, episodes: List[InteractionEpisode]):
        self.pattern_id = pattern_id
        self.creation_time = datetime.now()
        self.contributing_episodes = [ep.interaction_id for ep in episodes]
        self.pattern_embedding = self._compute_pattern_embedding(episodes)
        self.success_rate = np.mean([ep.success_score for ep in episodes if ep.success_score])
        self.average_alignment = np.mean([ep.belief_alignment for ep in episodes])
        self.trigger_conditions = self._extract_triggers(episodes)
        self.optimal_parameters = self._extract_optimal_parameters(episodes)
        self.usage_count = 0
        self.success_when_applied = []
    
    def _compute_pattern_embedding(self, episodes: List[InteractionEpisode]) -> np.ndarray:
        embeddings = []
        for ep in episodes:
            if ep.latent_representation is not None:
                embeddings.append(ep.latent_representation)
        if embeddings:
            return np.mean(embeddings, axis=0)
        return np.zeros(SILConfig.load().encoder.latent_dim)
    
    def _extract_triggers(self, episodes: List[InteractionEpisode]) -> Dict:
        triggers = defaultdict(list)
        for ep in episodes:
            if ep.context:
                for key, value in ep.context.items():
                    triggers[key].append(value)
        common_triggers = {}
        for key, values in triggers.items():
            if len(values) > len(episodes) * 0.7:
                counter = Counter(values)
                common_triggers[key] = counter.most_common(1)[0][0]
        return common_triggers
    
    def _extract_optimal_parameters(self, episodes: List[InteractionEpisode]) -> Dict:
        successful = [ep for ep in episodes if ep.success_score and ep.success_score > 0.7]
        if not successful:
            return {}
        parameters = defaultdict(list)
        for ep in successful:
            if 'execution_result' in ep.context:
                result = ep.context['execution_result']
                if isinstance(result, dict) and 'parameters' in result:
                    for param, value in result['parameters'].items():
                        if isinstance(value, (int, float)):
                            parameters[param].append(value)
        optimal = {}
        for param, values in parameters.items():
            optimal[param] = {
                'mean': np.mean(values),
                'std': np.std(values),
                'recommended': np.median(values)
            }
        return optimal

# ============= HUMAN MODEL =============
class HumanModel:
    def __init__(self, config: SILConfig = None):
        self.config = config or SILConfig.load()
        mem = self.config.memory
        self.preferences = {}
        self.communication_style = {}
        self.task_patterns = defaultdict(list)
        self.error_patterns = defaultdict(list)
        self.learning_rate = mem.human_learning_rate
        self.confidence_threshold = mem.human_confidence_threshold
        self._formality_indicators = mem.formality_indicators
        self._specificity_indicators = mem.specificity_indicators
        self.belief_evolution = []  
        self.adaptation_responsiveness = mem.default_adaptation_responsiveness
        self.clarification_patterns = {}
    
    def update_preferences(self, interaction_data: Dict):
        if 'preference_signals' in interaction_data:
            for pref_type, value in interaction_data['preference_signals'].items():
                if pref_type not in self.preferences:
                    self.preferences[pref_type] = value
                else:
                    self.preferences[pref_type] = (
                        (1 - self.learning_rate) * self.preferences[pref_type] + 
                        self.learning_rate * value
                    )
    
    def update_communication_style(self, interaction: InteractionEpisode):
        input_length = len(interaction.human_input.split())
        self._update_style_metric('verbosity', input_length)
        formal_indicators = self._formality_indicators
        formality_score = sum(1 for indicator in formal_indicators 
                             if indicator in interaction.human_input.lower()) / max(1, len(formal_indicators))
        self._update_style_metric('formality', formality_score)
        specific_indicators = self._specificity_indicators
        specificity_score = sum(1 for indicator in specific_indicators 
                               if indicator in interaction.human_input.lower())
        self._update_style_metric('specificity', specificity_score)
    
    def _update_style_metric(self, metric: str, value: float):
        if metric not in self.communication_style:
            self.communication_style[metric] = value
        else:
            self.communication_style[metric] = (
                (1 - self.learning_rate) * self.communication_style[metric] + 
                self.learning_rate * value
            )
    
    def predict_clarification_need(self, input_text: str) -> float:
        word_count = len(input_text.split())
        avg_verbosity = self.communication_style.get('verbosity', 10)
        if word_count < avg_verbosity * 0.5:
            return 0.8
        elif word_count > avg_verbosity * 2:
            return 0.3
        else:
            return 0.4
    
    def update_belief_evolution(self, belief_state):
        if belief_state:
            self.belief_evolution.append({
                'timestamp': datetime.now(),
                'confidence': belief_state.confidence if hasattr(belief_state, 'confidence') else 0.5,
                'embedding_snapshot': belief_state.task_embedding.copy() if hasattr(belief_state, 'task_embedding') else None
            })

# ============= AGENT BELIEF STATE =============
class AgentBeliefState:
    def __init__(self):
        self.current_task_belief = {}
        self.uncertainty_estimates = {}
        self.confidence_scores = {}
        self.action_history = deque(maxlen=100)
        self.human_model_estimate = {} 
        self.alignment_history = []
    
    def update_task_belief(self, task_representation: Dict, confidence: float):
        self.current_task_belief = task_representation
        self.confidence_scores['task_understanding'] = confidence
    
    def update_uncertainty(self, aspect: str, uncertainty: float):
        self.uncertainty_estimates[aspect] = uncertainty
    
    def get_overall_confidence(self) -> float:
        if not self.confidence_scores:
            return 0.5
        return np.mean(list(self.confidence_scores.values()))
    
    def update_human_model(self, observation: Dict):
        self.human_model_estimate.update(observation)
        if 'alignment_score' in observation:
            self.alignment_history.append({
                'timestamp': datetime.now(),
                'score': observation['alignment_score']
            })

# ============= MEMORY MODULE =============
class EpisodicSemanticMemory:
    def __init__(self, memory_dir: str = None,
                 max_episodic_memory: int = None,
                 semantic_update_threshold: float = None,
                 config: SILConfig = None):
        self.config = config or SILConfig.load()
        mem = self.config.memory
        self.memory_dir = memory_dir if memory_dir is not None else \
            rospy.get_param("sil/memory_directory", "models/memory")
        if not os.path.isabs(self.memory_dir):
            import rospkg
            self.memory_dir = os.path.join(
                rospkg.RosPack().get_path('sil_ros'), self.memory_dir)
        self.max_episodic_memory = max_episodic_memory if max_episodic_memory is not None \
            else mem.max_episodic_memory                     
        self.semantic_update_threshold = semantic_update_threshold if semantic_update_threshold is not None \
            else mem.semantic_update_threshold
        os.makedirs(self.memory_dir, exist_ok=True)
        self.episodic_memory = deque(maxlen=self.max_episodic_memory)
        self.semantic_memory = {
            'task_patterns': defaultdict(list),
            'success_patterns': defaultdict(list),
            'failure_patterns': defaultdict(list),
            'clarification_patterns': defaultdict(list),
            'adaptation_rules': {},
            'co_adaptation_patterns': [] 
        }

        self.human_model = HumanModel(config=self.config)
        self.agent_belief_state = AgentBeliefState()
        self.embedding_model = SentenceTransformer(mem.embedding_model)
        self.tfidf_vectorizer = TfidfVectorizer(max_features=1000, stop_words='english')
        self.tfidf_fitted = False
        self.memory_lock = threading.Lock()
        self.retrieval_w_s = mem.retrieval.semantic_weight  
        self.retrieval_w_b = mem.retrieval.belief_weight    
        self.retrieval_temperature = mem.retrieval.softmax_temperature
        self.retrieval_max_episodes = mem.retrieval.max_episodes
        self.max_retrieval_candidates = mem.max_retrieval_candidates
        self.max_semantic_patterns_per_type = mem.max_semantic_patterns_per_type
        self.retention_success_threshold = mem.retention_success_threshold
        self.retention_alignment_threshold = mem.retention_alignment_threshold
        self.co_adaptation_min_episodes = mem.co_adaptation_min_episodes
        self.co_adaptation_improvement = mem.co_adaptation_improvement
        self.latent_dim = self.config.encoder.latent_dim
        self.consolidation_threshold = mem.human_confidence_threshold
        self.pattern_similarity_threshold = rospy.get_param(
            "memory/pattern_similarity_threshold", 0.75)
        self.shared_task_space = None
        self.load_memory()
        rospy.loginfo(
            f"[Memory] EpisodicSemanticMemory initialized "
            f"(capacity={self.max_episodic_memory}, embed={mem.embedding_model})"
        )
    
    def store_interaction(self, interaction_data: Dict) -> str:
        with self.memory_lock:
            interaction_id = f"ep_{len(self.episodic_memory)}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            episode = InteractionEpisode(
                interaction_id=interaction_id,
                human_input=interaction_data.get('human_input', ''),
                agent_response=interaction_data.get('agent_response', ''),
                context=interaction_data.get('context', {}),
                feedback=interaction_data.get('feedback', {}),
                timestamp=datetime.now()
            )
            if self.shared_task_space and hasattr(self.shared_task_space, 'project_to_latent'):
                latent = self.shared_task_space.project_to_latent(episode.human_input)
                episode.latent_representation = latent.copy() if hasattr(latent, 'copy') else latent
            if (self.shared_task_space and
                    hasattr(self.shared_task_space, 'human_belief_state') and
                    self.shared_task_space.human_belief_state is not None):
                hb = self.shared_task_space.human_belief_state
                episode.belief_states['human'] = {
                    'confidence': float(hb.confidence),
                    'embedding': hb.task_embedding.copy() if hasattr(hb.task_embedding, 'copy') else hb.task_embedding
                }
                episode.belief_alignment = self.shared_task_space.compute_belief_alignment()
            if (self.shared_task_space and
                    hasattr(self.shared_task_space, 'agent_belief_state') and
                    self.shared_task_space.agent_belief_state is not None):
                ab = self.shared_task_space.agent_belief_state
                episode.belief_states['agent'] = {
                    'confidence': float(ab.confidence),
                    'embedding': ab.task_embedding.copy() if hasattr(ab.task_embedding, 'copy') else ab.task_embedding
                }
                if episode.belief_alignment == 0.0:
                    episode.belief_alignment = self.shared_task_space.compute_belief_alignment() \
                                             if hasattr(self.shared_task_space, 'compute_belief_alignment') else 0.0
            if 'execution_result' in interaction_data:
                episode.success_score = 1.0 if interaction_data['execution_result'].get('success') else 0.0
            self.episodic_memory.append(episode)
            self.human_model.update_communication_style(episode)
            if 'feedback' in interaction_data:
                self.human_model.update_preferences(interaction_data['feedback'])
            self._update_semantic_memory(episode)
            self._check_co_adaptation_patterns()
            rospy.loginfo(f"[Memory] Stored interaction {interaction_id}")
            return interaction_id
    
    def _check_co_adaptation_patterns(self):
        if len(self.episodic_memory) < self.co_adaptation_min_episodes:
            return
        recent = list(self.episodic_memory)[-self.co_adaptation_min_episodes:]
        alignments = [ep.belief_alignment for ep in recent if ep.belief_alignment > 0]
        if len(alignments) > self.co_adaptation_min_episodes // 2:
            if alignments[-1] > alignments[0] + self.co_adaptation_improvement:  
                pattern = {
                    'type': 'successful_convergence',
                    'improvement': alignments[-1] - alignments[0],
                    'episodes': [ep.interaction_id for ep in recent],
                    'timestamp': datetime.now()
                }
                self.semantic_memory['co_adaptation_patterns'].append(pattern)
                rospy.loginfo("[Memory] Identified successful co-adaptation pattern")
    
    def retrieve_relevant_context(self, current_input: str, max_episodes: int = None,
                                    temperature: float = None) -> List[InteractionEpisode]:
        with self.memory_lock:
            if not self.episodic_memory:
                return []
            if max_episodes is None:
                max_episodes = self.retrieval_max_episodes
            if temperature is None:
                temperature = self.retrieval_temperature
            MAX_CANDIDATES = self.max_retrieval_candidates
            all_episodes = list(self.episodic_memory)
            episodes = all_episodes[-MAX_CANDIDATES:] if len(all_episodes) > MAX_CANDIDATES else all_episodes
            episode_texts = [ep.human_input for ep in episodes]
            current_embedding = self.embedding_model.encode([current_input])    
            episode_embeddings = self.embedding_model.encode(episode_texts)    
            text_sims = cosine_similarity(current_embedding, episode_embeddings)[0] 
            current_agent_belief = None
            current_agent_confidence = 0.5
            if (self.shared_task_space and 
                    hasattr(self.shared_task_space, 'agent_belief_state') and
                    self.shared_task_space.agent_belief_state is not None):
                if hasattr(self.shared_task_space.agent_belief_state, 'task_embedding'):
                    current_agent_belief = self.shared_task_space.agent_belief_state.task_embedding
                    current_agent_confidence = getattr(
                        self.shared_task_space.agent_belief_state, 'confidence', 0.5
                    )
            belief_sims = np.zeros(len(episodes))
            for i, ep in enumerate(episodes):
                if current_agent_belief is not None and ep.latent_representation is not None:
                    latent_sim = cosine_similarity(
                        current_agent_belief.reshape(1, -1),
                        ep.latent_representation.reshape(1, -1)
                    )[0, 0]
                    stored_confidence = ep.belief_alignment if ep.belief_alignment > 0 else 0.5
                    belief_sims[i] = latent_sim * current_agent_confidence * stored_confidence
            w_s, w_b = self.retrieval_w_s, self.retrieval_w_b
            if current_agent_belief is not None:
                scores = w_s * text_sims + w_b * belief_sims
            else:
                scores = text_sims
            logits = scores / max(temperature, 1e-8)
            logits -= np.max(logits)                       
            exp_logits = np.exp(logits)
            probs = exp_logits / exp_logits.sum()
            n_retrieve = min(max_episodes, len(episodes))
            try:
                chosen_indices = np.random.choice(
                    len(episodes), size=n_retrieve, replace=False, p=probs
                )
            except ValueError:
                chosen_indices = np.argsort(scores)[::-1][:n_retrieve]
            chosen_indices = sorted(chosen_indices, key=lambda i: scores[i], reverse=True)
            relevant_episodes = [episodes[i] for i in chosen_indices]
            rospy.loginfo(
                f"[Memory] Retrieved {len(relevant_episodes)} episodes "
                f"(softmax τ={temperature:.2f}, top score={scores[chosen_indices[0]]:.3f})"
            )
            return relevant_episodes
    
    def get_adaptation_suggestions(self, current_context: Dict) -> List[str]:
        suggestions = []
        task_type = current_context.get('task_type', 'general')
        if self.semantic_memory['co_adaptation_patterns']:
            recent_pattern = self.semantic_memory['co_adaptation_patterns'][-1]
            if recent_pattern['type'] == 'successful_convergence':
                suggestions.append("Previous interactions show improving alignment. Continue current approach.")
        success_patterns = self.get_success_patterns(task_type)
        if success_patterns:
            suggestions.append("Based on past successes, consider being more specific about parameters.")
        if self.human_model.preferences:
            if self.human_model.preferences.get('prefers_confirmation', 0) > 0.7:
                suggestions.append("This user appreciates confirmation before execution.")
            if self.human_model.preferences.get('prefers_detailed_feedback', 0) > 0.7:
                suggestions.append("Provide detailed feedback during execution.")
        return suggestions
    
    def get_success_patterns(self, task_type: str) -> List[Dict]:
        return self.semantic_memory['success_patterns'].get(task_type, [])
    
    def get_failure_patterns(self, task_type: str) -> List[Dict]:
        return self.semantic_memory['failure_patterns'].get(task_type, [])
    
    def _update_semantic_memory(self, episode: InteractionEpisode):
        if episode.success_score is None:
            return
        task_type = episode.context.get('task_type', 'general')
        pattern = {
            'input_pattern': episode.human_input,
            'response_pattern': episode.agent_response,
            'context': episode.context,
            'success_score': episode.success_score,
            'belief_alignment': episode.belief_alignment,
            'timestamp': episode.timestamp
        }
        if episode.success_score >= self.semantic_update_threshold:
            self.semantic_memory['success_patterns'][task_type].append(pattern)
        else:
            self.semantic_memory['failure_patterns'][task_type].append(pattern)
        max_patterns = self.max_semantic_patterns_per_type
        for pattern_type in ['success_patterns', 'failure_patterns']:
            if len(self.semantic_memory[pattern_type][task_type]) > max_patterns:
                self.semantic_memory[pattern_type][task_type] = \
                    self.semantic_memory[pattern_type][task_type][-max_patterns:]
    
    def generate_learning_reflection(self, interaction_data: Dict) -> str:
        if not hasattr(self, 'shared_task_space') or not self.shared_task_space:
            return ""
        reflection_prompt = f"""
        Based on this interaction: {interaction_data}
        Generate a brief learning reflection (1 sentence) showing what was learned.
        Examples: "I noticed navigation works better with specific coordinates."
        """
        try:
            llm_interface = getattr(self, 'llm_interface', None)
            if llm_interface:
                reflection = llm_interface.call_llm(
                    "You are reflecting on learning.", reflection_prompt
                ).strip()
                
                if len(reflection) > 10:
                    return reflection
        except Exception as e:
            rospy.logerr(f"Error generating reflection: {e}")
        return ""
    
    def save_memory(self):
        try:
            episodic_file = os.path.join(self.memory_dir, 'episodic_memory.pkl')
            with open(episodic_file, 'wb') as f:
                pickle.dump(list(self.episodic_memory), f)
            semantic_file = os.path.join(self.memory_dir, 'semantic_memory.json')
            semantic_data = {}
            for key, value in self.semantic_memory.items():
                if isinstance(value, defaultdict):
                    semantic_data[key] = dict(value)
                else:
                    semantic_data[key] = value
            semantic_data = convert_numpy_types(semantic_data)
            with open(semantic_file, 'w') as f:
                json.dump(semantic_data, f, indent=2)
            human_model_file = os.path.join(self.memory_dir, 'human_model.json')
            human_data = {
                'preferences': self.human_model.preferences,
                'communication_style': self.human_model.communication_style,
                'task_patterns': dict(self.human_model.task_patterns),
                'adaptation_responsiveness': self.human_model.adaptation_responsiveness
            }
            human_data = convert_numpy_types(human_data)
            with open(human_model_file, 'w') as f:
                json.dump(human_data, f, indent=2)
            rospy.loginfo("[Memory] Memory saved to disk")    
        except Exception as e:
            rospy.logerr(f"[Memory] Failed to save memory: {e}")
    
    def load_memory(self):
        try:
            episodic_file = os.path.join(self.memory_dir, 'episodic_memory.pkl')
            if os.path.exists(episodic_file) and os.path.getsize(episodic_file) > 0:
                try:
                    with open(episodic_file, 'rb') as f:
                        episodes = pickle.load(f)
                        if isinstance(episodes, list):
                            self.episodic_memory.extend(episodes)
                except (EOFError, UnpicklingError) as e:
                    rospy.logwarn(f"[Memory] Skipping corrupted episodic file: {e}")  
            semantic_file = os.path.join(self.memory_dir, 'semantic_memory.json')
            if os.path.exists(semantic_file):
                with open(semantic_file, 'r') as f:
                    semantic_data = json.load(f)
                    for key, value in semantic_data.items():
                        if key in ['success_patterns', 'failure_patterns', 'clarification_patterns']:
                            self.semantic_memory[key] = defaultdict(list, value)
                        else:
                            self.semantic_memory[key] = value
            human_model_file = os.path.join(self.memory_dir, 'human_model.json')
            if os.path.exists(human_model_file):
                with open(human_model_file, 'r') as f:
                    human_data = json.load(f)
                    self.human_model.preferences = human_data.get('preferences', {})
                    self.human_model.communication_style = human_data.get('communication_style', {})
                    self.human_model.task_patterns = defaultdict(list, human_data.get('task_patterns', {}))
                    self.human_model.adaptation_responsiveness = human_data.get('adaptation_responsiveness', 0.5)
            rospy.loginfo(f"[Memory] Loaded {len(self.episodic_memory)} episodes from disk")
        except Exception as e:
            rospy.logwarn(f"[Memory] Could not load existing memory: {e}")
    
    def cleanup_old_memories(self, days_to_keep: int = None):
        if days_to_keep is None:
            days_to_keep = self.config.memory.episodic_retention_days
        cutoff_date = datetime.now() - timedelta(days=days_to_keep)
        with self.memory_lock:
            original_count = len(self.episodic_memory)
            preserved_episodes = deque(maxlen=self.max_episodic_memory)
            for ep in self.episodic_memory:
                should_keep = (
                    ep.timestamp > cutoff_date or
                    (ep.success_score and ep.success_score > self.retention_success_threshold) or
                    (ep.belief_alignment > self.retention_alignment_threshold) or
                    any(ep.interaction_id in pattern.get('episodes', []) 
                        for pattern in self.semantic_memory['co_adaptation_patterns'])
                ) 
                if should_keep:
                    preserved_episodes.append(ep)
            self.episodic_memory = preserved_episodes
            removed_count = original_count - len(self.episodic_memory)
            if removed_count > 0:
                rospy.loginfo(f"[Memory] Cleaned up {removed_count} old episodes")
    
    def get_memory_statistics(self) -> Dict:
        with self.memory_lock:
            stats = {
                'episodic_count': len(self.episodic_memory),
                'semantic_patterns': sum(len(patterns) for patterns in self.semantic_memory['success_patterns'].values()),
                'failure_patterns': sum(len(patterns) for patterns in self.semantic_memory['failure_patterns'].values()),
                'co_adaptation_patterns': len(self.semantic_memory['co_adaptation_patterns']),
                'human_preferences': len(self.human_model.preferences),
                'communication_style': self.human_model.communication_style,
                'agent_confidence': self.agent_belief_state.get_overall_confidence()
            } 
            if self.episodic_memory:
                recent = list(self.episodic_memory)[-20:]
                stats['recent_success_rate'] = np.mean([ep.success_score for ep in recent if ep.success_score is not None])
                stats['recent_alignment'] = np.mean([ep.belief_alignment for ep in recent if ep.belief_alignment > 0])
            return stats
        
    def find_positive_example(self, anchor_input: str, similarity_threshold: float = None) -> Optional[str]:
        if not self.episodic_memory:
            return None
        if similarity_threshold is None:
            similarity_threshold = self.config.training.positive_similarity_threshold
        candidates = [
            ep for ep in self.episodic_memory
            if ep.success_score is not None and ep.success_score > similarity_threshold
        ]
        if not candidates:
            return None
        anchor_embedding = self.embedding_model.encode([anchor_input])
        candidate_texts = [ep.human_input for ep in candidates]
        candidate_embeddings = self.embedding_model.encode(candidate_texts)
        sims = cosine_similarity(anchor_embedding, candidate_embeddings)[0]
        valid_mask = sims > similarity_threshold
        if not np.any(valid_mask):
            return None
        valid_indices = np.where(valid_mask)[0]
        best_idx = valid_indices[np.argmax(sims[valid_indices])]
        return candidates[best_idx].human_input

    def find_negative_example(self, anchor_input: str, similarity_threshold: float = None) -> Optional[str]:
        if not self.episodic_memory:
            return None
        if similarity_threshold is None:
            similarity_threshold = self.config.training.negative_similarity_threshold
        success_floor = self.config.training.negative_success_threshold
        anchor_embedding = self.embedding_model.encode([anchor_input])
        all_texts = [ep.human_input for ep in self.episodic_memory]
        all_embeddings = self.embedding_model.encode(all_texts)
        sims = cosine_similarity(anchor_embedding, all_embeddings)[0]
        episodes = list(self.episodic_memory)
        best_example = None
        lowest_sim = 1.0
        for i, ep in enumerate(episodes):
            if ep.success_score is not None and ep.success_score < success_floor:
                if sims[i] < similarity_threshold and sims[i] < lowest_sim:
                    lowest_sim = sims[i]
                    best_example = ep.human_input
        if best_example is not None:
            return best_example
        for i, ep in enumerate(episodes):
            if sims[i] < similarity_threshold and sims[i] < lowest_sim:
                lowest_sim = sims[i]
                best_example = ep.human_input
        return best_example
