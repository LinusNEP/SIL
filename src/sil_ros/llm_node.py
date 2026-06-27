#!/usr/bin/env python
import rospy
import os
import math
import numpy as np
import re
from typing import Dict, Any, Tuple, List
from sil_ros.llm_provider import LLMProviderFactory
from sentence_transformers import SentenceTransformer
from sklearn.metrics.pairwise import cosine_similarity as sklearn_cosine_similarity
from datetime import datetime
import json
from sil_ros.config import SILConfig

class SILLLMInterface:
    def __init__(self, api_key: str, destinations: Dict[str, Any], memory_module=None,
                 config: SILConfig = None):
        self.api_key = api_key 
        self.config = config or SILConfig.load()
        self.LLM_PROVIDER = rospy.get_param("models/llm_provider", "openai")
        self.MODEL_NAME = rospy.get_param("models/llm_name", "gpt-4")
        self.MAX_TOKENS = rospy.get_param("models/llm_max_tokens", 500)
        self.TEMPERATURE = rospy.get_param("models/llm_temperature", 0)
        self.llm_endpoint = rospy.get_param("models/llm_endpoint", "")
        self.llm_timeout = rospy.get_param("models/llm_timeout_seconds", 30)
        self.llm_max_retries = int(rospy.get_param("models/llm_max_retries", 2))
        self.llm_retry_base_delay = float(rospy.get_param("models/llm_retry_base_delay", 1.0))
        self.memory_module = memory_module
        self.interaction_history = []
        uc = self.config.uncertainty
        self.uncertainty_threshold = uc.uncertainty_threshold
        self.ensemble_size = uc.ensemble_size
        self.enable_uncertainty_estimation = uc.enable_estimation
        self.alpha_u = uc.alpha_u   
        self.beta_u = uc.beta_u      
        # U(x) = alpha_u * D + (1 - alpha_u) * (1 - C_ling) + beta_u * C_ctx
        self.temperature_variation = uc.temperature_variation
        self.destinations = destinations
        self.llm_client = LLMProviderFactory.create_provider(
            self.LLM_PROVIDER,
            api_key,
            self.MODEL_NAME,
            self.MAX_TOKENS,
            self.TEMPERATURE,
            endpoint=self.llm_endpoint or None,
            timeout=self.llm_timeout,
            max_retries=self.llm_max_retries,
            retry_base_delay=self.llm_retry_base_delay
        )
        self._embedding_model = None 
        self.confidence_indicators = {
            'high_confidence': uc.high_confidence_keywords,
            'medium_confidence': uc.medium_confidence_keywords,
            'low_confidence': uc.low_confidence_keywords,
        }
        self._conf_baseline = uc.confidence_baseline
        self._conf_high_w = uc.high_confidence_weight
        self._conf_medium_w = uc.medium_confidence_weight
        self._conf_low_w = uc.low_confidence_weight
        rospy.loginfo("[SIL-LLM] Enhanced LLM Interface initialized")
    
    @property
    def embedding_model(self) -> SentenceTransformer:
        """Lazy-load embedding model on first use to avoid blocking init."""
        if self._embedding_model is None:
            model_name = rospy.get_param(
                "models/embedding_model", "paraphrase-MiniLM-L6-v2"
            )
            self._embedding_model = SentenceTransformer(model_name)
            rospy.loginfo(f"[SIL-LLM] Loaded embedding model: {model_name}")
        return self._embedding_model

    def _compute_ensemble_dispersion(self, responses: List[str]) -> float:
        if len(responses) < 2:
            return 0.0
        try:
            embeddings = self.embedding_model.encode(responses)         
            sim_matrix = sklearn_cosine_similarity(embeddings)          
            K = len(responses)
            total = 0.0
            count = 0
            for i in range(K):
                for j in range(i + 1, K):
                    total += 1.0 - sim_matrix[i, j]                   
                    count += 1
            dispersion = total / count if count > 0 else 0.0
            return float(np.clip(dispersion, 0.0, 1.0))
        except Exception as e:
            rospy.logwarn(f"[SIL-LLM] Dispersion computation failed: {e}")
            return 0.0

    def _compute_contextual_novelty(self, input_text: str) -> float:
        if (self.memory_module is None or 
                not hasattr(self.memory_module, 'episodic_memory') or
                len(self.memory_module.episodic_memory) == 0):
            return 1.0                                                
        try:
            input_emb = self.embedding_model.encode([input_text])   
            past_texts = [
                ep.human_input for ep in self.memory_module.episodic_memory
                if hasattr(ep, 'human_input') and ep.human_input
            ]
            if not past_texts:
                return 1.0
            past_embs = self.embedding_model.encode(past_texts)        
            similarities = sklearn_cosine_similarity(input_emb, past_embs)[0] 
            max_sim = float(np.max(similarities))
            novelty = 1.0 - max_sim
            return float(np.clip(novelty, 0.0, 1.0))
        except Exception as e:
            rospy.logwarn(f"[SIL-LLM] Contextual novelty computation failed: {e}")
            return 0.5                                                 

    def _compute_overall_uncertainty(self, input_text: str,
                                     ensemble_dispersion: float,
                                     linguistic_confidence: float) -> float:
        contextual_novelty = self._compute_contextual_novelty(input_text)
        uncertainty = (
            self.alpha_u * ensemble_dispersion +
            (1.0 - self.alpha_u) * (1.0 - linguistic_confidence) +
            self.beta_u * contextual_novelty
        )
        uncertainty = float(np.clip(uncertainty, 0.0, 1.0))
        rospy.loginfo(
            f"[SIL-LLM] U(x) = {uncertainty:.3f}  "
            f"[D={ensemble_dispersion:.3f}, C_ling={linguistic_confidence:.3f}, "
            f"C_ctx={contextual_novelty:.3f}]"
        )
        return uncertainty
    
    def process_input_with_uncertainty(self, input_text: str, 
                                     system_message: str = None,
                                     return_confidence: bool = True) -> Tuple[Dict, float]:
        if not system_message:
            system_message = self.build_enhanced_system_message({})
        if self.enable_uncertainty_estimation:
            response, confidence = self._generate_response_with_ensemble(
                system_message, input_text
            )
        else:
            response_text = self.call_llm(system_message, input_text)
            response = {'type': 'ACTIONS' if 'Action' in response_text else 'RESPONSE',
                       'content': response_text}
            confidence = self._estimate_response_confidence(response_text)
        if self.memory_module:
            self.interaction_history.append({
                'input': input_text,
                'response': response,
                'confidence': confidence,
                'timestamp': datetime.now()
            })
        if return_confidence:
            return response, confidence
        return response
    
    def _generate_response_with_ensemble(self, system_message: str, input_text: str) -> Tuple[Dict, float]:
        responses = []
        confidences = []      
        temperatures = []
        for i in range(self.ensemble_size):
            temp_i = self.TEMPERATURE + (i * self.temperature_variation)
            temp_i = min(1.0, max(0.0, temp_i))
            temperatures.append(temp_i)
            try:
                temp_client = LLMProviderFactory.create_provider(
                    self.LLM_PROVIDER,
                    self.api_key,
                    self.MODEL_NAME,
                    self.MAX_TOKENS,
                    temp_i,
                    endpoint=self.llm_endpoint or None,
                    timeout=self.llm_timeout,
            max_retries=self.llm_max_retries,
            retry_base_delay=self.llm_retry_base_delay
                ) 
                response_text = temp_client.chat(system_message, input_text)
                responses.append(response_text)
                conf = self._estimate_response_confidence(response_text)
                confidences.append(conf)
            except Exception as e:
                rospy.logwarn(f"[SIL-LLM] Ensemble member {i} (T={temp_i:.2f}) failed: {e}")
                continue
        
        if not responses:
            response_text = self.call_llm(system_message, input_text)
            return {'type': 'ACTIONS' if 'Action' in response_text else 'RESPONSE',
                'content': response_text}, 0.5
        dispersion = self._compute_ensemble_dispersion(responses)
        avg_linguistic_confidence = float(np.mean(confidences))
        overall_uncertainty = self._compute_overall_uncertainty(
            input_text, dispersion, avg_linguistic_confidence
        )
        best_response = self._weighted_consensus_selection(
            responses, confidences, temperatures, overall_uncertainty
        )
        ensemble_confidence = 1.0 - overall_uncertainty
        response_type = 'ACTIONS' if 'Action' in best_response else 'RESPONSE'
        return {'type': response_type, 'content': best_response}, ensemble_confidence

    def _weighted_consensus_selection(self, responses: List[str],
                                      confidences: List[float],
                                      temperatures: List[float],
                                      overall_uncertainty: float) -> str:
        if len(responses) == 1:
            return responses[0]
        K = len(responses)
        max_temp = max(temperatures) if temperatures else 1.0
        raw_weights = [(max_temp - t + 0.1) for t in temperatures]
        w_sum = sum(raw_weights)
        temp_weights = [w / w_sum for w in raw_weights]               
        per_member_uncertainty = [1.0 - c for c in confidences]
        try:
            embeddings = self.embedding_model.encode(responses)          
            sim_matrix = sklearn_cosine_similarity(embeddings)           
        except Exception:
            return responses[int(np.argmax(confidences))]
        scores = np.zeros(K)
        for y_idx in range(K):
            for k in range(K):
                kernel_val = float(sim_matrix[k, y_idx])               
                scores[y_idx] += (
                    temp_weights[k] * kernel_val * (1.0 - per_member_uncertainty[k])
                )
        best_idx = int(np.argmax(scores))
        rospy.loginfo(
            f"[SIL-LLM] Weighted consensus: scores={np.round(scores, 3).tolist()}, "
            f"selected member {best_idx}"
        )
        return responses[best_idx]

    def _calculate_response_similarity(self, response1: str, response2: str) -> float:
        try:
            embs = self.embedding_model.encode([response1, response2])
            return float(sklearn_cosine_similarity(embs[:1], embs[1:])[0, 0])
        except Exception:
            return self._text_similarity_jaccard(response1, response2)
    
    def _extract_actions(self, response: str) -> List[str]:
        lines = response.split('\n')
        actions = []
        for line in lines:
            if re.match(r'Action\s*\d+:', line, re.IGNORECASE):
                actions.append(line.strip())
        return actions
    
    def _text_similarity_jaccard(self, text1: str, text2: str) -> float:
        words1 = set(text1.lower().split())
        words2 = set(text2.lower().split())
        if not words1 and not words2:
            return 1.0
        if not words1 or not words2:
            return 0.0
        intersection = len(words1.intersection(words2))
        union = len(words1.union(words2))
        return intersection / union if union > 0 else 0.0
    
    def _estimate_response_confidence(self, response_text: str) -> float:
        text_lower = response_text.lower()
        confidence_score = self._conf_baseline 
        for level, keywords in self.confidence_indicators.items():
            count = sum(1 for keyword in keywords if keyword in text_lower)
            if level == 'high_confidence':
                confidence_score += count * self._conf_high_w
            elif level == 'medium_confidence':
                confidence_score += count * self._conf_medium_w
            elif level == 'low_confidence':
                confidence_score -= count * self._conf_low_w
        if 'action' in text_lower:
            if re.search(r'\d+(?:\.\d+)?\s*(?:meter|degree|second)', text_lower):
                confidence_score += 0.1
            if any(action in text_lower for action in ['move', 'turn', 'navigate', 'rotate']):
                confidence_score += 0.1
            if any(obj in text_lower for obj in ['object', 'destination', 'coordinates']):
                confidence_score += 0.05
            if re.search(r'\b(move|navigate|go|turn)\s+(forward|backward|left|right|around)\b', text_lower):
                confidence_score += 0.1
            if re.search(r'\b\d+\s*(?:meters?|degrees?)\b', text_lower):
                confidence_score += 0.1
            if re.search(r'\b(move|navigate|go|turn)\s+\d+\s*(?:meters?|degrees?)\b', text_lower):
                confidence_score += 0.1
            if re.search(r'\b(move|navigate|go|turn)\s+(?:forward|backward|left|right|around)\s+\d+\s*(?:meters?|degrees?)\b', text_lower):
                confidence_score += 0.1                     
            if any(word in text_lower for word in ['coordinates', 'kitchen', 'living room']):
                confidence_score += 0.05
        hedging_words = ['might', 'could', 'perhaps', 'maybe', 'possibly']
        hedging_count = sum(1 for word in hedging_words if word in text_lower)
        confidence_score -= hedging_count * 0.1
        return max(0.0, min(1.0, confidence_score))
    
    def generate_clarification_prompt(self, uncertain_intent: Dict, 
                                    context: Dict, uncertainty_type: str = None) -> str:
        clarification_prompts = {
            'ambiguous_reference': """
            The user's request contains an ambiguous reference. Generate a specific clarification question 
            to resolve the ambiguity. Focus on the unclear element and ask for specific details.
            """,
            'missing_parameters': """
            The user's request is missing key parameters needed for execution. Generate a question
            that asks for the specific missing information in a helpful way.
            """,
            'conflicting_instructions': """
            The user's request contains conflicting or contradictory instructions. Generate a question
            that helps resolve the conflict by asking the user to prioritize or clarify.
            """,
            'unclear_goal': """
            The user's ultimate goal or intention is unclear. Generate a question that helps
            understand what they're trying to achieve.
            """
        }
        base_prompt = clarification_prompts.get(uncertainty_type, clarification_prompts['unclear_goal'])
        enhanced_prompt = f"""
        {base_prompt}
        Context:
        - User input: {uncertain_intent.get('original_input', 'Unknown')}
        - Parsed intent: {uncertain_intent}
        - Current situation: {context.get('current_situation', 'Normal operation')}
        
        Generate a clear, specific clarification question that will help resolve the uncertainty.
        Be conversational and helpful, not robotic.
        """
        try:
            clarification = self.call_llm(enhanced_prompt, "Generate clarification question:")
            return clarification.strip()
        except Exception as e:
            rospy.logerr(f"[SIL-LLM] Failed to generate clarification: {e}")
            return "Could you please provide more details about what you'd like me to do?"
    
    def call_llm(self, system_message: str, prompt: str) -> str:
        try:
            if self.LLM_PROVIDER == "llama.cpp":
                wrapped_prompt = f"[INST] <<SYS>>\n{system_message}\n<</SYS>>\nInput: {prompt}\nAnswer:[/INST]"
                response = self.llm_client.chat("", wrapped_prompt)
            else:
                response = self.llm_client.chat(system_message, prompt) 
            rospy.logdebug(f"[SIL-LLM] Generated response: {response[:100]}...")
            return response
        except Exception as e:
            rospy.logerr(f"[SIL-LLM] LLM call failed: {e}")
            return "I'm sorry, I encountered an error processing your request."
    
    def build_enhanced_system_message(self, input_params: Dict[str, str]) -> str:
        current_yaw = input_params.get("current_yaw", "unknown")
        cardinal_direction = input_params.get("cardinal_direction", "unknown")
        position_x = input_params.get("position_x", "unknown")
        position_y = input_params.get("position_y", "unknown")
        position_z = input_params.get("position_z", "unknown")
        base_parts = [
            self.get_system_message_status(current_yaw, cardinal_direction, 
                                         position_x, position_y, position_z),
            self.get_system_message_commands(),
            self.get_system_message_navigation(),
            self.get_example_messages()
        ]
        sil_enhancements = self._build_sil_enhancements(input_params)
        combined_message = "\n".join(base_parts + [sil_enhancements])
        return combined_message
    
    def _build_sil_enhancements(self, input_params: Dict) -> str:
        enhancements = ["\n--- SYMBIOTIC INTERACTIVE LEARNING ENHANCEMENTS ---"]
        if self.memory_module:
            human_prefs = self.memory_module.human_model.preferences
            if human_prefs:
                enhancements.append(f"LEARNED USER PREFERENCES: {human_prefs}")
            comm_style = self.memory_module.human_model.communication_style
            if comm_style:
                verbosity = comm_style.get('verbosity', 10)
                formality = comm_style.get('formality', 0.5)
                
                style_guidance = f"""
                COMMUNICATION ADAPTATION:
                - User typically uses {verbosity:.1f} words per request
                - Formality level: {formality:.1f} (0=casual, 1=formal)
                - Adapt your response style accordingly
                """
                enhancements.append(style_guidance)
            recent_successes = self._get_recent_success_patterns()
            if recent_successes:
                enhancements.append(f"RECENT SUCCESSFUL PATTERNS: {recent_successes}")
        uncertainty_guidance = """
        UNCERTAINTY HANDLING:
        - If you're uncertain about any aspect of the request, indicate this clearly
        - Use phrases like "I want to make sure I understand..." for clarification
        - Be specific about what information you need
        - Consider safety implications of uncertain actions
        """
        enhancements.append(uncertainty_guidance)
        proactive_guidance = """
        PROACTIVE ASSISTANCE:
        - Suggest helpful alternatives when appropriate
        - Warn about potential issues before they occur
        - Offer additional context that might be useful
        - Ask if the user wants updates during long operations
        """
        enhancements.append(proactive_guidance)
        return "\n".join(enhancements)
    
    def _get_recent_success_patterns(self) -> str:
        if not self.memory_module or not self.memory_module.episodic_memory:
            return ""
        recent_episodes = list(self.memory_module.episodic_memory)[-5:]
        successful_episodes = [ep for ep in recent_episodes 
                             if ep.success_score and ep.success_score > 0.7]
        
        if not successful_episodes:
            return ""
        patterns = []
        for ep in successful_episodes[-5:]:  
            pattern = f"'{ep.human_input[:50]}...' → '{ep.agent_response[:50]}...'"
            patterns.append(pattern)
        return "; ".join(patterns)
    
    def get_system_message_status(self, current_yaw: str, cardinal_direction: str, 
                                  position_x: str, position_y: str, position_z: str) -> str:
        max_speed = rospy.get_param("speeds/maximum_speed", 1.0)
        min_speed = rospy.get_param("speeds/minimum_speed", 0.2)
        max_ang = math.degrees(rospy.get_param("speeds/default_angular_speed", 0.5))
        base_message = (
            "You are SIL-Robo, a physical ROS based mobile robot equipped with advanced "
            "Symbiotic Interactive Learning (SIL) capabilities. You can understand natural language, "
            "learn from interactions, and proactively assist users. "
            f"Your maximum and minimum movement speed are {max_speed} m/s and {min_speed} m/s respectively, "
            f"and your angular speed is approximately {max_ang:.0f} deg/s. "
            f"Current orientation (yaw): {current_yaw} degrees, facing {cardinal_direction}. "
            f"Current position: x = {position_x}, y = {position_y}, z = {position_z}. "
            "When given a command, you should: "
            "1. Interpret the user's intent with confidence estimation "
            "2. Request clarification if uncertain (confidence < 70%) "
            "3. Generate proactive suggestions when helpful "
            "4. Execute actions with adaptive monitoring "
            "5. Learn from feedback for future improvements "
        )
        return base_message
    
    def get_system_message_commands(self) -> str:
        try:
            from sil_ros.skills import build_registry
            available = build_registry().describe_for_prompt()
        except Exception:
            available = "- (action registry unavailable)"
        return (
            "You are a helpful robot assistant. When users give you commands, you MUST:\n"
            "1. First provide a friendly, conversational response\n"
            "2. Then generate specific Action lines for execution\n"
            "\n"
            "Compose any task as a sequence of the available primitives — do not try to enumerate\n"
            "higher-level tasks. Emit each step as `Action N: PRIMITIVE(arg=value, ...)` using the\n"
            "exact primitive names below, or in plain language.\n"
            "\n"
            "AVAILABLE ACTION PRIMITIVES:\n"
            + available + "\n"
            "\n"
            "EXAMPLES:\n"
            "User: 'move front'\n"
            "Response: I'll move forward.\n"
            "Action 1: FORWARD(distance=1)\n"
            "\n"
            "User: 'do a little dance'\n"
            "Response: Here's a quick routine!\n"
            "Action 1: ROTATE(angle=90)\n"
            "Action 2: FORWARD(distance=0.5)\n"
            "Action 3: ROTATE(angle=180)\n"
            "Action 4: BACKWARD(distance=0.5)\n"
        )
    
    def get_system_message_navigation(self) -> str:
        destinations_list = ', '.join(self.destinations.keys()) if self.destinations else "No destinations configured"
        return (
            f"NAVIGATION CAPABILITIES: You can navigate to coordinates, named destinations "
            f"({destinations_list}), or detected objects. Your start location is (0,0,0). "
            "For navigation requests: "
            "1. Confirm destination understanding if ambiguous "
            "2. Suggest speed preferences if not specified "
            "3. Warn about potential obstacles or long distances "
            "4. Offer route alternatives when appropriate "
            "5. Provide progress updates for long journeys"
        )
    def get_example_messages(self) -> str:
        return (
            "EXAMPLE MESSAGES:\n"
            "1. User: 'Can you move to the kitchen?'\n"
            "   Response: 'Sure, I'll navigate to the kitchen now.'\n"
            "   Action 1: Navigate to kitchen\n"
            "\n"
            "2. User: 'Turn left and move forward 2 meters.'\n"
            "   Response: 'Turning left and moving forward 2 meters.'\n"
            "   Action 1: Turn left 90 degrees\n"
            "   Action 2: Move forward 2 meters\n"
            "\n"
            "3. User: 'What can you do?'\n"
            "   Response: 'I can help with navigation, object detection, and more!'\n"
            "   Action 1: Describe capabilities\n"
        )
    
    def process_input(self, input_text: str, **input_params) -> Dict[str, Any]:
        if hasattr(self, 'process_input_with_uncertainty'):
            response, confidence = self.process_input_with_uncertainty(
                input_text, return_confidence=True
            )
            response['confidence'] = confidence
            response['requires_clarification'] = confidence < self.uncertainty_threshold
            return response
        else:
            combined_system_message = self.build_enhanced_system_message(input_params)
            generated_text = self.call_llm(combined_system_message, input_text)
            rospy.loginfo("LLM output: %s", generated_text)
            if self.is_query_response(generated_text):
                return {'type': 'RESPONSE', 'content': generated_text}
            else:
                return {'type': 'ACTIONS', 'content': generated_text}
    
    def is_query_response(self, response_text: str) -> bool:
        if 'Action' not in response_text:
            return True

        clarification_indicators = [
            'could you clarify', 'could you specify', 'what do you mean',
            'i want to make sure', 'just to confirm', 'to be certain'
        ]
        response_lower = response_text.lower()
        has_clarification = any(indicator in response_lower 
                               for indicator in clarification_indicators)
        return has_clarification
    
    def get_interaction_history(self) -> List[Dict]:
        return self.interaction_history[-10:] 
    
    def clear_interaction_history(self):
        self.interaction_history.clear()
        rospy.loginfo("[SIL-LLM] Interaction history cleared")
    
    def update_uncertainty_threshold(self, new_threshold: float):
        old_threshold = self.uncertainty_threshold
        self.uncertainty_threshold = max(0.1, min(0.9, new_threshold))
        rospy.loginfo(f"[SIL-LLM] Updated uncertainty threshold: {old_threshold:.2f} → {self.uncertainty_threshold:.2f}")
    
    def get_confidence_statistics(self) -> Dict:
        if not self.interaction_history:
            return {'message': 'No interaction history available'}
        recent_confidences = [interaction['confidence'] 
                            for interaction in self.interaction_history[-20:]
                            if 'confidence' in interaction]
        if not recent_confidences:
            return {'message': 'No confidence data available'}
        return {
            'mean_confidence': np.mean(recent_confidences),
            'std_confidence': np.std(recent_confidences),
            'min_confidence': np.min(recent_confidences),
            'max_confidence': np.max(recent_confidences),
            'low_confidence_count': sum(1 for c in recent_confidences if c < self.uncertainty_threshold),
            'total_interactions': len(recent_confidences)
        }