#!/usr/bin/env python
from __future__ import annotations
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional
import yaml

try:  
    import rospy

    def _ros_available() -> bool:
        try:
            return rospy.core.is_initialized()
        except Exception:
            return False
except Exception:  
    rospy = None

    def _ros_available() -> bool:
        return False

def _default_config_path() -> Optional[str]:
    env_path = os.environ.get("SIL_CONFIG")
    if env_path and os.path.isfile(env_path):
        return env_path
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "..", "..", "config", "sil_config.yaml"),
        os.path.join(here, "..", "config", "sil_config.yaml"),
    ]
    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if os.path.isfile(candidate):
            return candidate
    return None

class ParamStore:
    def __init__(self, yaml_path: Optional[str] = None):
        self._yaml: Dict[str, Any] = {}
        path = yaml_path or _default_config_path()
        if path and os.path.isfile(path):
            with open(path, "r") as handle:
                self._yaml = yaml.safe_load(handle) or {}

    def _from_yaml(self, key: str, default: Any) -> Any:
        node: Any = self._yaml
        for part in key.strip("/").split("/"):
            if isinstance(node, dict) and part in node:
                node = node[part]
            else:
                return default
        return node

    def get(self, key: str, default: Any = None) -> Any:
        if _ros_available():
            try:
                return rospy.get_param("/" + key.strip("/"), default)
            except Exception:
                pass
        return self._from_yaml(key, default)

@dataclass
class EncoderConfig:
    input_dim: int = 768
    hidden_dims: List[int] = field(default_factory=lambda: [512, 384])
    latent_dim: int = 256
    dropout: float = 0.2
    learning_rate: float = 0.001
    sentence_encoder_model: str = "all-mpnet-base-v2"


@dataclass
class CoAdaptationConfig:
    enable: bool = True
    belief_update_eta: List[float] = field(
        default_factory=lambda: [0.6, 0.3, 0.1, 0.7, 0.2, 0.1]
    )
    alpha_agent: float = 0.1
    alpha_human: float = 0.05
    influence_init_std: float = 0.01
    influence_matrix_lr: float = 0.01
    confidence_blend_agent: float = 0.7
    confidence_blend_human: float = 0.8


@dataclass
class BeliefInitConfig:
    human_confidence: float = 0.8
    agent_confidence: float = 0.5
    init_noise_std: float = 0.1
    intent_distribution_dim: int = 10


@dataclass
class AlignmentConfig:
    misalignment_threshold: float = 0.6
    uncertainty_source_threshold: float = 0.3


@dataclass
class TrainingConfig:
    triplet_margin: float = 1.0
    ewc_lambda: float = 2000.0
    positive_similarity_threshold: float = 0.7
    negative_similarity_threshold: float = 0.3
    negative_success_threshold: float = 0.4


@dataclass
class UncertaintyConfig:
    enable_estimation: bool = True
    ensemble_size: int = 3
    uncertainty_threshold: float = 0.3
    temperature_variation: float = 0.1
    alpha_u: float = 0.4
    beta_u: float = 0.2
    confidence_baseline: float = 0.5
    high_confidence_weight: float = 0.15
    medium_confidence_weight: float = 0.05
    low_confidence_weight: float = 0.15
    high_confidence_keywords: List[str] = field(default_factory=list)
    medium_confidence_keywords: List[str] = field(default_factory=list)
    low_confidence_keywords: List[str] = field(default_factory=list)


@dataclass
class ContinualLearningConfig:
    enable: bool = True
    replay_buffer_capacity: int = 10000
    replay_batch_size: int = 32
    task_shift_window: int = 10
    task_shift_drop_threshold: float = 0.3
    checkpoint_keep_recent: int = 1000


@dataclass
class RetrievalConfig:
    semantic_weight: float = 0.6
    belief_weight: float = 0.4
    softmax_temperature: float = 0.1
    max_episodes: int = 5


@dataclass
class MemoryConfig:
    max_episodic_memory: int = 20000
    semantic_update_threshold: float = 0.8
    episodic_retention_days: int = 30
    max_semantic_patterns_per_type: int = 50
    max_retrieval_candidates: int = 200
    retention_success_threshold: float = 0.9
    retention_alignment_threshold: float = 0.8
    co_adaptation_min_episodes: int = 10
    co_adaptation_improvement: float = 0.2
    embedding_model: str = "paraphrase-MiniLM-L6-v2"
    human_learning_rate: float = 0.1
    human_confidence_threshold: float = 0.7
    default_adaptation_responsiveness: float = 0.5
    formality_indicators: List[str] = field(default_factory=list)
    specificity_indicators: List[str] = field(default_factory=list)
    retrieval: RetrievalConfig = field(default_factory=RetrievalConfig)


@dataclass
class SILConfig:
    encoder: EncoderConfig = field(default_factory=EncoderConfig)
    co_adaptation: CoAdaptationConfig = field(default_factory=CoAdaptationConfig)
    belief_init: BeliefInitConfig = field(default_factory=BeliefInitConfig)
    alignment: AlignmentConfig = field(default_factory=AlignmentConfig)
    training: TrainingConfig = field(default_factory=TrainingConfig)
    uncertainty: UncertaintyConfig = field(default_factory=UncertaintyConfig)
    continual_learning: ContinualLearningConfig = field(
        default_factory=ContinualLearningConfig
    )
    memory: MemoryConfig = field(default_factory=MemoryConfig)
    clarification_threshold: float = 0.3
    @classmethod
    def load(cls, yaml_path: Optional[str] = None) -> "SILConfig":
        store = ParamStore(yaml_path)
        g = store.get
        encoder = EncoderConfig(
            input_dim=int(g("sil/encoder/input_dim", 768)),
            hidden_dims=list(g("sil/encoder/hidden_dims", [512, 384])),
            latent_dim=int(g("sil/encoder/latent_dim", 256)),
            dropout=float(g("sil/encoder/dropout", 0.2)),
            learning_rate=float(g("sil/encoder/learning_rate", 0.001)),
            sentence_encoder_model=str(
                g("sil/encoder/sentence_encoder_model", "all-mpnet-base-v2")
            ),
        )

        co_adaptation = CoAdaptationConfig(
            enable=bool(g("sil/co_adaptation/enable", True)),
            belief_update_eta=list(
                g("sil/co_adaptation/belief_update_eta", [0.6, 0.3, 0.1, 0.7, 0.2, 0.1])
            ),
            alpha_agent=float(g("sil/co_adaptation/alpha_agent", 0.1)),
            alpha_human=float(g("sil/co_adaptation/alpha_human", 0.05)),
            influence_init_std=float(g("sil/co_adaptation/influence_init_std", 0.01)),
            influence_matrix_lr=float(g("sil/co_adaptation/influence_matrix_lr", 0.01)),
            confidence_blend_agent=float(
                g("sil/co_adaptation/confidence_blend_agent", 0.7)
            ),
            confidence_blend_human=float(
                g("sil/co_adaptation/confidence_blend_human", 0.8)
            ),
        )

        belief_init = BeliefInitConfig(
            human_confidence=float(g("sil/belief_init/human_confidence", 0.8)),
            agent_confidence=float(g("sil/belief_init/agent_confidence", 0.5)),
            init_noise_std=float(g("sil/belief_init/init_noise_std", 0.1)),
            intent_distribution_dim=int(
                g("sil/belief_init/intent_distribution_dim", 10)
            ),
        )

        alignment = AlignmentConfig(
            misalignment_threshold=float(
                g("sil/alignment/misalignment_threshold", 0.6)
            ),
            uncertainty_source_threshold=float(
                g("sil/alignment/uncertainty_source_threshold", 0.3)
            ),
        )

        training = TrainingConfig(
            triplet_margin=float(g("sil/training/triplet_margin", 1.0)),
            ewc_lambda=float(g("sil/training/ewc_lambda", 2000.0)),
            positive_similarity_threshold=float(
                g("sil/training/positive_similarity_threshold", 0.7)
            ),
            negative_similarity_threshold=float(
                g("sil/training/negative_similarity_threshold", 0.3)
            ),
            negative_success_threshold=float(
                g("sil/training/negative_success_threshold", 0.4)
            ),
        )

        uncertainty = UncertaintyConfig(
            enable_estimation=bool(g("sil/uncertainty/enable_estimation", True)),
            ensemble_size=int(g("sil/uncertainty/ensemble_size", 3)),
            uncertainty_threshold=float(
                g("sil/uncertainty/uncertainty_threshold", 0.3)
            ),
            temperature_variation=float(
                g("sil/uncertainty/temperature_variation", 0.1)
            ),
            alpha_u=float(g("sil/uncertainty/alpha_u", 0.4)),
            beta_u=float(g("sil/uncertainty/beta_u", 0.2)),
            confidence_baseline=float(g("sil/uncertainty/confidence_baseline", 0.5)),
            high_confidence_weight=float(
                g("sil/uncertainty/high_confidence_weight", 0.15)
            ),
            medium_confidence_weight=float(
                g("sil/uncertainty/medium_confidence_weight", 0.05)
            ),
            low_confidence_weight=float(
                g("sil/uncertainty/low_confidence_weight", 0.15)
            ),
            high_confidence_keywords=list(
                g("sil/uncertainty/high_confidence_keywords", [])
            ),
            medium_confidence_keywords=list(
                g("sil/uncertainty/medium_confidence_keywords", [])
            ),
            low_confidence_keywords=list(
                g("sil/uncertainty/low_confidence_keywords", [])
            ),
        )

        continual_learning = ContinualLearningConfig(
            enable=bool(g("sil/continual_learning/enable", True)),
            replay_buffer_capacity=int(
                g("sil/continual_learning/replay_buffer_capacity", 10000)
            ),
            replay_batch_size=int(g("sil/continual_learning/replay_batch_size", 32)),
            task_shift_window=int(g("sil/continual_learning/task_shift_window", 10)),
            task_shift_drop_threshold=float(
                g("sil/continual_learning/task_shift_drop_threshold", 0.3)
            ),
            checkpoint_keep_recent=int(
                g("sil/continual_learning/checkpoint_keep_recent", 1000)
            ),
        )

        retrieval = RetrievalConfig(
            semantic_weight=float(g("memory/retrieval/semantic_weight", 0.6)),
            belief_weight=float(g("memory/retrieval/belief_weight", 0.4)),
            softmax_temperature=float(g("memory/retrieval/softmax_temperature", 0.1)),
            max_episodes=int(g("memory/retrieval/max_episodes", 5)),
        )

        memory = MemoryConfig(
            max_episodic_memory=int(g("memory/max_episodic_memory", 20000)),
            semantic_update_threshold=float(g("memory/semantic_update_threshold", 0.8)),
            episodic_retention_days=int(g("memory/episodic_retention_days", 30)),
            max_semantic_patterns_per_type=int(
                g("memory/max_semantic_patterns_per_type", 50)
            ),
            max_retrieval_candidates=int(g("memory/max_retrieval_candidates", 200)),
            retention_success_threshold=float(
                g("memory/retention_success_threshold", 0.9)
            ),
            retention_alignment_threshold=float(
                g("memory/retention_alignment_threshold", 0.8)
            ),
            co_adaptation_min_episodes=int(g("memory/co_adaptation_min_episodes", 10)),
            co_adaptation_improvement=float(g("memory/co_adaptation_improvement", 0.2)),
            embedding_model=str(g("memory/embedding_model", "paraphrase-MiniLM-L6-v2")),
            human_learning_rate=float(g("memory/human_model/learning_rate", 0.1)),
            human_confidence_threshold=float(
                g("memory/human_model/confidence_threshold", 0.7)
            ),
            default_adaptation_responsiveness=float(
                g("memory/human_model/default_adaptation_responsiveness", 0.5)
            ),
            formality_indicators=list(
                g("memory/human_model/formality_indicators",
                  ["please", "could you", "would you", "thank you"])
            ),
            specificity_indicators=list(
                g("memory/human_model/specificity_indicators",
                  ["exactly", "precisely", "specifically", "at coordinates"])
            ),
            retrieval=retrieval,
        )

        return cls(
            encoder=encoder,
            co_adaptation=co_adaptation,
            belief_init=belief_init,
            alignment=alignment,
            training=training,
            uncertainty=uncertainty,
            continual_learning=continual_learning,
            memory=memory,
            clarification_threshold=float(g("sil/clarification_threshold", 0.3)),
        )

def get_param(key: str, default: Any = None, yaml_path: Optional[str] = None) -> Any:
    return ParamStore(yaml_path).get(key, default)
