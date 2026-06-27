"""SIL — Symbiotic Interactive Learning (ROS package ``sil_ros``).

Reference:
    Nwankwo et al., "SIL: Symbiotic Interactive Learning for Language-
    Conditioned Human-Agent Co-Adaptation." https://linusnep.github.io/SIL/
"""

from .config import SILConfig, ParamStore, get_param

__all__ = ["SILConfig", "ParamStore", "get_param"]
