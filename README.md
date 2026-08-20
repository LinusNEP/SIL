## SIL: Symbiotic Interactive Learning for Language-Conditioned Human-Agent Co-Adaptation

> Most language-conditioned HRI follows a one-way *master–apprentice* pattern: the human commands, the agent executes. **SIL** instead treats human and agent as *co-adaptive* systems that align joint belief states in a **shared latent task space**, enabling proactive clarification, adaptive suggestions, and shared plan refinement. It grounds foundation models for spatial perception and reasoning through a triplet-loss-trained latent encoder, and uses episodic/semantic memory regularised by **elastic weight consolidation (EWC)** to mitigate catastrophic forgetting.

[![SIL Official](https://img.shields.io/badge/SIL%20Official-Website-lightblue?style=flat&logo=globe&logoColor=white)](https://linusnep.github.io/SIL/)
[![arXiv](https://img.shields.io/badge/arXiv-2511.05203-b31b1b.svg)](https://arxiv.org/abs/2511.05203)
[![ROS 1](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](http://www.ros.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://index.ros.org/doc/ros2/)
[![Python](https://img.shields.io/badge/Python-≥3.8-blue.svg)](https://www.python.org/)
[![License: CC BY 4.0](https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/)
[![PyPI](https://img.shields.io/badge/PyPI-PIP-orange.svg)](https://pypi.org/)

🌐 [Website](https://linusnep.github.io/SIL/) | 📄[Paper](https://arxiv.org/abs/2511.05203)| 🚀 [Get Started](docs/GET-STARTED.md) | ⚙️ [Install](docs/GET-STARTED.md) | ⚖️ [License](https://creativecommons.org/licenses/by/4.0/)

## Getting Started
To reproduce the experiments and demos shown on the [project website](https://linusnep.github.io/SIL/), we recommend following the installation instructions provided at [GET-STARTED.md](docs/GET-STARTED.md).

## Summary Video
[![Watch the Video](https://github.com/LinusNEP/SIL/blob/main/docs/sil-sim.png?raw=true)](https://youtu.be/aua734moxwU?si=yGa7YPqHaqM9TaHa)

## Citation
If you use SIL in your research, consider citing our paper:
```bibtex
@article{nwankwo2026silsymbioticinteractivelearning,
      title={SIL: Symbiotic Interactive Learning for Language-Conditioned Human-Agent Co-Adaptation}, 
      author={Linus Nwankwo and Bjoern Ellensohn and Christian Rauch and Elmar Rueckert},
      year={2026},
      eprint={2511.05203},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2511.05203}, 
}
```
## Contributing
We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md) for details.

### Development Setup
1.  Fork the repository

2.  Create a feature branch: `git checkout -b feature/amazing-feature`

3.  Commit changes: `git commit -m 'Add amazing feature'`

4.  Push to branch: `git push origin feature/amazing-feature`

5.  Open a Pull Request

Before opening a PR, please run the checks: `python -m compileall src scripts` and `python tools/check_config_keys.py` (verifies every `get_param` key exists in the config).

## License
This work is licensed under a [Creative Commons Attribution International 4.0 License](https://creativecommons.org/licenses/by/4.0/).

## Acknowledgments
- This work was supported by the MINEVIEW project (#FO999927835), funded by the Republic of Austria, Federal Ministry of Environment, Innovation and Technology
- Built upon foundational research in human-robot interaction and continual learning
- Uses open-source components including ROS, PyTorch, and various foundation models

**Repository:**
- [TCC](https://github.com/LinusNEP/TCC-IRoNL)
- [ROMR](https://github.com/LinusNEP/ROMR.git)
- [Unitree_ros](https://github.com/macc-n/ros_unitree.git)
