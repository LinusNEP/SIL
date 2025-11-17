## Beyond Master and Apprentice: Grounding Foundation Models for Symbiotic Interactive Learning in a Shared Latent Space (SIL)
[![SIL Official](https://img.shields.io/badge/SIL%20Official-Website-lightblue?style=flat&logo=globe&logoColor=white)](https://sites.google.com/view/anonymousresearcher/home)
[![ROS 1](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](http://www.ros.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://index.ros.org/doc/ros2/)
[![Python](https://img.shields.io/badge/Python-≥3.8-blue.svg)](https://www.python.org/)
[![License: CC BY 4.0](https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/)
[![PyPI](https://img.shields.io/badge/PyPI-PIP-orange.svg)](https://pypi.org/)
[![GitHub Stars](https://img.shields.io/github/stars/LinusNEP/SIL?style=social)](https://github.com/LinusNEP/SIL/stargazers)
[![Last Commit](https://img.shields.io/github/last-commit/LinusNEP/SIL.svg)](https://github.com/LinusNEP/SIL/commits)

## Summary Video
[![Watch the Video](https://github.com/LinusNEP/SIL/blob/main/docs/sil-sim.png?raw=true)](https://drive.google.com/file/d/1v0kMQgpQIrWXoIKuYJBQMEm0-pLp6JVO/view?usp=sharing)

## 🚀 Getting Started
To reproduce the experiments and demonstrations shown on the [project website](https://sites.google.com/view/anonymousresearcher/home), we recommend following the setup instructions provided here.

### Prerequisites
- **ROS Noetic** (recommended) or ROS2 Humble & Jazzy (currently been implemented)
- **Python 3.8+**
- **PyTorch** with CUDA support (required for SAM, CLIP, and MiDaS)
- **OpenAI API key** or compatible LLM provider

### Installation
1.  For ROS 1, create a workspace and clone the repository:
   ```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
git clone https://github.com/LinusNEP/SIL.git
cd ~/catkin_ws
```
2.  Create and activate a virtual environment:
   ```bash
python3 -m venv sil_env
source sil_env/bin/activate
```
3.  Install dependencies:
```bash
pip install -r requirements.txt
```
4.  Set up ROS workspace:
```bash
catkin_make
source devel/setup.bash
```
5.  Configure API keys:
```bash
export OPENAI_API_KEY="your-api-key-here"
# Or configure in sil_config.yaml
```
## ⚙️ Configurations
### Key Configuration Parameters (`sil_config.yaml`)
```yaml
# Core SIL Settings
sil:
  enable: true
  pre_execution_confirmation: true
  post_execution_suggestions: true
  enable_ensemble_uncertainty: true
  clarification_threshold: 0.5

# Memory Configuration  
memory:
  max_episodic_memory: 1000
  semantic_update_threshold: 0.8
  memory_directory: "/tmp/sil_memory"
  auto_save_memory: true
  memory_save_interval: 300

# LLM and perception Configurations
models:
  llm_provider: "openai"  # openai, deepseek, claude, gemini, llama.cpp
  llm_name: "gpt-4o"
  llm_max_tokens: 500
  llm_temperature: 0.5
  llm_api_key: ""          # or via env var (e.g. OPENAI_API_KEY)
  sam_checkpoint: "config/sam_vit_b_01ec64.pth"
  embedding_model: "paraphrase-MiniLM-L6-v2"

# Core ROS topics
topics:
  camera_color: "/camera/color/image_raw"
  camera_depth: "/camera/depth/image_rect_raw"
  cmd_vel: "/nav_vel"
  odom: "/odom"

  sil_response: "/sil_response"
  sil_clarification: "/sil_clarification"
  sil_suggestions: "/sil_suggestions"
  sil_feedback: "/sil_feedback"
  sil_command: "/sil_command"
  sil_status: "/sil_status"
  sil_metrics: "/sil_metrics"
```
### Topics Overview
- **Input / Output:**
    - `/llm_output` (`std_msgs/String`) – textual response stream.
    - `/sil_response` (`std_msgs/String`) – SIL-specific responses.
    - `/sil_clarification` (`std_msgs/String`) – clarification questions.
    - `/sil_suggestions` (`std_msgs/String`) – proactive suggestions.

- **SIL feedback and control:**
   - `/sil_feedback` (`std_msgs/String`) – JSON feedback from chatGUI.
   - `/sil_command` (`std_msgs/String`) – SIL-level commands (e.g. toggles).
   - `/sil_status` (`std_msgs/String`) – status JSON (active, task id, etc.).
   - `/sil_metrics` (`std_msgs/String`) – metrics/logging stream.
   - `/emergency_stop` (`std_msgs/String`) – triggers emergency stop.

- **Perception & navigation:**
    - `topics/camera_color`, `topics/camera_depth` – input RGBD streams.
    - `/llm_image_output` (`sensor_msgs/Image`) – images for chatGUI.
    - `/odom` (`nav_msgs/Odometry`) – robot odometry used by perception and action executor.
    - `/nav_vel` (`geometry_msgs/Twist`) – velocity commands for base.
    - `move_base` action server (`MoveBaseAction`) – goal navigation.
**Important:** Update camera intrinsics, base frames, and destinations to match your robot setup.

### Running SIL
1.  Launch SIL core:
```bash
source devel/setup.bash
roslaunch sil_ros sil_robot.launch
```
2.  Start the chat interface:
```bash
source devel/setup.bash
roslaunch sil_ros sil_chatGUI.launch
```
3. Interact with the SIL agent using natural language! Commands, clarifications, suggestions, and memory retrieval are all handled in real time.

### Customising Destinations
Define robot-relevant spaces (rooms, locations, zones) in `sil_config.yaml`:
```yaml
destinations:
  living_room:
    display_name: "Living Room"
    aliases: ["lounge", "sitting area"]
    coords: {x: 1.0, y: 2.5, z: 0.0}
```
Destinations are automatically interpreted by the Action Executor and used for navigation planning.
## 🔧 Advanced Usage
**Custom Belief Models**
Extend the shared latent space with domain-specific representations:
```python
class CustomBeliefState(BeliefState):
    def __init__(self):
        super().__init__()
        self.domain_specific_embedding = None
        self.custom_confidence_metrics = {}
```
**Adding New Memory Types**
Implement custom memory structures:
```python
class ProceduralMemory(EpisodicSemanticMemory):
    def store_procedure(self, command_pattern, action_sequence):
        # Custom procedural memory implementation
        pass
```

## 📝 Citation
If you use SIL in your research, please cite our paper:
```bibtex
@article{nwankwo2025beyond,
  title={Beyond Master and Apprentice: Grounding Foundation Models for Symbiotic Interactive Learning in a Shared Latent Space},
  author={Nwankwo, Linus and Ellensohn, Bj{\"o}rn and Rauch, Christian and Rueckert, Elmar},
  journal={arXiv preprint arXiv:2511.05203},
  year={2025}
}
```
## 🤝 Contributing
We welcome contributions! Please see our [Contributing Guidelines](https://...) for details.

### Development Setup
1.  Fork the repository

2.  Create a feature branch: `git checkout -b feature/amazing-feature`

3.  Commit changes: `git commit -m 'Add amazing feature'`

4.  Push to branch: `git push origin feature/amazing-feature`

5.  Open a Pull Request

## 📄 License
This work is licensed under a [Creative Commons Attribution International 4.0 License](https://creativecommons.org/licenses/by/4.0/).

## 🙏 Acknowledgments
- This work was supported by the xxxxxx organisation
- Built upon foundational research in human-robot interaction and continual learning
- Uses open-source components including ROS, PyTorch, and various foundation models

**Repository:**
- [TCC](https://github.com/LinusNEP/TCC-IRoNL)
- [ROMR](https://github.com/LinusNEP/ROMR.git)
- [Unitree_ros](https://github.com/macc-n/ros_unitree.git)

