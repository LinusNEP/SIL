# Beyond Master and Apprentice: Grounding Foundation Models for Symbiotic Interactive Learning in a Shared Latent Space (SIL)
## Summary Video
[![Watch the Video](https://example.com/thumbnail.jpg)](https://drive.google.com/file/d/1v0kMQgpQIrWXoIKuYJBQMEm0-pLp6JVO/view?usp=sharing)

## 🚀 Getting Started
To replicate the results shown on the [project website](https://sites.google.com/view/anonymousresearcher/home), we .....

### Prerequisites
- **ROS Noetic** (recommended) or ROS2 Humble & Jazzy (currently been implemented)
- **Python 3.8+**
- **PyTorch** with CUDA support (for SAM and CLIP)
- **OpenAI API key** or compatible LLM provider

### Installation
1.  Clone the repository:
   ```bash
git clone https://github.com/.....
cd SIL
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
3. Interact with the SIL agent using natural language!

## ⚙️ Configurations
### Key Configuration Parameters (`sil_config.yaml`)
```yaml
# Core SIL Settings
sil:
  enable: true
  clarification_threshold: 0.5
  proactive_suggestions_enabled: true
  enable_ensemble_uncertainty: true

# Memory Configuration  
memory:
  max_episodic_memory: 1000
  semantic_update_threshold: 0.8

# LLM Configuration
models:
  llm_provider: "openai"  # openai, deepseek, claude, gemini, llama.cpp
  llm_name: "gpt-4o"
```
### Customising Destinations
Add any environment-specific locations in `sil_config.yaml`:
```yaml
destinations:
  living_room:
    display_name: "Living Room"
    aliases: ["lounge", "sitting area"]
    coords: {x: 1.0, y: 2.5, z: 0.0}
```
## 🧪 Capabilities
**Evaluated Task Domains:**
1.  Embodied Instruction Following (EIF)
   - Simple navigation: "go to the kitchen"
   - Complex multi-step: "navigate there and come back here"
   - Conditional reasoning: "stop 2 meters before reaching"

2.  Memory-Based Interactive Information Retrieval (MIIR)
   - Retrospective queries: "what was the last location you visited?"
   - Procedural recall: command alias retention
     
3.   Query-Oriented Reasoning (QOR)
   - Deductive: "navigate to the closer location"
   - Hypothetical: "which locations would be visible from here?"
   - Inductive: generalising from observed patterns

4.  Proactive Dialogue and Suggestion (PDS)
   - Clarification requests for ambiguous commands
   - Context-aware suggestions

5.  Long-Term Preference Learning (LPL)
   - Adapting to communication styles
   - Personalisation: "when I say move, I mean fastest speed"

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
@article{sil2024,
  title={Beyond Master and Apprentice: Grounding Foundation Models for Symbiotic Interactive Learning in a Shared Latent Space},
  author={Author1 and Author2},
  journal={arXiv preprint},
  year={2024}
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

