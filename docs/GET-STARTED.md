
## 🚀 Getting Started
For a hitch-free use of SIL, we recommend following the setup instructions provided here.

### Prerequisites
- **ROS Noetic** (recommended) or ROS2 Humble & Jazzy (currently been implemented)
- **Python 3.8+**
- **PyTorch** with CUDA-capable GPU support (required for SAM, CLIP, MiDaS, and accelerated LLM usage)
- **OpenAI API key** or compatible LLM provider (DeepSeek, Claude, Gemini, llama.cpp, etc.)

### Installation (Native on host)
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
4.  Build the ROS workspace:
```bash
cd ~/catkin_ws
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
cd ~/catkin_ws
source devel/setup.bash
roslaunch sil_ros sil_robot.launch
```
2.  Start the chat interface:
```bash
source devel/setup.bash
roslaunch sil_ros sil_chatGUI.launch
```
3. Interact with the SIL agent using natural language (speech or text)!
   Commands, clarifications, suggestions, and memory retrieval are all handled in real time.

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

## 🐳 Docker
SIL provides a GPU-enabled Docker environment that bundles ROS, CUDA, PyTorch, and all Python dependencies. This is useful if you:
- Don’t want to modify your host ROS/Python setup.
- Need reproducible experiments across machines.
- Deploy on servers or lab machines with limited root access.
### Prerequisites
- Install NVIDIA drivers and the NVIDIA container toolkit: `sudo apt install nvidia-driver-535`
- NVIDIA Container Toolkit: `sudo apt-get install -y nvidia-container-toolkit`, `sudo nvidia-ctk runtime configure --runtime=docker`, `sudo systemctl restart docker`.
### Build the SIL Docker image
```bash
docker build -t sil_ros:gpu .
```
### Run SIL in Docker (with GPU and GUI)
```bash
docker run -it --rm \
    --gpus all \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --network=host \
    sil_ros:gpu
```
Ensure X11 is enabled: `xhost +local:docker`.
Inside the container:
```bash
roslaunch sil_ros sil_robot.launch
roslaunch sil_ros sil_chatGUI.launch
```
To persist memory and configuration across container runs:
```bash
mkdir -p ~/sil_memory ~/sil_config
docker run -it --rm \
    --gpus all \
    -v ~/sil_memory:/root/sil_memory \
    -v ~/sil_config:/root/sil_config \
    --network=host \
    sil_ros:gpu
```
For iterative development:
```bash
docker run -it --rm \
    --gpus all \
    -v $(pwd):/root/catkin_ws/src/sil_ros \
    --network=host \
    sil_ros:gpu
```
Then, inside the container:
```bash
cd /root/catkin_ws
catkin_make
```
Accessing the host microphone (speech input):
```bash
docker run -it --rm \
    --gpus all \
    --device /dev/snd \
    --group-add audio \
    --network=host \
    sil_ros:gpu
```
Accessing the host camera (USB):
```bash
docker run -it --rm \
    --gpus all \
    --device=/dev/video0 \
    --network=host \
    sil_ros:gpu
```
Pass API keys at runtime:
```bash
docker run -it --rm \
    --gpus all \
    -e OPENAI_API_KEY="your-key" \
    --network=host \
    sil_ros:gpu
```

