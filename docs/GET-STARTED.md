# 🚀 Getting Started
For a hitch-free use of SIL, we recommend following the setup instructions provided here.

### Prerequisites
- **ROS Noetic** (recommended) or ROS 2 Humble & Jazzy (currently being implemented)
- **Python 3.8+**
- **PyTorch** with a CUDA-capable GPU (required for SAM, CLIP, MiDaS, and accelerated LLM usage)
- **A navigation stack**: `move_base` plus a localization source publishing `map → odom` (e.g. AMCL), and an RGB-D camera
- **OpenAI API key** or a compatible LLM provider (DeepSeek, Claude, Gemini, llama.cpp, etc.)

### Installation (native on host)
1. For ROS 1, create a workspace and clone the repository **as the package `sil_ros`**:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
git clone https://github.com/LinusNEP/SIL.git sil_ros
```
2. Create and activate a virtual environment:
```bash
python3 -m venv sil_env
source sil_env/bin/activate
```
3. Install dependencies:
```bash
cd ~/catkin_ws/src/sil_ros
pip install -r requirements.txt
```
4. Download the SAM checkpoint (ViT-B by default) into `models/`:
```bash
wget -P models https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth
```
5. Build the ROS workspace:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
6. Configure API keys:
```bash
export OPENAI_API_KEY="your-api-key-here"
# Or set models/llm_api_key in config/sil_config.yaml
```

## ⚙️ Configuration
**All the tunable hyperparameters are in [`config/sil_config.yaml`](../config/sil_config.yaml)** and can be read
through `sil_ros/config.py`. The defaults reproduce the paper's Table III. Edit the YAML to change behaviour; no code changes are needed.

### Key configuration parameters
```yaml
# Core SIL settings
sil:
  enable: true
  pre_execution_confirmation: true
  post_execution_suggestions: true
  clarification_threshold: 0.3
  uncertainty:
    enable_estimation: true
    ensemble_size: 3            
  follow_up:
    enabled: true               
    num_suggestions: 2
    use_emojis: false
    avoid_self_praise: true     

# Memory
memory:
  max_episodic_memory: 20000    
  semantic_update_threshold: 0.8
  embedding_model: "paraphrase-MiniLM-L6-v2"

# LLM + perception
models:
  llm_provider: "openai"        # openai | deepseek | claude | gemini | llama.cpp
  llm_name: "gpt-4o"
  llm_max_tokens: 500
  llm_temperature: 0.5
  llm_api_key: ""               # or via env var (e.g. OPENAI_API_KEY)
  sam_checkpoint: "models/sam_vit_b_01ec64.pth"

# Open-vocabulary perception backend
perception:
  backend: "clip_sam"           # clip_sam | yolo_world | yoloe | sam3
  open_vocabulary: true
  use_command_vocabulary: true  

# Navigation execution
execution:
  goal_frame: "map"             
  navigation_timeout: 0.0      

# Core ROS topics (update to match your robot)
topics:
  camera_color: "/rgb/image"
  camera_depth: "/depth/image"
  cmd_vel: "/cmd_vel"
  odom: "/odom"
  sil_response: "/sil_response"
  sil_clarification: "/sil_clarification"
  sil_suggestions: "/sil_suggestions"
```

### Topics overview
- **Input / output:**
  - `/llm_output` (`std_msgs/String`) – textual response stream.
  - `/sil_response` (`std_msgs/String`) – SIL-specific responses.
  - `/sil_clarification` (`std_msgs/String`) – clarification questions.
  - `/sil_suggestions` (`std_msgs/String`) – proactive suggestions.
- **SIL feedback and control:**
  - `/sil_feedback` (`std_msgs/String`) – JSON feedback from the chat GUI.
  - `/sil_command` (`std_msgs/String`) – SIL-level commands (e.g. toggles).
  - `/sil_status` (`std_msgs/String`) – status JSON (active, task id, etc.).
  - `/sil_metrics` (`std_msgs/String`) – metrics/logging stream.
  - `/emergency_stop` (`std_msgs/String`) – triggers an emergency stop.
- **Perception & navigation:**
  - `topics/camera_color`, `topics/camera_depth` – input RGB-D streams.
  - `/llm_image_output` (`sensor_msgs/Image`) – images for the chat GUI.
  - `/odom` (`nav_msgs/Odometry`) – odometry used by perception and the action executor.
  - `topics/cmd_vel` (`geometry_msgs/Twist`) – base velocity commands.
  - `move_base` action server (`MoveBaseAction`) – goal navigation.

**Important:** update camera intrinsics, base frames, topics, and destinations to match your robot.

### Running SIL
This launch file starts the SIL controller and the chat GUI together:
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch sil_ros sil_robot.launch
# headless (no GUI), with RViz:
# roslaunch sil_ros sil_robot.launch launch_gui:=false launch_rviz:=true
```
Then interact with the agent using natural language (speech or text). Commands,
clarifications, suggestions, and memory retrieval are handled in real time, e.g.
*"go to the professor's office, describe what you see, and return to the start."*

### Open-vocabulary perception
Detection is open, not a fixed class list. Switch the backend in the config:

| Backend | Capability | Install |
|---|---|---|
| `clip_sam` *(default)* | SAM masks classified with CLIP (open/configurable vocabulary) | included |
| `yolo_world` | YOLO-World open-vocab detection | `pip install ultralytics` |
| `sam3` | SAM 3 promptable concept segmentation | [facebookresearch/sam3](https://github.com/facebookresearch/sam3) |

All backends share the same 3D projection, Kalman tracking, and TF logic, so switching is one
config line plus the matching install.

### Customising destinations
Define robot-relevant spaces (rooms, locations, zones) in `sil_config.yaml`, e.g.,:
```yaml
destinations:
  living_room:
    display_name: "Living Room"
    aliases: ["lounge", "sitting area"]
    coords: {x: 1.0, y: 2.5, z: 0.0}
```

## Advanced usage
**Custom belief models** — extend the shared latent space with domain-specific representations:
```python
class CustomBeliefState(BeliefState):
    def __init__(self):
        super().__init__()
        self.domain_specific_embedding = None
        self.custom_confidence_metrics = {}
```
**New memory types** — implement custom memory structures:
```python
class ProceduralMemory(EpisodicSemanticMemory):
    def store_procedure(self, command_pattern, action_sequence):
        # Custom procedural memory implementation
        pass
```

## 🐳 Docker
SIL ships a GPU-enabled Docker setup that bundles ROS, CUDA, PyTorch, and all Python
dependencies. Useful if you don't want to modify your host setup, need reproducible
experiments, or deploy on lab machines with limited root access.

### Prerequisites
- NVIDIA drivers: `sudo apt install nvidia-driver-535`
- NVIDIA Container Toolkit: `sudo apt-get install -y nvidia-container-toolkit`, then
  `sudo nvidia-ctk runtime configure --runtime=docker` and `sudo systemctl restart docker`.

### Quick start with docker compose
The provided `docker-compose.yml` wires up the GUI (X11), GPU, audio, host networking, and
persistent volumes for learned memory and model caches:
```bash
xhost +local:docker
export OPENAI_API_KEY="your-key"
docker compose up --build
```
Edit `config/sil_config.yaml` on the host (it is bind-mounted) and relaunch, no rebuild needed.
Remove the `deploy.resources` block in the compose file to run CPU-only.

### Manual docker build / run
```bash
docker build -t sil_ros:gpu .
docker run -it --rm \
    --gpus all \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --network=host \
    sil_ros:gpu
```
Inside the container: `roslaunch sil_ros sil_robot.launch`.

Persist memory and config across runs:
```bash
docker run -it --rm --gpus all \
    -v ~/sil_memory:/tmp/sil_memory \
    -v ~/sil_models:/tmp/sil_models \
    --network=host sil_ros:gpu
```
Microphone (speech input): add `--device /dev/snd --group-add audio`.
Camera (USB): add `--device=/dev/video0`.
API keys: add `-e OPENAI_API_KEY="your-key"`.
