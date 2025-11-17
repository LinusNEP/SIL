# ============================================================
# Base image: CUDA + cuDNN runtime on Ubuntu 20.04 (ROS Noetic compatible)
# ============================================================
FROM nvidia/cuda:11.8.0-cudnn8-runtime-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Optional but recommended for NVIDIA runtime integration
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics

# ============================================================
# System dependencies
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    lsb-release \
    gnupg2 \
    ca-certificates \
    software-properties-common \
    locales \
    # Python
    python3 \
    python3-pip \
    python3-dev \
    python3-tk \
    python3-distutils \
    # Audio / GUI
    portaudio19-dev \
    libasound-dev \
    pulseaudio \
    # OpenCV deps
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgl1 \
    # Misc
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

# ============================================================
# Install ROS Noetic (desktop-full)
# ============================================================
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list' && \
    curl -sSL "https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc" | apt-key add - && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    ros-noetic-catkin \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=noetic

# Initialize rosdep
RUN rosdep init && rosdep update

# ============================================================
# Python tooling & CUDA-enabled PyTorch stack
# ============================================================
RUN python3 -m pip install --upgrade pip

# Install PyTorch with CUDA (11.8) + base DL stack
RUN pip install --no-cache-dir \
    torch==2.2.0+cu118 \
    torchvision==0.17.0+cu118 \
    torchaudio==2.2.0+cu118 \
    --extra-index-url https://download.pytorch.org/whl/cu118

# ============================================================
# Copy repository and install Python dependencies
# (Assumes this Dockerfile is at the root of the SIL repo)
# ============================================================
WORKDIR /root/catkin_ws

# Catkin workspace structure
RUN mkdir -p /root/catkin_ws/src

# Copy the entire repo as a ROS package into src/sil_ros
# If your package name is different, adjust "sil_ros" accordingly.
COPY . /root/catkin_ws/src/sil_ros

# Install Python requirements (requirements.txt must be in repo root)
RUN pip install --no-cache-dir -r /root/catkin_ws/src/sil_ros/requirements.txt

# ============================================================
# Build catkin workspace
# ============================================================
# Source ROS, then build, then clean temp files
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make"

# ============================================================
# Runtime environment setup
# ============================================================
# Automatically source ROS and workspace in every shell
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc && \
    echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc

# Optional: entrypoint to ensure environment is sourced
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

