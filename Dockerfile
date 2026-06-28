# =====================================================================
# SIL — Symbiotic Interactive Learning
# Build:  docker build -t sil:latest .
# Run:    see docker-compose.yml (handles GUI/X11, GPU, and devices)
# =====================================================================
FROM ros:noetic-robot

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    PYTHONUNBUFFERED=1

# --- System & ROS dependencies -------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-catkin-tools build-essential git wget \
        ros-noetic-move-base-msgs ros-noetic-move-base \
        ros-noetic-tf ros-noetic-cv-bridge ros-noetic-vision-opencv \
        ros-noetic-dynamic-reconfigure ros-noetic-actionlib \
        ros-noetic-amcl ros-noetic-map-server \
        python3-tk mpg123 espeak ffmpeg libsm6 libxext6 portaudio19-dev \
    && rm -rf /var/lib/apt/lists/*

# --- Catkin workspace ----------------------------------------------------------
ENV CATKIN_WS=/root/catkin_ws
WORKDIR $CATKIN_WS/src

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

COPY . $CATKIN_WS/src/sil_ros

# --- Build the workspace -------------------------------------------------------
WORKDIR $CATKIN_WS
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make"

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source $CATKIN_WS/devel/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "sil_ros", "sil_robot.launch"]
