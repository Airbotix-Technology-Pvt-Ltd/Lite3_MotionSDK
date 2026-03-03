FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# ── Base tools ─────────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    curl gnupg lsb-release wget git \
    python3-pip libdw-dev \
    && rm -rf /var/lib/apt/lists/*


# ── ROS2 Humble ───────────────────────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ── Python deps for MuJoCo sim ────────────────────────────────────────────────
RUN pip3 install "numpy<2.0" mujoco scipy

# ── backward-cpp for crash debugging ─────────────────────────────────────────
RUN wget https://raw.githubusercontent.com/bombela/backward-cpp/master/backward.hpp && \
    mv backward.hpp /usr/include

# ── Clone sdk_deploy (MuJoCo sim node + drdds msgs) ──────────────────────────
WORKDIR /workspace
RUN git clone https://github.com/DeepRoboticsLab/sdk_deploy.git

# Fix gitlab→github for eigen submodule
RUN cd sdk_deploy && \
    find . -name ".gitmodules" | xargs grep -l "eigen" 2>/dev/null | \
    xargs sed -i 's|https://gitlab.com/libeigen/eigen.git|https://github.com/live-clones/eigen.git|g' \
    2>/dev/null || true

RUN cd sdk_deploy && \
    git submodule sync && \
    git submodule update --init --recursive

# ── libevdev needed by lite3_sdk_deploy CMakeLists ───────────────────────────
# Separate layer — keeps everything above cached on rebuild
RUN apt-get update && apt-get install -y libevdev-dev && rm -rf /var/lib/apt/lists/*

# ── Build sdk_deploy (drdds msgs + rl_deploy node) ───────────────────────────
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    cd sdk_deploy && \
    colcon build \
    --packages-up-to lite3_sdk_deploy \
    --cmake-args -DBUILD_PLATFORM=x86"

# ── Copy our custom gait package ──────────────────────────────────────────────
COPY . /workspace/lite3_custom_gait_src/

# ── Build our custom gait package ─────────────────────────────────────────────
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    source /workspace/sdk_deploy/install/setup.bash && \
    mkdir -p /workspace/custom_gait_ws/src && \
    cp -r /workspace/lite3_custom_gait_src /workspace/custom_gait_ws/src/lite3_custom_gait && \
    cd /workspace/custom_gait_ws && \
    colcon build --cmake-args -DBUILD_PLATFORM=x86"

# ── Copy launch script ────────────────────────────────────────────────────────
COPY docker/launch.sh /workspace/launch.sh
RUN chmod +x /workspace/launch.sh

# ── Shell convenience (.bashrc) ───────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash"                   >> /root/.bashrc && \
    echo "source /workspace/sdk_deploy/install/setup.bash"     >> /root/.bashrc && \
    echo "source /workspace/custom_gait_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=1"                              >> /root/.bashrc

WORKDIR /workspace

# ── Mesa software OpenGL — very last layer so nothing above is invalidated ────
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx libgl1-mesa-dri libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/*

# ── Default: launch sim + gait together ──────────────────────────────────────
CMD ["bash", "/workspace/launch.sh"]
