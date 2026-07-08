# Development container for the AGV greenhouse workspace.
# Mirrors the CI environment (.github/workflows/ci.yaml): ROS 2 Humble
# base image + colcon/rosdep + Node 20 for the TypeScript packages.
FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-rosdep \
      git \
      curl \
      ca-certificates \
    && curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/*

# rosdep is initialized in the base image; refresh the cache at build time
# so postCreate only resolves workspace deps.
RUN rosdep update

# Source ROS in every shell.
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc
