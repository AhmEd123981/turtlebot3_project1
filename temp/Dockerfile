FROM ros:jazzy

# Install ros_gz_sim and dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python dependencies
RUN pip3 install --no-cache-dir numpy
