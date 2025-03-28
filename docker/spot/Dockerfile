FROM ros:humble

# Set noninteractive mode for APT
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary and useful packages
RUN apt update && \
    apt install --no-install-recommends -y \
    ssh vim python3-pip python3-vcstool wget git \
    ros-$ROS_DISTRO-velodyne \
    ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-rmw-cyclonedds-cpp 

# Set up workspace
WORKDIR /ros_ws/src

# Initialize rosdep
RUN rosdep update

# Clone driver code
RUN git clone https://github.com/bdaiinstitute/spot_ros2.git .
RUN git submodule update --init --recursive

# Run install script and pass in the architecture
RUN ARCH=$(dpkg --print-architecture) && echo "Building driver with $ARCH" && /ros_ws/src/install_spot_ros2.sh --$ARCH

# Build packages with Colcon
WORKDIR /ros_ws/
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

ENV ROS_DOMAIN_ID=42
ENV ROS_LOCALHOST_ONLY=0
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
