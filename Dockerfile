FROM ros:humble

# Install necessary and useful packages
RUN apt update && \
    apt install --no-install-recommends -y \
    ssh vim python3-pip python3-vcstool wget git iputils-ping\
    ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-rmw-cyclonedds-cpp 

RUN pip install websockets

COPY . /workspace/src/ar_visualisation
WORKDIR /workspace/src
RUN git clone https://github.com/maxcap12/ar_visualisation_msgs.git

WORKDIR /workspace/src/ar_visualisation_msgs
RUN . /opt/ros/humble/setup.sh && colcon build

WORKDIR /workspace
RUN . /workspace/src/ar_visualisation_msgs/install/setup.sh && colcon build

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc
RUN echo "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc