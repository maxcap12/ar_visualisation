FROM osrf/ros:humble-desktop AS base

# Install necessary and useful packages
RUN apt update && \
    apt install --no-install-recommends -y \
    vim python3-pip python3-vcstool wget git \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp 


FROM base AS sgraphs

RUN pip3 install scikit-learn torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2

WORKDIR /workspace/src
RUN git clone https://github.com/snt-arg/lidar_situational_graphs s_graphs
WORKDIR /workspace/src/s_graphs
RUN vcs import . < .rosinstall_ros2

RUN echo "alias s_graphs='bash -c \"ros2 launch lidar_situational_graphs s_graphs_launch.py lidar_topic:=/cloud/undistorted base_frame:=base_link compute_odom:=false\"'" >> ~/.bashrc

FROM sgraphs AS ar_view

RUN python3 -m pip install future websockets
WORKDIR /workspace/src
RUN git clone https://github.com/maxcap12/ar_visualisation_msgs.git
RUN git clone https://github.com/maxcap12/ar_visualisation.git

FROM ar_view AS build

WORKDIR /workspace
RUN rosdep install --from-paths . -y --ignore-src -r
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

ENV ROS_DOMAIN_ID=42
ENV ROS_LOCALHOST_ONLY=0
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
