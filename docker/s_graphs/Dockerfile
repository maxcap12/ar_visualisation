FROM ros:humble

# Install necessary and useful packages
RUN apt update && \
    apt install --no-install-recommends -y \
    ssh \
    vim \
    python3-pip \
    python3-vcstool \
    wget \
    git \
    nano \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp 

RUN python3 -m pip install future py-trees scikit-learn scipy==1.13.1 numpy==1.23.4

# setup ssh key
ARG ssh_prv_key
ARG ssh_pub_key

RUN mkdir -p -m 0700 /root/.ssh && \
    ssh-keyscan -H github.com >> /root/.ssh/known_hosts

RUN echo "$ssh_prv_key" > /root/.ssh/id_rsa && \
    echo "$ssh_pub_key" > /root/.ssh/id_rsa.pub && \
    chmod 600 /root/.ssh/id_rsa && \
    chmod 600 /root/.ssh/id_rsa.pub

#  Creating working spaces directory and clone s_graphs inside it
RUN mkdir -p /workspaces
WORKDIR /workspaces
RUN git clone git@github.com:snt-arg/situational_graphs.git s_graphs

# Fetching all packages specified in the .rosinstall file and running build everything with colcon build.
WORKDIR /workspaces/s_graphs
RUN vcs import . < .rosinstall_ros2
WORKDIR /workspaces/s_graphs
RUN sudo apt install python3-rosdep
RUN rosdep install --from-paths . -y --ignore-src -r
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Install mprocs
WORKDIR /
RUN wget https://github.com/pvolok/mprocs/releases/download/v0.6.4/mprocs-0.6.4-linux64.tar.gz
RUN tar -xf mprocs* && \
    rm mprocs*.tar.gz && \
    mv mprocs /usr/local/bin

# alias for mprocs
RUN echo "alias mprocs_real='mprocs -c /workspaces/s_graphs/.real_mprocs.yaml'" >> /.bashrc
RUN echo "alias mprocs_virtual='mprocs -c /workspaces/s_graphs/.virtual_mprocs.yaml'" >> /.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
RUN echo "alias s_graphs='bash -c \"source /workspaces/s_graphs/install/setup.bash && ros2 launch lidar_situational_graphs s_graphs_launch.py lidar_topic:=/cloud/undistorted base_frame:=base_link compute_odom:=false\"'" >> ~/.bashrc

ENV ROS_DOMAIN_ID=42
ENV ROS_LOCALHOST_ONLY=0

# remove the ssh keys
RUN rm -rf /.ssh/