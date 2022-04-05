FROM ros:melodic
ARG ssh_prv_key

# install ros
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full \
    python3-catkin-tools \
    libtool-bin \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'

# install other packages
RUN apt-get update && apt-get install -y \
        nano  \
        git \
        ros-melodic-hector-slam \
        autoconf \
        openssh-server

# Authorize SSH Host
RUN mkdir -p /root/.ssh && \
    chmod 0700 /root/.ssh && \
    ssh-keyscan gitlab.lrz.de > /root/.ssh/known_hosts

# Add the keys and set permissions
RUN echo "$ssh_prv_key" > /root/.ssh/id_rsa && \
    chmod 600 /root/.ssh/id_rsa

# clone your git using SSH connection
WORKDIR /home/ros
RUN git clone -b develop git@gitlab.lrz.de:autonomous-systems-2021-group-pi/autonomous-systems-2021-group-pi.git
# you might need additional git operations here

# Remove SSH keys from docker
RUN rm -rf /root/.ssh/

# set working directory to project
WORKDIR /home/ros/autonomous-systems-2021-group-pi/Project/simulation/unity_template/catkin_ws
RUN git submodule init
RUN git submodule update

RUN catkin config --extend /opt/ros/melodic

