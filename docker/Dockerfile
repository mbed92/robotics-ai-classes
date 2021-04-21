FROM ros:noetic

# SETUP ENVS
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# INSTALL SOME ESSENTIAL PROGRAMS
RUN apt-get update     && \
    apt-get upgrade -y && \
    apt-get install -y    \
        lsb-core g++ openssh-server gedit vim git net-tools moreutils \
        python3-pip python3-catkin-tools python3-osrf-pycommon && \
    rm -rf /var/lib/apt/lists/*
RUN pip3 install pybotics

# SETUP SSH
RUN echo root:root | /usr/sbin/chpasswd
RUN echo "PermitRootLogin yes" >> /etc/ssh/sshd_config && service ssh restart
RUN echo "service ssh start" >> ~/.bashrc

# CREATE CATKIN WS
RUN mkdir -p /catkin_ws/src/universal_robot
RUN mkdir -p /catkin_ws/src/move_robot

# COPY UNIVERSAL ROBOT PACKAGE
COPY ./docker/submodules/universal_robot /catkin_ws/src/universal_robot
COPY ./move_robot /catkin_ws/src/move_robot

# INSTALL DEPENDENCIES
WORKDIR /catkin_ws
RUN apt update -qq && rosdep update && rosdep install --from-paths src --ignore-src -y

# BUILD WORKSPACE
RUN /bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash; catkin build"

# FILL BASHRC
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
