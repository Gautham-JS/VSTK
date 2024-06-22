ARG ROS_DISTRO=humble
 
########################################
# Base Image for TurtleBot3 Simulation #
########################################
FROM osrf/ros:${ROS_DISTRO}-desktop as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

#RUN apt-get update && apt-get install -y sudo
#RUN adduser --disabled-password --gecos '' docker
#RUN adduser docker sudo
#RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
#USER docker


 
# Create Colcon workspace with external dependencies
RUN mkdir -p /vstk/
COPY ./src/ /vstk/src/
COPY ./include/ /vstk/include/
COPY ./thirdparty/ /vstk/thirdparty/
COPY ./CMakeLists.txt /vstk/CMakeLists.txt
COPY ./build.sh /vstk/build.sh
 
# Build the base Colcon workspace, installing dependencies first.
WORKDIR /vstk
RUN source /opt/ros/${ROS_DISTRO}/setup.bash

RUN sudo apt-get update -y && sudo apt install software-properties-common -y
RUN /bin/bash /vstk/build.sh

