#!/bin/bash

apt-get install -y  libopencv-dev
apt-get install -y  libopenmpi-dev

# PCL installation.
# ENV variable DEBIAN_FRONTEND=noninteractive should be set,
# otherwise it could require to interactivly choose the settings for keyboard-configuration package 
DEBIAN_FRONTEND=noninteractive apt-get install -y libpcl-dev

# Eigen installation. Since the package include Eigen headers as <Eigen/...> instead of <eigen3/Eigen/Core>, let's create a symbolic.
sudo apt-get install -y libeigen3-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

# Install ROS requirements
apt-get install -y  python3-pip                             \
                    python3-lxml                            \
                    psmisc                                  \
                    ros-$ROS_DISTRO-rviz                    \
                    ros-$ROS_DISTRO-tf                      \
                    ros-$ROS_DISTRO-tf2-ros                 \
                    ros-$ROS_DISTRO-rosbash
