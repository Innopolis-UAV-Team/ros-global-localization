ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO
LABEL description="HPC Registration approach for global localization ROS package"
LABEL maintainer="ponomarevda96@gmail.com"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/hybrid_pc_registration
RUN apt-get update                          &&  \
    apt-get upgrade -y                      &&  \
    apt-get install -y  git                     \
                        ros-$ROS_DISTRO-catkin  \
                        python3-catkin-tools

RUN if [[ "$ROS_DISTRO" = "melodic" ]] ; then apt-get install -y python-pip python-catkin-tools ; fi

# Install hybrid_pc_registration package
COPY install_requirements.sh    install_requirements.sh
RUN ./install_requirements.sh

# COPY repository
COPY include/ include/
COPY libs/ libs/
COPY src/ src/
COPY test/ test/
COPY CMakeLists.txt CMakeLists.txt
COPY package.xml package.xml

# Build
RUN apt-get install -y ros-$ROS_DISTRO-pcl-conversions
RUN apt-get install -y ros-$ROS_DISTRO-pcl-ros
RUN source /opt/ros/$ROS_DISTRO/setup.bash  && cd ../../ && catkin build

COPY scripts/ scripts/
COPY launch/ launch/

CMD echo "main process has been started"            &&  \
    echo "container has been finished"
