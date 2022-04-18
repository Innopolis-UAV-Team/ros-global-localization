#!/bin/bash

print_help() {
   echo "Wrapper under docker API for HPCR approach for global localization ROS package.
It encapsulates all necessary docker flags for working with the package and properly handles image versions.
https://tfs.university.innopolis.ru/tfs/IndustrialRoboticsLab/Colocalization/_git/ros-global-localization

usage: docker.sh [build | pull | push | run | test | interactive | kill | help]

Commands:
build           Build docker image.
pull            Pull docker image.
push            Push docker image.
run             Run inclinometer
rviz            Run inclinometer with rviz.
test            Run tests
interactive     Run container in interactive mode.
kill            Kill all containers.
help            Print this message and exit"
}

setup_config() {
    TAG_NAME=v0.0.1
    DOCKERHUB_REPOSITOTY=industrial_robotics_lab/hpcr
    if uname -m | grep -q 'aarch64'; then
        TAG_NAME="$TAG_NAME""arm64"
    elif uname -m | grep -q 'x86_64'; then
        TAG_NAME="$TAG_NAME""amd64"
    else
        echo "unknown architecture"
        exit
    fi
    DOCKER_CONTAINER_NAME=$DOCKERHUB_REPOSITOTY:$TAG_NAME

    DOCKER_FLAGS="--privileged -v $DEV_PATH_SYMLINK:$DEV_PATH_SYMLINK       \
                 --net=host                                                 \
                 -v "/tmp/.X11-unix:/tmp/.X11-unix:rw"                      \
                 -e DISPLAY=$DISPLAY                                        \
                 -e QT_X11_NO_MITSHM=1)"

    echo "TAG_NAME is" $TAG_NAME
    echo "DOCKERHUB_REPOSITOTY is" $DOCKERHUB_REPOSITOTY
}

build_docker_image() {
    setup_config
    docker build -t $DOCKER_CONTAINER_NAME .
}

pull_docker_image() {
    setup_config
    docker pull $DOCKER_CONTAINER_NAME
}

push_docker_image() {
    setup_config
    docker push $DOCKER_CONTAINER_NAME
}

run() {
    setup_config
    docker container run --rm $DOCKER_FLAGS $DOCKER_CONTAINER_NAME ./scripts/run.sh
}

run_rviz() {
    setup_config
    echo "Not ready yet"
}

test() {
    setup_config
    echo "Not ready yet"
}

run_interactive() {
    setup_config
    echo "Not ready yet"
}

kill_all_containers() {
    docker kill $(docker ps -q)
}


cd "$(dirname "$0")"

if [ "$1" = "build" ]; then
    build_docker_image
elif [ "$1" = "pull" ]; then
    pull_docker_image
elif [ "$1" = "push" ]; then
    push_docker_image
elif [ "$1" = "run" ]; then
    run
elif [ "$1" = "rviz" ]; then
    run_rviz
elif [ "$1" = "test" ]; then
    test
elif [ "$1" = "interactive" ]; then
    run_interactive
elif [ "$1" = "kill" ]; then
    kill_all_containers
else
    print_help
fi
