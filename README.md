# Hybrid Point Cloud Registration approach for global localization

# Table of Contents

* [1. Introduction](#1-introduction)
* [2. Requirements](#2-requirements)
* [3. ROS API](#3-ros-api)
* [4. Running ROS package with docker](#4-running-ros-package-with-docker)
* [5. Running ROS package without docker](#5-running-ros-package-without-docker)
* [6. Running ROS package with prerecorded dataset](#5-trying-ros-package-with-prerecorded-dataset)
* [7. Getting Started without ROS](#7-getting-started-without-ros)

## 1. Introduction

Hybrid approach of Point Cloud Registration consists of main parts: downsampling cascade, WHI feature descriptor, fast multidimensional nearest neighbors graph-search, Max Clique Inlier Selection, transformation estimator of Fast Global Registration algorithm. It also include options to use FPFH feature descriptor, transformation estimator of Teaser++ and Plane-to-Plane ICP cascade for registration refinement. This approach allows for indoor global localization of devices that build a spatial map.

## 2. Requirements

- ubuntu 20.04
- ros noetic

## 3. ROS API

**Subscribed Topics**

- /global_map ([sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)): Topic of global map.
- /local_map ([sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)): Topic of current map.

**Published Topics**

- /tf2 [(geometry_msgs/TransformStamped](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/TransformStamped.html)): Tf2 publish frames: frame_id -> child_id

**Services**

- /relocalize ([std_srvs/Trigger](http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)): Service to manually run relocalization.

**Parameters**

- ~rate (float, default: 100) Frame rate of TF publiser
- ~frame_id (string, default: "map") Name of parent TF.
- ~child_id (string, default: "map_slam") Frame of child TF.

## 4. Running ROS package with docker

The easiest way to play with the package is to use docker. You can either build and run it manually, or use `docker.sh` script.

Typically, you need to run one of following commands with this script:

- `./docker.sh build` to build a docker image
- `./docker.sh run` to run it
- `./docker.sh help` to get additional info


## 5. Running ROS package without docker

Your installation process might be base don `Dockerfile`.

The core of the package is `src/node.cpp` node. To use it, you should load ros parameters.

The best way to use this node is to either use `launch/run.launch` file or to write your own.

## 6. Running ROS package with prerecorded dataset

Let's say you have 2 pcd files in your folder named `Map.pcd` and `GlobalMap.pcd`.

You could play with this package by running following commands in different terminals:

```bash
./docker.sh run
```

```bash
rosrun pcl_ros pcd_to_pointcloud Map.pcd 1.0        _frame_id:=map cloud_pcd:=/tgt_ros_cloud __name:=my_node2
```

```bash
rosrun pcl_ros pcd_to_pointcloud GlobalMap.pcd 1.0  _frame_id:=map_slam cloud_pcd:=/src_ros_cloud __name:=my_node1
```

## 7. Getting Started without ROS

### 7.1. Dependencies installation on Ubuntu/Debian:


1) Installation of Point Cloud Library (PCL):
```
$ sudo apt install libpcl-dev
```
2) Dependence for external WHI feature library:
```
$ sudo apt install libopencv-dev
```
3) Support to use multithreading:
```
$ sudo apt install libopenmpi-dev
```

### 7.2. Build on Ubuntu/Debian

From root folder:
```
$ mkdir build 
$ cd build
$ cmake ..
$ make
```
### 7.3. Running on Ubuntu/Debian
From build folder:

```
$  ./HAPCR <target_point_cloud> <source_point_cloud>
```
* \<target_point_cloud\> and \<source_point_cloud> are the point cloud files of pcd format.
