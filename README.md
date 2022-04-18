# Hybrid Point Cloud Registration approach for global localization

# Table of Contents

* [Introduction](#introduction)
* [Requirements](#requirements)
* [ROS API](#ros-api)
* [Getting Started](#getting-started)

## Introduction

Hybrid approach of Point Cloud Registration consists of main parts: downsampling cascade, WHI feature descriptor, fast multidimensional nearest neighbors graph-search, Max Clique Inlier Selection, transformation estimator of Fast Global Registration algorithm. It also include options to use FPFH feature descriptor, transformation estimator of Teaser++ and Plane-to-Plane ICP cascade for registration refinement. This approach allows for indoor global localization of devices that build a spatial map.

## Requirements

- ubuntu 20.04
- ros noetic

## ROS API

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

## Getting Started

### Dependencies installation on Ubuntu/Debian:


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

### Build on Ubuntu/Debian

From root folder:
```
$ mkdir build 
$ cd build
$ cmake ..
$ make
```
### Running on Ubuntu/Debian
From build folder:

```
$  ./HAPCR <target_point_cloud> <source_point_cloud>
```
* \<target_point_cloud\> and \<source_point_cloud> are the point cloud files of pcd format.