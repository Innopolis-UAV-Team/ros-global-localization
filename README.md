# Hybrid Point Cloud Registration approach for global localization
# Table of Contents

* [Introduction](##introduction)
* [Getting Started](##Getting Started)
## Introduction
Hybrid approach of Point Cloud Registration consists of main parts: downsampling cascade, WHI feature descriptor, fast multidimensional nearest neighbors graph-search, Max Clique Inlier Selection, transformation estimator of Fast Global Registration algorithm. It also include options to use FPFH feature descriptor, transformation estimator of Teaser++ and Plane-to-Plane ICP cascade for registration refinement. This approach allows for indoor global localization of devices that build a spatial map.

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