/**
 * @file node.cpp
 * @author ponomarevda96@gmail.com
 * @note ROS wrapper under PRCegistration library
 */

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include "PCRegistration.h"


typedef pcl::PointXYZ Point_pcl;
typedef pcl::PointCloud<Point_pcl> PointCloud_pcl;


int main(int argc, char **argv) {
    ros::init(argc, argv, "node");
    ros::NodeHandle nh;

    PointCloud_pcl::Ptr cloud_src_pcl(new PointCloud_pcl);
    PointCloud_pcl::Ptr cloud_tgt_pcl(new PointCloud_pcl);

    return 0;
}
