/**
 * @file node.cpp
 * @author ponomarevda96@gmail.com
 * @note ROS wrapper under HAPCL Registration library
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include "PCRegistration.h"


class HAPCLRegistrationROS {
  public:
    HAPCLRegistrationROS() {};
    int8_t init(ros::NodeHandle& nh);
    int8_t process();
    void src_cloud_cb(const sensor_msgs::PointCloud2Ptr& src_ros_point_cloud);
    void tgt_cloud_cb(const sensor_msgs::PointCloud2Ptr& tgt_ros_point_cloud);

  private:
    void convert_cloud_ros_to_pcl(const sensor_msgs::PointCloud2Ptr& ros_point_cloud,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud);
    void convert_transf_matrix_to_tf(const Eigen::Matrix4f& trans_matrix);

    RegistrationPC _registration_pc;
    ros::Subscriber _src_ros_cloud_sub;
    ros::Subscriber _tgt_ros_cloud_sub;
    tf2_ros::TransformBroadcaster _tf_pub;
    sensor_msgs::PointCloud2Ptr _src_ros_cloud;
    sensor_msgs::PointCloud2Ptr _tgt_ros_cloud;
    geometry_msgs::TransformStamped _transform;
    std::string _parent_frame_id;
    std::string _child_frame_id;
};


int8_t HAPCLRegistrationROS::init(ros::NodeHandle& nh) {
    _src_ros_cloud_sub = nh.subscribe("src_ros_cloud", 1, &HAPCLRegistrationROS::src_cloud_cb, this);
    _tgt_ros_cloud_sub = nh.subscribe("tgt_ros_cloud", 1, &HAPCLRegistrationROS::tgt_cloud_cb, this);

    int res = 0;
    if (!nh.getParam("parent_frame_id", _parent_frame_id)) {
        ROS_ERROR("Parameter parent_frame_id is missing.");
        res = -1;
    }
    if (!nh.getParam("child_frame_id", _child_frame_id)) {
        ROS_ERROR("Parameter child_frame_id is missing.");
        res = -1;
    }

    return res;
}

int8_t HAPCLRegistrationROS::process() {
    if (!_tgt_ros_cloud || !_src_ros_cloud) {
        ROS_INFO("Input ROS cloud is not appeared yet.");
        return -1;
    }

    if (!_tgt_ros_cloud->height || !_tgt_ros_cloud->width ||
            !_src_ros_cloud->height || !_src_ros_cloud->width) {
        ROS_WARN("Input ROS cloud is empty.");
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ> tgt_pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ> src_pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_pcl_cloud_ptr = tgt_pcl_cloud.makeShared();
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_pcl_cloud_ptr = src_pcl_cloud.makeShared();

    convert_cloud_ros_to_pcl(_tgt_ros_cloud, tgt_pcl_cloud_ptr);
    convert_cloud_ros_to_pcl(_src_ros_cloud, src_pcl_cloud_ptr);

    if (!tgt_pcl_cloud_ptr->size() || !src_pcl_cloud_ptr->size()) {
        ROS_WARN("Cannot create a KDTree with an empty input cloud.");
        return -1;
    }

    ROS_INFO("started conversion");
    Eigen::Matrix4f trans_matrix = _registration_pc.Registration(tgt_pcl_cloud_ptr, src_pcl_cloud_ptr);
    ROS_INFO("finished conversion");

    convert_transf_matrix_to_tf(trans_matrix);
    _tf_pub.sendTransform(_transform);

    _src_ros_cloud = NULL;
    _tgt_ros_cloud = NULL;
    return 0;
}

void HAPCLRegistrationROS::src_cloud_cb(const sensor_msgs::PointCloud2Ptr& src_ros_cloud_ptr) {
    _src_ros_cloud = src_ros_cloud_ptr;
}

void HAPCLRegistrationROS::tgt_cloud_cb(const sensor_msgs::PointCloud2Ptr& tgt_ros_cloud_ptr) {
    _tgt_ros_cloud = tgt_ros_cloud_ptr;
}

void HAPCLRegistrationROS::convert_transf_matrix_to_tf(const Eigen::Matrix4f& trans_matrix) {
    Eigen::Affine3d eigen_affine_transform;
    eigen_affine_transform.matrix() = trans_matrix.cast<double>();
    _transform = tf2::eigenToTransform(eigen_affine_transform);
    _transform.header.frame_id = _parent_frame_id;
    _transform.child_frame_id = _child_frame_id;
    _transform.header.stamp = ros::Time::now();
}

void HAPCLRegistrationROS::convert_cloud_ros_to_pcl(const sensor_msgs::PointCloud2Ptr& ros_point_cloud,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*ros_point_cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *pcl_pointcloud);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "node");
    ros::NodeHandle nh;

    HAPCLRegistrationROS hapcl;
    if (-1 == hapcl.init(nh)) {
        return 0;
    }

    while (ros::ok()) {
        ros::spinOnce();
        if (0 != hapcl.process()) {
            ros::Duration(1.0).sleep();
        }
    }

    return 0;
}
