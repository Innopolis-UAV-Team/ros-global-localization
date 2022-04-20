/**
 * @file PCGlobalLocalizationROS.cpp
 * @author ponomarevda96@gmail.com
 * @note ROS wrapper under HAPCL Registration library
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
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


enum ErrorCode_t: int8_t {
    STATUS_ERROR = -1,
    STATUS_OK = 0,
};


class PCGlobalLocalizationROS {
  public:
    PCGlobalLocalizationROS() {};

    /**
     * @brief Initialize the node in the mode defined by the ROS parameters.
     * @return STATUS_ERROR, if something wrong happend and whole application should be finished.
     *  return STATUS_OK, if everything is ok.
     */
    ErrorCode_t init(ros::NodeHandle& nh);

    /**
     * @brief Spin with the initialized parameters.
     * This call may take some time if you are using it in self-publishing mode (_mode_pub_by_request=false).
     */
    void spin_once();

  private:
    ErrorCode_t perform_transformation();
    void publish_tf();
    void convert_cloud_ros_to_pcl(const sensor_msgs::PointCloud2Ptr& ros_point_cloud,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud);
    void convert_transf_matrix_to_tf(const Eigen::Matrix4f& trans_matrix);

    RegistrationPC _registration_pc;

    ros::Subscriber _src_ros_cloud_sub;
    ros::Subscriber _tgt_ros_cloud_sub;
    ros::ServiceServer _relocalize_srv;
    tf2_ros::TransformBroadcaster _tf_pub;

    sensor_msgs::PointCloud2Ptr _src_ros_cloud;
    sensor_msgs::PointCloud2Ptr _tgt_ros_cloud;
    geometry_msgs::TransformStamped _transform;

    std::string _parent_frame_id;
    std::string _child_frame_id;
    double _next_tf_pub_time_sec{ros::Time::now().toSec()};
    double _tf_pub_rate;
    bool _mode_pub_by_request;

  public:
    ///< These methods are public because they are handled via ROS spin. Don't use them directly.
    void src_cloud_cb(const sensor_msgs::PointCloud2Ptr& src_ros_point_cloud);
    void tgt_cloud_cb(const sensor_msgs::PointCloud2Ptr& tgt_ros_point_cloud);
    bool relocalize_cb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
};


ErrorCode_t PCGlobalLocalizationROS::init(ros::NodeHandle& nh) {
    auto res = STATUS_OK;
    if (!nh.getParam("parent_frame_id", _parent_frame_id)) {
        ROS_ERROR("Parameter parent_frame_id is missing.");
        res = STATUS_ERROR;
    }
    if (!nh.getParam("child_frame_id", _child_frame_id)) {
        ROS_ERROR("Parameter child_frame_id is missing.");
        res = STATUS_ERROR;
    }
    if (!nh.getParam("tf_pub_rate", _tf_pub_rate) || (_tf_pub_rate < 0.01)) {
        ROS_ERROR("Parameter tf_pub_rate is missing or invalid.");
        res = STATUS_ERROR;
    }
    if (!nh.getParam("mode_pub_by_request", _mode_pub_by_request)) {
        ROS_ERROR("Parameter mode_pub_by_request is missing.");
        res = STATUS_ERROR;
    }

    _src_ros_cloud_sub = nh.subscribe("global_map", 1, &PCGlobalLocalizationROS::src_cloud_cb, this);
    _tgt_ros_cloud_sub = nh.subscribe("local_map", 1, &PCGlobalLocalizationROS::tgt_cloud_cb, this);
    _relocalize_srv = nh.advertiseService("relocalize", &PCGlobalLocalizationROS::relocalize_cb, this);

    return res;
}

void PCGlobalLocalizationROS::spin_once() {
    if (!_mode_pub_by_request) {
        if (STATUS_ERROR == perform_transformation()) {
            ros::Duration(1.0).sleep();
        }
    }
    publish_tf();
}

ErrorCode_t PCGlobalLocalizationROS::perform_transformation() {
    if (!_tgt_ros_cloud || !_src_ros_cloud) {
        ROS_WARN("HAPCL: Input ROS cloud is not appeared yet.");
        return STATUS_ERROR;
    }

    if (!_tgt_ros_cloud->height || !_tgt_ros_cloud->width ||
            !_src_ros_cloud->height || !_src_ros_cloud->width) {
        ROS_WARN("HAPCL: Input ROS cloud is empty.");
        return STATUS_ERROR;
    }

    pcl::PointCloud<pcl::PointXYZ> tgt_pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ> src_pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_pcl_cloud_ptr = tgt_pcl_cloud.makeShared();
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_pcl_cloud_ptr = src_pcl_cloud.makeShared();

    convert_cloud_ros_to_pcl(_tgt_ros_cloud, tgt_pcl_cloud_ptr);
    convert_cloud_ros_to_pcl(_src_ros_cloud, src_pcl_cloud_ptr);

    if (!tgt_pcl_cloud_ptr->size() || !src_pcl_cloud_ptr->size()) {
        ROS_WARN("Cannot create a KDTree with an empty input cloud.");
        return STATUS_ERROR;
    }

    Eigen::Matrix4f trans_matrix = _registration_pc.Registration(tgt_pcl_cloud_ptr, src_pcl_cloud_ptr);
    convert_transf_matrix_to_tf(trans_matrix);

    _src_ros_cloud = NULL;
    _tgt_ros_cloud = NULL;
    return STATUS_OK;
}

void PCGlobalLocalizationROS::publish_tf() {
    double crnt_time_sec = ros::Time::now().toSec();
    if (crnt_time_sec < _next_tf_pub_time_sec ||
            !_transform.header.frame_id.size() ||
            !_transform.child_frame_id.size()) {
        return;
    }

    ///< It is guaranted (by init() method) that _tf_pub_rate is not zero
    _next_tf_pub_time_sec = crnt_time_sec + 1 / _tf_pub_rate;

    if (!_transform.header.frame_id.size() || !_transform.child_frame_id.size()) {
        ROS_WARN_THROTTLE(5, "HAPCL: TF is not ready yet.");
        return;
    }

    _transform.header.stamp = ros::Time::now();
    _tf_pub.sendTransform(_transform);

}

void PCGlobalLocalizationROS::src_cloud_cb(const sensor_msgs::PointCloud2Ptr& src_ros_cloud_ptr) {
    _src_ros_cloud = src_ros_cloud_ptr;
}

void PCGlobalLocalizationROS::tgt_cloud_cb(const sensor_msgs::PointCloud2Ptr& tgt_ros_cloud_ptr) {
    _tgt_ros_cloud = tgt_ros_cloud_ptr;
}

bool PCGlobalLocalizationROS::relocalize_cb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    if (_mode_pub_by_request) {
        response.success = (perform_transformation() == STATUS_OK) ? true : false;
    } else {
        ROS_WARN("HAPCL: the node is initialized in a self-publishing mode. The requst has been ignored.");
        response.success = false;
    }
    return response.success;
}


void PCGlobalLocalizationROS::convert_transf_matrix_to_tf(const Eigen::Matrix4f& trans_matrix) {
    Eigen::Affine3d eigen_affine_transform;
    eigen_affine_transform.matrix() = trans_matrix.cast<double>();
    _transform = tf2::eigenToTransform(eigen_affine_transform);
    _transform.header.frame_id = _parent_frame_id;
    _transform.child_frame_id = _child_frame_id;
}

void PCGlobalLocalizationROS::convert_cloud_ros_to_pcl(const sensor_msgs::PointCloud2Ptr& ros_point_cloud,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*ros_point_cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *pcl_pointcloud);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "node");
    ros::NodeHandle nh;

    PCGlobalLocalizationROS hapcl;
    if (STATUS_ERROR == hapcl.init(nh)) {
        return STATUS_ERROR;
    }

    while (ros::ok()) {
        ros::spinOnce();
        hapcl.spin_once();
    }

    return STATUS_OK;
}
