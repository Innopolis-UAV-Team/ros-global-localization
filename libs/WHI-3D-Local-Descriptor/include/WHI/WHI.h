#pragma once
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <vector>
#include "opencv2/core/core.hpp"

using namespace std;

struct WHIFeature
{
	std::vector<double> feature;
};


class WHIFeatureEstimation
{
public:
	void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in);
	void setIndices(const pcl::PointIndicesConstPtr& indices);
	void setDimBasic(int k);
	void setSearchRadius(double radius);
	void setmr(double mr);
	void compute(pcl::PointCloud<WHIFeature>::Ptr& feature);
	pcl::PointCloud<pcl::Normal>::Ptr getCloudNormals(); 
	WHIFeatureEstimation();
	~WHIFeatureEstimation();
private:
	void init();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
	pcl::PointIndices indices_in;
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	std::vector<std::vector<int>> cloud_kdtree_indices;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
	int dimBasic;
	double searchRadius;
	double meshresolution;
	bool indices_flag = false;
};
