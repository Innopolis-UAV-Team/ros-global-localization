#ifndef __PCREGISTRATION_H__
#define __PCREGISTRATION_H__

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZ Point_pcl;
typedef pcl::PointCloud<Point_pcl> PointCloud_pcl;
typedef pcl::PointXYZRGBNormal PointNormal_pcl;
typedef pcl::PointCloud<PointNormal_pcl> PointCloudNormal_pcl;

// Class for solving registration problem and for reconstraction rooms
class RegistrationPC 
{ 
public:
   // Method to point cloud registration based on HNSW-nearest point search in graph 
   // and MCIS-Max Clique Inlier Selection algorithms
   Eigen::Matrix4f Registration(const PointCloud_pcl::ConstPtr cloud_tgt_pcl, 
                                const PointCloud_pcl::ConstPtr cloud_src_pcl);
   // Method to extand map location
   void ReconstractionRoom(PointCloud_pcl::Ptr cloud_tgt_pcl, 
                           const PointCloud_pcl::ConstPtr cloud_src_pcl, 
                           const Eigen::Matrix4f & transformation_matrix);
   // Method for visualization registration PC
   void Visualization(const PointCloud_pcl::ConstPtr cloud_tgt_pcl,
                      const PointCloud_pcl::ConstPtr cloud_src_pcl,
                      const Eigen::Matrix4f & transformation_matrix);
private:
   // Method to extract features FPFH
   void FeatureExtractFPFH(const PointCloud_pcl::ConstPtr cloud_pcl,
   			     PointCloudNormal_pcl::Ptr cloud_with_normals_pcl, 
                            std::vector<Eigen::Vector3f> & points, 
                            std::vector<Eigen::VectorXf> & features,
                            const float & down_radius,
                            const float & normal_radius,
                            const float & feature_radius);
   // Method to extract features WHI
   void FeatureExtractWHI(const PointCloud_pcl::ConstPtr cloud_pcl,
   			 PointCloudNormal_pcl::Ptr cloud_with_normals_pcl,
                        std::vector<Eigen::Vector3f> & points,
                        std::vector<Eigen::VectorXf> & features,
                        const float & down_radius,
                        const float & feature_radius);
                        // Method to extract features FPFH
   std::vector<std::pair<int, int> > CalculateCorrespondencesHNSW(
   				const std::vector<Eigen::VectorXf> & features_src,
   				const std::vector<Eigen::VectorXf> & features_tgt);
   // Method to refine the global point cloud registration
   Eigen::Matrix4f LocalRefinementPointToPlaneICP (PointCloudNormal_pcl::ConstPtr source_cloud, 
                                                    PointCloudNormal_pcl::ConstPtr target_cloud, 
                                                    Eigen::Matrix4f transformation_guess,
                                                    float max_correspondence_distance);
};

#endif
