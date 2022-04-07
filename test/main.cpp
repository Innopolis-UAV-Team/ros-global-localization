
#include <chrono>

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "PCRegistration.h"

typedef pcl::PointXYZ Point_pcl;
typedef pcl::PointCloud<Point_pcl> PointCloud_pcl;

int main(int argc, char *argv[]) {
    if (argc != 3)
    {
        std::cout << argv[0] << " <source_cloud> <target_cloud> " << std::endl;
        return 0;
    }
    std::string src_cloud_path = argv[1];
    std::string tgt_cloud_path = argv[2];

    PointCloud_pcl::Ptr cloud_src_pcl (new PointCloud_pcl);
    PointCloud_pcl::Ptr cloud_tgt_pcl (new PointCloud_pcl);
    // Loading Cloud_Points of room
    if (pcl::io::loadPCDFile<Point_pcl>(src_cloud_path, *cloud_src_pcl) == -1) //* load the file
    {
        PCL_ERROR("Error loading cloud %s.\n");
        return (-1);
    }
    if (pcl::io::loadPCDFile<Point_pcl>(tgt_cloud_path, *cloud_tgt_pcl) == -1) //* load the file
    {
        PCL_ERROR("Error loading cloud %s.\n");
        return (-1);
    }
            
    RegistrationPC registration_pc;
    auto start = std::chrono::high_resolution_clock::now();
    Eigen::Matrix4f trans_matrix = registration_pc.Registration(cloud_tgt_pcl, cloud_src_pcl);
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std:: cout << "Registration time: " << elapsed.count()*1000 <<  " ms" << std::endl;
    bool use_visualization = true;
    if (use_visualization)
        registration_pc.Visualization(cloud_tgt_pcl, cloud_src_pcl, trans_matrix);
    return 0;
}


