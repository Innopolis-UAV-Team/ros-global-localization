#include <pcl/filters/voxel_grid.h> // library for downsampling
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/normal_3d.h> // library for normal estimation
#include <pcl/features/fpfh.h> // method FPFH for feature extraction
#include <pcl/common/transforms.h> // for tansformation of point cloud
#include <pcl/visualization/pcl_visualizer.h> // for visualization clouds points
#include <pcl/registration/icp.h> // for use icp - local refinement method

#include "PCRegistration.h"
#include "app.h" // Methods of FGR class
#include <WHI/WHI.h>
#include <teaser/registration.h>
#include "hnswlib.h"
#include <thread>
#include "app_exception.h"
#include <chrono>

enum error_code {error_type_feature_descriptor = 1, error_type_registration_approach = 2};

enum RegistrationApproaches {FGR = 0, TEASER};
enum FeatureDescriptors {FPFH = 2, WHI};

// Point Cloud Registration parameters
const RegistrationApproaches registration_approach = FGR; // Options: FGR, TEASER
const FeatureDescriptors feature_descriptor = WHI; // Options: FPFH, WHI
const bool use_local_refinement = false; // local registration refinement ICP Point-to-Plane method

// Parameters of FPFH feature extraction
const float down_radius_fpfh(0.2); // radius of downsampling cloud points
const float normal_radius_fpfh(0.9); // search radius for compute normals
const float feature_radius_fpfh(0.6); // search radius for compute FPFH features
const int bins_histogram_fpfh(33); // number of bins for histogram of one FPFH feature

// Parameters of WHI feature extraction
const float down_radius_whi(0.2); // radius of downsampling cloud points
const float feature_radius_whi(1.5); // search radius for compute WHI features
const int dim_basic_whi(3); // parameter of WHI feature dimension

// Parameters of room reconstarction process
const float min_distance(0.003); // threshold for points which will added to target cloud

// Parameters of HNSW graph search algorithm
// More information about parameters you can read in ALGO_PARAMS.md from source of HNSW library
const int M_hnsw(16);
const int ef_construction_hnsw(120);
const int random_seed_hnsw(100);

// Helper function to convert Eigen(Point Cloud) to Teaser(Point Cloud)
void Convert_EigenPC_to_TeaserPC(teaser::PointCloud & cloud_teaser, const std::vector<Eigen::Vector3f> & cloud_eigen);

// Hybrid Teaser-FGR approach to registration of two clouds points
Eigen::Matrix4f RegistrationPC::Registration(const PointCloud_pcl::ConstPtr cloud_tgt_pcl, 
                                                const PointCloud_pcl::ConstPtr cloud_src_pcl)
{
    std::vector<Eigen::Vector3f> points_src; // source points in vector-Eigen representation
    std::vector<Eigen::Vector3f> points_tgt; // target points in vector-Eigen representation
    std::vector<Eigen::VectorXf> features_src; // source features in vector-Eigen representation
    std::vector<Eigen::VectorXf> features_tgt; // target features in vector-Eigen representation
    PointCloudNormal_pcl::Ptr cloud_with_normals_tgt_pcl (new PointCloudNormal_pcl);
    PointCloudNormal_pcl::Ptr cloud_with_normals_src_pcl (new PointCloudNormal_pcl);
    switch (feature_descriptor)
    {
        case FPFH:
            FeatureExtractFPFH(cloud_tgt_pcl, cloud_with_normals_tgt_pcl, points_tgt, features_tgt, 
                                down_radius_fpfh, normal_radius_fpfh, feature_radius_fpfh);
            FeatureExtractFPFH(cloud_src_pcl, cloud_with_normals_src_pcl, points_src, features_src,
                                down_radius_fpfh, normal_radius_fpfh, feature_radius_fpfh);
            break;
        case WHI:
            FeatureExtractWHI(cloud_tgt_pcl, cloud_with_normals_tgt_pcl, points_tgt, features_tgt, 
                                down_radius_whi, feature_radius_whi);
            FeatureExtractWHI(cloud_src_pcl, cloud_with_normals_src_pcl, points_src, features_src, 
                                down_radius_whi, feature_radius_whi);
            break;
        default:
            throw AppException("Wrong option of feature descriptor.", error_type_feature_descriptor);
            break;
    }
    std::vector<std::pair<int, int> > correspondences;
    correspondences = CalculateCorrespondencesHNSW(features_src, features_tgt);
    teaser::PointCloud cloud_src_teaser;
    teaser::PointCloud cloud_tgt_teaser;
    Convert_EigenPC_to_TeaserPC(cloud_src_teaser, points_src);
    Convert_EigenPC_to_TeaserPC(cloud_tgt_teaser, points_tgt);
    // Run TEASER++ registration
    // Prepare solver parameters
    teaser::RobustRegistrationSolver::Params params;
    params.use_max_clique = true;
    params.noise_bound = 0.2;
    params.cbar2 = 1.2;
    params.estimate_scaling = false;
    params.inlier_selection_mode =
            teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_HEU;
    params.rotation_tim_graph =
            teaser::RobustRegistrationSolver::INLIER_GRAPH_FORMULATION::COMPLETE;
    params.rotation_max_iterations = 100;
    params.rotation_gnc_factor = 1.4;
    params.rotation_estimation_algorithm =
            teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_cost_threshold = 0.0005;

    // Solve with TEASER++
    teaser::RobustRegistrationSolver solver(params);
    solver.solve(cloud_src_teaser, cloud_tgt_teaser, correspondences);
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
    switch (registration_approach)
    {
        case FGR:
        {
            // Get inlier correspondences from Teaser algorithm
            std::vector<std::pair<int, int>> inlier_correspondences;
            std::vector<int> inlier_indices = solver.getInlierMaxClique();
            for (int i = 0; i < inlier_indices.size(); ++i)
                inlier_correspondences.push_back(correspondences[inlier_indices[i]]);
            // Fast Global Registration Pairwise Optimization
            fgr::CApp app; // FGR object
            app.LoadCorres(points_src, points_tgt, inlier_correspondences);
            app.NormalizePoints();
            app.OptimizePairwise(true); // find transforamtion matrix
            transformation_matrix = app.GetOutputTrans();
            break;
        }
        case TEASER:
        {
            auto solution = solver.getSolution();
            transformation_matrix.block<3,3>(0,0) = solution.rotation.cast<float>();
            transformation_matrix.block<3,1>(0,3) = solution.translation.cast<float>();
            break;
        }
        default:
            throw AppException("Wrong option of registration approach.", error_type_registration_approach);
            break;
    }
    // Cascade of ICP Point-to-Plane refinement
    if (use_local_refinement){
        transformation_matrix = LocalRefinementPointToPlaneICP(cloud_with_normals_src_pcl, cloud_with_normals_tgt_pcl, transformation_matrix, 0.15);
        transformation_matrix = LocalRefinementPointToPlaneICP(cloud_with_normals_src_pcl, cloud_with_normals_tgt_pcl, transformation_matrix, 0.05);
        transformation_matrix = LocalRefinementPointToPlaneICP(cloud_with_normals_src_pcl, cloud_with_normals_tgt_pcl, transformation_matrix, 0.025);
    }
    return transformation_matrix;
}

// Function to extand map location
void RegistrationPC::ReconstractionRoom(PointCloud_pcl::Ptr cloud_tgt_pcl, 
                                            const PointCloud_pcl::ConstPtr cloud_src_pcl, 
                                            const Eigen::Matrix4f & transformation_matrix)
{
    PointCloud_pcl::Ptr cloud_transformed_pcl (new PointCloud_pcl);
    pcl::transformPointCloud(*cloud_src_pcl, *cloud_transformed_pcl, transformation_matrix);

    // Set searching of near point in tgt cloud to point in src cloud
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud_tgt_pcl);
    const int k_search(1); // number nearest points for search method Kdtree
    std::vector<int> pointIdxNKNSearch(k_search);
    std::vector<float> pointNKNSquaredDistance(k_search);
    Point_pcl search_point;

    for (int v = 0; v < cloud_transformed_pcl->size(); ++v)
    {
        search_point = cloud_transformed_pcl->points[v];
        if ( kdtree.nearestKSearch (search_point, k_search, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            if (pointNKNSquaredDistance[0] > min_distance)
            {
                cloud_tgt_pcl->push_back(search_point);
            }
        }
    }
}

// Function for visualization of registration of clouds points
void RegistrationPC::Visualization(const PointCloud_pcl::ConstPtr cloud_tgt_pcl, 
                                        const PointCloud_pcl::ConstPtr cloud_src_pcl, 
                                        const Eigen::Matrix4f & transformation_matrix)
{
    PointCloud_pcl::Ptr cloud_result_pcl (new PointCloud_pcl);
    // Executing the transformation
    pcl::transformPointCloud(*cloud_src_pcl, *cloud_result_pcl, transformation_matrix);
    // To create window for demonstration of Points Cloud
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    // Create two vertically separated viewports
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;
    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<Point_pcl> cloud_in_color_h (cloud_tgt_pcl, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                               (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud_tgt_pcl, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (cloud_tgt_pcl, cloud_in_color_h, "cloud_in_v2", v2);
    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<Point_pcl> cloud_tr_color_h (cloud_src_pcl, 20, 180, 20);
    viewer.addPointCloud (cloud_src_pcl, cloud_tr_color_h, "cloud_tr_v1", v1);
    // FGR aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<Point_pcl> cloud_fgr_color_h (cloud_result_pcl, 180, 20, 20);
    viewer.addPointCloud (cloud_result_pcl, cloud_fgr_color_h, "cloud_icp_v2", v2);
    // Adding text descriptions in each viewport
    viewer.addText ("White: First point cloud\nGreen: Second point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText ("White: First point cloud\nRed: FGR aligned second point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
    float zoom;
    zoom = 15.0;
    // Set camera position and orientation
    viewer.setCameraPosition (-zoom-3.68332, zoom+2.94092, zoom+5.71266, zoom+0.289847, zoom+0.921947, -zoom-0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size
    // Display the visualiser
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce();
    }
}

// Method to extract features FPFH
void RegistrationPC::FeatureExtractFPFH(const PointCloud_pcl::ConstPtr cloud_pcl,
                            PointCloudNormal_pcl::Ptr cloud_with_normals_pcl, 
                            std::vector<Eigen::Vector3f> & points, 
                            std::vector<Eigen::VectorXf> & features,
                            const float & down_radius,
                            const float & normal_radius,
                            const float & feature_radius)
{
    // Downsampling
    PointCloud_pcl::Ptr cloud_filtered_pcl (new PointCloud_pcl);
    pcl::VoxelGrid<Point_pcl> sor;
    pcl::ApproximateVoxelGrid<Point_pcl> apr;
    apr.setInputCloud (cloud_pcl);
    apr.setLeafSize (0.15, 0.15, 0.15);
    apr.filter (*cloud_filtered_pcl);
    sor.setInputCloud (cloud_filtered_pcl);
    sor.setLeafSize (0.16, 0.16, 0.16);
    sor.filter (*cloud_filtered_pcl);
    sor.setInputCloud (cloud_filtered_pcl);
    sor.setLeafSize (0.17, 0.17, 0.17);
    sor.filter (*cloud_filtered_pcl);
    sor.setInputCloud (cloud_filtered_pcl);
    sor.setLeafSize (down_radius, down_radius, down_radius);
    sor.filter (*cloud_filtered_pcl);

     // Compute the normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_pcl (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud_filtered_pcl);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (normal_radius);
    ne.compute (*cloud_normals_pcl);

    pcl::concatenateFields(*cloud_filtered_pcl, *cloud_normals_pcl, *cloud_with_normals_pcl);

    // Feature extraction
    pcl::FPFHEstimation<Point_pcl , pcl::Normal, pcl::FPFHSignature33> fest;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    fest.setRadiusSearch(feature_radius);
    fest.setInputCloud(cloud_filtered_pcl);
    fest.setInputNormals(cloud_normals_pcl);
    fest.setSearchMethod(tree);
    fest.compute(*object_features);

    for (int v = 0; v < cloud_filtered_pcl->size(); ++v)
    {
        Eigen::Vector3f temp_point;
        temp_point(0) = cloud_filtered_pcl->points[v].x;
        temp_point(1) = cloud_filtered_pcl->points[v].y;
        temp_point(2) = cloud_filtered_pcl->points[v].z;
        points.push_back(temp_point);

        Eigen::VectorXf temp_feature(bins_histogram_fpfh);
        const pcl::FPFHSignature33 & feature_fpfh = object_features->points[v];
        for (int i = 0; i < bins_histogram_fpfh; ++i)
        {
            temp_feature(i) = feature_fpfh.histogram[i];
        }
        features.push_back(temp_feature);
    }

}


// Method to extract features WHI
void RegistrationPC::FeatureExtractWHI(const PointCloud_pcl::ConstPtr cloud_pcl,
                        PointCloudNormal_pcl::Ptr cloud_with_normals_pcl,
                        std::vector<Eigen::Vector3f> & points,
                        std::vector<Eigen::VectorXf> & features,
                        const float & down_radius,
                        const float & feature_radius)
{
    // Downsampling
    PointCloud_pcl::Ptr cloud_filtered_pcl (new PointCloud_pcl);
    pcl::VoxelGrid<Point_pcl> sor;
    pcl::ApproximateVoxelGrid<Point_pcl> apr;
    apr.setInputCloud (cloud_pcl);
    apr.setLeafSize (0.15, 0.15, 0.15);
    apr.filter (*cloud_filtered_pcl);
    sor.setInputCloud (cloud_filtered_pcl);
    sor.setLeafSize (0.16, 0.16, 0.16);
    sor.filter (*cloud_filtered_pcl);
    sor.setInputCloud (cloud_filtered_pcl);
    sor.setLeafSize (0.17, 0.17, 0.17);
    sor.filter (*cloud_filtered_pcl);
    sor.setInputCloud (cloud_filtered_pcl);
    sor.setLeafSize (down_radius, down_radius, down_radius);
    sor.filter (*cloud_filtered_pcl);

    //WHI feature;
    pcl::PointCloud<WHIFeature>::Ptr features_whi(new pcl::PointCloud<WHIFeature>);

    // Feature extraction
    WHIFeatureEstimation est_Feature;
    est_Feature.setInputCloud(cloud_filtered_pcl);
    est_Feature.setDimBasic(dim_basic_whi);
    est_Feature.setSearchRadius(feature_radius);
    est_Feature.compute(features_whi);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_pcl = est_Feature.getCloudNormals();
    
    pcl::concatenateFields(*cloud_filtered_pcl, *cloud_normals_pcl, *cloud_with_normals_pcl);

    for (int v = 0; v < cloud_filtered_pcl->size(); ++v)
    {
        Eigen::Vector3f temp_point;
        temp_point(0) = cloud_filtered_pcl->points[v].x;
        temp_point(1) = cloud_filtered_pcl->points[v].y;
        temp_point(2) = cloud_filtered_pcl->points[v].z;
        points.push_back(temp_point);

        int feat_whi_size = features_whi->points[v].feature.size();
        Eigen::VectorXf temp_feature(feat_whi_size);
        for (int i = 0; i < feat_whi_size; ++i)
        {
            temp_feature(i) = features_whi->points[v].feature[i];
        }
        features.push_back(temp_feature);
    }

}

// Local registration refinement ICP Point-to-Plane method
Eigen::Matrix4f RegistrationPC::LocalRefinementPointToPlaneICP (PointCloudNormal_pcl::ConstPtr source_cloud, 
                                                                    PointCloudNormal_pcl::ConstPtr target_cloud, 
                                                                    Eigen::Matrix4f transformation_guess,
                                                                    float max_correspondence_distance)
{
    pcl::IterativeClosestPointWithNormals<PointNormal_pcl, PointNormal_pcl> icp;
    PointCloudNormal_pcl::Ptr transformed_cloud(new PointCloudNormal_pcl);
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    icp.setUseSymmetricObjective(true);
    icp.setMaxCorrespondenceDistance (max_correspondence_distance);
    icp.setMaximumIterations(10);
    icp.setTransformationEpsilon (1e-8);
    icp.align(*transformed_cloud, transformation_guess);
    Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();
    return transformation_matrix;
}

// inline function code to parallel for cycle
template<class Function>
inline void ParallelFor(size_t start, size_t end, size_t numThreads, Function fn) {
    if (numThreads <= 0) {
        numThreads = std::thread::hardware_concurrency();
    }

    if (numThreads == 1) {
        for (size_t id = start; id < end; id++) {
            fn(id, 0);
        }
    } else {
        std::vector<std::thread> threads;
        std::atomic<size_t> current(start);

        // keep track of exceptions in threads
        // https://stackoverflow.com/a/32428427/1713196
        std::exception_ptr lastException = nullptr;
        std::mutex lastExceptMutex;

        for (size_t threadId = 0; threadId < numThreads; ++threadId) {
            threads.push_back(std::thread([&, threadId] {
                while (true) {
                    size_t id = current.fetch_add(1);

                    if ((id >= end)) {
                        break;
                    }

                    try {
                        fn(id, threadId);
                    } catch (...) {
                        std::unique_lock<std::mutex> lastExcepLock(lastExceptMutex);
                        lastException = std::current_exception();
                        /*
                         * This will work even when current is the largest value that
                         * size_t can fit, because fetch_add returns the previous value
                         * before the increment (what will result in overflow
                         * and produce 0 instead of current + 1).
                         */
                        current = end;
                        break;
                    }
                }
            }));
        }
        for (auto &thread : threads) {
            thread.join();
        }
        if (lastException) {
            std::rethrow_exception(lastException);
        }
    }
}

// Method to calculate correspondences between source and target clouds
std::vector<std::pair<int, int> > RegistrationPC::CalculateCorrespondencesHNSW(
                                                const std::vector<Eigen::VectorXf> & features_src,
   				                                const std::vector<Eigen::VectorXf> & features_tgt)
{
    const std::vector<Eigen::VectorXf> * p_features[2]; 
    const size_t k = 1; // number nearest neighbours
    const int first_idx = 0; // start index of features container

    // Set first the biggest cloud to build HNSW graph from it
    bool swapped = false;
    if (features_src.size() > features_tgt.size()) {
        p_features[0] = &features_src;
        p_features[1] = &features_tgt;     
        swapped = true;
    }else{
        p_features[0] = &features_tgt;
        p_features[1] = &features_src;      
    }
    // Build HNSW graph 
    hnswlib::L2Space space((*p_features[0])[first_idx].size());
    hnswlib::AlgorithmInterface<float>* alg_hnsw = new hnswlib::HierarchicalNSW<float>(
                                                                    &space,2 * p_features[0]->size(),
                                                                    M_hnsw, 
                                                                    ef_construction_hnsw,
                                                                    random_seed_hnsw);
    int num_threads = std::thread::hardware_concurrency();
    ParallelFor(first_idx, p_features[0]->size(), num_threads, [&](size_t i, size_t threadId) {
            alg_hnsw->addPoint((void *)((*p_features[0])[i].data()), i);
            });
    // Search nearest points in HNSW graph
    std::vector<std::pair<int, int> > correspondences;
    for (int j = 0; j < p_features[1]->size(); ++j) {
        auto nearest_point = alg_hnsw->searchKnnCloserFirst((*p_features[1])[j].data(), k);
        correspondences.push_back(std::pair<int, int>(j, nearest_point[0].second));
        }
    if (swapped) {
        std::vector<std::pair<int, int>> temp;
        for (size_t i = 0; i < correspondences.size(); i++)
            temp.push_back(std::pair<int, int>(correspondences[i].second, 
                                                correspondences[i].first));
    correspondences = temp;
    }
    delete alg_hnsw;
    return correspondences;
}

void Convert_EigenPC_to_TeaserPC(teaser::PointCloud & cloud_teaser, const std::vector<Eigen::Vector3f> & cloud_eigen)
{
    for (int i = 0; i < cloud_eigen.size(); i++)
    {
        cloud_teaser.push_back({cloud_eigen[i][0],
                                cloud_eigen[i][1], 
                                cloud_eigen[i][2]});
    }
}