#include "WHI/WHI.h"
#include <pcl/features/normal_3d.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

/* *************************for class ThreePointsFeatureEstimation************************** */
void WHIFeatureEstimation::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& in)
{
	cloud_in = in;
}
void WHIFeatureEstimation::setIndices(const pcl::PointIndicesConstPtr& indicesn)
{
	if (indicesn->indices.size() > 0)
	{
		indices_in = *indicesn;
		indices_flag = true;
	}
	else
	{
		indices_flag = false;
	}

}
void WHIFeatureEstimation::setDimBasic(int k)
{
	dimBasic = k;
}
void WHIFeatureEstimation::setSearchRadius(double radius)
{
	searchRadius = radius;
}
void WHIFeatureEstimation::setmr(double mr)
{
	meshresolution = mr;
}
void WHIFeatureEstimation::init()
{
	tree.setInputCloud(cloud_in);
	if (!indices_flag)
	{
		for (int i = 0; i < cloud_in->size(); i++)
		{
			indices_in.indices.push_back(i);
		}

	}
}

void WHIFeatureEstimation::compute(pcl::PointCloud<WHIFeature>::Ptr& feature)
{
	init();
	double lambda = 0.3;
	double sigmax = (2 * searchRadius) / sqrt(12);
	double sigmay = sigmax;
	double reso = searchRadius / dimBasic;
	feature->clear();
	cloud_normals->clear();

	for (int i = 0; i < indices_in.indices.size(); i++)
	{
		pcl::PointXYZ QueryPoint = cloud_in->at(indices_in.indices[i]);
		vector<int> Kind;
		vector<float> Kdist;
		tree.radiusSearch(QueryPoint, searchRadius, Kind, Kdist);
		cloud_kdtree_indices.push_back(Kind);
		if (Kind.size() > 0) // default by author was > 5
		{
			//LRF estimation
			const Eigen::Vector3f& central_point = (*cloud_in)[indices_in.indices[i]].getVector3fMap();
	
			int valid_nn_points = Kind.size() - 1;
			int valid_conv_point = floor(valid_nn_points*0.3);
			Eigen::Matrix<float, Eigen::Dynamic, 3> vij(valid_nn_points, 3);
			Eigen::Matrix3f cov_m = Eigen::Matrix3f::Zero();

			vector<double> distances(Kind.size());

			for (size_t i_idx = 0; i_idx < valid_nn_points; ++i_idx)
			{
				Eigen::Vector3f pt = cloud_in->points[Kind[i_idx + 1]].getVector3fMap();
				vij.row(i_idx).matrix() = (pt - central_point);
				distances[i_idx] = searchRadius - sqrt(Kdist[i_idx + 1]);
				if (i_idx < valid_conv_point)
					cov_m += distances[i_idx] * (vij.row(i_idx).transpose() * vij.row(i_idx));
			}

			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov_m);

			// Disambiguation
			Eigen::Vector3f v1 = Eigen::Vector3f::Zero();
			Eigen::Vector3f v3 = Eigen::Vector3f::Zero();
			Eigen::Vector3f v2 = Eigen::Vector3f::Zero();

			v1.matrix() = solver.eigenvectors().col(2);
			v3.matrix() = solver.eigenvectors().col(0);
			v2.matrix() = v3.cross(v1);

			vector<float> ftemp(valid_nn_points, 0.0);
			vector<vector<float>> feature_space(3, ftemp);

			double sum0 = 0.0;
			double sum1 = 0.0;
			for (int di = 0; di < valid_nn_points; di++)
			{
				feature_space[0][di] = (vij.row(di).dot(v1));
				feature_space[1][di] = (vij.row(di).dot(v2));
				feature_space[2][di] = (vij.row(di).dot(v3));
				if (di < valid_conv_point)
				{
					sum0 += feature_space[0][di];
					sum1 += feature_space[2][di];
				}
			}

			int sign0, sign1, sign2;
			if (sum0 >= 0)
				sign0 = 1;
			else
				sign0 = -1;
			if (sum1 >= 0)
				sign2 = 1;
			else
				sign2 = -1;
			sign1 = sign0 * sign2;

			pcl::Normal normal_point{sign2*v3[0], sign2*v3[1], sign2*v3[2]};
            cloud_normals->push_back(normal_point);

			vector<int> tempn(2 * dimBasic, 0);
			vector<vector<int>> imagen(2 * dimBasic, tempn);
			vector<double> tempimg(2 * dimBasic, 0.0);
			vector<vector<double>> image(2 * dimBasic, tempimg);

			for (int di = 0; di < valid_nn_points; di++)
			{

				double lambda_ = lambda + (1 - lambda)*(distances[di] / searchRadius);
				int index_x = floor((sign0*feature_space[0][di] + searchRadius) / reso);
				int index_y = floor((sign1*feature_space[1][di] + searchRadius) / reso);

				if (index_x >= 2 * dimBasic) 
					index_x = index_x - 1;
				else if (index_x <= 0) 
					index_x = index_x + 1;
				if (index_y >= 2 * dimBasic) 
					index_y = index_y - 1;
				else if (index_y <= 0) 
					index_y = index_y + 1;

				image[index_x][index_y] = image[index_x][index_y] + lambda_ * sign2*feature_space[2][di];
				imagen[index_x][index_y] ++;
			}

			WHIFeature fea;

			cv::Mat His = cv::Mat::zeros(2 * dimBasic, 2 * dimBasic, CV_64F);

			for (int ix = 0; ix < 2 * dimBasic; ix++)
			{
				for (int iy = 0; iy < 2 * dimBasic; iy++)
				{
					if (imagen[ix][iy] > 0)
						His.at<double>(ix, iy) = image[ix][iy] / imagen[ix][iy];
				}
			}

			cv::GaussianBlur(His, His, Size(5, 5), sigmax, sigmay);
			fea.feature.resize(4 * dimBasic*dimBasic);
			fea.feature = (vector<double>)His.reshape(1, 1);
			feature->push_back(fea);
		}
	}
}

pcl::PointCloud<pcl::Normal>::Ptr WHIFeatureEstimation::getCloudNormals()
{
	pcl::PointCloud<pcl::Normal>::Ptr cloud_noramls_ptr = cloud_normals;
	cloud_normals.reset();
	return cloud_noramls_ptr;
} 

WHIFeatureEstimation::WHIFeatureEstimation() : cloud_normals(new pcl::PointCloud<pcl::Normal>)
{

}
WHIFeatureEstimation::~WHIFeatureEstimation()
{

}