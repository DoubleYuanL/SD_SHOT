#include "headers.h"

typedef pcl::PointXYZ PointType;
typedef pcl::PointNormal PointNormalType;
typedef pcl::Normal NormalType;

template< typename T >
void minVect(const T* v, int len, int& ind)
{
	assert(len > 0);

	T min = v[0];
	for (int i = 1; i < len; i++)
		if (v[i] < min)
		{
			min = v[i];
			ind = i;
		}
}

class sdshot_descriptor
{
public:
	std::bitset< 352 > bits;
};



class sddescriptors
{
public:
	int m; // for SHOT descriptor, the dimention of descriptor is 352
	int L; //L means the lenth of binary unit;
	int n;  // n means the  num of  standard deviation
	pcl::Correspondences corresp, corr;
	int matchsize;
	Eigen::Matrix4f mat;

public:
	pcl::PointCloud<PointType>				cloud1, cloud2;
	pcl::PointCloud<NormalType>				cloud1_normals, cloud2_normals;
	pcl::PointCloud<PointNormalType>		cloud1_point_normals, cloud2_point_normals;
	pcl::PointCloud<PointType>				cloud1_keypoints, cloud2_keypoints;


	pcl::PointCloud<pcl::SHOT352>           cloud1_shot, cloud2_shot;
	std::vector<sdshot_descriptor>          cloud1_sdshot, cloud2_sdshot;


	template <typename T>
	void compute_normals(float radius, T cloud1, T cloud2)
	{
		// Estimate the normals.
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setRadiusSearch(radius);
		normalEstimation.setNumberOfThreads(12);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		normalEstimation.setSearchMethod(kdtree);

		normalEstimation.setInputCloud(cloud1.makeShared());
		normalEstimation.compute(cloud1_normals);

		normalEstimation.setInputCloud(cloud2.makeShared());
		normalEstimation.compute(cloud2_normals);
	}

	void compute_filters_voxel_grid(float leaf_size)
	{
		pcl::VoxelGrid<PointType> voxel_grid;
		voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);

		voxel_grid.setInputCloud(cloud1.makeShared());
		voxel_grid.filter(cloud1_keypoints);

		voxel_grid.setInputCloud(cloud2.makeShared());
		voxel_grid.filter(cloud2_keypoints);
	}

	void calculate_SHOT(float radius)
	{
		// SHOT estimation object.
		pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
		shot.setRadiusSearch(radius);
		shot.setNumberOfThreads(12);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		shot.setSearchMethod(kdtree);

		shot.setInputCloud(cloud1_keypoints.makeShared());
		shot.setSearchSurface(cloud1.makeShared());
		shot.setInputNormals(cloud1_normals.makeShared());
		shot.compute(cloud1_shot);

		shot.setInputCloud(cloud2_keypoints.makeShared());
		shot.setSearchSurface(cloud2.makeShared());
		shot.setInputNormals(cloud2_normals.makeShared());
		shot.compute(cloud2_shot);
	}
	void compute_sdshot()
	{
		compute_SDSHOT_from_SHOT(cloud1_shot, cloud1_sdshot);
		compute_SDSHOT_from_SHOT(cloud2_shot, cloud2_sdshot);
	}
	void compute_SDSHOT_from_SHOT(pcl::PointCloud<pcl::SHOT352>& descriptors_here, std::vector<sdshot_descriptor>& sdshot_descriptors)
	{
		sdshot_descriptors.resize(descriptors_here.size());
		int m = 352; // for SHOT descriptor, the dimention of descriptor is 352
		int L = 352; //L means the lenth of binary unit;
		int n = 1;  // n means the  num of  standard deviation
		float all_sum_mean = 0, all_mean_mean = 0, all_mean_v = 0, all_sd_mean = 0, all_cv_mean = 0;
		for (int i = 0; i < (int)descriptors_here.size(); i++)
		{
			std::bitset< 352 > bit;
			bit.reset();
			for (int j = 0; j <= m / L - 1; ++j)
			{
				float sum = 0, mean = 0, variance = 0, sd = 0;
				for (int k = j * L; k <= (j + 1) * L - 1; k++)
				{
					sum += descriptors_here[i].descriptor[k];
				}
				mean = sum / m;
				for (int k = j * L; k <= (j + 1) * L - 1; k++)
					variance += (descriptors_here[i].descriptor[k] - mean) * (descriptors_here[i].descriptor[k] - mean);
				variance = variance / L;
				sd = sqrt(variance);
				for (int k = j * L; k <= (j + 1) * L - 1; k++)
					if (descriptors_here[i].descriptor[k] > mean + n * sd)
						bit.set(k);
				sdshot_descriptors[i].bits = bit;
			}
		}
	}
	
};