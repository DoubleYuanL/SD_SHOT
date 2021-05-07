#pragma once
#ifndef headers
#define headers

#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/copy_point.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/rops_estimation.h>
#include <pcl/features/rsd.h>
#include <pcl/features/usc.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/3dsc.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/impl/bilateral.hpp>
#include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/clipper3D.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/impl/covariance_sampling.hpp>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/impl/frustum_culling.hpp>

#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/impl/sampling_surface_normal.hpp>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/smoothed_surfaces_keypoint.h>
#include <pcl/keypoints/impl/smoothed_surfaces_keypoint.hpp>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/trajkovic_3d.h>

#include <pcl/octree/octree.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/PointIndices.h>

#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/histogram_visualizer.h>

#include <boost/version.hpp>
#include <boost/thread/thread.hpp>

#include <algorithm>
#include <ctime>
#include <math.h>
#include <numeric>
#include <omp.h>
#include <string>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <streambuf>
#include <iostream>
#include <bitset>
#include <time.h>
#include <valarray>
#include <vector>




#include <pcl/point_types_conversion.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>   //����
#include <pcl/surface/gp3.h>
#include <pcl/features/rops_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>//�ؼ�����
#include <pcl/visualization/pcl_plotter.h>//ֱ��ͼ���ӻ�
#include <time.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/correspondence.h>   //��Ӧ��ʾ����ʵ��֮���ƥ�䣨���磬�㣬�������ȣ���
#include <pcl/kdtree/kdtree_flann.h>             //��׼����
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/recognition/cg/geometric_consistency.h>  //����һ����
#include <pcl/features/board.h>
#include <pcl/recognition/cg/hough_3d.h>    //hough����
#include <pcl/filters/uniform_sampling.h>
#include <pcl/common/transforms.h>             //ת������


//#include <dirent.h> // for looping over the files in the directory
//#include <sys/time.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

#define PI 3.14159265

#endif 
