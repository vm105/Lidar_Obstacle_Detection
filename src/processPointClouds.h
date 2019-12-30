// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <cstdlib>
#include <ctime>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include <cmath>
#include "quiz/cluster/kdtree.h"
template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};

class Plane
{
	public:
		Plane(float x1, float x2, float x3,
			float y1, float y2, float y3, float z1, float z2, float z3)
		{
			A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
			B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
			C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
			D = -((A * x1) + (B * y1) + (C * z1));
		}

		float get_dist_from(float x, float y, float z)
		{
			return std::fabs((A*x + B*y + C*z + D))/ sqrtf(A*A + B*B + C*C);
		}

	private:
		float A;
		float B;
		float C;
		float D;
};
typedef std::unordered_set<int> my_visited_set_t;

namespace my_pcl
{

	template<typename PointT>
	std::vector<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr, int, float);

	template<typename PointT>
	void proximity(typename pcl::PointCloud<PointT>::Ptr, int, my_pcl::KdTree*,float , my_visited_set_t&, std::vector<int>&, int);

	template<typename PointT>
	std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr, my_pcl::KdTree*, float, int, int);

	template<typename PointT>
	void setup_kdtree(typename pcl::PointCloud<PointT>::Ptr, my_pcl::KdTree*, int);
}

#endif /* PROCESSPOINTCLOUDS_H_ */