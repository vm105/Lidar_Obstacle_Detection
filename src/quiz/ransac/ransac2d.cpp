/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cstdlib>
#include <ctime>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include <cmath>

class Line
{
	public:
		Line(float x1, float x2, float y1, float y2)
			:A(y1 - y2), B(x2 - x1), C(x1*y2 - x2*y1)
			{

			}

		float get_dist_from(float x, float y)
		{	
			return std::fabs(A*x + B*y + C)/ sqrtf(A*A + B*B);
		}

	private:
		float A;
		float B;
		float C;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;

	std:srand(std::time(nullptr));
	std::size_t point_size = cloud->size();
	typedef std::unordered_set<std::tuple<std::size_t, std::size_t, std::size_t>, boost::hash<std::tuple<size_t, size_t, size_t> > > my_set_t;
	my_set_t point_hist;

	// For max iterations 
	for (int i = 0; i < maxIterations; ++i)
	{
		//get random points
		std::size_t rand_index_1;
		std::size_t rand_index_2;
		std::size_t rand_index_3;
		std::tuple<size_t, size_t, size_t> index_tuple;
		my_set_t::iterator find_itr;

		//make sure the random points are not repeated or the same
		do
		{		
			rand_index_1 = std::rand() % point_size;
			rand_index_2 = std::rand() % point_size;
			rand_index_3 = std::rand() % point_size;
			index_tuple = std::make_tuple(rand_index_1, rand_index_2, rand_index_3);

			find_itr = point_hist.find(index_tuple);
		}
		while(!point_hist.empty() && (find_itr != point_hist.end() || (rand_index_1 == rand_index_2 
			|| rand_index_2 == rand_index_3 || rand_index_1 == rand_index_3)));

		point_hist.insert(index_tuple);

		//create plane
		Plane my_plane{cloud->at(rand_index_1).x, cloud->at(rand_index_2).x, cloud->at(rand_index_3).x,
						cloud->at(rand_index_1).y, cloud->at(rand_index_2).y, cloud->at(rand_index_3).y,
						cloud->at(rand_index_1).z, cloud->at(rand_index_2).z, cloud->at(rand_index_3).z};
		//find inliers
		std::unordered_set<int> inliers{};

		for (int j = 0; j < cloud->size(); ++j)
		{
			pcl::PointXYZ new_point = cloud->at(j);
			float dist = my_plane.get_dist_from(new_point.x, new_point.y, new_point.z);
			std::cout << dist;
			if (dist < distanceTol)
			{
				inliers.insert(j);
			}	
		}

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
			std::cout << inliers.size();
		}

	}
	return inliersResult;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	// srand(time(NULL));
	
	// TODO: Fill in this function
	std:srand(std::time(nullptr));
	std::size_t point_size = cloud->size();
	typedef std::unordered_set<std::pair<std::size_t, std::size_t>, boost::hash<std::pair<size_t, size_t> > > my_set_t;
	my_set_t point_hist;

	// For max iterations 
	for (int i = 0; i < maxIterations; ++i)
	{
		//get random point
		std::size_t rand_index_1;
		std::size_t rand_index_2;
		std::pair<size_t, size_t> index_pair;
		my_set_t::iterator find_itr;
		//infinite loop
		do
		{		
			rand_index_1 = std::rand() % point_size;
			rand_index_2 = std::rand() % point_size;
			index_pair = {rand_index_1, rand_index_2};

			find_itr = point_hist.find(index_pair);
		}
		while(!point_hist.empty() && (find_itr != point_hist.end() || (rand_index_1 == rand_index_2)));

		point_hist.insert(index_pair);

		//create line
		Line my_line{cloud->at(rand_index_1).x, cloud->at(rand_index_2).x, cloud->at(rand_index_1).y, cloud->at(rand_index_2).y};
		//find inliers
		std::unordered_set<int> inliers{};

		for (int j = 0; j < cloud->size(); ++j)
		{
			pcl::PointXYZ new_point = cloud->at(j);
			float dist = my_line.get_dist_from(new_point.x, new_point.y);
			std::cout << dist;
			if (dist < distanceTol)
			{
				inliers.insert(j);
			}	
		}

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
			std::cout << inliers.size();
		}

	}
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.2);



	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
