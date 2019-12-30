// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

// #define USE_PCL

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::VoxelGrid<PointT> grid_obj{};
    typename pcl::CropBox<PointT> box_obj{};
    typename pcl::CropBox<PointT> roof_obj {};

    typename pcl::PointCloud<PointT>::Ptr down_sampled_cloud {new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr cropped_cloud {new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud {new pcl::PointCloud<PointT>()};

    typename pcl::ExtractIndices<PointT> extract{};
    typename pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    boost::shared_ptr<std::vector<int> > roof_indices{new std::vector<int>};



    grid_obj.setInputCloud(cloud);
    grid_obj.setLeafSize(filterRes, filterRes, filterRes);
    grid_obj.filter(*down_sampled_cloud);

    box_obj.setInputCloud(down_sampled_cloud);
    box_obj.setMin(minPoint);
    box_obj.setMax(maxPoint);
    box_obj.filter(*cropped_cloud);

    roof_obj.setInputCloud(cropped_cloud);
    roof_obj.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof_obj.setMax(Eigen::Vector4f(2.6, 1.7 ,-.4, 1));
    roof_obj.filter(*roof_indices);

    extract.setInputCloud(cropped_cloud);
    extract.setIndices(roof_indices);
    extract.setNegative(true);
    extract.filter(*filtered_cloud);
    


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filtered_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::ExtractIndices<PointT> extract{};
    typename pcl::PointCloud<PointT>::Ptr cloud_road (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_obst (new pcl::PointCloud<PointT>());

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(false);
    extract.filter(*cloud_road);

    extract.setNegative(true);
    extract.filter(*cloud_obst);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obst, cloud_road);
    return segResult;
}

template<typename PointT>
void my_pcl::setup_kdtree(typename pcl::PointCloud<PointT>::Ptr cloud, my_pcl::KdTree* tree, int dimension)
{
    for (int i = 0; i < cloud->size(); ++i)
    {
        std::vector<float> point;
        point.push_back(cloud->at(i).x);
        point.push_back(cloud->at(i).y);
        point.push_back(cloud->at(i).z);
        tree->insert(point, i);
    }
}

template<typename PointT>
void my_pcl::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree, float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max)
{
	if (cluster.size() < max)
    {
        cluster.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point;
        point.push_back(cloud->at(target_ndx).x);
        point.push_back(cloud->at(target_ndx).y);
        point.push_back(cloud->at(target_ndx).z);
        std::vector<int> neighbor_ndxs = tree->search(point, distanceTol);

        for (int neighbor_ndx : neighbor_ndxs)
        {
            // if point was not visited
            if (visited.find(neighbor_ndx) == visited.end())
            {
                proximity<PointT>(cloud, neighbor_ndx, tree, distanceTol, visited, cluster, max);
            }

            if (cluster.size() >= max)
            {
                return;
            }
        }
    }
}

template<typename PointT>
std::vector<pcl::PointIndices> my_pcl::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, my_pcl::KdTree* tree, float distanceTol, int min, int max)
{
	// TODO: Fill out this function to return list of indices for each cluster
	my_visited_set_t visited{};

	std::vector<pcl::PointIndices> clusters;
	for (int point_ndx = 0; point_ndx < cloud->size(); ++point_ndx)
	{
		// if a point wasn't processed
		if (visited.find(point_ndx) == visited.end())
		{
			std::vector<int> cluster_indices{};

			my_pcl::proximity<PointT>(cloud, point_ndx, tree, distanceTol, visited, cluster_indices, max);

            if (cluster_indices.size() >= min)
            {
                pcl::PointIndices cluster;
                cluster.indices = cluster_indices;
			    clusters.push_back(cluster);
            }

		}
	}
	return clusters;
}

template<typename PointT>
std::vector<int> my_pcl::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::vector<int> inliersResult;

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
		std::vector<int> inliers{};

		for (int j = 0; j < cloud->size(); ++j)
		{
			PointT new_point = cloud->at(j);
			float dist = my_plane.get_dist_from(new_point.x, new_point.y, new_point.z);
			if (dist < distanceTol)
			{
                inliers.push_back(j);
			}	
		}

		if (inliers.size() >= inliersResult.size())
		{
			inliersResult = inliers;
			std::cout << inliers.size();
		}

	}
	return inliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

#ifdef USE_PCL
    pcl::SACSegmentation<PointT> seg;

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefs (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);

    seg.setDistanceThreshold(distanceThreshold);
    seg.setMaxIterations(maxIterations);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefs);
#else
    inliers->indices = my_pcl::RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);
#endif

    if (inliers->indices.empty())
    {
        std::cout << "could not estimate a plane model for data" << std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointIndices> clusterIndicesVector;

#ifdef USE_PCL
    typename pcl::search::KdTree<PointT>::Ptr tree {new pcl::search::KdTree<PointT>};
    pcl::EuclideanClusterExtraction<PointT> ec;
    
    tree->setInputCloud(cloud);
    
    ec.setSearchMethod(tree);
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setInputCloud(cloud);

    ec.extract(clusterIndicesVector);
#else
    my_pcl::KdTree* tree = new my_pcl::KdTree;
    tree->set_dimension(3);
    my_pcl::setup_kdtree<PointT>(cloud, tree, 3);
    clusterIndicesVector = my_pcl::euclideanCluster<PointT>(cloud, tree, clusterTolerance, minSize, maxSize);
#endif
    //extract into clutsers
    for (auto clusterIndex : clusterIndicesVector)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster{new pcl::PointCloud<PointT>};
        for (auto index: clusterIndex.indices)
        {
            cluster->push_back(cloud->at(index));
        }
        cluster->width = clusterIndex.indices.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    
    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}