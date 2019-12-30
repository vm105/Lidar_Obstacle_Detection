// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//uncomment this to use PCL Libraries for clustering and segmentation
//  #define USE_PCL

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

    typename pcl::VoxelGrid<PointT> gridObj{};
    typename pcl::CropBox<PointT> boxObj{};
    typename pcl::CropBox<PointT> roofObj {};

    typename pcl::PointCloud<PointT>::Ptr downSampledCloud {new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr croppedCloud {new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr filteredCloud {new pcl::PointCloud<PointT>()};

    typename pcl::ExtractIndices<PointT> extract{};
    typename pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    boost::shared_ptr<std::vector<int> > roofIndices{new std::vector<int>};

    gridObj.setInputCloud(cloud);
    gridObj.setLeafSize(filterRes, filterRes, filterRes);
    gridObj.filter(*downSampledCloud);

    boxObj.setInputCloud(downSampledCloud);
    boxObj.setMin(minPoint);
    boxObj.setMax(maxPoint);
    boxObj.filter(*croppedCloud);

    roofObj.setInputCloud(croppedCloud);
    roofObj.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roofObj.setMax(Eigen::Vector4f(2.6, 1.7 ,-.4, 1));
    roofObj.filter(*roofIndices);

    extract.setInputCloud(croppedCloud);
    extract.setIndices(roofIndices);
    extract.setNegative(true);
    extract.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCloud;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::ExtractIndices<PointT> extract{};
    typename pcl::PointCloud<PointT>::Ptr cloudRoad (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudObst (new pcl::PointCloud<PointT>());

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(false);
    extract.filter(*cloudRoad);

    extract.setNegative(true);
    extract.filter(*cloudObst);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudObst, cloudRoad);
    return segResult;
}

template<typename PointT>
void my_pcl::setupKdtree(typename pcl::PointCloud<PointT>::Ptr cloud, my_pcl::KdTree* tree, int dimension)
{
    //insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i)
    {
        tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
    }
}

template<typename PointT>
void my_pcl::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree, float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max)
{
	if (cluster.size() < max)
    {
        cluster.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point {cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};
    
        // get all neighboring indices of point
        std::vector<int> neighborNdxs = tree->search(point, distanceTol);

        for (int neighborNdx : neighborNdxs)
        {
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end())
            {
                proximity<PointT>(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
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
	my_visited_set_t visited{};

	std::vector<pcl::PointIndices> clusters;
	for (int pointNdx = 0; pointNdx < cloud->size(); ++pointNdx)
	{
		// if a point wasn't processed
		if (visited.find(pointNdx) == visited.end())
		{
			std::vector<int> clusterIndices{};
            // find clusters
			my_pcl::proximity<PointT>(cloud, pointNdx, tree, distanceTol, visited, clusterIndices, max);

            if (clusterIndices.size() >= min)
            {
                pcl::PointIndices cluster;
                cluster.indices = clusterIndices;
			    clusters.push_back(cluster);
            }
		}
	}
	return clusters;
}

template<typename PointT>
std::vector<int> my_pcl::ransacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::vector<int> inliersResult;

	std:srand(std::time(nullptr));

	std::size_t pointSize = cloud->size();
	typedef std::unordered_set<std::tuple<std::size_t, std::size_t, std::size_t>, boost::hash<std::tuple<size_t, size_t, size_t> > > my_set_t;
	my_set_t pointHist;

	// For max iterations 
	for (int i = 0; i < maxIterations; ++i)
	{
		//get random points
		std::size_t randIndex1;
		std::size_t randIndex2;
		std::size_t randIndex3;
		std::tuple<size_t, size_t, size_t> indexTuple;
		my_set_t::iterator findItr;

		//make sure the random points are not repeated or the same
		do
		{		
			randIndex1 = std::rand() % pointSize;
			randIndex2 = std::rand() % pointSize;
			randIndex3 = std::rand() % pointSize;
			indexTuple = std::make_tuple(randIndex1, randIndex2, randIndex3);

			findItr = pointHist.find(indexTuple);
		}
		while(!pointHist.empty() && (findItr != pointHist.end() || (randIndex1 == randIndex2 
			|| randIndex2 == randIndex3 || randIndex1 == randIndex3)));

		pointHist.insert(indexTuple);

		//create plane
		my_pcl::Plane myPlane{cloud->at(randIndex1).x, cloud->at(randIndex2).x, cloud->at(randIndex3).x,
						cloud->at(randIndex1).y, cloud->at(randIndex2).y, cloud->at(randIndex3).y,
						cloud->at(randIndex1).z, cloud->at(randIndex2).z, cloud->at(randIndex3).z};
		//find inliers
		std::vector<int> inliers{};

		for (int j = 0; j < cloud->size(); ++j)
		{
			PointT newPoint = cloud->at(j);
			float dist = myPlane.getDistFrom(newPoint.x, newPoint.y, newPoint.z);
			if (dist < distanceTol)
			{
                inliers.push_back(j);
			}	
		}

		if (inliers.size() >= inliersResult.size())
		{
			inliersResult = inliers;
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

    pcl::ModelCoefficients::Ptr coefs (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);

    seg.setDistanceThreshold(distanceThreshold);
    seg.setMaxIterations(maxIterations);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefs);
#else
    inliers->indices = my_pcl::ransacPlane<PointT>(cloud, maxIterations, distanceThreshold);
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
    my_pcl::KdTree tree;
    tree.set_dimension(3);
    my_pcl::setupKdtree<PointT>(cloud, &tree, 3);
    clusterIndicesVector = my_pcl::euclideanCluster<PointT>(cloud, &tree, clusterTolerance, minSize, maxSize);
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