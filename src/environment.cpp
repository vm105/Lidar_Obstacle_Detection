/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    inputCloud = pointProcessorI.FilterCloud(inputCloud, 0.32f, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 7, 1, 1));

    //uncomment line below to render input PCD
    //renderPointCloud(viewer, inputCloud, "inputCloud");
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI.SegmentPlane(inputCloud, 100, 0.4);
    renderPointCloud(viewer, segmentedCloud.first, "obst" , Color{1,0,0});
    renderPointCloud(viewer, segmentedCloud.second, "road" , Color{0,1,0});

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentedCloud.first, .62, 6, 500);
    static int clusterId {0};
    std::srand(time(nullptr));

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr clusterPtr : cloudClusters)
    {
        float rand_r = static_cast<float>(std::rand() % 10) / 10;
        float rand_g = static_cast<float>(std::rand() % 5) / 10;
        float rand_b = static_cast<float>(std::rand() % 10) / 10;

        Color color{rand_r, rand_g, rand_b};
        renderPointCloud(viewer, clusterPtr, "cluster_" + std::to_string(clusterId++), color);
        Box box = pointProcessorI.BoundingBox(clusterPtr);
        renderBox(viewer, box, clusterId);
    }

}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    Lidar* lidar_ptr =  new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr  scanData = lidar_ptr->scan();

    ProcessPointClouds<pcl::PointXYZ> pointProcessor{};
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedCloud = pointProcessor.SegmentPlane(scanData, 100, 0.2);
    renderPointCloud(viewer, segmentedCloud.first, "obst" , Color{1,1,1});
    renderPointCloud(viewer, segmentedCloud.second, "road" , Color{0,1,0});

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentedCloud.first, 1, 3, 30);
    std::vector<Color> clusterColors = {{1.0, 0, 0}, {1.0, 1.0, 0}, {0, 0, 1.0}};
    int clusterId {0};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr : cloudClusters)
    {
        std::cout << "Cluster size: ";
        pointProcessor.numPoints(clusterPtr);
        renderPointCloud(viewer, clusterPtr, "cluster_" + std::to_string(clusterId), clusterColors[clusterId++]);
        Box box = pointProcessor.BoundingBox(clusterPtr);
        renderBox(viewer, box, clusterId);
    }
  
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI> pointProcessorI{};
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    std::vector<boost::filesystem::path>::iterator streamIterator = stream.begin();

    while (!viewer->wasStopped())
    {
        //clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //load pcd and run obstacle detection
        inputCloud = pointProcessorI.loadPcd(streamIterator++->string());
        cityBlock(viewer, pointProcessorI, inputCloud);

        if (streamIterator ==  stream.end())
        {
            streamIterator = stream.begin();
        }
         
        viewer->spinOnce();
    }
}