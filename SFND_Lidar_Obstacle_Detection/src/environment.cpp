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

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0.0);
    auto pointCloud = lidar->scan();
    Car egoCar = cars.at(0);
    Vect3 lidarPosition = lidar->position;
    //renderRays(viewer, lidarPosition, pointCloud);
    //renderPointCloud(viewer, pointCloud, "lidarPointCloud", Color(1, 1, 1));

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> processPointClouds;

    // Segmenting
    auto separatedPointClouds = processPointClouds.SegmentPlane(pointCloud, 100, 0.2);
    //renderPointCloud(viewer, separatedPointClouds.first, "obstaclePointCloud", Color(1, 0, 0));
    renderPointCloud(viewer, separatedPointClouds.second, "planePointCloud", Color(1, 1, 1));
    
    // Clustering
    auto cloudClusters = processPointClouds.Clustering(separatedPointClouds.first, 2.0, 3, 30);

    const std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };

    for(int clusterId = 0; clusterId < cloudClusters.size(); ++clusterId) {
        renderPointCloud(viewer, cloudClusters.at(clusterId), "cloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);

        //Box box = processPointClouds.BoundingBox(cloudClusters.at(clusterId));
        //renderBox(viewer, box, clusterId, colors[clusterId%colors.size()], 0.5);

        BoxQ boxq = processPointClouds.optimalBox(cloudClusters.at(clusterId));
        renderBox(viewer, boxq, 10*clusterId+1, colors[clusterId%colors.size()], 0.5);
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // -------------------------------------------------
    // ----- Open 3D viewer and dsiplay City Block -----
    // -------------------------------------------------
    
    /* Filter cloud */
    Eigen::Vector4f minPoint(-15, -6, -3, 1);
    Eigen::Vector4f maxPoint(30, 6, 0.5, 1);
    auto filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.25, minPoint, maxPoint);
    //renderPointCloud(viewer, filterCloud, "inputCloud");

    Box regionOfInterest; regionOfInterest.x_min = minPoint[0]; regionOfInterest.x_max = maxPoint[0]; regionOfInterest.y_min = minPoint[1]; regionOfInterest.y_max = maxPoint[1]; regionOfInterest.z_min = minPoint[2]; regionOfInterest.z_max = maxPoint[2];
    //renderBox(viewer, regionOfInterest, 100, Color(1,1,1), 0.5);

    Eigen::Vector4f roofMinPoint(-1.5, -1.7, -1, 1);
    Eigen::Vector4f roofMaxPoint(2.6, 1.7, -0.4, 1);
    Box roof; roof.x_min = roofMinPoint[0]; roof.x_max = roofMaxPoint[0]; roof.y_min = roofMinPoint[1]; roof.y_max = roofMaxPoint[1]; roof.z_min = roofMinPoint[2]; roof.z_max = roofMaxPoint[2];
    renderBox(viewer, roof, 101, Color(1,1,0), 0.5);

    /* Segmentation */
    auto separatedPointClouds = pointProcessorI->SegmentPlane(filterCloud, 30, 0.3);
    //auto separatedPointClouds = pointProcessorI->SegmentPlane(inputCloud, 30, 0.2);
    //renderPointCloud(viewer, separatedPointClouds.first, "nonPlaneCloud", Color(1,0,1));
    renderPointCloud(viewer, separatedPointClouds.second, "planePointCloud", Color(1, 1, 1));

    /* Clustering */
    auto cloudClusters = pointProcessorI->Clustering(separatedPointClouds.first, 0.35, 10, 500);

    const std::vector<Color> colors = { Color(1,0,0) };

    for(int clusterId = 0; clusterId < cloudClusters.size(); ++clusterId) {
        renderPointCloud(viewer, cloudClusters.at(clusterId), "cloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);

        Box box = pointProcessorI->BoundingBox(cloudClusters.at(clusterId));
        renderBox(viewer, box, clusterId, colors[clusterId%colors.size()], 0.5);

        //BoxQ boxq = pointProcessorI->optimalBox(cloudClusters.at(clusterId));
        //renderBox(viewer, boxq, 10*clusterId+1, colors[clusterId%colors.size()], 0.5);
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
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1/");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    //simpleHighway(viewer);

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}