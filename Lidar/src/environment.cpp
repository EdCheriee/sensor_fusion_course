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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, 
                ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, 
                const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // std::unique_ptr<ProcessPointClouds<pcl::PointXYZI>> pointProcessorI = std::make_unique<ProcessPointClouds<pcl::PointXYZI>>();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // Experiment with the ? values and find what works best
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3 , Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f ( 30, 8, 1, 1));
    auto segmentedCloud = pointProcessorI->RansacPlane(filterCloud, 100, 0.5);

    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentedCloud.first, 0.8, 20, 400);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->ClusteringCustom(segmentedCloud.first, 0.8, 20, 400);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[0]);
        ++clusterId;
    }


    renderPointCloud(viewer,segmentedCloud.second,"filterCloud");
    // renderPointCloud(viewer,segmentedCloud.second,"inputCloud");
}




void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer, bool useCustomRansac = false)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    const auto lidar = new Lidar(cars, 0.0);
    const auto pclCloud = lidar->scan();
    // renderRays(viewer, lidar->position, pclCloud);
    // renderPointCloud(viewer, pclCloud, "someName", Color(255, 0, 0));
    // TODO:: Create point processor
    std::unique_ptr<ProcessPointClouds<pcl::PointXYZ>> pointProcessor = std::make_unique<ProcessPointClouds<pcl::PointXYZ>>();

    if(!useCustomRansac)
    {
        auto segmentedCloud = pointProcessor->SegmentPlane(pclCloud, 100, 0.3);
        renderPointCloud(viewer, segmentedCloud.first, "roadCloud", Color(1, 0, 0));
        renderPointCloud(viewer, segmentedCloud.second, "obstacleCloud", Color(0, 1, 0));
    }
    else
    {
        auto segmentedCloud = pointProcessor->RansacPlane(pclCloud, 100, 0.5);

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentedCloud.first, 1.0, 3, 30);

        int clusterId = 0;
        std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,1,1)};

        for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
        {
            std::cout << "cluster size ";
            pointProcessor->numPoints(cluster);
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
            ++clusterId;
        }
        // renderPointCloud(viewer, segmentedCloud.second, "roadCloud", Color(1, 1, 1));
        // renderPointCloud(viewer, segmentedCloud.second, "obstacleCloud", Color(0, 1, 1));
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
    // pcl::visualization::PCLVisualizer::Ptr viewerCustom (new pcl::visualization::PCLVisualizer ("Custom Ransac 3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


    // initCamera(setAngle, viewerCustom);
    // simpleHighway(viewerCustom, true);
    // cityBlock(viewer, pointProcessorI, inputCloudI);

    while (!viewer->wasStopped())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        std::cout<< (*streamIterator).string() << std::endl;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();


        viewer->spinOnce();
        // viewerCustom->spin();
    } 
}