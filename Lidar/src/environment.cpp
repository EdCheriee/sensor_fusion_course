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
        auto segmentedCloud = pointProcessor->RansacPlane(pclCloud, 50, 0.3);

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentedCloud.first, 1.0, 3, 30);

        int clusterId = 0;
        std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,1,1)};

        for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
        {
            std::cout << "cluster size ";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
            ++clusterId;
        }
        // renderPointCloud(viewer, segmentedCloud.first, "roadCloud", Color(1, 0, 1));
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
    pcl::visualization::PCLVisualizer::Ptr viewerCustom (new pcl::visualization::PCLVisualizer ("Custom Ransac 3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer);

    initCamera(setAngle, viewerCustom);
    simpleHighway(viewerCustom, true);

    while (!viewer->wasStopped())
    {
        viewer->spin();
        viewerCustom->spin();
    } 
}