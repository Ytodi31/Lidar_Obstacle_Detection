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
    Lidar *ptr_lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = ptr_lidar->scan();

    // renderRays(viewer, ptr_lidar->position , cloud);
    //renderPointCloud(viewer, cloud, "cloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* ptr_pointcloud = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = ptr_pointcloud->SegmentPlane(cloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "ObstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "PlaneCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = ptr_pointcloud->Clustering(segmentCloud.first, 1, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
      std::cout << "cluster size: " ;
      ptr_pointcloud->numPoints(cluster);
      Box box = ptr_pointcloud->BoundingBox(cluster);
      renderBox(viewer,box,clusterId);
      renderPointCloud(viewer, cluster, "ObstCloud"+std::to_string(clusterId), colors[clusterId]);
      ++clusterId;
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
 void cityblock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* ptr_pointcloudI, pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
   Eigen::Vector4f min(-20, -6, -4, 1);
   Eigen::Vector4f max(20, 6, 4, 1);
   pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = ptr_pointcloudI ->FilterCloud(inputCloud, 0.2, min, max);
   // std::cout << "Filtered cloud size: " << filterCloud->size() << endl;
   std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = ptr_pointcloudI->SegmentPlane(filterCloud, 80, 0.1);
   //renderPointCloud(viewer, segmentCloud.first, "ObstacleCloud", Color(1,0,0));
   //renderPointCloud(viewer, segmentCloud.second, "PlaneCloud", Color(0,1,0));
   //renderPointCloud(viewer, filterCloud, "CityPCD");

   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = ptr_pointcloudI->Clustering(segmentCloud.first, 0.5 , 25, 1000);

   int clusterId = 0;
   std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
   renderPointCloud(viewer, segmentCloud.second, "PlaneCloud", Color(0,1,0));
   for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
     // std::cout << "cluster size: " ;
     ptr_pointcloudI->numPoints(cluster);
     Box box = ptr_pointcloudI->BoundingBox(cluster);
     renderBox(viewer,box,clusterId);
     renderPointCloud(viewer, cluster, "ObstCloud"+std::to_string(clusterId), colors[clusterId]);
     ++clusterId;
   }
 }

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* ptr_pointcloudI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = ptr_pointcloudI ->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIt = stream.begin();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = ptr_pointcloudI ->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //cityblock(viewer, ptr_pointcloudI, inputCloud);
    while (!viewer->wasStopped ())
    {
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

       pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = ptr_pointcloudI->loadPcd((*streamIt).string());
       cityblock(viewer, ptr_pointcloudI, inputCloud);
       streamIt +=1;
       if(streamIt==stream.end())
       streamIt = stream.begin();
        viewer->spinOnce ();
    }
}
