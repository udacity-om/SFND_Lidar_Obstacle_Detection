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

#undef SHOW_HIGHWAY_SCENE
#undef SHOW_LIDAR_RAYS
#undef SHOW_LIDAR_POINTS
#undef SHOW_SEGMENTED_LIDAR_CLOUDS
#define SHOW_ONLY_OBSTACLE_CLUSTERS
#define SHOW_BOUNDING_BOXES

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
#ifdef SHOW_HIGHWAY_SCENE
    bool renderScene = true;
#else
	bool renderScene = false;
#endif
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
	Lidar* lidar = new Lidar(cars, 0.0f);
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
#ifdef SHOW_LIDAR_RAYS
	renderRays(viewer, lidar->position, inputCloud);
#endif
#ifdef SHOW_LIDAR_POINTS
	renderPointCloud(viewer, inputCloud, "inputCloud");
#endif

    // TODO:: Create point processor
	ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
	std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =
		pointProcessor->SegmentPlane(inputCloud, 25, 0.2);
#ifdef SHOW_SEGMENTED_LIDAR_CLOUDS
	renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
	renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
#endif /* SHOW_SEGMENTED_LIDAR_CLOUDS */

#ifdef SHOW_ONLY_OBSTACLE_CLUSTERS
	renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obstClusters = pointProcessor->Clustering(segmentCloud.first, 1, 5, 80);

	unsigned int obstClusterId = 0;
	std::vector<Color> obstColors = { Color(1,0,1), Color(1,1,0), Color(1,1,1) };

	for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : obstClusters) {
		if (obstClusterId < obstColors.size()) {
			std::cout << "Cluster size : ";
			pointProcessor->numPoints(cluster);
			renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(obstClusterId), obstColors[obstClusterId]);

#ifdef SHOW_BOUNDING_BOXES
			Box box = pointProcessor->BoundingBox(cluster);
			renderBox(viewer, box, obstClusterId);
#endif /* SHOW_BOUNDING_BOXES */

			++obstClusterId;
		}
	}
#endif /* SHOW_ONLY_OBSTACLE_CLUSTERS */
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

#ifdef SHOW_FILE_PATH
#include <iostream>
#include <filesystem>
namespace fs = std::experimental::filesystem;
#endif

#define SHOW_REAL_CITY_BLOCK
#undef SHOW_REAL_RAW_POINT_CLOUD
#undef SHOW_REAL_FILTERED_POINT_CLOUD
#undef SHOW_REAL_SEGMENTED_POINT_CLOUD
#define SHOW_REAL_ONLY_OBSTACLE_CLUSTERS
#define SHOW_REAL_BOUNDING_BOXES

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
	//----------------------------------------------
	//----Open 3D viewer and display City Block-----
	//----------------------------------------------

	/* Show real PCD data */
#ifdef SHOW_REAL_RAW_POINT_CLOUD
	renderPointCloud(viewer, inputCloud, "inputCloud");
#endif

	/* Filter the PCD data */
	pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZI>());
	filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2f, Eigen::Vector4f(-10,-5,-3,1.0), Eigen::Vector4f(30, 7, 1, 1.0));
#ifdef SHOW_REAL_FILTERED_POINT_CLOUD
	renderPointCloud(viewer, filterCloud, "filterCloud");
#endif

	/* Segment the filtered cloud into two parts, road and obstacles */
	ProcessPointClouds<pcl::PointXYZI>* pointProcessor(new ProcessPointClouds<pcl::PointXYZI>);
	std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
		pointProcessor->SegmentPlane(filterCloud, 25, 0.2);
#ifdef SHOW_REAL_SEGMENTED_POINT_CLOUD
	renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
	renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
#endif

	/* Cluster the obstacle cloud */
#ifdef SHOW_REAL_ONLY_OBSTACLE_CLUSTERS
	renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstClusters = pointProcessor->Clustering(segmentCloud.first, 0.3, 10, 1000);

	unsigned int obstClusterId = 0;
	std::vector<Color> obstColors = {Color(1,0,0), Color(0,0,1), Color(0,1,1), Color(1,0,1), Color(1,1,0), Color(1,1,1)};

	for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : obstClusters) {
		std::cout << "Cluster size : ";
		pointProcessor->numPoints(cluster);
		//renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(obstClusterId), obstColors[obstClusterId % obstColors.size()]);

#ifdef SHOW_REAL_BOUNDING_BOXES
		Box box = pointProcessor->BoundingBox(cluster);
		renderBox(viewer, box, obstClusterId);
#endif /* SHOW_BOUNDING_BOXES */

		++obstClusterId;
	}
#endif /* SHOW_ONLY_OBSTACLE_CLUSTERS */
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
#ifndef SHOW_REAL_CITY_BLOCK
    simpleHighway(viewer);
#endif

#ifdef SHOW_FILE_PATH
	std::cout << fs::current_path() << std::endl;
#endif

#ifdef SHOW_REAL_CITY_BLOCK
	ProcessPointClouds<pcl::PointXYZI>* pointProcessorI(new ProcessPointClouds<pcl::PointXYZI>);
	std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
	auto streamIterator = stream.begin();
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
#endif

    while (!viewer->wasStopped ())
    {
#ifdef SHOW_REAL_CITY_BLOCK
		std::cout << "SHOW_REAL_CITY_BLOCK" << std::endl;
		// Clear viewer
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		// Load pcd and run obstacle detection process
		inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
		cityBlock(viewer, pointProcessorI, inputCloudI);

		streamIterator++;
		if (streamIterator == stream.end()) {
			streamIterator = stream.begin();
		}

		viewer->spinOnce();
#else
        viewer->spinOnce ();
#endif
    } 
}