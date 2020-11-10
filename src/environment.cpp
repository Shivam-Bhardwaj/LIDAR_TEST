/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <thread>

float a, b, c;

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer) {

  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

//  ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
//  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd(
//    "../src/sensors/data/pcd/data_1/0000000000.pcd");
  auto filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.1,
                                                  Eigen::Vector4f(-10, -6, -2, 1),
                                                  Eigen::Vector4f(17, 6, 2, 1));
  auto segmented_plane = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
  auto clusters = pointProcessorI->Clustering(segmented_plane.first, a, int(b), int(c));

  int clusterId = 0;
  for (const pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster : clusters) {
    std::cout << "cluster size ";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), Color(1, 1, 1));
    ++clusterId;
    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
  }
  renderPointCloud(viewer, segmented_plane.second, "cloud", Color(0, 1, 0));

}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer) {

  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
    case XY :
      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
      break;
    case TopDown :
      viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
      break;
    case Side :
      viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
      break;
    case FPS :
      viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS)
    viewer->addCoordinateSystem(1.0);
}


int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;
  int option = 0;
  while ((option = getopt(argc, argv, "a:b:c:")) != -1) {
    switch (option) {
      case 'a' :
        a = std::stof(std::string(optarg));
        break;
      case 'b' :
        b = std::stof(std::string(optarg));
        break;
      case 'c':
        c = std::stof(std::string(optarg));
        break;
      default:
        return EXIT_SUCCESS;
    }
  }
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  auto *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
  while (!viewer->wasStopped()) {

    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);
//    std::this_thread::sleep_for(std::chrono::milliseconds (50));
    streamIterator++;
    if (streamIterator == stream.end())
      streamIterator = stream.begin();

    viewer->spinOnce();
  }
}