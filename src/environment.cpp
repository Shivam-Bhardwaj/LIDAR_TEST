/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "quiz/cluster/kdtree.h"
#include <thread>
#include <unordered_set>

float a, b, c;


std::vector<std::vector<int>>
euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol);


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol);

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  auto startTime = std::chrono::steady_clock::now();
  auto filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2,
                                                  Eigen::Vector4f(-10, -6, -2, 1),
                                                  Eigen::Vector4f(17, 6, 2, 1));

  auto seg = Ransac(filterCloud, 100, 0.2);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (int point: seg) {
    inliers->indices.push_back(point);
  }


  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(filterCloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*inputCloud);
  KdTree *tree = new KdTree;

  std::vector<std::vector<float>> input_points;

  for (int i = 0; i < inputCloud->points.size(); i++) {
    tree->insert({inputCloud->points.at(i).x,
                  inputCloud->points.at(i).y,
                  inputCloud->points.at(i).z}, i);
    input_points.push_back({{inputCloud->points.at(i).x,
                                inputCloud->points.at(i).y,
                                inputCloud->points.at(i).z}});
  }


  std::vector<std::vector<int>> clusters = euclideanCluster(input_points, tree, 3.0);

//  auto segmented_plane = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);

//  auto clusters = pointProcessorI->Clustering(inputCloud, 0.3, 10, 1000);
//
//  int clusterId = 0;
//  for (const pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster : clusters) {
//    std::cout << "cluster size ";
//    pointProcessorI->numPoints(cluster);
//    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), Color(1, 1, 1));
//    ++clusterId;
//    Box box = pointProcessorI->BoundingBox(cluster);
//    renderBox(viewer, box, clusterId);
//  }
//  extract.setNegative(false);
//  extract.filter(*inputCloud);
//  renderPointCloud(viewer,inputCloud, "cloud", Color(0, 1, 0));
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "pipeline took " << elapsedTime.count() << " milliseconds" << std::endl;
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer);


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
  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
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

void helper_cluster(int i, const std::vector<std::vector<float>> &points, std::vector<int> &cluster,
                    std::vector<bool> processed, KdTree *tree, float distanceTol) {
  processed[i] = true;
  cluster.push_back(i);

  std::vector<int> to_be_traversed = tree->search(points[i], distanceTol);
  for (int index: to_be_traversed) {
    if (!processed[index]) {
      helper_cluster(index, points, cluster, processed, tree, distanceTol);
    }
  }
}

std::vector<std::vector<int>>
euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol) {

  // TODO: Fill out this function to return list of indices for each cluster
  std::vector<std::vector<int>> clusters;
  std::vector<bool> processed(points.size(), false);
  int i{0};
  while (i < points.size()) {
    if (processed[i]) {
      i++;
      continue;
    }
    std::vector<int> cluster;
    helper_cluster(i++, points, cluster, processed, tree, distanceTol);
    clusters.push_back(cluster);
  }
  return clusters;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function
  while (maxIterations--) {
    std::unordered_set<int> inliers;
    while (inliers.size() < 3) {
      inliers.insert(rand() % cloud->points.size());
    }

    auto itr = inliers.begin();
    float x1{cloud->points.at(*itr).x}, y1{cloud->points.at(*itr).y}, z1{cloud->points.at(*itr).z};
    itr++;
    float x2{cloud->points.at(*itr).x}, y2{cloud->points.at(*itr).y}, z2{cloud->points.at(*itr).z};
    itr++;
    float x3{cloud->points.at(*itr).x}, y3{cloud->points.at(*itr).y}, z3{cloud->points.at(*itr).z};
    float i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    float A{i}, B{j}, C{k}, D{-1 * (i * x1 + j * y1 + k * z1)};

    for (int index = 0; index < cloud->points.size(); ++index) {
      if (inliers.count(index)) continue;

      auto current_point = cloud->points.at(index);
      float x4{current_point.x}, y4{current_point.y}, z4{current_point.z};
      float d = fabs(A * x4 + B * y4 + C * z4 + D) / sqrt(A * A + B * B + C * C);
      if (d < distanceTol) {
        inliers.insert(index);
      }
    }
    if (inliersResult.size() < inliers.size()) {
      inliersResult = inliers;
    }
  }
  return inliersResult;
}

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
