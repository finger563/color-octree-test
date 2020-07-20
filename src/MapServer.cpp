#include <color-octree-test/MapServer.h>
#include <iostream>
#include <exception>
#include <string>

// for convenience
using namespace octomap;
using json = nlohmann::json;

namespace roshi {

MapServer::MapServer()
  : m_octree(NULL), m_maxRange(-1.0),
    m_res(0.05),
    m_treeDepth(0),
    m_maxTreeDepth(0),
    m_pointcloudMinX(-std::numeric_limits<double>::max()),
    m_pointcloudMaxX(std::numeric_limits<double>::max()),
    m_pointcloudMinY(-std::numeric_limits<double>::max()),
    m_pointcloudMaxY(std::numeric_limits<double>::max()),
    m_pointcloudMinZ(-std::numeric_limits<double>::max()),
    m_pointcloudMaxZ(std::numeric_limits<double>::max()),
    m_occupancyMinZ(-std::numeric_limits<double>::max()),
    m_occupancyMaxZ(std::numeric_limits<double>::max()),
    m_minSizeX(0.0), m_minSizeY(0.0),
    m_compressMap(true), m_savePointCloud(false),
    m_pc_filename("pc_scan.node") {
}

void MapServer::init(const std::string& configFileName) {
  json config = json({});
  if (configFileName.length()) {
    printf("MapServer: Parsing config file '%s'\n", configFileName.c_str());
    std::ifstream configFile(configFileName);
    try {
      configFile >> config;
    } catch (std::exception& e) {
      printf("Could not find / load config file '%s': %s\n", configFileName.c_str(), e.what());
    }
  } else {
    printf("MapServer: no config file provided, using defaults.\n");
  }
  init(config);
}

void MapServer::init(const json& config) {
  // settings for the point cloud
  m_pointcloudMinX = config.value("pointcloud_min_x", m_pointcloudMinX);
  m_pointcloudMaxX = config.value("pointcloud_max_x", m_pointcloudMaxX);
  m_pointcloudMinY = config.value("pointcloud_min_y", m_pointcloudMinY);
  m_pointcloudMaxY = config.value("pointcloud_max_y", m_pointcloudMaxY);
  m_pointcloudMinZ = config.value("pointcloud_min_z", m_pointcloudMinZ);
  m_pointcloudMaxZ = config.value("pointcloud_max_z", m_pointcloudMaxZ);
  printf("Filtering sensor data in the following ranges (min, max):\n"
         "\tx: [%.2f, %.2f]\n"
         "\ty: [%.2f, %.2f]\n"
         "\tz: [%.2f, %.2f]\n",
         m_pointcloudMinX, m_pointcloudMaxX,
         m_pointcloudMinY, m_pointcloudMaxY,
         m_pointcloudMinZ, m_pointcloudMaxZ);

  m_occupancyMinZ = config.value("occupancy_min_z", m_occupancyMinZ);
  m_occupancyMaxZ = config.value("occupancy_max_z", m_occupancyMaxZ);

  m_minSizeX = config.value("min_x_size", m_minSizeX);
  m_minSizeY = config.value("min_y_size", m_minSizeY);

  // settings for the sensor
  m_maxRange = config.value("sensor_model/max_range", m_maxRange);
  double probHit, probMiss, thresMin, thresMax;
  probHit = config.value("sensor_model/hit", 0.7);
  probMiss = config.value("sensor_model/miss", 0.4);
  thresMin = config.value("sensor_model/min", 0.12);
  thresMax = config.value("sensor_model/max", 0.97);

  printf("Using sensor_model/max_range: %0.2f m\n", m_maxRange);

  // setting for the map
  m_res = config.value("resolution", m_res);
  m_compressMap = config.value("compress_map", m_compressMap);

  printf("Using voxel resolution: %0.2f m\n", m_res);

  m_savePointCloud = config.value("save_pointcloud", m_savePointCloud);
  m_pc_filename = config.value("pointcloud_filename", m_pc_filename);

  // initialize octomap object & params
  m_octree = std::make_shared<OcTreeT>(m_res);
  m_octree->setProbHit(probHit);
  m_octree->setProbMiss(probMiss);
  m_octree->setClampingThresMin(thresMin);
  m_octree->setClampingThresMax(thresMax);
  m_treeDepth = m_octree->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;
}

MapServer::~MapServer() {
}

void MapServer::insertScanNode(octomap::ScanNode& node) {
  auto start = std::chrono::steady_clock::now();
  { // protect this code
    std::lock_guard<std::mutex> lock(_octree_mutex);
    // insert octomap::pointcloud into octree
    m_octree->insertPointCloud(*node.scan, node.pose.trans(), m_maxRange, true, true);
    // since we set lazy_eval to true, we need to explicitly call this
    m_octree->updateInnerOccupancy();
    // now we prune the tree (if configured to do so)
    if (m_compressMap)
      m_octree->prune();
  }
  auto total_elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);
  printf("Pointcloud insertion in MapServer done (%zu pts, %f sec)\n",
         node.scan->size(), total_elapsed.count());
}

void MapServer::reset() {
  m_octree->clear();
  printf("Cleared octomap\n");
}

} // namespace roshi
