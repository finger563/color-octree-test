#pragma once

#include <nlohmann/json.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <color-octree-test/ColorOcTree.h>

#include <mutex>

namespace roshi {
  class MapServer {

  public:
    typedef roshi::ColorOcTree OcTreeT;

    MapServer();
    ~MapServer();

    void init(const std::string& configFileName);
    void init(const nlohmann::json& config);

    void reset();

    void insertScanNode(octomap::ScanNode& node);

  protected:
    // map storage in octree
    std::shared_ptr<OcTreeT> m_octree;
    std::mutex _octree_mutex;

    // sensor max range
    double m_maxRange;

    // octree resolution and configuration
    double m_res;
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;

    double m_pointcloudMinX;
    double m_pointcloudMaxX;
    double m_pointcloudMinY;
    double m_pointcloudMaxY;
    double m_pointcloudMinZ;
    double m_pointcloudMaxZ;
    double m_occupancyMinZ;
    double m_occupancyMaxZ;
    double m_minSizeX;
    double m_minSizeY;

    bool m_compressMap;

    // do we save the pointclouds as ScanNode files?
    bool m_savePointCloud;
    std::string m_pc_filename;
  };
}
