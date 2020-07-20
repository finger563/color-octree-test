/**
 * test_mapserver_insertion_performance: A Tool to test MapServer insertion perforamnce
 */

#include <color-octree-test/MapServer.h>
#include <iostream>

nlohmann::json load_config(const std::string & config_filename) {
  auto config = nlohmann::json({});
  if (!config_filename.empty()) {
    std::cout << "ROSHI: Parsing config file " << config_filename << "\n";
    std::ifstream config_file(config_filename);
    try {
      config_file >> config;
    } catch (std::exception & e) {
      std::cout << "ROSHI: Could not find / load config file " << config_filename
                << " " << e.what() << "\n";
    }
  } else {
    std::cout << "ROSHI: No config file provided, using defaults\n";
  }
  return config;
}

int main(int argc, char **argv) {
  // get config file name from command line arguments
  std::string config_filename = "";
  if (argc > 1) {
    config_filename = argv[1];
  }

  // initialize the map server
  roshi::MapServer server;
  server.init(config_filename);

  auto config = load_config(config_filename);

  try {
    // load the file containing the points
    std::filebuf fb;
    std::string scan_filename = config.value("pointcloud_filename", "pc_scan.node");
    fb.open(scan_filename, std::ios::in);
    if (!fb.is_open()) {
      std::cerr << "Error: could not find scan file '" << scan_filename << "', please make sure to generate one.\n";
      return -1;
    }
    std::istream is(&fb);
    auto node = octomap::ScanNode();
    node.readBinary(is);

    {
      auto start = std::chrono::steady_clock::now();
      size_t num_tests = 10;
      for (size_t i=0; i < num_tests; i++) {
        // now see how long it takes to insert the points
        server.insertScanNode(node);
      }
      auto total_elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);
      printf("Pointcloud insertion in MapServer done (%zu pts, %f sec average over %zu iterations)\n",
             node.scan->size(), total_elapsed.count() / (double)num_tests, num_tests);
    }

    // cleanup
    fb.close();
  } catch (std::runtime_error &e) {
    std::cout << "test_mapserver_insertion_performance exception: " << e.what() << "\n";
    return -1;
  }

  return 0;
}
