#include "grpc_server.hh"
#include "mid360.hh"
#include <chrono>
#include <iostream>
#include <thread>

void printUsage() {
  std::cout << "Usage: app [config] [powersave] (opt)[accupts]" << std::endl;
}
int main(int argc, char **argv) {

  if (argc < 2) {
    printUsage();
    exit(0);
  }

  Mid360 lidar(argv[1]);
  lidar.init();

  if (argc > 2) {
    const std::string arg = argv[2];
    if (arg == "powersave") {
      std::cout << "power save mode..." << std::endl;
      lidar.setMode(Mode::PowerSave);
      exit(0);
    }
  }

  lidar.startSampling();

  gRPCServer server;
  server.start();

  const int accumulate = argc > 4 ? atoi(argv[3]) : 100;

  PointCloud3 accumulated;
  unsigned int idx = 0;
  while (true) {

    const auto points = lidar.getScan();

    if (points.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      accumulated.insert(accumulated.end(), points.begin(), points.end());
      if (idx++ % accumulate == 0) {
        server.put_scan(accumulated);
        accumulated.clear();
      }
    }
  }
}