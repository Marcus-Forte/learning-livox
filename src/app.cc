#include <chrono>
#include <iostream>
#include <thread>

#include "ILidar.hh"
#include "grpc_server.hh"
#include "mid40.hh"

void printUsage() {
  std::cout << "Usage: app [mode: 0, 1] [accupts]" << std::endl;
}
int main(int argc, char** argv) {
  if (argc < 3) {
    printUsage();
    exit(0);
  }

  const unsigned int mode = atoi(argv[1]);
  const size_t accumulate = atoi(argv[2]);
  Mid40 lidar(accumulate);
  lidar.init();

  if (mode == 0) {
    lidar.setMode(Mode::PowerSave);
    exit(0);
  } else {
    lidar.setMode(Mode::Normal);
  }

  lidar.startSampling();

  gRPCServer server;
  server.start();

  PointCloud3 accumulated;
  while (true) {
    auto cloud = lidar.getScan();

    if (cloud.points.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      server.put_scan(accumulated.points);
    }
  }
}
