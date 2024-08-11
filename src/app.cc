#include <chrono>
#include <iostream>
#include <thread>

#include "ILidar.hh"
#include "grpc_server.hh"
#include "mid40.hh"

void printUsage() { std::cout << "Usage: app [powersave]" << std::endl; }
int main(int argc, char** argv) {
  Mid40 lidar;
  lidar.init();

  if (argc >= 2) {
    const std::string arg(argv[1]);
    if (arg == "powersave") {
      lidar.setMode(Mode::PowerSave);
      exit(0);
    } else {
      printUsage();
      exit(0);
    }
  }

  lidar.setMode(Mode::Normal);

  lidar.startSampling();
  unsigned int idx = 0;

  gRPCServer server;
  server.start();

  const int accumulate = 100;
  PointCloud3 accumulated;
  while (true) {
    auto points = lidar.getScan();

    if (points.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      std::cout << "Scan count: " << idx++ << std::endl;
      accumulated.insert(accumulated.end(), points.begin(), points.end());
      if (idx++ % accumulate == 0) {
        server.put_scan(accumulated);
        accumulated.clear();
      }
    }
  }
}