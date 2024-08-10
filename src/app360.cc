#include "grpc_server.hh"
#include "mid360.hh"
#include <chrono>
#include <iostream>
#include <thread>

void printUsage() {
  std::cout << "Usage: app [config] (opt)[accusamples]" << std::endl;
}
int main(int argc, char **argv) {

  if (argc < 3) {
    printUsage();
    exit(0);
  }

  const int accumulate = atoi(argv[2]);
  std::cout << "Accu samples: " << accumulate << std::endl;
  Mid360 lidar(argv[1], accumulate);
  lidar.init();

  lidar.setMode(Mode::Normal);

  lidar.startSampling();

  gRPCServer server;
  server.start();

  std::chrono::high_resolution_clock::time_point last =
      std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point current;

  while (true) {

    const auto imu = lidar.getImuSample();

    if (imu.has_value()) {
      //   std::cout << std::format("Imu ok!: {} {} {} {} {} {}\n", imu->ax,
      //                            imu->ay, imu->az, imu->gx, imu->gy,
      //                            imu->gz);
      // }
      // current = std::chrono::high_resolution_clock::now();
      // std::cout << "Imu time diff: "
      //           << std::chrono::duration_cast<std::chrono::microseconds>(
      //                  current - last)
      //                  .count()
      //           << " us" << std::endl;

      // last = current;
    }
    const auto points = lidar.getScan();

    if (!points.empty()) {

      current = std::chrono::high_resolution_clock::now();
      std::cout << "AccScan time diff: "
                << std::chrono::duration_cast<std::chrono::microseconds>(
                       current - last)
                       .count()
                << " us" << std::endl;

      last = current;

      std::cout << "Sending: " << points.size() << " points" << std::endl;

      server.put_scan(points);
    }
  }
}
