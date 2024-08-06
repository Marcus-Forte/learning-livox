#include "mid40.hh"
#include <chrono>
#include <format>
#include <iostream>
#include <thread>

int main() {
  Mid40 lidar;
  lidar.init();

  //   lidar.registerDataCallback();
  lidar.startSampling();

  unsigned int idx = 0;
  while (true) {

    auto points = lidar.getScan();

    if (points.empty()) {
      std::cout << "no pts! " << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      for (const auto &pt : points) {
        std::cout << std::format("{},{},{}\n", pt.x, pt.y, pt.z);
      }
    }

    std::cout << "Scan count: " << idx++ << std::endl;
  }
}