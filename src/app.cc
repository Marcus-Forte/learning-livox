#include "mid40.hh"
#include <chrono>
#include <iostream>
#include <thread>
int main() {
  Mid40 lidar;
  lidar.init();

  lidar.startSampling();

  unsigned int idx = 0;
  while (true) {

    auto points = lidar.getScan();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::cout << "Scan count: " << idx++ << std::endl;
  }
}