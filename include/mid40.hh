#pragma once

#include "ILidar.hh"
#include <deque>
#include <thread>

class Mid40 : public ILidar {

public:
  Mid40();
  void init() override;
  PointCloud3 getScan() override;
  void startSampling() override;
  void stopSampling() override;

private:
  std::thread worker_;
  std::deque<PointCloud3> queue_;

  size_t queue_limit_ = 100;
};