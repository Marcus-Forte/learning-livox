#pragma once

#include "ILidar.hh"
#include <deque>

class Mid40 : public ILidar {

public:
  Mid40();
  void init() override;
  PointCloud3 getScan() override;
  void startSampling() override;
  void stopSampling() override;
  void setMode(Mode mode) override;

private:
  std::deque<PointCloud3> queue_;

  size_t queue_limit_ = 100;
};