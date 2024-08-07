#pragma once

#include "ILidar.hh"
#include <deque>
#include <thread>

class Mid360 : public ILidar {

public:
  Mid360(const std::string &&config);
  void init() override;
  PointCloud3 getScan() override;
  void startSampling() override;
  void stopSampling() override;
  void setMode(Mode mode) override;

private:
  const std::string config_;
  std::deque<PointCloud3> queue_;

  size_t queue_limit_ = 100;

  uint8_t connection_handle_;
};