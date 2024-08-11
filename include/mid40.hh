#pragma once

#include <deque>

#include "ILidar.hh"

class Mid40 : public ILidar {
 public:
  Mid40(size_t accumulate_scan_count);
  void init() override;
  PointCloud3 getScan() override;
  void startSampling() override;
  void stopSampling() override;
  void setMode(Mode mode) override;

 private:
  std::deque<PointCloud3> queue_;
  PointCloud3 accumulated_;
  const size_t accumulate_scan_count_;

  size_t scan_count_;
  size_t queue_limit_ = 100;
};