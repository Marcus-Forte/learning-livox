#pragma once

#include "ILidar.hh"
#include <deque>
#include <optional>
#include <string>

class Mid360 : public ILidar {

public:
  enum class ScanPattern { Repetitive, NonRepetitive, LowFrameRate };
  Mid360(const std::string &&config, size_t accumulate_scans);
  void init() override;
  PointCloud3 getScan() override;
  void startSampling() override;
  void stopSampling() override;
  void setMode(Mode mode) override;
  void setScanPattern(ScanPattern pattern) const;

  std::optional<ImuData> getImuSample();

private:
  const std::string config_;
  PointCloud3 accumulated_;
  std::deque<PointCloud3> queue_;
  std::deque<ImuData> queue_imu_;

  const size_t queue_limit_ = 50;
  const size_t accumulate_scans_;

  size_t scan_count_;

  uint32_t connection_handle_;
};