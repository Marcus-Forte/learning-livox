#pragma once
#include <cstdint>
#include <vector>

struct Point3 {
  float x;
  float y;
  float z;
  uint8_t intensity;
};

struct ImuData {
  float gx;  // rad/s
  float gy;
  float gz;
  float ax;  // g
  float ay;
  float az;
  uint64_t timestamp;
};

enum class Mode { Normal, PowerSave };

struct PointCloud3 {
  std::vector<Point3> points;
  uint64_t timestamp;
};

class ILidar {
 public:
  virtual void init() = 0;
  virtual void startSampling() = 0;
  virtual void stopSampling() = 0;
  /** returns lidar scan. Associated timestamp is assumed to be the time
   * point[0] was measured. Unit: ns (1/1000000000 sec) */
  virtual PointCloud3 getScan() = 0;
  virtual void setMode(Mode mode) = 0;
};
