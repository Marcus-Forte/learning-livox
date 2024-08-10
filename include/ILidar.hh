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
  float gx; // rad/s
  float gy;
  float gz;
  float ax; // g
  float ay;
  float az;
};

enum class Mode { Normal, PowerSave };

using PointCloud3 = std::vector<Point3>;

class ILidar {
public:
  virtual void init() = 0;
  virtual void startSampling() = 0;
  virtual void stopSampling() = 0;
  virtual PointCloud3 getScan() = 0;
  virtual void setMode(Mode mode) = 0;
};