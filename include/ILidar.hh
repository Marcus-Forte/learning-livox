#pragma once
#include <stdint.h>
#include <vector>

struct Point3 {
  float x;
  float y;
  float z;
  uint8_t intensity;
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