#pragma once
#include <vector>
struct Point3 {
  float x;
  float y;
  float z;
};

using PointCloud3 = std::vector<Point3>;

class ILidar {
public:
  virtual void init() = 0;
  virtual void startSampling() = 0;
  virtual void stopSampling() = 0;
  virtual PointCloud3 getScan() = 0;
};