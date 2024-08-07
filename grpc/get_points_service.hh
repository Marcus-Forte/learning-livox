#pragma once
#include "ILidar.hh"
#include "points.grpc.pb.h"

class ScanService : public lidar::LidarService::Service {
public:
  ScanService();
  ::grpc::Status
  getScan(::grpc::ServerContext *context,
          const ::google::protobuf::Empty *request,
          ::grpc::ServerWriter<lidar::PointCloud3> *writer) override;

  void putScan(const std::vector<Point3> &scan);

private:
  std::deque<std::vector<Point3>> scan_queue_;
};