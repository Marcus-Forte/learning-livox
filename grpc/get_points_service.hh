#pragma once
#include "ILidar.hh"
#include "lidar.grpc.pb.h"
#include <memory>

class ScanService : public lidar::LidarService::Service {
public:
  ScanService();
  ::grpc::Status
  getScan(::grpc::ServerContext *context,
          const ::google::protobuf::Empty *request,
          ::grpc::ServerWriter<lidar::PointCloud3> *writer) override;

  void putScan(const PointCloud3 &scan);

private:
  std::unique_ptr<PointCloud3> scan_data_;
};