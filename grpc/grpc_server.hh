#pragma once

#include "ILidar.hh" // # TODO fix dependency?
#include "get_points_service.hh"
#include <future>
#include <grpcpp/grpcpp.h>
#include <grpcpp/server_builder.h>

class gRPCServer {
public:
  gRPCServer();

  void start();
  void stop();

  void put_scan(const std::vector<Point3> &scan);

private:
  ScanService scan_service_;
  std::unique_ptr<grpc::Server> server_;
  std::future<void> task_;
};