#include "get_points_service.hh"
#include <chrono>
#include <grpcpp/support/status.h>
#include <thread>

#include "colormap.hh"

ScanService::ScanService() = default;

grpc::Status
ScanService::getScan(::grpc::ServerContext *context,
                     const ::google::protobuf::Empty * /*request*/,
                     ::grpc::ServerWriter<lidar::PointCloud3> *writer) {
  static bool s_client_connected = false;
  if (s_client_connected) {
    return {grpc::StatusCode::RESOURCE_EXHAUSTED, "Only one client supported"};
  }

  std::cout << "Start stream." << std::endl;
  s_client_connected = true;
  while (!context->IsCancelled()) {

    while (!scan_queue_.empty()) {
      auto scan = scan_queue_.front();
      lidar::PointCloud3 point_cloud;

      for (const auto &point : scan) {
        auto pt = point_cloud.add_points();
        pt->set_x(point.x);
        pt->set_y(point.y);
        pt->set_z(point.z);

        float r, g, b;
        Int2RGB(static_cast<float>(point.intensity), r, g, b);
        // 2D Lidar
        pt->set_r(r);
        pt->set_g(g);
        pt->set_b(b);
      }
      writer->Write(point_cloud);
      scan_queue_.pop_front();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::cout << "Canceled stream." << std::endl;
  s_client_connected = false;
  return ::grpc::Status::OK;
}

void ScanService::putScan(const std::vector<Point3> &scan) {
  scan_queue_.push_front(scan);
  if (scan_queue_.size() > 1) {
    scan_queue_.pop_back();
  }
}