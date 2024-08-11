#include "get_points_service.hh"

#include <grpcpp/support/status.h>

#include <chrono>
#include <memory>
#include <thread>

#include "ILidar.hh"
#include "colormap.hh"

ScanService::ScanService() = default;

grpc::Status ScanService::getScan(
    ::grpc::ServerContext *context,
    const ::google::protobuf::Empty * /*request*/
    ,
    ::grpc::ServerWriter<lidar::PointCloud3> *writer) {
  static bool s_client_connected = false;
  if (s_client_connected) {
    return {grpc::StatusCode::RESOURCE_EXHAUSTED, "Only one client supported"};
  }

  std::cout << "Start stream." << std::endl;
  s_client_connected = true;
  while (!context->IsCancelled()) {
    if (scan_data_ != nullptr) {
      lidar::PointCloud3 point_cloud;

      for (const auto &point : *scan_data_) {
        auto *msg_pt = point_cloud.add_points();
        msg_pt->set_x(point.x);
        msg_pt->set_y(point.y);
        msg_pt->set_z(point.z);

        float pt_r;
        float pt_g;
        float pt_b;
        Int2RGB(static_cast<float>(point.intensity), pt_r, pt_g, pt_b);
        // 2D Lidar
        msg_pt->set_r(pt_r);
        msg_pt->set_g(pt_g);
        msg_pt->set_b(pt_b);
      }
      writer->Write(point_cloud);
      scan_data_.reset();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::cout << "Canceled stream." << std::endl;
  s_client_connected = false;
  return ::grpc::Status::OK;
}

void ScanService::putScan(const PointCloud3 &scan) {
  scan_data_ = std::make_unique<PointCloud3>(scan);
}