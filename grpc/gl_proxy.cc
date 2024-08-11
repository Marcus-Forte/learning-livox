
#include <google/protobuf/empty.pb.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/security/credentials.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/support/channel_arguments.h>

#include <chrono>
#include <memory>
#include <thread>

#include "colormap.hh"
#include "lidar.grpc.pb.h"
#include "opengl_srv_points.grpc.pb.h"

// Simple application that subscribes to lidar service and publishes to opengl
// service: https://github.com/Marcus-Forte/learning-opengl

gl::PointCloud3 fromLidarService(const lidar::PointCloud3 &msg) {
  gl::PointCloud3 ret;
  ret.mutable_points()->Reserve(msg.points_size());

  for (size_t i = 0; i < msg.points_size(); ++i) {
    auto *point = ret.add_points();
    point->set_x(msg.points(i).x());
    point->set_y(msg.points(i).y());
    point->set_z(msg.points(i).z());
    float pt_r;
    float pt_g;
    float pt_b;

    Int2RGB(msg.points(i).intensity(), pt_r, pt_g, pt_b);
    point->set_r(pt_r);
    point->set_g(pt_g);
    point->set_b(pt_b);
  }
  return ret;
}
void printUsage() {
  std::cout << "gl_proxy [lidar server ip:port] [opengl server ip:port]"
            << std::endl;
}
int main(int argc, char **argv) {
  if (argc < 3) {
    printUsage();
    exit(-1);
  }
  const std::string lidarSrvIp(argv[1]);
  const std::string openGlSrvIp(argv[2]);

  grpc::ChannelArguments args;
  args.SetMaxReceiveMessageSize(1024 * 1024 * 1024);
  args.SetMaxSendMessageSize(1024 * 1024 * 1024);

  auto channel = grpc::CreateCustomChannel(
      lidarSrvIp, grpc::InsecureChannelCredentials(), args);
  auto lidar_stub = lidar::LidarService::NewStub(channel);

  auto channel_opengl = grpc::CreateCustomChannel(
      openGlSrvIp, grpc::InsecureChannelCredentials(), args);
  auto opengl_stub = gl::addToScene::NewStub(channel_opengl);
  google::protobuf::Empty empty_response;
  std::unique_ptr<grpc::ClientContext> lidar_context =
      std::make_unique<grpc::ClientContext>();
  auto reader = lidar_stub->getScan(lidar_context.get(), empty_response);

  std::unique_ptr<grpc::ClientContext> opengl_context =
      std::make_unique<grpc::ClientContext>();

  auto writer =
      opengl_stub->streamPointClouds(opengl_context.get(), &empty_response);
  lidar::PointCloud3 msg;
  while (true) {
    if (!reader->Read(&msg)) {
      auto state = channel_opengl->GetState(true);
      std::cerr << "Error reading from lidar server " << lidarSrvIp
                << " code: " << state << std::endl;
      if (state == GRPC_CHANNEL_READY) {
        std::cout << "Reconnecting..." << std::endl;
        channel =
            grpc::CreateChannel(lidarSrvIp, grpc::InsecureChannelCredentials());
        lidar_context = std::make_unique<grpc::ClientContext>();
        lidar_stub = lidar::LidarService::NewStub(channel);
        reader = lidar_stub->getScan(lidar_context.get(), empty_response);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    const auto tick = std::chrono::high_resolution_clock::now();
    auto gl_cloud = fromLidarService(msg);
    const auto delta = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::high_resolution_clock::now() - tick)
                           .count();
    std::cout << "Conversion took: " << delta << " us" << std::endl;
    gl_cloud.set_entity_name("lidar");
    if (!writer->Write(gl_cloud)) {
      auto state = channel_opengl->GetState(true);
      std::cerr << "Error writing to opengl server " << openGlSrvIp
                << " code: " << state << std::endl;
      if (state == GRPC_CHANNEL_READY) {
        // Reconnect
        std::cout << "Reconnecting..." << std::endl;
        channel_opengl = grpc::CreateChannel(
            openGlSrvIp, grpc::InsecureChannelCredentials());
        opengl_context = std::make_unique<grpc::ClientContext>();
        opengl_stub = gl::addToScene::NewStub(channel_opengl);
        writer = opengl_stub->streamPointClouds(opengl_context.get(),
                                                &empty_response);
      }
    }
  }
}