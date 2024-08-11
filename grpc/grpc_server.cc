#include "grpc_server.hh"

#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>

#include <future>
#include <memory>

gRPCServer::gRPCServer() = default;

void gRPCServer::start() {
  task_ = std::async(std::launch::async, [this] {
    grpc::ServerBuilder builder;
    builder.AddListeningPort("0.0.0.0:50051",
                             ::grpc::InsecureServerCredentials());
    builder.RegisterService(&scan_service_);

    server_ = builder.BuildAndStart();
    builder.SetMaxSendMessageSize(1024 * 1024 * 1024);
    builder.SetMaxReceiveMessageSize(1024 * 1024 * 1024);
    std::cout << "Listening on port 50051..." << std::endl;
    server_->Wait();
  });
}

void gRPCServer::stop() {
  server_->Shutdown();
  task_.get();
}

void gRPCServer::put_scan(const std::vector<Point3> &scan) {
  scan_service_.putScan(scan);
}