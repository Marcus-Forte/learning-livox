#include "mid360.hh"
#include "livox_lidar_api.h"
#include <algorithm>
#include <condition_variable>
#include <iostream>
#include <livox_lidar_def.h>
#include <mutex>
#include <string>

std::mutex g_mutex;
std::condition_variable g_cv;
bool g_sync = false;

static PointCloud3 convertData(const LivoxLidarEthernetPacket *eth_packet,
                               unsigned int data_pts) {
  const LivoxLidarCartesianHighRawPoint *data_ =
      reinterpret_cast<const LivoxLidarCartesianHighRawPoint *>(
          eth_packet->data);

  PointCloud3 cloud(data_pts);
  std::transform(data_, data_ + data_pts, cloud.begin(),
                 [](const LivoxLidarCartesianHighRawPoint &pt) {
                   return Point3{static_cast<float>(pt.x) / 1000.0f,
                                 static_cast<float>(pt.y) / 1000.0f,
                                 static_cast<float>(pt.z) / 1000.0f,
                                 pt.reflectivity};
                 });
  return {cloud};
}

Mid360::Mid360(const std::string &&config) : config_{config} {}

void Mid360::startSampling() {};
void Mid360::stopSampling() {};

void Mid360::setMode(Mode mode) {
  SetLivoxLidarInfoChangeCallback(
      [](const uint32_t handle, const LivoxLidarInfo *info, void *client_data) {
        if (info == nullptr)
          return;

        std::cout << "Lidar IP: " << info->lidar_ip << std::endl;
        std::cout << "DevType: " << info->dev_type << std::endl;
        std::cout << "SN: " << info->sn << std::endl;
        std::cout << "handle: " << std::to_string(handle) << std::endl;
        // TODO!
      },
      {});
  std::unique_lock<std::mutex> lock(g_mutex);
  g_cv.wait(lock, [] { return g_sync; });
};

void Mid360::init() {
  std::cout << "config:  " << config_ << std::endl;
  if (!LivoxLidarSdkInit(config_.c_str())) {
    LivoxLidarSdkUninit();
    std::cout << "Livox-SDK init fail!" << std::endl;
    throw std::runtime_error("Unable to initialize Mid360!");
  }

  if (!LivoxLidarSdkStart()) {
    throw std::runtime_error("Unable to initialize Mid40!");
  }

  SetLivoxLidarPointCloudCallBack(
      [](const uint32_t handle, const uint8_t dev_type,
         LivoxLidarEthernetPacket *data, void *client_data) {
        if (!data)
          return;

        auto this_ = reinterpret_cast<decltype(this)>(client_data);

        auto cloud = convertData(data, data->dot_num);
        {
          std::lock_guard<std::mutex> lock(g_mutex);
          this_->queue_.push_front(cloud);

          if (this_->queue_.size() > this_->queue_limit_) {
            this_->queue_.pop_back();
          }
        }
      },
      this);
}

PointCloud3 Mid360::getScan() {
  if (queue_.empty())
    return {};

  const auto last = queue_.front();
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    queue_.pop_front();
  }
  return last;
}