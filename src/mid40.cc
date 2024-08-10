#include "mid40.hh"
#include "ILidar.hh"
#include "livox_sdk.h"
#include <condition_variable>
#include <iostream>
#include <livox_def.h>
#include <mutex>

std::condition_variable g_cv;
std::mutex g_mutex;
bool g_sync;

uint8_t connection_handle_ = 0;

static PointCloud3 convertData(const LivoxEthPacket *eth_packet,
                               unsigned int data_pts) {
  const auto *data_ = reinterpret_cast<const LivoxRawPoint *>(eth_packet->data);

  PointCloud3 cloud(data_pts);
  for (auto &point : cloud) {
    point.x = static_cast<float>(data_->x) / 1000.0F;
    point.y = static_cast<float>(data_->y) / 1000.0F;
    point.z = static_cast<float>(data_->z) / 1000.0F;
    point.intensity = data_->reflectivity;
    data_++;
  }
  return {cloud};
}

Mid40::Mid40() = default;

// TODO check state after powering on.
void Mid40::startSampling() {
  std::cout << "LidarStartSampling" << std::endl;
  LidarStartSampling(
      connection_handle_,
      [](livox_status status, uint8_t handle, uint8_t response,
         void *client_data) { std::cout << "start sampling!" << std::endl; },
      nullptr);
}

void Mid40::stopSampling() { LidarStopSampling(connection_handle_, {}, {}); }

void Mid40::setMode(Mode mode) {
  if (mode == Mode::Normal) {
    LidarSetMode(connection_handle_, LidarMode::kLidarModeNormal, {}, {});
  } else if (mode == Mode::PowerSave) {
    std::cout << "powersave\n";
    LidarSetMode(connection_handle_, LidarMode::kLidarModePowerSaving,
                 [](livox_status status, uint8_t handle, uint8_t response,
                    void *client_data) {
                   std::cout
                       << "Powersave mode status: " << std::to_string(status)
                       << std::endl;
                 },
                 {});
  }
}
void Mid40::init() {

  if (!Init()) {
    Uninit();
    std::cout << "Livox-SDK init fail!" << std::endl;
    throw std::runtime_error("Unable to initialize Mid40!");
  }

  if (!Start()) {
    Uninit();
    throw std::runtime_error("Unable to initialize Mid40!");
  }

  std::cout << "Init OK!\n";
  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d\n", _sdkversion.major, _sdkversion.minor,
         _sdkversion.patch);

  std::cout << "SetBroadcastCallback" << std::endl;
  SetBroadcastCallback([](const BroadcastDeviceInfo *info) -> void {
    if (info == nullptr) {
      std::cout << "nullptr" << std::endl;
      return;
    }
    std::cout << "Found Livox IP: " << info->ip << std::endl;

    const auto result =
        AddLidarToConnect(info->broadcast_code, &connection_handle_);
    if (result == kStatusSuccess && connection_handle_ < kMaxLidarCount) {
      std::cout << "Lidar connected: " << info->broadcast_code << std::endl;

    } else {
      printf("Add lidar to connect is failed : %d %d \n", result,
             connection_handle_);
    }

    if (info->dev_type == kDeviceTypeHub) {
      printf("In lidar mode, couldn't connect a hub : %s\n",
             info->broadcast_code);
      return;
    }
  });

  g_sync = false;
  SetDeviceStateUpdateCallback([](const DeviceInfo *info, DeviceEvent type) {
    if (info == nullptr) {
      return;
    }

    if (type == kEventConnect) {
      printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
      std::lock_guard<std::mutex> lock(g_mutex);
      g_sync = true;
      g_cv.notify_one();
    } else if (type == kEventDisconnect) {
      printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
    } else if (type == kEventStateChange) {
      printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
    }
  });
  {
    std::unique_lock<std::mutex> lock(g_mutex);
    g_cv.wait(lock, [] { return g_sync; });
  }
  std::cout << "Connected!" << std::endl;

  std::cout << "Set cartesian coordinates " << std::endl;
  SetCartesianCoordinate(connection_handle_, {}, nullptr);
  SetDataCallback(
      connection_handle_,
      [](uint8_t handle, LivoxEthPacket *data, uint32_t data_num,
         void *client_data) {
        auto *this_ = reinterpret_cast<decltype(this)>(client_data);
        // Convert
        if (data == nullptr) {
          // std::cout << "cvt: " << data_num << "pts" << std::endl;
          auto cloud = convertData(data, data_num);
          {
            std::lock_guard<std::mutex> lock(g_mutex);
            this_->queue_.push_front(cloud);

            if (this_->queue_.size() > this_->queue_limit_) {
              this_->queue_.pop_back();
            }
          }
        }
      },
      this);
}

PointCloud3 Mid40::getScan() {
  if (queue_.empty()) {
    return {};
  }

  {
    std::lock_guard<std::mutex> lock(g_mutex);
    const auto last = queue_.front();
    queue_.pop_front();
    return last;
  }
}