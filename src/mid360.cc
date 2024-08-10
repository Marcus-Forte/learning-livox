#include "mid360.hh"
#include "ILidar.hh"
#include "livox_lidar_api.h"
#include <condition_variable>
#include <iostream>
#include <livox_lidar_def.h>
#include <mutex>
#include <string>

std::mutex g_mutex;
std::mutex g_lidar_mutex;
std::mutex g_imu_mutex;
std::condition_variable g_cv;
bool g_sync;
uint32_t g_connection_handle_;

// TODO pre-allocate accumulate_scans * 96.
static PointCloud3 convertData(const LivoxLidarEthernetPacket *eth_packet,
                               unsigned int data_pts) {
  const auto *data_ = reinterpret_cast<const LivoxLidarCartesianHighRawPoint *>(
      eth_packet->data);
  PointCloud3 cloud(data_pts);
  for (auto &point : cloud) {
    point.x = static_cast<float>(data_->x) / 1000.0F;
    point.y = static_cast<float>(data_->y) / 1000.0F;
    point.z = static_cast<float>(data_->z) / 1000.0F;
    point.intensity = data_->reflectivity;
    data_++;
  }
  return cloud;
}

Mid360::Mid360(const std::string &&config, size_t accumulate_scans)
    : config_{config}, accumulate_scans_(accumulate_scans), scan_count_(0) {}

void Mid360::startSampling() {
  if (!LivoxLidarSdkStart()) {
    throw std::runtime_error("Unable to initialize Mid360!");
  }
};
void Mid360::stopSampling() { /* TODO */ };

void Mid360::setMode(Mode mode) {
  // Wake up is actually idle...
  const LivoxLidarWorkMode _mode = mode == Mode::Normal
                                       ? LivoxLidarWorkMode::kLivoxLidarNormal
                                       : LivoxLidarWorkMode::kLivoxLidarWakeUp;

  g_sync = false;
  SetLivoxLidarWorkMode(
      g_connection_handle_, _mode,
      [](livox_status status, uint32_t handle,
         LivoxLidarAsyncControlResponse *response, void *client_data) {
        if (response == nullptr) {
        }
        printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, "
               "error_key:%u\n",
               status, handle, response->ret_code, response->error_key);
        std::lock_guard<std::mutex> lock(g_mutex);
        g_sync = true;
        g_cv.notify_one();
      },
      nullptr);
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

  g_sync = false;
  SetLivoxLidarInfoChangeCallback(
      [](const uint32_t handle, const LivoxLidarInfo *info, void *client_data) {
        if (info == nullptr) {
          return;
        }
        std::lock_guard<std::mutex> lock(g_mutex);
        std::cout << "Lidar IP: " << info->lidar_ip << std::endl;
        std::cout << "DevType: " << info->dev_type << std::endl;
        std::cout << "SN: " << info->sn << std::endl;
        std::cout << "handle: " << std::to_string(handle) << std::endl;
        g_connection_handle_ = handle;
        g_cv.notify_one();
        g_sync = true;
      },
      nullptr);

  {
    std::unique_lock<std::mutex> lock(g_mutex);
    g_cv.wait(lock, [] { return g_sync; });
  }

  g_sync = false;
  SetLivoxLidarScanPattern(
      g_connection_handle_,
      LivoxLidarScanPattern::kLivoxLidarScanPatternRepetive,
      [](livox_status status, uint32_t handle,
         LivoxLidarAsyncControlResponse *response, void *client_data) {
        std::lock_guard<std::mutex> lock(g_mutex);

        printf("SetLivoxLidarScanPattern, status:%u, handle:%u, ret_code:%u, "
               "error_key:%u\n",
               status, handle, response->ret_code, response->error_key);
        g_cv.notify_one();
        g_sync = true;
      },
      nullptr);

  {
    std::unique_lock<std::mutex> lock(g_mutex);
    g_cv.wait(lock, [] { return g_sync; });
  }

  SetLivoxLidarImuDataCallback(
      [](const uint32_t handle, const uint8_t dev_type,
         LivoxLidarEthernetPacket *data, void *client_data) {
        if (data == nullptr) {
          return;
        }
        auto *this_ = reinterpret_cast<decltype(this)>(client_data);

        auto *data_ = reinterpret_cast<LivoxLidarImuRawPoint *>(data->data);

        {
          std::lock_guard<std::mutex> lock(g_imu_mutex);
          this_->queue_imu_.push_front({data_->gyro_x, data_->gyro_y,
                                        data_->gyro_z, data_->acc_x,
                                        data_->acc_y, data_->acc_z});
        }
      },
      this);

  // TODO maybe accumulation should be done here.
  SetLivoxLidarPointCloudCallBack(
      [](const uint32_t handle, const uint8_t dev_type,
         LivoxLidarEthernetPacket *data, void *client_data) {
        if (data == nullptr) {
          return;
        }
        auto *this_ = reinterpret_cast<decltype(this)>(client_data);

        const auto cloud = convertData(data, data->dot_num);
        this_->accumulated_.insert(this_->accumulated_.end(), cloud.begin(),
                                   cloud.end());

        if (this_->scan_count_++ % this_->accumulate_scans_ == 0) {
          std::lock_guard<std::mutex> lock(g_lidar_mutex);
          this_->queue_.push_front(this_->accumulated_);

          if (this_->queue_.size() > this_->queue_limit_) {
            this_->queue_.pop_back();
          }

          this_->accumulated_.clear();
        }
      },
      this);
}

PointCloud3 Mid360::getScan() {
  if (queue_.empty()) {
    return {};
  }

  {
    std::lock_guard<std::mutex> lock(g_lidar_mutex);
    const auto last = queue_.front();
    queue_.pop_front();
    return last;
  }
}

std::optional<ImuData> Mid360::getImuSample() {
  if (queue_imu_.empty()) {
    return {};
  }
  {
    std::lock_guard<std::mutex> lock(g_imu_mutex);
    const auto last = queue_imu_.front();
    queue_imu_.pop_front();
    return last;
  }
}