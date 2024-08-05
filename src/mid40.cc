#include "mid40.hh"

#include <iostream>

uint8_t connection_handle_ = 0;

void Mid40::data_callback(uint8_t handle, LivoxEthPacket *data,
                          uint32_t data_num, void *client_data) {}
Mid40::Mid40() {}

void Mid40::startSampling() {
  // SetCartesianCoordinate(
  //     connection_handle_,
  //     [](livox_status status, uint8_t handle, uint8_t response,
  //        void *client_data) {
  //       std::cout << "Set cartesian coordinates " << std::endl;
  //       LidarStartSampling(
  //           connection_handle_,
  //           [](livox_status status, uint8_t handle, uint8_t response,
  //              void *client_data) {
  //             std::cout << "start sampling!" << std::endl;
  //           },
  //           nullptr);
  //     },
  //     nullptr);
}

void Mid40::stopSampling() {}
void Mid40::init() {
  if (!Init()) {
    Uninit();
    std::cout << "Livox-SDK init fail!" << std::endl;
    throw std::runtime_error("Unable to initialize Mid40!");
  }

  std::cout << "Init OK!" << std::endl;
  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d\n", _sdkversion.major, _sdkversion.minor,
         _sdkversion.patch);

  SetBroadcastCallback([](const BroadcastDeviceInfo *info) {
    if (info == nullptr) {
      std::cout << "nullptr" << std::endl;
      return;
    }

    std::cout << "IP: " << info->ip << std::endl;

    uint8_t handle = 0;
    const auto result =
        AddLidarToConnect(info->broadcast_code, &connection_handle_);
    if (result == kStatusSuccess && handle < kMaxLidarCount) {
      std::cout << "Set data callback!" << std::endl;
      SetDataCallback(
          connection_handle_,
          [](uint8_t handle, LivoxEthPacket *data, uint32_t data_num,
             void *client_data) {
            LivoxEthPacket *eth_packet = data;
            if (eth_packet) {
              std::cout << "data!" << std::endl;
            }
          },
          nullptr);

    } else {
      printf("Add lidar to connect is failed : %d %d \n", result, handle);
    }

    if (info->dev_type == kDeviceTypeHub) {
      printf("In lidar mode, couldn't connect a hub : %s\n",
             info->broadcast_code);
      return;
    }
  });

  SetDeviceStateUpdateCallback([](const DeviceInfo *info, DeviceEvent type) {
    if (info == nullptr) {
      return;
    }

    uint8_t handle = info->handle;
    if (handle >= kMaxLidarCount) {
      return;
    }

    if (type == kEventConnect) {
      printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
    } else if (type == kEventDisconnect) {
      printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
    } else if (type == kEventStateChange) {
      printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
    }

    SetCartesianCoordinate(
        connection_handle_,
        [](livox_status status, uint8_t handle, uint8_t response,
           void *client_data) {
          std::cout << "Set cartesian coordinates " << std::endl;
          LidarStartSampling(
              connection_handle_,
              [](livox_status status, uint8_t handle, uint8_t response,
                 void *client_data) {
                std::cout << "start sampling!" << std::endl;
              },
              nullptr);
        },
        nullptr);
  });

  if (!Start()) {
    Uninit();
    throw std::runtime_error("Unable to initialize Mid40!");
  }
}

PointCloud3 Mid40::getScan() {
  PointCloud3 pts;
  return pts;
}