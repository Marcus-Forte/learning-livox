#pragma once

#include "ILidar.hh"
#include "livox_sdk.h"
#include <thread>

class Mid40 : public ILidar {

public:
  Mid40();
  void init() override;
  PointCloud3 getScan() override;
  void startSampling() override;
  void stopSampling() override;

private:
  std::thread worker_;
  void data_callback(uint8_t handle, LivoxEthPacket *data, uint32_t data_num,
                     void *client_data);
  // uint8_t connection_handle_;
};