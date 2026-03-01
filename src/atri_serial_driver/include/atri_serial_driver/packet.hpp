// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#ifndef ATRI_SERIAL_DRIVER__PACKET_HPP_
#define ATRI_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace atri_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  float roll;
  float pitch;
  float yaw;
  uint32_t timestamp;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  uint8_t state : 1;  // 0-untracking 1-tracking
  float yaw;
  float pitch;
  uint32_t cap_timestamp;
  uint16_t t_offset;
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace atri_serial_driver

#endif  // ATRI_SERIAL_DRIVER__PACKET_HPP_