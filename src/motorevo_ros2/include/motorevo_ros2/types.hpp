#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace motorevo_ros2
{

struct Frame
{
  uint32_t id{0};
  bool extended{false};
  bool fd{false};
  bool bitrate_switch{false};
  std::size_t size{0};
  std::array<uint8_t, 64> data{};
};

struct ProtocolLimits
{
  double position_min{-12.5};
  double position_max{12.5};
  double velocity_min{-10.0};
  double velocity_max{10.0};
  double kp_min{0.0};
  double kp_max{250.0};
  double kd_min{0.0};
  double kd_max{50.0};
  double torque_min{-50.0};
  double torque_max{50.0};
};

struct MotorCommand
{
  uint8_t id{1};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double kp{0.0};
  double kd{0.0};
};

struct MotorState
{
  uint8_t id{0};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double temperature{0.0};
  uint16_t status{0};
  bool enabled{false};
  bool valid{false};
  uint64_t sequence{0};
};

enum class StateCommand : uint8_t
{
  kEnable = 0xFC,
  kDisable = 0xFD,
  kSetZero = 0xFE,
  kClearFault = 0xFB,
  kQuery = 0xFA,
};

struct TransportConfig
{
  std::string backend{"meow_usb"};
  std::string device{"/dev/USB2CAN0"};
  std::string library_path{"libusb_fdcan.so"};
  uint8_t channel{1};
  bool can_fd{true};
  bool bitrate_switch{true};
  uint32_t nominal_bitrate{1000000};
  uint32_t data_bitrate{5000000};
};

}  // namespace motorevo_ros2
