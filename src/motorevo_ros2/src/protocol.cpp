#include "motorevo_ros2/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motorevo_ros2
{
namespace
{

void validate_range(double low, double high, const char * name)
{
  if (!std::isfinite(low) || !std::isfinite(high) || low >= high) {
    throw std::invalid_argument(std::string("invalid protocol range: ") + name);
  }
}

uint32_t encode(double value, double low, double high, unsigned bits)
{
  if (!std::isfinite(value)) {
    value = 0.0;
  }
  const double clamped = std::clamp(value, low, high);
  const uint32_t full_scale = (1u << bits) - 1u;
  return static_cast<uint32_t>((clamped - low) * full_scale / (high - low));
}

double decode(uint32_t value, double low, double high, unsigned bits)
{
  const uint32_t full_scale = (1u << bits) - 1u;
  return low + static_cast<double>(value) * (high - low) / full_scale;
}

void validate_id(uint8_t id, bool broadcast)
{
  if (id == 0 || (broadcast && id > 8)) {
    throw std::out_of_range(broadcast ? "broadcast motor ID must be 1..8" : "motor ID must be 1..255");
  }
}

}  // namespace

Protocol::Protocol(ProtocolLimits limits)
: limits_(limits)
{
  validate_range(limits_.position_min, limits_.position_max, "position");
  validate_range(limits_.velocity_min, limits_.velocity_max, "velocity");
  validate_range(limits_.kp_min, limits_.kp_max, "kp");
  validate_range(limits_.kd_min, limits_.kd_max, "kd");
  validate_range(limits_.torque_min, limits_.torque_max, "torque");
}

std::array<uint8_t, 8> Protocol::pack_mit(const MotorCommand & command) const
{
  const uint32_t p = encode(command.position, limits_.position_min, limits_.position_max, 16);
  const uint32_t v = encode(command.velocity, limits_.velocity_min, limits_.velocity_max, 12);
  const uint32_t kp = encode(command.kp, limits_.kp_min, limits_.kp_max, 12);
  const uint32_t kd = encode(command.kd, limits_.kd_min, limits_.kd_max, 12);
  const uint32_t t = encode(command.effort, limits_.torque_min, limits_.torque_max, 12);

  return {
    static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p),
    static_cast<uint8_t>(v >> 4), static_cast<uint8_t>(((v & 0x0F) << 4) | (kp >> 8)),
    static_cast<uint8_t>(kp), static_cast<uint8_t>(kd >> 4),
    static_cast<uint8_t>(((kd & 0x0F) << 4) | (t >> 8)), static_cast<uint8_t>(t)};
}

std::optional<MotorState> Protocol::unpack_feedback(const Frame & frame) const
{
  if (frame.extended || frame.size != 8 || frame.id == 0 || frame.id > 255) {
    return std::nullopt;
  }

  const uint32_t p = (static_cast<uint32_t>(frame.data[0]) << 8) | frame.data[1];
  const uint32_t v = (static_cast<uint32_t>(frame.data[2]) << 4) | (frame.data[3] >> 4);
  const uint32_t t = (static_cast<uint32_t>(frame.data[3] & 0x0F) << 8) | frame.data[4];
  const uint16_t status =
    static_cast<uint16_t>((static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7]);

  MotorState state;
  state.id = static_cast<uint8_t>(frame.id);
  state.position = decode(p, limits_.position_min, limits_.position_max, 16);
  state.velocity = decode(v, limits_.velocity_min, limits_.velocity_max, 12);
  state.effort = decode(t, limits_.torque_min, limits_.torque_max, 12);
  state.temperature = static_cast<double>(frame.data[5]) - 40.0;
  state.status = status;
  state.enabled = (status & 0x0001u) != 0u;
  state.valid = true;
  return state;
}

Frame Protocol::make_broadcast_control(const std::vector<MotorCommand> & commands) const
{
  Frame frame;
  frame.id = 0x20;
  frame.fd = true;
  frame.bitrate_switch = true;
  frame.size = 64;

  MotorCommand neutral;
  for (uint8_t id = 1; id <= 8; ++id) {
    neutral.id = id;
    const auto bytes = pack_mit(neutral);
    std::copy(bytes.begin(), bytes.end(), frame.data.begin() + (id - 1) * 8);
  }
  for (const auto & command : commands) {
    validate_id(command.id, true);
    const auto bytes = pack_mit(command);
    std::copy(bytes.begin(), bytes.end(), frame.data.begin() + (command.id - 1) * 8);
  }
  return frame;
}

Frame Protocol::make_individual_control(
  const MotorCommand & command, bool fd, bool brs) const
{
  validate_id(command.id, false);
  Frame frame;
  frame.id = 0x200u + command.id;
  frame.fd = fd;
  frame.bitrate_switch = fd && brs;
  frame.size = 8;
  const auto bytes = pack_mit(command);
  std::copy(bytes.begin(), bytes.end(), frame.data.begin());
  return frame;
}

Frame Protocol::make_broadcast_state(
  const std::vector<uint8_t> & motor_ids, StateCommand command) const
{
  Frame frame;
  frame.id = 0x10;
  frame.fd = true;
  frame.bitrate_switch = true;
  frame.size = 64;
  frame.data.fill(0xFF);
  for (uint8_t id = 1; id <= 8; ++id) {
    frame.data[(id - 1) * 8 + 7] = static_cast<uint8_t>(StateCommand::kQuery);
  }
  for (const uint8_t id : motor_ids) {
    validate_id(id, true);
    frame.data[(id - 1) * 8 + 7] = static_cast<uint8_t>(command);
  }
  return frame;
}

Frame Protocol::make_individual_state(
  uint8_t motor_id, StateCommand command, bool fd, bool brs) const
{
  validate_id(motor_id, false);
  Frame frame;
  frame.id = 0x100u + motor_id;
  frame.fd = fd;
  frame.bitrate_switch = fd && brs;
  frame.size = 8;
  frame.data.fill(0xFF);
  frame.data[7] = static_cast<uint8_t>(command);
  return frame;
}

}  // namespace motorevo_ros2
