#pragma once

#include "motorevo_ros2/protocol.hpp"
#include "motorevo_ros2/transport.hpp"

#include <memory>
#include <unordered_map>
#include <vector>

namespace motorevo_ros2
{

class MotorBus
{
public:
  MotorBus(
    std::unique_ptr<Transport> transport, Protocol protocol,
    TransportConfig transport_config, bool broadcast_mode = true,
    int inter_frame_delay_us = 200);
  ~MotorBus();

  void open();
  void close() noexcept;
  void send_commands(const std::vector<MotorCommand> & commands);
  void send_state_command(const std::vector<uint8_t> & motor_ids, StateCommand command);
  std::size_t poll(int timeout_us = 0, std::size_t max_frames = 64);
  const std::unordered_map<uint8_t, MotorState> & states() const noexcept {return states_;}
  bool is_open() const noexcept;

private:
  std::unique_ptr<Transport> transport_;
  Protocol protocol_;
  TransportConfig transport_config_;
  bool broadcast_mode_;
  int inter_frame_delay_us_;
  uint64_t receive_sequence_{0};
  std::unordered_map<uint8_t, MotorState> states_;
};

}  // namespace motorevo_ros2
