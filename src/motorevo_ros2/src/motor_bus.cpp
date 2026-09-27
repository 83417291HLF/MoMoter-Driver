#include "motorevo_ros2/motor_bus.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

namespace motorevo_ros2
{

MotorBus::MotorBus(
  std::unique_ptr<Transport> transport, Protocol protocol,
  TransportConfig transport_config, bool broadcast_mode, int inter_frame_delay_us)
: transport_(std::move(transport)),
  protocol_(std::move(protocol)),
  transport_config_(std::move(transport_config)),
  broadcast_mode_(broadcast_mode),
  inter_frame_delay_us_(inter_frame_delay_us)
{
  if (!transport_) {
    throw std::invalid_argument("transport must not be null");
  }
}

MotorBus::~MotorBus()
{
  close();
}

void MotorBus::open()
{
  transport_->open(transport_config_);
}

void MotorBus::close() noexcept
{
  if (transport_) {
    transport_->close();
  }
}

void MotorBus::send_commands(const std::vector<MotorCommand> & commands)
{
  if (broadcast_mode_) {
    auto frame = protocol_.make_broadcast_control(commands);
    frame.bitrate_switch = transport_config_.bitrate_switch;
    transport_->send(frame);
    return;
  }
  for (std::size_t i = 0; i < commands.size(); ++i) {
    transport_->send(protocol_.make_individual_control(
      commands[i], transport_config_.can_fd, transport_config_.bitrate_switch));
    if (i + 1 < commands.size() && inter_frame_delay_us_ > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(inter_frame_delay_us_));
    }
  }
}

void MotorBus::send_state_command(
  const std::vector<uint8_t> & motor_ids, StateCommand command)
{
  if (broadcast_mode_) {
    auto frame = protocol_.make_broadcast_state(motor_ids, command);
    frame.bitrate_switch = transport_config_.bitrate_switch;
    transport_->send(frame);
    return;
  }
  for (std::size_t i = 0; i < motor_ids.size(); ++i) {
    transport_->send(protocol_.make_individual_state(
      motor_ids[i], command, transport_config_.can_fd, transport_config_.bitrate_switch));
    if (i + 1 < motor_ids.size() && inter_frame_delay_us_ > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(inter_frame_delay_us_));
    }
  }
}

std::size_t MotorBus::poll(int timeout_us, std::size_t max_frames)
{
  std::size_t count = 0;
  Frame frame;
  while (count < max_frames && transport_->receive(frame, count == 0 ? timeout_us : 0)) {
    const auto state = protocol_.unpack_feedback(frame);
    if (state) {
      auto timestamped = *state;
      timestamped.sequence = ++receive_sequence_;
      states_[timestamped.id] = timestamped;
    }
    ++count;
  }
  return count;
}

bool MotorBus::is_open() const noexcept
{
  return transport_ && transport_->is_open();
}

}  // namespace motorevo_ros2
