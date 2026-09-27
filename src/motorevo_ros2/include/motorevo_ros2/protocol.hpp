#pragma once

#include "motorevo_ros2/types.hpp"

#include <array>
#include <optional>
#include <vector>

namespace motorevo_ros2
{

class Protocol
{
public:
  explicit Protocol(ProtocolLimits limits = {});

  const ProtocolLimits & limits() const noexcept {return limits_;}
  std::array<uint8_t, 8> pack_mit(const MotorCommand & command) const;
  std::optional<MotorState> unpack_feedback(const Frame & frame) const;
  Frame make_broadcast_control(const std::vector<MotorCommand> & commands) const;
  Frame make_individual_control(const MotorCommand & command, bool fd, bool brs) const;
  Frame make_broadcast_state(
    const std::vector<uint8_t> & motor_ids, StateCommand command) const;
  Frame make_individual_state(
    uint8_t motor_id, StateCommand command, bool fd, bool brs) const;

private:
  ProtocolLimits limits_;
};

}  // namespace motorevo_ros2

