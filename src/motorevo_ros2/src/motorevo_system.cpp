#include "motorevo_ros2/motor_bus.hpp"
#include "motorevo_ros2/transport.hpp"

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace motorevo_ros2
{
namespace
{

double as_double(
  const std::unordered_map<std::string, std::string> & values,
  const std::string & key, double fallback)
{
  const auto found = values.find(key);
  return found == values.end() ? fallback : std::stod(found->second);
}

int as_int(
  const std::unordered_map<std::string, std::string> & values,
  const std::string & key, int fallback)
{
  const auto found = values.find(key);
  return found == values.end() ? fallback : std::stoi(found->second);
}

bool as_bool(
  const std::unordered_map<std::string, std::string> & values,
  const std::string & key, bool fallback)
{
  const auto found = values.find(key);
  if (found == values.end()) {return fallback;}
  return found->second == "true" || found->second == "1" || found->second == "yes";
}

std::string as_string(
  const std::unordered_map<std::string, std::string> & values,
  const std::string & key, const std::string & fallback)
{
  const auto found = values.find(key);
  return found == values.end() ? fallback : found->second;
}

}  // namespace

class MotorevoSystem final : public hardware_interface::SystemInterface
{
public:
  ~MotorevoSystem() override
  {
    emergency_disable();
    if (bus_) {bus_->close();}
  }

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    try {
      transport_config_.backend = as_string(info_.hardware_parameters, "backend", "meow_usb");
      transport_config_.device = as_string(info_.hardware_parameters, "device", "/dev/USB2CAN0");
      transport_config_.library_path = as_string(
        info_.hardware_parameters, "library_path", "libusb_fdcan.so");
      transport_config_.channel = static_cast<uint8_t>(
        as_int(info_.hardware_parameters, "channel", 1));
      transport_config_.can_fd = as_bool(info_.hardware_parameters, "can_fd", true);
      transport_config_.bitrate_switch = as_bool(
        info_.hardware_parameters, "bitrate_switch", true);
      transport_config_.nominal_bitrate = static_cast<uint32_t>(
        as_int(info_.hardware_parameters, "nominal_bitrate", 1000000));
      transport_config_.data_bitrate = static_cast<uint32_t>(
        as_int(info_.hardware_parameters, "data_bitrate", 5000000));
      broadcast_mode_ = as_bool(info_.hardware_parameters, "broadcast_mode", true);
      feedback_timeout_ = std::chrono::milliseconds(
        as_int(info_.hardware_parameters, "feedback_timeout_ms", 100));
      halt_on_fault_ = as_bool(info_.hardware_parameters, "halt_on_fault", true);
      inter_frame_delay_us_ = as_int(info_.hardware_parameters, "inter_frame_delay_us", 200);

      limits_.position_min = as_double(info_.hardware_parameters, "position_min", -12.5);
      limits_.position_max = as_double(info_.hardware_parameters, "position_max", 12.5);
      limits_.velocity_min = as_double(info_.hardware_parameters, "velocity_min", -10.0);
      limits_.velocity_max = as_double(info_.hardware_parameters, "velocity_max", 10.0);
      limits_.kp_min = as_double(info_.hardware_parameters, "kp_min", 0.0);
      limits_.kp_max = as_double(info_.hardware_parameters, "kp_max", 250.0);
      limits_.kd_min = as_double(info_.hardware_parameters, "kd_min", 0.0);
      limits_.kd_max = as_double(info_.hardware_parameters, "kd_max", 50.0);
      limits_.torque_min = as_double(info_.hardware_parameters, "torque_min", -50.0);
      limits_.torque_max = as_double(info_.hardware_parameters, "torque_max", 50.0);
      Protocol validation(limits_);
      (void)validation;

      std::unordered_set<int> used_ids;
      joints_.reserve(info_.joints.size());
      for (const auto & joint_info : info_.joints) {
        Joint joint;
        joint.name = joint_info.name;
        const int id = as_int(joint_info.parameters, "motor_id", -1);
        if (id < 1 || id > 255 || !used_ids.insert(id).second) {
          throw std::invalid_argument("each joint needs a unique motor_id in 1..255");
        }
        if (broadcast_mode_ && id > 8) {
          throw std::invalid_argument("broadcast_mode requires motor IDs in 1..8");
        }
        joint.id = static_cast<uint8_t>(id);
        joint.kp = as_double(joint_info.parameters, "kp", 20.0);
        joint.kd = as_double(joint_info.parameters, "kd", 1.0);
        joint.direction = as_double(joint_info.parameters, "direction", 1.0);
        joint.offset = as_double(joint_info.parameters, "offset", 0.0);
        if (joint.direction != 1.0 && joint.direction != -1.0) {
          throw std::invalid_argument("joint direction must be 1 or -1");
        }
        joints_.push_back(std::move(joint));
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(logger_, "Hardware configuration error: %s", error.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (auto & joint : joints_) {
      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.position);
      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity);
      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.effort);
      interfaces.emplace_back(joint.name, "temperature", &joint.temperature);
      interfaces.emplace_back(joint.name, "fault_code", &joint.fault_code);
      interfaces.emplace_back(joint.name, "enabled", &joint.enabled);
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (auto & joint : joints_) {
      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.position_command);
      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity_command);
      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.effort_command);
    }
    return interfaces;
  }

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override
  {
    try {
      bus_ = std::make_unique<MotorBus>(
        make_transport(transport_config_.backend), Protocol(limits_), transport_config_,
        broadcast_mode_, inter_frame_delay_us_);
      bus_->open();
      RCLCPP_INFO(logger_, "Opened %s transport", transport_config_.backend.c_str());
      return hardware_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(logger_, "Failed to open motor bus: %s", error.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &) override
  {
    if (bus_) {bus_->close();}
    bus_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override
  {
    if (!bus_ || !bus_->is_open()) {return hardware_interface::CallbackReturn::ERROR;}
    try {
      std::vector<MotorCommand> commands;
      std::vector<uint8_t> ids;
      commands.reserve(joints_.size());
      ids.reserve(joints_.size());
      for (auto & joint : joints_) {
        ids.push_back(joint.id);
        MotorCommand passive;
        passive.id = joint.id;
        commands.push_back(passive);
      }
      // Ask for feedback while the power stage is still disabled. Activation is rejected
      // if the current position is unknown, which prevents a jump toward zero.
      bus_->send_commands(commands);
      for (int attempt = 0; attempt < 5; ++attempt) {
        bus_->poll(10000);
        bool all_received = true;
        for (const auto & joint : joints_) {
          const auto feedback = bus_->states().find(joint.id);
          all_received = all_received && feedback != bus_->states().end() && feedback->second.valid;
        }
        if (all_received) {break;}
      }
      const auto start = std::chrono::steady_clock::now();
      commands.clear();
      for (auto & joint : joints_) {
        const auto feedback = bus_->states().find(joint.id);
        if (feedback == bus_->states().end() || !feedback->second.valid) {
          throw std::runtime_error(
                  "no feedback from motor " + std::to_string(joint.id) +
                  "; refusing to enable without a known position");
        }
        if (halt_on_fault_ && (feedback->second.status & 0xFFFEu) != 0u) {
          throw std::runtime_error(
                  "motor " + std::to_string(joint.id) + " has an active fault; clear it first");
        }
        joint.position = joint.direction * feedback->second.position + joint.offset;
        joint.velocity = joint.direction * feedback->second.velocity;
        joint.effort = joint.direction * feedback->second.effort;
        joint.position_command = joint.position;
        joint.velocity_command = 0.0;
        joint.effort_command = 0.0;
        joint.last_feedback = start;
        joint.last_sequence = feedback->second.sequence;
        commands.push_back(to_motor_command(joint));
      }
      bus_->send_commands(commands);
      bus_->send_state_command(ids, StateCommand::kEnable);
      active_ = true;
      return hardware_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(logger_, "Activation failed: %s", error.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    try {
      if (bus_ && bus_->is_open()) {
        std::vector<uint8_t> ids;
        for (const auto & joint : joints_) {ids.push_back(joint.id);}
        bus_->send_state_command(ids, StateCommand::kDisable);
      }
      return hardware_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(logger_, "Failed to disable motors: %s", error.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    try {
      bus_->poll(0);
      const auto now = std::chrono::steady_clock::now();
      for (auto & joint : joints_) {
        const auto found = bus_->states().find(joint.id);
        if (found != bus_->states().end() && found->second.valid &&
          found->second.sequence != joint.last_sequence)
        {
          const auto & state = found->second;
          joint.position = joint.direction * state.position + joint.offset;
          joint.velocity = joint.direction * state.velocity;
          joint.effort = joint.direction * state.effort;
          joint.temperature = state.temperature;
          joint.fault_code = static_cast<double>(state.status & 0xFFFEu);
          joint.enabled = state.enabled ? 1.0 : 0.0;
          joint.last_feedback = now;
          joint.last_sequence = state.sequence;
          if (active_ && halt_on_fault_ && joint.fault_code != 0.0) {
            RCLCPP_ERROR(logger_, "Motor %u reported fault 0x%04X", joint.id, state.status);
            emergency_disable();
            return hardware_interface::return_type::ERROR;
          }
        }
        if (active_ && now - joint.last_feedback > feedback_timeout_) {
          RCLCPP_ERROR(logger_, "Feedback timeout for motor %u", joint.id);
          emergency_disable();
          return hardware_interface::return_type::ERROR;
        }
      }
      return hardware_interface::return_type::OK;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(logger_, "Read failed: %s", error.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    try {
      std::vector<MotorCommand> commands;
      commands.reserve(joints_.size());
      for (const auto & joint : joints_) {commands.push_back(to_motor_command(joint));}
      bus_->send_commands(commands);
      return hardware_interface::return_type::OK;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(logger_, "Write failed: %s", error.what());
      return hardware_interface::return_type::ERROR;
    }
  }

private:
  struct Joint
  {
    std::string name;
    uint8_t id{0};
    double kp{20.0};
    double kd{1.0};
    double direction{1.0};
    double offset{0.0};
    double position{std::numeric_limits<double>::quiet_NaN()};
    double velocity{std::numeric_limits<double>::quiet_NaN()};
    double effort{std::numeric_limits<double>::quiet_NaN()};
    double temperature{std::numeric_limits<double>::quiet_NaN()};
    double fault_code{0.0};
    double enabled{0.0};
    double position_command{0.0};
    double velocity_command{0.0};
    double effort_command{0.0};
    std::chrono::steady_clock::time_point last_feedback{};
    uint64_t last_sequence{0};
  };

  MotorCommand to_motor_command(const Joint & joint) const
  {
    MotorCommand command;
    command.id = joint.id;
    command.position = joint.direction * (joint.position_command - joint.offset);
    command.velocity = joint.direction * joint.velocity_command;
    command.effort = joint.direction * joint.effort_command;
    command.kp = joint.kp;
    command.kd = joint.kd;
    return command;
  }

  void emergency_disable() noexcept
  {
    active_ = false;
    if (!bus_ || !bus_->is_open()) {return;}
    try {
      std::vector<uint8_t> ids;
      ids.reserve(joints_.size());
      for (const auto & joint : joints_) {ids.push_back(joint.id);}
      if (!ids.empty()) {bus_->send_state_command(ids, StateCommand::kDisable);}
    } catch (...) {
      // The controller manager already receives ERROR; the motor CAN watchdog is the final stop.
    }
  }

  const rclcpp::Logger logger_{rclcpp::get_logger("MotorevoSystem")};
  TransportConfig transport_config_;
  ProtocolLimits limits_;
  std::unique_ptr<MotorBus> bus_;
  std::vector<Joint> joints_;
  std::chrono::milliseconds feedback_timeout_{100};
  int inter_frame_delay_us_{200};
  bool broadcast_mode_{true};
  bool halt_on_fault_{true};
  bool active_{false};
};

}  // namespace motorevo_ros2

PLUGINLIB_EXPORT_CLASS(motorevo_ros2::MotorevoSystem, hardware_interface::SystemInterface)
