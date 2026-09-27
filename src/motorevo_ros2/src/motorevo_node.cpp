#include "motorevo_ros2/motor_bus.hpp"
#include "motorevo_ros2/transport.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace motorevo_ros2
{

class MotorNode final : public rclcpp::Node
{
public:
  MotorNode()
  : Node("motorevo_driver")
  {
    const auto joint_names = declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint1"});
    const auto ids = declare_parameter<std::vector<int64_t>>("motor_ids", {1});
    const auto kp = declare_parameter<std::vector<double>>("kp", {20.0});
    const auto kd = declare_parameter<std::vector<double>>("kd", {1.0});
    if (joint_names.empty() || joint_names.size() != ids.size()) {
      throw std::invalid_argument("joint_names and motor_ids must be non-empty and have equal length");
    }
    joint_names_ = joint_names;
    commands_.resize(ids.size());
    motor_ids_.reserve(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] < 1 || ids[i] > 255) {throw std::invalid_argument("motor_ids must be 1..255");}
      const uint8_t id = static_cast<uint8_t>(ids[i]);
      if (std::find(motor_ids_.begin(), motor_ids_.end(), id) != motor_ids_.end()) {
        throw std::invalid_argument("motor_ids must be unique");
      }
      motor_ids_.push_back(id);
      commands_[i].id = id;
      commands_[i].kp = value_at(kp, i, "kp");
      commands_[i].kd = value_at(kd, i, "kd");
      id_to_index_[id] = i;
      name_to_index_[joint_names_[i]] = i;
    }

    ProtocolLimits limits;
    limits.position_min = declare_parameter("limits.position_min", -12.5);
    limits.position_max = declare_parameter("limits.position_max", 12.5);
    limits.velocity_min = declare_parameter("limits.velocity_min", -10.0);
    limits.velocity_max = declare_parameter("limits.velocity_max", 10.0);
    limits.kp_min = declare_parameter("limits.kp_min", 0.0);
    limits.kp_max = declare_parameter("limits.kp_max", 250.0);
    limits.kd_min = declare_parameter("limits.kd_min", 0.0);
    limits.kd_max = declare_parameter("limits.kd_max", 50.0);
    limits.torque_min = declare_parameter("limits.torque_min", -50.0);
    limits.torque_max = declare_parameter("limits.torque_max", 50.0);

    TransportConfig config;
    config.backend = declare_parameter("transport.backend", "meow_usb");
    config.device = declare_parameter("transport.device", "/dev/USB2CAN0");
    config.library_path = declare_parameter("transport.library_path", "libusb_fdcan.so");
    const auto channel = declare_parameter<int64_t>("transport.channel", 1);
    if (channel < 1 || channel > 2) {throw std::invalid_argument("transport.channel must be 1 or 2");}
    config.channel = static_cast<uint8_t>(channel);
    config.can_fd = declare_parameter("transport.can_fd", true);
    config.bitrate_switch = declare_parameter("transport.bitrate_switch", true);
    config.nominal_bitrate = static_cast<uint32_t>(
      declare_parameter<int64_t>("transport.nominal_bitrate", 1000000));
    config.data_bitrate = static_cast<uint32_t>(
      declare_parameter<int64_t>("transport.data_bitrate", 5000000));
    const bool broadcast = declare_parameter("broadcast_mode", true);
    const int delay_us = declare_parameter("inter_frame_delay_us", 200);
    command_timeout_ = std::chrono::milliseconds(
      declare_parameter<int64_t>("command_timeout_ms", 100));
    disable_on_timeout_ = declare_parameter("disable_on_timeout", true);
    const double update_rate = declare_parameter("update_rate", 500.0);
    if (update_rate <= 0.0) {throw std::invalid_argument("update_rate must be positive");}

    bus_ = std::make_unique<MotorBus>(
      make_transport(config.backend), Protocol(limits), config, broadcast, delay_us);
    bus_->open();

    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "~/command", rclcpp::SensorDataQoS(),
      std::bind(&MotorNode::on_command, this, std::placeholders::_1));
    state_pub_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 10);
    enable_srv_ = service("~/enable", StateCommand::kEnable);
    disable_srv_ = service("~/disable", StateCommand::kDisable);
    clear_fault_srv_ = service("~/clear_fault", StateCommand::kClearFault);
    set_zero_srv_ = service("~/set_zero", StateCommand::kSetZero);

    last_command_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate), std::bind(&MotorNode::update, this));
    RCLCPP_INFO(
      get_logger(), "Opened %s backend for %zu motor(s); motors remain disabled",
      config.backend.c_str(), motor_ids_.size());
  }

  ~MotorNode() override
  {
    try {
      if (bus_ && bus_->is_open()) {bus_->send_state_command(motor_ids_, StateCommand::kDisable);}
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to disable motors during shutdown: %s", error.what());
    }
  }

private:
  static double value_at(const std::vector<double> & values, std::size_t index, const char * name)
  {
    if (values.size() == 1) {return values.front();}
    if (index < values.size()) {return values[index];}
    throw std::invalid_argument(std::string(name) + " must contain one value or one value per joint");
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service(
    const std::string & name, StateCommand command)
  {
    return create_service<std_srvs::srv::Trigger>(
      name, [this, command](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        try {
          bus_->send_state_command(motor_ids_, command);
          if (command == StateCommand::kEnable) {
            enabled_ = true;
            timed_out_ = false;
            last_command_ = now();
          } else if (command == StateCommand::kDisable) {
            enabled_ = false;
          }
          response->success = true;
          response->message = "command sent";
        } catch (const std::exception & error) {
          response->success = false;
          response->message = error.what();
        }
      });
  }

  void on_command(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    for (std::size_t source = 0; source < message->name.size(); ++source) {
      const auto found = name_to_index_.find(message->name[source]);
      if (found == name_to_index_.end()) {continue;}
      const std::size_t target = found->second;
      if (source < message->position.size()) {commands_[target].position = message->position[source];}
      if (source < message->velocity.size()) {commands_[target].velocity = message->velocity[source];}
      if (source < message->effort.size()) {commands_[target].effort = message->effort[source];}
    }
    last_command_ = now();
    timed_out_ = false;
  }

  void update()
  {
    try {
      bus_->poll(0);
      if (enabled_) {
        const bool stale = (now() - last_command_).nanoseconds() >
          std::chrono::duration_cast<std::chrono::nanoseconds>(command_timeout_).count();
        if (stale && disable_on_timeout_) {
          if (!timed_out_) {
            bus_->send_state_command(motor_ids_, StateCommand::kDisable);
            RCLCPP_ERROR(get_logger(), "Command timeout: motors disabled");
          }
          timed_out_ = true;
          enabled_ = false;
        } else {
          bus_->send_commands(commands_);
        }
      }
      publish_state();
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Bus error: %s", error.what());
    }
  }

  void publish_state()
  {
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = now();
    joints.name = joint_names_;
    joints.position.resize(joint_names_.size(), 0.0);
    joints.velocity.resize(joint_names_.size(), 0.0);
    joints.effort.resize(joint_names_.size(), 0.0);

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = joints.header.stamp;
    for (const auto & [id, state] : bus_->states()) {
      const auto index = id_to_index_.find(id);
      if (index == id_to_index_.end()) {continue;}
      joints.position[index->second] = state.position;
      joints.velocity[index->second] = state.velocity;
      joints.effort[index->second] = state.effort;

      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "motorevo/" + joint_names_[index->second];
      status.hardware_id = "motor_" + std::to_string(id);
      const uint16_t faults = state.status & 0xFFFEu;
      status.level = faults ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
        diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = faults ? "motor fault" : (state.enabled ? "enabled" : "disabled");
      diagnostic_msgs::msg::KeyValue temperature;
      temperature.key = "temperature_c";
      temperature.value = std::to_string(state.temperature);
      status.values.push_back(std::move(temperature));
      diagnostic_msgs::msg::KeyValue status_value;
      status_value.key = "status";
      status_value.value = std::to_string(state.status);
      status.values.push_back(std::move(status_value));
      diagnostics.status.push_back(std::move(status));
    }
    state_pub_->publish(joints);
    diagnostics_pub_->publish(diagnostics);
  }

  std::vector<std::string> joint_names_;
  std::vector<uint8_t> motor_ids_;
  std::vector<MotorCommand> commands_;
  std::unordered_map<uint8_t, std::size_t> id_to_index_;
  std::unordered_map<std::string, std::size_t> name_to_index_;
  std::unique_ptr<MotorBus> bus_;
  bool enabled_{false};
  bool timed_out_{false};
  bool disable_on_timeout_{true};
  std::chrono::milliseconds command_timeout_{100};
  rclcpp::Time last_command_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_zero_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace motorevo_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<motorevo_ros2::MotorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("motorevo_driver"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
