#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace franka_vr
{
class Pi05WatchdogVelocityController final : public controller_interface::ControllerInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= 7; ++i) config.names.push_back("fr3_joint" + std::to_string(i) + "/velocity");
    return config;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  CallbackReturn on_init() override
  {
    try {
      auto_declare<double>("command_timeout_s", 0.05);
      auto_declare<double>("max_velocity_rad_s", 0.6);
      auto_declare<double>("max_acceleration_rad_s2", 2.0);
      auto_declare<double>("max_jerk_rad_s3", 300.0);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_node()->get_logger(), "parameter declaration failed: %s", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    timeout_ = get_node()->get_parameter("command_timeout_s").as_double();
    vmax_ = get_node()->get_parameter("max_velocity_rad_s").as_double();
    amax_ = get_node()->get_parameter("max_acceleration_rad_s2").as_double();
    jmax_ = get_node()->get_parameter("max_jerk_rad_s3").as_double();
    if (!(timeout_ > 0.02 && timeout_ <= 0.10 && vmax_ > 0 && vmax_ <= 0.6 &&
          amax_ > 0 && amax_ <= 2.0 && jmax_ > 0 && jmax_ <= 300.0)) {
      RCLCPP_ERROR(get_node()->get_logger(), "invalid direct velocity safety parameters");
      return CallbackReturn::ERROR;
    }
    command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/commands", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        Command command;
        command.stamp = std::chrono::steady_clock::now();
        // 只接受完整的 7 关节向量；任何缺维、NaN/Inf 或超过 task2 速度包络
        // 的命令都标成无效，update() 随后会把目标设为零并受 jerk 限制减速停车。
        command.valid = msg->data.size() == 7;
        if (command.valid) {
          for (size_t i = 0; i < 7; ++i) {
            const double value = msg->data[i];
            if (!std::isfinite(value) || std::abs(value) > vmax_ + 1e-6) {
              command.valid = false;
              break;
            }
            command.velocity[i] = value;
          }
        }
        commands_.writeFromNonRT(command);
      });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (command_interfaces_.size() != 7) return CallbackReturn::ERROR;
    commands_.writeFromNonRT(Command{});
    velocity_.fill(0.0);
    acceleration_.fill(0.0);
    for (auto & joint : command_interfaces_) joint.set_value(0.0);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    for (auto & joint : command_interfaces_) joint.set_value(0.0);
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration & period) override
  {
    const double dt = period.seconds();
    if (!(dt > 0.0 && dt <= 0.01)) {
      for (auto & joint : command_interfaces_) joint.set_value(0.0);
      velocity_.fill(0.0);
      acceleration_.fill(0.0);
      return controller_interface::return_type::OK;
    }
    const Command * command = commands_.readFromRT();
    const bool fresh = command && command->valid &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - command->stamp).count() <= timeout_;
    for (size_t i = 0; i < 7; ++i) {
      const double target = fresh ? command->velocity[i] : 0.0;
      // Franka 会检查命令速度的加速度和 jerk 连续性。上游目标以 100 Hz
      // 更新，在这里用 1 kHz 的加速度/jerk 限制消除阶梯边沿；目标速度稳定后
      // 仍能到达模型值，不会把 0.253 rad/s 峰值压到旧的 0.15 rad/s 上限。
      const double delta_v = target - velocity_[i];
      double desired_acceleration = std::clamp(delta_v / dt, -amax_, amax_);
      const double braking_envelope = std::sqrt(2.0 * jmax_ * std::abs(delta_v));
      desired_acceleration = delta_v >= 0.0 ?
        std::min(desired_acceleration, braking_envelope) :
        std::max(desired_acceleration, -braking_envelope);
      const double max_acceleration_change = jmax_ * dt;
      acceleration_[i] = std::clamp(
        desired_acceleration,
        std::max(-amax_, acceleration_[i] - max_acceleration_change),
        std::min(amax_, acceleration_[i] + max_acceleration_change));
      velocity_[i] = std::clamp(velocity_[i] + acceleration_[i] * dt, -vmax_, vmax_);
      command_interfaces_[i].set_value(velocity_[i]);
    }
    return controller_interface::return_type::OK;
  }

private:
  struct Command
  {
    std::array<double, 7> velocity{};
    std::chrono::steady_clock::time_point stamp{};
    bool valid{false};
  };

  double timeout_{0.05}, vmax_{0.6}, amax_{2.0}, jmax_{300.0};
  std::array<double, 7> velocity_{}, acceleration_{};
  realtime_tools::RealtimeBuffer<Command> commands_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
};
}  // namespace franka_vr

PLUGINLIB_EXPORT_CLASS(franka_vr::Pi05WatchdogVelocityController, controller_interface::ControllerInterface)
