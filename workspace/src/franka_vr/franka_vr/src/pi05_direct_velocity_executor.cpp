#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "control_msgs/msg/joint_jog.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr size_t N = 7;
const std::array<std::string, N> JOINTS = {
  "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
const std::array<double, N> LOWER = {-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159};
const std::array<double, N> UPPER = {2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159};

bool is_finite(double value) { return std::isfinite(value); }

template<typename T>
std::string join(const T& values)
{
  std::ostringstream stream;
  stream << std::setprecision(9);
  for (size_t i = 0; i < values.size(); ++i) stream << (i ? ";" : "") << values[i];
  return stream.str();
}
}  // namespace

class DirectVelocityExecutor final : public rclcpp::Node
{
public:
  DirectVelocityExecutor() : Node("pi05_direct_velocity_executor"), steady_clock_(RCL_STEADY_TIME)
  {
    armed_ = declare_parameter("armed", false);
    rate_ = declare_parameter("control_rate_hz", 100.0);
    timeout_ = declare_parameter("command_timeout_s", 0.12);
    max_command_stamp_age_ = declare_parameter("max_command_stamp_age_s", 0.20);
    max_command_future_skew_ = declare_parameter("max_command_future_skew_s", 0.05);
    feedback_timeout_ = declare_parameter("max_feedback_age_s", 0.10);
    collision_timeout_ = declare_parameter("collision_monitor_timeout_s", 0.15);
    vmax_ = declare_parameter("max_joint_velocity_rad_s", 0.6);
    amax_ = declare_parameter("max_joint_acceleration_rad_s2", 2.0);
    margin_ = declare_parameter("joint_limit_margin_rad", 0.15);
    soft_error_ = declare_parameter("soft_tracking_error_rad", 0.03);
    hard_error_ = declare_parameter("hard_tracking_error_rad", 0.08);
    max_deadline_misses_ = declare_parameter("max_consecutive_deadline_misses", 3);
    log_path_ = declare_parameter("csv_log_path", std::string(""));
    if (rate_ < 50 || rate_ > 250 || timeout_ > 0.12 || timeout_ <= 0 || max_command_stamp_age_ <= 0 ||
        max_command_future_skew_ < 0 || vmax_ <= 0 || amax_ <= 0 ||
      soft_error_ <= 0 || hard_error_ <= soft_error_) {
      throw std::runtime_error("invalid or non-conservative direct executor parameters");
    }

    cmd_sub_ = create_subscription<control_msgs::msg::JointJog>(
      "/pi05/direct_joint_velocity_cmd", rclcpp::SystemDefaultsQoS(),
      std::bind(&DirectVelocityExecutor::command, this, std::placeholders::_1));
    auto sensor_qos = rclcpp::SensorDataQoS();
    feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/franka/joint_states", sensor_qos,
      [this](sensor_msgs::msg::JointState::SharedPtr message) { feedback(message, true); });
    fallback_feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", sensor_qos,
      [this](sensor_msgs::msg::JointState::SharedPtr message) { feedback(message, false); });
    collision_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/pi05/collision_stop", 10, std::bind(&DirectVelocityExecutor::collision, this, std::placeholders::_1));
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/pi05/direct_executor/status", 10);
    commanded_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/pi05/direct_executor/commanded_qdot", 10);
    guarded_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/pi05/direct_executor/guarded_qdot", 10);
    // Dry-run cannot publish to the hardware command controller.
    if (armed_) {
      velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/fr3_direct_velocity_controller/commands", rclcpp::SystemDefaultsQoS());
    }
    if (!log_path_.empty()) {
      csv_.open(log_path_, std::ios::out | std::ios::app);
      if (!csv_) throw std::runtime_error("cannot open executor CSV log: " + log_path_);
      csv_ << "steady_s,state,fault,command_age_s,target_qdot,guarded_qdot,actual_qdot,actual_position,commanded_position,tracking_error\n";
    }
    last_tick_ = steady_clock_.now();
    started_ = last_tick_;
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_), std::bind(&DirectVelocityExecutor::tick, this));
    RCLCPP_WARN(get_logger(), "direct executor is %s; velocity publisher %s", armed_ ? "ARMED" : "DISARMED",
                armed_ ? "created" : "NOT created");
  }

private:
  bool controller_subscriber_ready()
  {
    for (const auto & endpoint : get_subscriptions_info_by_topic(
           "/fr3_direct_velocity_controller/commands")) {
      if (endpoint.node_name() == "fr3_direct_velocity_controller") return true;
    }
    return false;
  }

  void latch(const std::string& reason)
  {
    if (!fault_) RCLCPP_ERROR(get_logger(), "FAULT LATCHED: %s; restart required", reason.c_str());
    fault_ = true;
    fault_reason_ = reason;
    target_.fill(0.0);
  }

  void command(const control_msgs::msg::JointJog::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fault_) return;
    if (message->joint_names.size() != N || message->velocities.size() != N ||
        !std::equal(message->joint_names.begin(), message->joint_names.end(), JOINTS.begin())) {
      latch("invalid joint names, order, or command dimension");
      return;
    }
    bool stop_only = true;
    for (size_t i = 0; i < N; ++i) {
      if (!is_finite(message->velocities[i])) { latch("non-finite velocity command"); return; }
      stop_only = stop_only && message->velocities[i] == 0.0;
    }
    // An unstamped all-zero command remains a valid fail-safe stop request.
    // Any command capable of motion must carry a fresh ROS timestamp.
    if (!stop_only) {
      if (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0) {
        latch("missing velocity command timestamp"); return;
      }
      const double stamp_age = (now() - rclcpp::Time(message->header.stamp)).seconds();
      if (!is_finite(stamp_age) || stamp_age > max_command_stamp_age_ || stamp_age < -max_command_future_skew_) {
        latch("stale or future velocity command timestamp"); return;
      }
    }
    for (size_t i = 0; i < N; ++i) target_[i] = message->velocities[i];
    last_command_ = steady_clock_.now();
    have_command_ = true;
    command_is_stop_ = stop_only;
  }

  void feedback(const sensor_msgs::msg::JointState::SharedPtr message, bool primary)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!primary && (primary_feedback_seen_ || (steady_clock_.now() - started_).seconds() < 2.0)) return;
    std::unordered_map<std::string, size_t> index;
    for (size_t i = 0; i < message->name.size(); ++i) {
      if (!index.emplace(message->name[i], i).second &&
          std::find(JOINTS.begin(), JOINTS.end(), message->name[i]) != JOINTS.end()) {
        latch("duplicate joint name in feedback"); return;
      }
    }
    std::array<double, N> position{}, velocity{};
    for (size_t i = 0; i < N; ++i) {
      auto found = index.find(JOINTS[i]);
      if (found == index.end() || found->second >= message->position.size()) return;
      const size_t j = found->second;
      position[i] = message->position[j];
      velocity[i] = j < message->velocity.size() ? message->velocity[j] : 0.0;
      if (!is_finite(position[i]) || !is_finite(velocity[i])) { latch("non-finite joint feedback"); return; }
    }
    const bool switch_to_primary = primary && !primary_feedback_seen_;
    primary_feedback_seen_ = primary_feedback_seen_ || primary;
    actual_position_ = position;
    actual_velocity_ = velocity;
    last_feedback_ = steady_clock_.now();
    if (!have_feedback_ || (switch_to_primary && !have_command_)) {
      commanded_position_ = actual_position_;
      guarded_.fill(0.0); acceleration_.fill(0.0);
      have_feedback_ = true;
    }
  }

  void collision(const std_msgs::msg::Bool::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_collision_heartbeat_ = steady_clock_.now();
    have_collision_heartbeat_ = true;
    if (message->data) latch("collision monitor stop");
  }

  void tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = steady_clock_.now();
    double dt = (now - last_tick_).seconds();
    last_tick_ = now;
    if (dt <= 0 || dt > 0.1) { latch("invalid control period"); dt = 1.0 / rate_; }
    if (dt > 2.5 / rate_) {
      if (++deadline_misses_ >= max_deadline_misses_) latch("consecutive control deadline misses");
    } else deadline_misses_ = 0;

    // ros2_control controller discovery/activation can occasionally take more
    // than 10 seconds on the real robot. Keep runtime watchdogs strict, but
    // allow the one-time startup handshake enough time to finish.
    const bool startup_expired = (now - started_).seconds() > 30.0;
    const bool dependencies_ready = armed_ && have_feedback_ && have_collision_heartbeat_ && velocity_pub_ &&
      controller_subscriber_ready();
    if (!runtime_ready_ && dependencies_ready) {
      runtime_ready_ = true;
      commanded_position_ = actual_position_;
      guarded_.fill(0.0);
      acceleration_.fill(0.0);
      RCLCPP_INFO(get_logger(), "armed dependencies ready; strict runtime watchdogs enabled");
    }
    if (armed_ && !runtime_ready_ && startup_expired) latch("armed dependencies did not become ready");
    if (runtime_ready_ && (now - last_feedback_).seconds() > feedback_timeout_) latch("joint feedback timeout");
    if (runtime_ready_ && (now - last_collision_heartbeat_).seconds() > collision_timeout_)
      latch("collision monitor heartbeat timeout");
    if (runtime_ready_ && !controller_subscriber_ready())
      latch("velocity controller subscriber disappeared");
    // Arm one nominal control period early so a 100 Hz sampling boundary still
    // guarantees entry into limited stopping no later than the 120 ms contract.
    const double sampled_watchdog = std::max(0.0, timeout_ - 1.0 / rate_);
    // A fresh nonzero command must be continuously supervised. An explicit
    // all-zero command is already the fail-safe parked state, so silence after
    // that stop request must not convert an orderly horizon/completion stop
    // into a latched timeout fault.
    if (have_command_ && !command_is_stop_ && (now - last_command_).seconds() >= sampled_watchdog)
      latch("policy command timeout");
    if (!armed_) target_.fill(0.0);
    if (!have_feedback_) {
      if (armed_ && velocity_pub_) publish_velocity();
      publish_status(now);
      return;
    }

    if (fault_) {
      guarded_.fill(0.0);
      acceleration_.fill(0.0);
      commanded_position_ = actual_position_;
      publish_arrays();
      if (armed_ && velocity_pub_) publish_velocity();
      publish_status(now);
      log(now, 0.0);
      return;
    }

    double max_tracking = 0;
    for (size_t i = 0; i < N; ++i) max_tracking = std::max(max_tracking, std::abs(commanded_position_[i] - actual_position_[i]));
    if (max_tracking > hard_error_) latch("hard tracking error");
    double tracking_scale = max_tracking <= soft_error_ ? 1.0 :
      std::clamp((hard_error_ - max_tracking) / (hard_error_ - soft_error_), 0.0, 1.0);

    commanded_target_ = target_;
    for (size_t i = 0; i < N; ++i) {
      // 使用 task2 配置的 0.6 rad/s 速度包络和 2.0 rad/s² 单层加速度保护。
      // 不再施加 jerk 限制；底层 1 kHz 控制器只负责新鲜度检查与速度写入。
      const double effective_vmax = std::max(0.0, vmax_ - amax_ * dt);
      const double desired_v = fault_ ? 0.0 : std::clamp(target_[i], -effective_vmax, effective_vmax) * tracking_scale;
      double desired_a = std::clamp((desired_v - guarded_[i]) / dt, -amax_, amax_);
      double next_v = guarded_[i] + desired_a * dt;
      if ((next_v - desired_v) * (guarded_[i] - desired_v) <= 0) next_v = desired_v;
      const double low = LOWER[i] + margin_, high = UPPER[i] - margin_;
      if (actual_position_[i] <= low || actual_position_[i] >= high) latch("feedback inside joint-limit margin");
      const double predicted = commanded_position_[i] + next_v * dt;
      if ((predicted < low && next_v < 0) || (predicted > high && next_v > 0)) {
        next_v = 0; acceleration_[i] = 0;
      } else acceleration_[i] = desired_a;
      guarded_[i] = next_v;
      commanded_position_[i] = std::clamp(commanded_position_[i] + guarded_[i] * dt, low, high);
    }
    if (fault_) guarded_.fill(0.0);

    publish_arrays();
    if (armed_ && velocity_pub_) publish_velocity();
    publish_status(now);
    log(now, max_tracking);
  }

  void publish_arrays()
  {
    std_msgs::msg::Float64MultiArray commanded, guarded;
    commanded.data.assign(commanded_target_.begin(), commanded_target_.end());
    guarded.data.assign(guarded_.begin(), guarded_.end());
    commanded_pub_->publish(commanded); guarded_pub_->publish(guarded);
  }

  void publish_velocity()
  {
    std_msgs::msg::Float64MultiArray command;
    command.data.assign(N, 0.0);
    if (runtime_ready_ && !fault_) {
      std::copy(guarded_.begin(), guarded_.end(), command.data.begin());
    }
    velocity_pub_->publish(command);
  }

  void publish_status(const rclcpp::Time& stamp)
  {
    diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "pi05_direct_velocity_executor"; status.hardware_id = "fr3";
    status.level = fault_ ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      (armed_ ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN);
    status.message = fault_ ? fault_reason_ :
      (armed_ ? (runtime_ready_ ? "armed" : "armed_waiting_for_dependencies") : "disarmed_no_velocity_publisher");
    diagnostic_msgs::msg::KeyValue armed; armed.key = "armed"; armed.value = armed_ ? "true" : "false";
    diagnostic_msgs::msg::KeyValue publisher; publisher.key = "velocity_publisher_created";
    publisher.value = velocity_pub_ ? "true" : "false";
    status.values = {armed, publisher}; array.status.push_back(status); status_pub_->publish(array);
  }

  void log(const rclcpp::Time& stamp, double tracking)
  {
    if (!csv_) return;
    csv_ << std::setprecision(9) << stamp.seconds() << ',' << (fault_ ? "fault" : (armed_ ? "armed" : "disarmed")) << ','
         << fault_reason_ << ',' << (have_command_ ? (steady_clock_.now() - last_command_).seconds() : -1.0)
         << ',' << join(commanded_target_) << ',' << join(guarded_) << ',' << join(actual_velocity_)
         << ',' << join(actual_position_) << ',' << join(commanded_position_) << ',' << tracking << '\n';
    csv_.flush();
  }

  std::mutex mutex_;
  rclcpp::Clock steady_clock_;
  bool armed_{false}, fault_{false}, have_command_{false}, have_feedback_{false}, have_collision_heartbeat_{false},
    primary_feedback_seen_{false}, runtime_ready_{false}, command_is_stop_{true};
  std::string fault_reason_, log_path_;
  double rate_, timeout_, max_command_stamp_age_, max_command_future_skew_, feedback_timeout_, collision_timeout_,
    vmax_, amax_, margin_, soft_error_, hard_error_;
  int max_deadline_misses_, deadline_misses_{0};
  std::array<double, N> target_{}, commanded_target_{}, guarded_{}, acceleration_{}, actual_position_{},
    actual_velocity_{}, commanded_position_{};
  rclcpp::Time last_tick_{0, 0, RCL_STEADY_TIME}, last_command_{0, 0, RCL_STEADY_TIME},
    last_feedback_{0, 0, RCL_STEADY_TIME}, last_collision_heartbeat_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time started_{0, 0, RCL_STEADY_TIME};
  std::ofstream csv_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_, fallback_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr commanded_pub_, guarded_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<DirectVelocityExecutor>()); }
  catch (const std::exception& error) { RCLCPP_FATAL(rclcpp::get_logger("pi05_direct_executor"), "%s", error.what()); }
  rclcpp::shutdown();
  return 0;
}
