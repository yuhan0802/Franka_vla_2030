#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit/utils/logger.hpp>
#include "franka_vr/srv/set_step_size.hpp"
#include "franka_vr/srv/set_target_pose.hpp"
#include <Eigen/Geometry>

using namespace moveit_servo;

namespace
{
// All interactive clients publish their targets in the shared world frame.
// MoveIt Servo transforms this twist into its planning frame internally.
constexpr auto K_TARGET_FRAME = "world";
// fr3_hand only exists when the optional gripper is loaded.  fr3_link8 is
// always present and is therefore the safe Cartesian control tip for both
// gripper and no-gripper launches.
constexpr auto K_TIP_FRAME = "fr3_link8";
constexpr double POSITION_TRACKING_GAIN = 2.0;
constexpr double ORIENTATION_TRACKING_GAIN = 2.0;
constexpr double SOFT_STOP_DURATION_SECONDS = 0.12;
// Generate one action segment every 1/80 second. The VR target source remains
// at 100 Hz and the low-level FCI control loop remains at 1 kHz.
constexpr double TELEOP_COMMAND_HZ = 80.0;
// VR collection limits at the 80 Hz action cadence.
constexpr double MAX_JOINT_VELOCITY_RAD_S = 2.0;
// This preserves the 3.75 rad/s^2 acceleration limit. Keep it aligned
// with MoveIt's joint_limits.yaml so neither layer unexpectedly dominates.
constexpr double MAX_JOINT_VELOCITY_CHANGE_RAD_S = 3.75 / TELEOP_COMMAND_HZ;
constexpr double MAX_JOINT_POSITION_STEP_RAD =
    MAX_JOINT_VELOCITY_RAD_S / TELEOP_COMMAND_HZ;
}  // namespace

// These values are interpreted as Cartesian velocity limits by the target
// tracker.  Keep the defaults conservative for the first real-robot test.
// At 80 Hz, these velocity caps correspond to at most 0.00375 m and
// 0.0121875 rad per action segment.
static Eigen::Vector3d linear_step_size{ 0.30, 0.30, 0.30 };
static Eigen::AngleAxisd x_step_size(0.975, Eigen::Vector3d::UnitX());
static Eigen::AngleAxisd y_step_size(0.975, Eigen::Vector3d::UnitY());
static Eigen::AngleAxisd z_step_size(0.975, Eigen::Vector3d::UnitZ());
static geometry_msgs::msg::Pose target_pose;
// The service callback and the Servo tracking thread run concurrently.  The
// old plain bool could be cached by the tracking thread, so newly received
// manual targets were never observed in optimized builds.
static std::atomic_bool target_set{ false };
static std::atomic_bool target_stop_requested{ false };
static std::atomic_bool target_start_requested{ false };
static std::atomic_bool target_motion_requested{ false };
static std::mutex target_pose_mutex;

void publish_joint_velocities(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& publisher,
    const Eigen::VectorXd& velocities, const rclcpp::Time& send_time)
{
  std_msgs::msg::Float64MultiArray message;
  // Producer-side timestamp followed by the seven joint velocities.
  message.data.reserve(static_cast<std::size_t>(velocities.size()) + 1);
  message.data.push_back(send_time.seconds());
  message.data.insert(message.data.end(), velocities.data(), velocities.data() + velocities.size());
  publisher->publish(message);
}

void limit_teleop_joint_command(KinematicState& command, const KinematicState& reference,
                                Eigen::VectorXd& previous_velocity)
{
  if (command.positions.size() != command.velocities.size() ||
      command.positions.size() != reference.positions.size())
  {
    return;
  }
  if (previous_velocity.size() != command.velocities.size())
  {
    previous_velocity = Eigen::VectorXd::Zero(command.velocities.size());
  }
  for (Eigen::Index index = 0; index < command.velocities.size(); ++index)
  {
    const double velocity = std::clamp(command.velocities(index), -MAX_JOINT_VELOCITY_RAD_S,
                                       MAX_JOINT_VELOCITY_RAD_S);
    command.velocities(index) = std::clamp(
        velocity, previous_velocity(index) - MAX_JOINT_VELOCITY_CHANGE_RAD_S,
        previous_velocity(index) + MAX_JOINT_VELOCITY_CHANGE_RAD_S);
    command.positions(index) = std::clamp(command.positions(index),
                                          reference.positions(index) - MAX_JOINT_POSITION_STEP_RAD,
                                          reference.positions(index) + MAX_JOINT_POSITION_STEP_RAD);
  }
  previous_velocity = command.velocities;
}

void set_step_size_callback(const rclcpp::Node::SharedPtr& node,
                            const std::shared_ptr<franka_vr::srv::SetStepSize::Request> request,
                            std::shared_ptr<franka_vr::srv::SetStepSize::Response> response)
{
  try
  {
    linear_step_size = Eigen::Vector3d{ request->linear_x, request->linear_y, request->linear_z };
    x_step_size = Eigen::AngleAxisd(request->angular_x, Eigen::Vector3d::UnitX());
    y_step_size = Eigen::AngleAxisd(request->angular_y, Eigen::Vector3d::UnitY());
    z_step_size = Eigen::AngleAxisd(request->angular_z, Eigen::Vector3d::UnitZ());
    response->success = true;
    response->message = "Step sizes updated successfully";
    RCLCPP_INFO(node->get_logger(), "Updated step sizes: linear[%.3f, %.3f, %.3f], angular[%.3f, %.3f, %.3f]",
                linear_step_size.x(), linear_step_size.y(), linear_step_size.z(), x_step_size.angle(),
                y_step_size.angle(), z_step_size.angle());
  }
  catch (const std::exception& e)
  {
    response->success = false;
    response->message = std::string("Failed to update step sizes: ") + e.what();
    RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
  }
}

void set_target_pose_callback(const rclcpp::Node::SharedPtr& node,
                              const std::shared_ptr<franka_vr::srv::SetTargetPose::Request> request,
                              std::shared_ptr<franka_vr::srv::SetTargetPose::Response> response)
{
  if (request->stop)
  {
    target_start_requested.store(false, std::memory_order_release);
    target_motion_requested.store(false, std::memory_order_release);
    target_stop_requested.store(true, std::memory_order_release);
    response->success = true;
    response->message = "Current Cartesian pose hold requested";
    RCLCPP_INFO(node->get_logger(), "Current Cartesian pose hold requested by client");
    return;
  }

  const auto& stamped_pose = request->target_pose;
  geometry_msgs::msg::Pose pose = stamped_pose.pose;
  const double orientation_norm = std::sqrt(
      pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w);
  if (stamped_pose.header.frame_id != K_TARGET_FRAME || !std::isfinite(pose.position.x) ||
      !std::isfinite(pose.position.y) || !std::isfinite(pose.position.z) || !std::isfinite(pose.orientation.x) ||
      !std::isfinite(pose.orientation.y) || !std::isfinite(pose.orientation.z) ||
      !std::isfinite(pose.orientation.w) || orientation_norm < 1e-6)
  {
    response->success = false;
    response->message = std::string("Target pose must be finite and expressed in ") + K_TARGET_FRAME;
    RCLCPP_WARN(node->get_logger(), "Rejected target pose in frame '%s'", stamped_pose.header.frame_id.c_str());
    return;
  }

  pose.orientation.x /= orientation_norm;
  pose.orientation.y /= orientation_norm;
  pose.orientation.z /= orientation_norm;
  pose.orientation.w /= orientation_norm;

  {
    std::lock_guard<std::mutex> lock(target_pose_mutex);
    target_pose = pose;
  }
  target_stop_requested.store(false, std::memory_order_release);
  target_motion_requested.store(true, std::memory_order_release);
  if (!target_set.exchange(true, std::memory_order_acq_rel))
  {
    target_start_requested.store(true, std::memory_order_release);
  }
  response->success = true;
  response->message = "Target pose set successfully";
  RCLCPP_INFO(node->get_logger(), "Target pose set: [x: %.3f, y: %.3f, z: %.3f]", pose.position.x,
              pose.position.y, pose.position.z);
}

Eigen::Vector3d compute_position_error(const geometry_msgs::msg::Pose& current, const geometry_msgs::msg::Pose& target)
{
  return Eigen::Vector3d(target.position.x - current.position.x, target.position.y - current.position.y,
                         target.position.z - current.position.z);
}

bool get_current_tip_pose(tf2_ros::Buffer& tf_buffer, const rclcpp::Node::SharedPtr& node,
                          geometry_msgs::msg::Pose& pose)
{
  try
  {
    const auto transform = tf_buffer.lookupTransform(K_TARGET_FRAME, K_TIP_FRAME, tf2::TimePointZero);
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;
    return true;
  }
  catch (const tf2::TransformException& error)
  {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                         "Cannot read actual %s -> %s transform: %s", K_TARGET_FRAME, K_TIP_FRAME, error.what());
    return false;
  }
}

void publish_latched_hold(const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& publisher,
                          const KinematicState& hold_state, const rclcpp::Time& stamp)
{
  if (hold_state.joint_names.empty() || hold_state.joint_names.size() != static_cast<size_t>(hold_state.positions.size()))
  {
    return;
  }
  trajectory_msgs::msg::JointTrajectory hold;
  hold.header.stamp = stamp;
  hold.joint_names = hold_state.joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.reserve(hold_state.positions.size());
  for (const auto position : hold_state.positions)
  {
    point.positions.push_back(position);
  }
  point.velocities.assign(hold_state.positions.size(), 0.0);
  // Refresh this future point at 100 Hz so the effort trajectory controller
  // never runs out of a hold command between VR input samples.
  point.time_from_start.nanosec = 100000000U;
  hold.points.push_back(std::move(point));
  publisher->publish(hold);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("moveit_servo_demo");
  moveit::setNodeLoggerName(demo_node->get_name());

  auto step_size_service = demo_node->create_service<franka_vr::srv::SetStepSize>(
      "set_step_size", std::bind(&set_step_size_callback, demo_node, std::placeholders::_1, std::placeholders::_2));
  auto target_pose_service = demo_node->create_service<franka_vr::srv::SetTargetPose>(
      "set_target_pose", std::bind(&set_target_pose_callback, demo_node, std::placeholders::_1, std::placeholders::_2));

  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();

  auto trajectory_outgoing_cmd_pub = demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      servo_params.command_out_topic, rclcpp::SystemDefaultsQoS());
  auto requested_joint_velocity_pub = demo_node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/franka_vr/joint_velocity_command_requested", rclcpp::SystemDefaultsQoS());
  auto sent_joint_velocity_pub = demo_node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/franka_vr/joint_velocity_command_sent", rclcpp::SystemDefaultsQoS());

  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      createPlanningSceneMonitor(demo_node, servo_params);
  Servo servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);
  tf2_ros::Buffer tf_buffer(demo_node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  std::this_thread::sleep_for(std::chrono::seconds(3));

  auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params.move_group_name);
  if (!joint_model_group)
  {
    RCLCPP_ERROR(demo_node->get_logger(), "Move group '%s' is unavailable", servo_params.move_group_name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  servo.setCommandType(CommandType::TWIST);

  auto pose_tracker = [&]() {
    KinematicState joint_state;
    rclcpp::WallRate tracking_rate(TELEOP_COMMAND_HZ);
    std::deque<KinematicState> joint_cmd_rolling_window;
    KinematicState current_state = servo.getCurrentRobotState(true);
    servo.resetSmoothing(current_state);
    StatusCode previous_status = StatusCode::INVALID;
    std::optional<KinematicState> latched_hold_state;
    bool soft_stop_active = false;
    Eigen::VectorXd previous_sent_velocity;
    rclcpp::Time soft_stop_deadline = demo_node->now();

    while (rclcpp::ok())
    {
      if (target_stop_requested.exchange(false, std::memory_order_acq_rel))
      {
        joint_cmd_rolling_window.clear();
        current_state = servo.getCurrentRobotState(false);
        // Preserve measured joint velocity when seeding the smoothing plugin;
        // commanding a zero Cartesian twist below then produces a bounded
        // deceleration instead of an instantaneous velocity reset.
        servo.resetSmoothing(current_state);
        latched_hold_state.reset();
        target_set.store(true, std::memory_order_release);
        soft_stop_active = true;
        soft_stop_deadline = demo_node->now() + rclcpp::Duration::from_seconds(SOFT_STOP_DURATION_SECONDS);
        RCLCPP_INFO(demo_node->get_logger(), "Servo starting %.0f ms controlled stop",
                    SOFT_STOP_DURATION_SECONDS * 1000.0);
      }

      // A fresh hand target takes priority if the trigger is pressed again
      // during the short deceleration window.
      if (target_motion_requested.exchange(false, std::memory_order_acq_rel))
      {
        soft_stop_active = false;
      }

      if (target_start_requested.exchange(false, std::memory_order_acq_rel))
      {
        // The smoothing plugin is bypassed while the joint-hold path is
        // active. Re-seed it from measured feedback before the first Servo
        // command so it cannot replay a stale position or velocity.
        joint_cmd_rolling_window.clear();
        current_state = servo.getCurrentRobotState(false);
        current_state.velocities *= 0.0;
        servo.resetSmoothing(current_state);
        latched_hold_state.reset();
        soft_stop_active = false;
        RCLCPP_INFO(demo_node->get_logger(), "Servo tracker armed from measured robot state");
      }

      if (!target_set.load(std::memory_order_acquire))
      {
        // The real FR3 trajectory controller has an effort command interface.
        // Simply ceasing to publish trajectories after a VR timeout lets its
        // short command horizon expire, which removes the commanded hold and
        // can let the arm sag under gravity. Latch the measured joints once
        // and continuously publish that zero-velocity joint trajectory. A
        // stationary hold cannot introduce a collision, so it must not be
        // suppressed by Servo's collision-status path.
        if (!latched_hold_state)
        {
          current_state = servo.getCurrentRobotState(false);
          current_state.velocities *= 0.0;
          latched_hold_state = current_state;
          RCLCPP_INFO(demo_node->get_logger(), "Servo entered latched joint hold state");
        }
        publish_latched_hold(trajectory_outgoing_cmd_pub, *latched_hold_state, demo_node->now());
        tracking_rate.sleep();
        continue;
      }

      latched_hold_state.reset();

      // Keep the short-horizon command prediction only while it is still in
      // the future.  Once it expires, start the next IK step from measured
      // joint feedback instead of perpetuating a stale predicted trajectory.
      const auto current_time = demo_node->now();
      if (!joint_cmd_rolling_window.empty() && joint_cmd_rolling_window.back().time_stamp > current_time)
      {
        current_state = joint_cmd_rolling_window.back();
      }
      else
      {
        joint_cmd_rolling_window.clear();
        current_state = servo.getCurrentRobotState(false);
        if (!soft_stop_active)
        {
          current_state.velocities *= 0.0;
        }
      }
      robot_state->setJointGroupPositions(joint_model_group, current_state.positions);
      robot_state->setJointGroupVelocities(joint_model_group, current_state.velocities);

      Eigen::Vector3d linear_vel = Eigen::Vector3d::Zero();
      Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();

      if (soft_stop_active && current_time >= soft_stop_deadline)
      {
        // Once velocity has been ramped down, discard the prediction and hold
        // the pose actually reached at the end of the deceleration.
        joint_cmd_rolling_window.clear();
        current_state = servo.getCurrentRobotState(false);
        current_state.velocities *= 0.0;
        servo.resetSmoothing(current_state);
        robot_state->setJointGroupPositions(joint_model_group, current_state.positions);
        robot_state->setJointGroupVelocities(joint_model_group, current_state.velocities);

        geometry_msgs::msg::Pose hold_pose;
        if (get_current_tip_pose(tf_buffer, demo_node, hold_pose))
        {
          std::lock_guard<std::mutex> target_guard(target_pose_mutex);
          target_pose = hold_pose;
          soft_stop_active = false;
          RCLCPP_INFO(demo_node->get_logger(), "Servo controlled stop complete; holding Cartesian pose");
        }
        else
        {
          target_set.store(false, std::memory_order_release);
          soft_stop_active = false;
          RCLCPP_WARN(demo_node->get_logger(), "Falling back to joint hold because the tool pose is unavailable");
          tracking_rate.sleep();
          continue;
        }
      }

      if (!soft_stop_active)
      {
        geometry_msgs::msg::Pose requested_target;
        {
          std::lock_guard<std::mutex> target_guard(target_pose_mutex);
          requested_target = target_pose;
        }
        geometry_msgs::msg::Pose current_pose;
        if (!get_current_tip_pose(tf_buffer, demo_node, current_pose))
        {
          tracking_rate.sleep();
          continue;
        }
        Eigen::Vector3d pos_error = compute_position_error(current_pose, requested_target);
      // Keep streaming zero/error-correcting Servo trajectories even after
      // the target enters tolerance.  Clearing target_set here previously
      // switched to the separate latched-hold path between low-rate VLA
      // updates; on the effort-controlled real arm that transition can leave
      // a command-horizon gap and allow gravity sag.  Leaving the Cartesian
      // target active gives Servo continuous position feedback until a new
      // target, explicit stop, or safety halt arrives.

        const double configured_angular_speed = std::max({ x_step_size.angle(), y_step_size.angle(), z_step_size.angle() });
        const double max_linear_speed = linear_step_size.maxCoeff() > 0 ? linear_step_size.maxCoeff() : 0.05;
        const double max_angular_speed = configured_angular_speed > 0 ? configured_angular_speed : 0.4;
                              
      // A proportional target tracker is responsive for the small keyboard
      // increments while the Servo acceleration filter removes abrupt starts
      // and stops.  The configured values remain strict speed caps.
        const double position_error_norm = pos_error.norm();
        const double linear_speed = std::min(max_linear_speed, POSITION_TRACKING_GAIN * position_error_norm);
        if (position_error_norm > 1e-6)
        {
          linear_vel = pos_error / position_error_norm * linear_speed;
        }

        tf2::Quaternion q_current, q_target;
        tf2::fromMsg(current_pose.orientation, q_current);
        tf2::fromMsg(requested_target.orientation, q_target);
        tf2::Quaternion q_diff = q_target * q_current.inverse();
        q_diff.normalize();
        if (q_diff.w() < 0.0)
        {
          q_diff.setX(-q_diff.x());
          q_diff.setY(-q_diff.y());
          q_diff.setZ(-q_diff.z());
          q_diff.setW(-q_diff.w());
        }
        const tf2::Vector3 angular_vel_axis = q_diff.getAxis();
        const double angular_vel_magnitude = q_diff.getAngle();
        const double angular_speed = std::min(max_angular_speed, ORIENTATION_TRACKING_GAIN * angular_vel_magnitude);
        angular_vel = Eigen::Vector3d(angular_vel_axis.x(), angular_vel_axis.y(), angular_vel_axis.z()) * angular_speed;
      }

      TwistCommand target_twist{ K_TARGET_FRAME,
                                 { linear_vel.x(), linear_vel.y(), linear_vel.z(), angular_vel.x(), angular_vel.y(),
                                   angular_vel.z() } };

      joint_state = servo.getNextJointState(robot_state, target_twist);
      // Preserve the Servo output before our final collection-specific limit,
      // then publish both values. The recorder uses the latter as training
      // action and retains the former for audit/replay.
      publish_joint_velocities(requested_joint_velocity_pub, joint_state.velocities, demo_node->now());
      limit_teleop_joint_command(joint_state, current_state, previous_sent_velocity);
      const StatusCode status = servo.getStatus();

      if (status != previous_status)
      {
        RCLCPP_INFO_STREAM(demo_node->get_logger(), "Servo status: " << servo.getStatusMessage());
        previous_status = status;
      }

      if (status != StatusCode::INVALID && status != StatusCode::HALT_FOR_COLLISION)
      {
        updateSlidingWindow(joint_state, joint_cmd_rolling_window, servo_params.max_expected_latency, current_time);
        if (const auto msg = composeTrajectoryMessage(servo_params, joint_cmd_rolling_window))
        {
          trajectory_outgoing_cmd_pub->publish(msg.value());
          // Emit a recording event only after the trajectory command was
          // actually handed to the robot controller.
          publish_joint_velocities(sent_joint_velocity_pub, joint_state.velocities, demo_node->now());
        }
      }
      else
      {
        target_set.store(false, std::memory_order_release);
        joint_cmd_rolling_window.clear();
        current_state = servo.getCurrentRobotState(false);
        current_state.velocities *= 0.0;
        servo.resetSmoothing(current_state);
        RCLCPP_WARN(demo_node->get_logger(), "Servo stopped target tracking after safety/status halt");
      }
      tracking_rate.sleep();
    }
  };

  std::thread tracker_thread(pose_tracker);

  // Match the 100 Hz Quest target producer and target-tracking loop.
  rclcpp::WallRate command_rate(100);
  RCLCPP_INFO_STREAM(demo_node->get_logger(), servo.getStatusMessage());

  while (rclcpp::ok())
  {
    rclcpp::spin_some(demo_node);
    command_rate.sleep();
  }

  rclcpp::shutdown();
  if (tracker_thread.joinable())
    tracker_thread.join();
  return 0;
}
