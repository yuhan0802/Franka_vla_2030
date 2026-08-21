#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
double parse_double(const char* value, const char* name)
{
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0')
  {
    throw std::invalid_argument(std::string("Invalid ") + name + ": " + value);
  }
  return parsed;
}
}  // namespace

int main(int argc, char** argv)
{
  const auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  rclcpp::init(argc, argv);
  if (non_ros_args.size() < 4 || non_ros_args.size() > 5)
  {
    std::cerr << "Usage: moveit_cartesian_offset DX DY DZ [execute=true|false]\n";
    return 2;
  }

  try
  {
    const double dx = parse_double(non_ros_args[1].c_str(), "DX");
    const double dy = parse_double(non_ros_args[2].c_str(), "DY");
    const double dz = parse_double(non_ros_args[3].c_str(), "DZ");
    const bool execute = non_ros_args.size() == 5 && non_ros_args[4] == "execute=true";

    auto node = std::make_shared<rclcpp::Node>(
        "moveit_cartesian_offset", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, "fr3_arm");
    move_group.setEndEffectorLink("fr3_link8");
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.01);

    const std::string planning_frame = move_group.getPlanningFrame();
    move_group.setPoseReferenceFrame(planning_frame);
    const auto current = move_group.getCurrentPose("fr3_link8");
    auto target = current.pose;
    target.position.x += dx;
    target.position.y += dy;
    target.position.z += dz;
    move_group.setStartStateToCurrentState();
    RCLCPP_INFO(node->get_logger(),
                "Planning frame: %s, current xyz: [%.6f, %.6f, %.6f], target xyz: [%.6f, %.6f, %.6f]",
                planning_frame.c_str(),
                current.pose.position.x, current.pose.position.y, current.pose.position.z,
                target.position.x, target.position.y, target.position.z);

    if (!move_group.setJointValueTarget(target, "fr3_link8"))
    {
      RCLCPP_ERROR(node->get_logger(), "IK_FAILED");
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 5;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group.plan(plan));
    if (!planned)
    {
      RCLCPP_ERROR(node->get_logger(), "PLAN_FAILED");
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 3;
    }

    const auto& points = plan.trajectory.joint_trajectory.points;
    const double duration = points.empty() ? 0.0 :
        rclcpp::Duration(points.back().time_from_start).seconds();
    RCLCPP_INFO(node->get_logger(), "PLAN_OK points=%zu duration=%.3fs execute=%s",
                points.size(), duration, execute ? "true" : "false");

    int result = 0;
    if (execute)
    {
      const bool executed = static_cast<bool>(move_group.execute(plan));
      if (!executed)
      {
        RCLCPP_ERROR(node->get_logger(), "EXECUTION_FAILED");
        result = 4;
      }
      else
      {
        RCLCPP_INFO(node->get_logger(), "EXECUTION_OK");
      }
    }

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return result;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
