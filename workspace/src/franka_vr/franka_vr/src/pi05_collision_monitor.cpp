#include <chrono>
#include <memory>

#include "moveit/planning_scene_monitor/planning_scene_monitor.hpp"
#include "moveit/collision_detection/collision_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class Pi05CollisionMonitor final : public rclcpp::Node
{
public:
  Pi05CollisionMonitor() : Node("pi05_collision_monitor")
  {
    rate_ = declare_parameter("rate_hz", 20.0);
    if (rate_ < 20.0) throw std::runtime_error("collision monitor rate must be at least 20 Hz");
    publisher_ = create_publisher<std_msgs::msg::Bool>("/pi05/collision_stop", rclcpp::QoS(1).reliable());
    started_ = now();
  }

  void initialize()
  {
    monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");
    if (!monitor_->getPlanningScene()) throw std::runtime_error("planning scene could not be created");
    monitor_->startStateMonitor("/joint_states");
    monitor_->startSceneMonitor();
    monitor_->startWorldGeometryMonitor();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_), std::bind(&Pi05CollisionMonitor::tick, this));
  }

private:
  void tick()
  {
    std_msgs::msg::Bool stop;
    if (!monitor_->getStateMonitor() || !monitor_->getStateMonitor()->haveCompleteState()) {
      stop.data = (now() - started_).seconds() > 30.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "collision monitor is waiting for complete robot state");
      publisher_->publish(stop);
      return;
    }
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    request.contacts = true;
    request.max_contacts = 1;
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(monitor_);
      scene->checkSelfCollision(request, result);
    }
    stop.data = result.collision;
    if (stop.data && !latched_) RCLCPP_ERROR(get_logger(), "SELF COLLISION DETECTED; stop latched");
    latched_ = latched_ || stop.data;
    stop.data = latched_;
    publisher_->publish(stop);  // false messages are the executor's required safety heartbeat.
  }

  double rate_;
  bool latched_{false};
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time started_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Pi05CollisionMonitor>();
    node->initialize();
    rclcpp::spin(node);
  }
  catch (const std::exception& error) { RCLCPP_FATAL(rclcpp::get_logger("pi05_collision_monitor"), "%s", error.what()); }
  rclcpp::shutdown();
  return 0;
}
