#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

namespace
{
constexpr char kFeedbackTopic[] =
    "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback";
constexpr char kGoalMarker[] = "EE:goal_fr3_hand_tcp";
constexpr char kEndEffectorLink[] = "fr3_hand_tcp";
}  // namespace

class RvizPoseGoalExecutor : public rclcpp::Node
{
public:
  RvizPoseGoalExecutor() : Node("rviz_pose_goal_executor")
  {
  }

  void initialize()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "fr3_arm");
    move_group_->setEndEffectorLink(kEndEffectorLink);
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.1);
    move_group_->setMaxAccelerationScalingFactor(0.1);
    move_group_->setGoalPositionTolerance(0.002);
    move_group_->setGoalOrientationTolerance(0.02);

    feedback_subscription_ = create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
        kFeedbackTopic, rclcpp::SystemDefaultsQoS(),
        std::bind(&RvizPoseGoalExecutor::feedbackCallback, this, std::placeholders::_1));
    feedback_publisher_ = create_publisher<visualization_msgs::msg::InteractiveMarkerFeedback>(
        kFeedbackTopic, rclcpp::SystemDefaultsQoS());
    RCLCPP_INFO(get_logger(), "Dragging the MoveIt end-effector marker will plan and execute on mouse release.");
  }

private:
  void feedbackCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
  {
    if (feedback->marker_name != kGoalMarker ||
        feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)
      return;

    std::scoped_lock lock(execution_mutex_);
    RCLCPP_INFO(get_logger(), "Received end-effector target: [%.3f, %.3f, %.3f]", feedback->pose.position.x,
                feedback->pose.position.y, feedback->pose.position.z);

    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(feedback->pose, kEndEffectorLink);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group_->plan(plan));
    if (!planned)
    {
      RCLCPP_WARN(get_logger(), "No plan found for the dragged end-effector target.");
      move_group_->clearPoseTargets();
      return;
    }

    const bool executed = static_cast<bool>(move_group_->execute(plan));
    if (!executed)
      RCLCPP_ERROR(get_logger(), "Failed to execute the planned end-effector motion.");
    else
    {
      scheduleGoalStateSync();
      RCLCPP_INFO(get_logger(), "Dragged end-effector target executed successfully.");
    }
    move_group_->clearPoseTargets();
  }

  void scheduleGoalStateSync()
  {
    // execute() completes inside this node's feedback callback.  At that
    // instant its current-state subscription has not yet been dispatched, so
    // getCurrentPose() would still return the pre-motion Start state.  Defer
    // the read briefly, allowing the Gazebo joint-state update to be handled.
    goal_sync_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      goal_sync_timer_->cancel();
      syncGoalStateToCurrentPose();
    });
  }

  void syncGoalStateToCurrentPose()
  {
    visualization_msgs::msg::InteractiveMarkerFeedback goal_feedback;
    goal_feedback.header.stamp = now();
    goal_feedback.header.frame_id = move_group_->getPlanningFrame();
    goal_feedback.marker_name = kGoalMarker;
    goal_feedback.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE;
    goal_feedback.pose = move_group_->getCurrentPose(kEndEffectorLink).pose;
    feedback_publisher_->publish(goal_feedback);
    RCLCPP_INFO(get_logger(), "Synchronized RViz Goal state to the completed robot pose.");
  }

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr feedback_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr feedback_publisher_;
  rclcpp::TimerBase::SharedPtr goal_sync_timer_;
  std::mutex execution_mutex_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RvizPoseGoalExecutor>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
