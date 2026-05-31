#pragma once

#include <servo_control/servo.hpp>
#include <rclcpp/rclcpp.hpp>

//ros interfaces
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <moveit_msgs/msg/servo_status.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace servo_control
{

class ServoNode
{
public:
  explicit ServoNode(const rclcpp::NodeOptions& options);

  ~ServoNode();

  // Disable copy construction.
  ServoNode(const ServoNode&) = delete;

  // Disable copy assignment.
  ServoNode& operator=(ServoNode&) = delete;

  // This function is required to make this class a valid NodeClass
  // see https://docs.ros2.org/latest/api/rclcpp_components/register__node__macro_8hpp.html
  // Skip linting due to unconventional function naming
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();  // NOLINT

private:
  /**
   * \brief Loop that handles different types of incoming commands.
   */
  void servoLoop();

  /**
   * \brief The service to pause servoing, this does not exit the loop or stop the servo loop thread.
   * The loop will be alive even after pausing, but no commands will be processed.
   */
  void pauseServo(const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                  const std::shared_ptr<std_srvs::srv::SetBool::Response>& response);

  /**
   * \brief The service to set the command type for Servo.
   * Supported command types can be found in utils/datatypes.hpp
   * This service must be used to set the command type before sending any servoing commands.
   */
  void switchCommandType(const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request>& request,
                         const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Response>& response);

  void jointJogCallback(const control_msgs::msg::JointJog::ConstSharedPtr& msg);
  void twistCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

  std::optional<KinematicState> processJointJogCommand(const moveit::core::RobotStatePtr& robot_state);
  std::optional<KinematicState> processTwistCommand(const moveit::core::RobotStatePtr& robot_state);
  std::optional<KinematicState> processPoseCommand(const moveit::core::RobotStatePtr& robot_state);

  // Variables

  const rclcpp::Node::SharedPtr node_;
  std::unique_ptr<Servo> servo_;
  servo::Params servo_params_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  KinematicState last_commanded_state_;  // Used when commands go stale;
  control_msgs::msg::JointJog latest_joint_jog_;
  geometry_msgs::msg::TwistStamped latest_twist_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_jog_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr multi_array_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::ServoStatus>::SharedPtr status_publisher_;

  rclcpp::Service<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_command_type_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_servo_;

  // Used for communication with thread
  std::atomic<bool> stop_servo_;
  std::atomic<bool> servo_paused_;
  std::atomic<bool> new_joint_jog_msg_, new_twist_msg_, new_pose_msg_;

  // Threads used by ServoNode
  std::thread servo_loop_thread_;

  // Locks for threads safety
  std::mutex lock_;

  // rolling window of joint commands
  std::deque<KinematicState> joint_cmd_rolling_window_;
};

}  // namespace moveit_servo
