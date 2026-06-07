
#include <chrono>
#include <cmath>
#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace
{
// Servo topics
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const size_t ROS_QUEUE_SIZE = 10;
const std::string PLANNING_FRAME_ID = "panda_link0";
const std::string EE_FRAME_ID = "gripper_control_frame";

// Gripper action
const std::string GRIPPER_ACTION = "/panda_hand_controller/gripper_command";
const std::string SWITCH_SERVICE = "/servo_node/switch_command_type";
const double GRIPPER_OPEN = 0.04;   // Max opening position
const double GRIPPER_CLOSE = 0.0;   // Closed position
const double GRIPPER_STEP = 0.01;   // Step size for incremental control

// Joy axis/button indices
constexpr int AXIS_LEFT_STICK_X = 0;    // Joint1 or Twist X rotation
constexpr int AXIS_LEFT_STICK_Y = 1;    // Joint2 or Twist Y rotation
constexpr int AXIS_LEFT_TRIGGER = 2;    // Unused
constexpr int AXIS_RIGHT_STICK_X = 3;   // Joint3 or Twist Z rotation
constexpr int AXIS_RIGHT_STICK_Y = 4;   // Unused
constexpr int AXIS_RIGHT_TRIGGER = 5;   // Unused
constexpr int AXIS_DPAD_X = 6;          // Joint5 or Twist Y linear
constexpr int AXIS_DPAD_Y = 7;          // Joint6 or Twist X linear

constexpr int BUTTON_A = 0;             // Switch command frame
constexpr int BUTTON_B = 1;             // Joint7+ or Twist Z+
constexpr int BUTTON_X = 2;             // Joint7- or Twist Z-
constexpr int BUTTON_Y = 3;             // Unused
constexpr int BUTTON_LEFT_BUMPER = 4;   // Gripper open
constexpr int BUTTON_RIGHT_BUMPER = 5;  // Gripper close
constexpr int BUTTON_BACK = 6;          // Select - switch mode
constexpr int BUTTON_START = 7;         // Enable/Disable

// Threshold for axis detection
constexpr double AXIS_THRESHOLD = 0.7;

// Control speeds
constexpr double JOINT_VELOCITY = 0.5;      // rad/s
constexpr double TWIST_LINEAR_SPEED = 0.5;  // m/s
constexpr double TWIST_ANGULAR_SPEED = 0.5; // rad/s

// Control modes
enum class ControlMode
{
  TWIST = 0,
  JOINT = 1,
  // Future: POSE = 2,
};
}  // namespace

class JoyServoController : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

  JoyServoController() : Node("joy_servo_controller")
  {
    // Publishers
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
    joint_pub_ = create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);

    // Service client for switching command type
    switch_input_ = create_client<moveit_msgs::srv::ServoCommandType>(SWITCH_SERVICE);

    // Gripper action client
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(this, GRIPPER_ACTION);

    // Joy subscriber
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("/joy", ROS_QUEUE_SIZE,
                                                           std::bind(&JoyServoController::joyCallback, this, std::placeholders::_1));

    // Initialize state
    enabled_ = false;
    mode_ = ControlMode::TWIST;
    command_frame_ = PLANNING_FRAME_ID;
    current_gripper_pos_ = GRIPPER_CLOSE;
    prev_start_ = false;
    prev_select_ = false;
    prev_button_a_ = false;

    // Previous button states for edge detection
    prev_buttons_.resize(8, 0);

    RCLCPP_INFO(get_logger(), "Joy Servo Controller initialized");
    RCLCPP_INFO(get_logger(), "Press START to enable (enables TWIST mode), SELECT to switch mode");

    // Timer to initialize servo command type on startup (retry until service available)
    init_timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&JoyServoController::initServoMode, this));
  }

  void initServoMode()
  {
    if (!switch_input_->wait_for_service(std::chrono::milliseconds(100)))
    {
      return;  // Retry next tick
    }
    // Service is available, set initial mode (async)
    auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    request->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;

    switch_input_->async_send_request(
        request,
        [this](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture future) {
          if (future.get()->success)
          {
            RCLCPP_INFO(get_logger(), "Servo initialized to TWIST mode");
          }
          else
          {
            RCLCPP_WARN(get_logger(), "Failed to initialize servo mode");
          }
        });
    init_timer_->cancel();  // Stop retrying
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Check button count
    if (msg->buttons.size() < 8 || msg->axes.size() < 8)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Joy message has insufficient buttons/axes");
      return;
    }

    // START button - Enable/Disable toggle (edge detection)
    bool current_start = msg->buttons[BUTTON_START];
    if (current_start && !prev_start_)
    {
      enabled_ = !enabled_;
      if (enabled_)
      {
        RCLCPP_INFO(get_logger(), "===== ENABLED =====");
      }
      else
      {
        RCLCPP_INFO(get_logger(), "===== DISABLED - Stopping =====");
        sendStopCommand();
      }
    }
    prev_start_ = current_start;

    // If not enabled, don't process other inputs
    if (!enabled_)
    {
      prev_buttons_ = msg->buttons;
      prev_select_ = msg->buttons[BUTTON_BACK];
      return;
    }

    // SELECT button - Switch mode (edge detection)
    bool current_select = msg->buttons[BUTTON_BACK];
    if (current_select && !prev_select_)
    {
      switchMode();
    }
    prev_select_ = current_select;

    // Process input based on mode
    switch (mode_)
    {
      case ControlMode::TWIST:
        processTwistInput(msg);
        break;
      case ControlMode::JOINT:
        processJointInput(msg);
        break;
      // Future: case ControlMode::POSE:
    }

    prev_buttons_ = msg->buttons;
  }

  void switchMode()
  {
    switch (mode_)
    {
      case ControlMode::TWIST:
        sendSwitchService(moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG);
        mode_ = ControlMode::JOINT;
        RCLCPP_INFO(get_logger(), "Switched to JOINT mode");
        break;
      case ControlMode::JOINT:
        sendSwitchService(moveit_msgs::srv::ServoCommandType::Request::TWIST);
        mode_ = ControlMode::TWIST;
        RCLCPP_INFO(get_logger(), "Switched to TWIST mode");
        break;
    }
  }

  void sendSwitchService(uint8_t command_type)
  {
    auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    request->command_type = command_type;

    switch_input_->async_send_request(
        request,
        [this](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture future) {
          if (!future.get()->success)
          {
            RCLCPP_WARN(get_logger(), "Failed to switch servo mode");
          }
        });
  }

  void processTwistInput(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    // Button A - Switch command frame (edge detection)
    bool current_button_a = msg->buttons[BUTTON_A];
    if (current_button_a && !prev_button_a_)
    {
      if (command_frame_ == PLANNING_FRAME_ID)
      {
        command_frame_ = EE_FRAME_ID;
        RCLCPP_INFO(get_logger(), "Command frame: %s", command_frame_.c_str());
      }
      else
      {
        command_frame_ = PLANNING_FRAME_ID;
        RCLCPP_INFO(get_logger(), "Command frame: %s", command_frame_.c_str());
      }
    }
    prev_button_a_ = current_button_a;

    // Axis 7 (DPAD_Y) - X linear
    if (std::abs(msg->axes[AXIS_DPAD_Y]) > AXIS_THRESHOLD)
    {
      twist_msg->twist.linear.x = TWIST_LINEAR_SPEED * msg->axes[AXIS_DPAD_Y];
    }

    // Axis 6 (DPAD_X) - Y linear
    if (std::abs(msg->axes[AXIS_DPAD_X]) > AXIS_THRESHOLD)
    {
      twist_msg->twist.linear.y = TWIST_LINEAR_SPEED * msg->axes[AXIS_DPAD_X];
    }

    // Button B (1) - Z+ linear
    if (msg->buttons[BUTTON_B])
    {
      twist_msg->twist.linear.z = TWIST_LINEAR_SPEED;
    }

    // Button X (2) - Z- linear
    if (msg->buttons[BUTTON_X])
    {
      twist_msg->twist.linear.z = -TWIST_LINEAR_SPEED;
    }

    // Axis 0 (Left stick X) - X rotation
    if (std::abs(msg->axes[AXIS_LEFT_STICK_X]) > AXIS_THRESHOLD)
    {
      twist_msg->twist.angular.x = TWIST_ANGULAR_SPEED * msg->axes[AXIS_LEFT_STICK_X];
    }

    // Axis 1 (Left stick Y) - Y rotation
    if (std::abs(msg->axes[AXIS_LEFT_STICK_Y]) > AXIS_THRESHOLD)
    {
      twist_msg->twist.angular.y = TWIST_ANGULAR_SPEED * msg->axes[AXIS_LEFT_STICK_Y];
    }

    // Axis 3 (Right stick X) - Z rotation
    if (std::abs(msg->axes[AXIS_RIGHT_STICK_X]) > AXIS_THRESHOLD)
    {
      twist_msg->twist.angular.z = TWIST_ANGULAR_SPEED * msg->axes[AXIS_RIGHT_STICK_X];
    }

    // Always publish (zero velocities when no input = stop)
    twist_msg->header.stamp = now();
    twist_msg->header.frame_id = command_frame_;
    twist_pub_->publish(std::move(twist_msg));
  }

  void processJointInput(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
    joint_msg->joint_names = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                               "panda_joint5", "panda_joint6", "panda_joint7" };
    joint_msg->velocities.resize(7, 0.0);

    // Axis 0 (Left stick X) - Joint1
    if (std::abs(msg->axes[AXIS_LEFT_STICK_X]) > AXIS_THRESHOLD)
    {
      joint_msg->velocities[0] = JOINT_VELOCITY * msg->axes[AXIS_LEFT_STICK_X];
    }

    // Axis 1 (Left stick Y) - Joint2
    if (std::abs(msg->axes[AXIS_LEFT_STICK_Y]) > AXIS_THRESHOLD)
    {
      joint_msg->velocities[1] = JOINT_VELOCITY * msg->axes[AXIS_LEFT_STICK_Y];
    }

    // Axis 3 (Right stick X) - Joint3
    if (std::abs(msg->axes[AXIS_RIGHT_STICK_X]) > AXIS_THRESHOLD)
    {
      joint_msg->velocities[2] = JOINT_VELOCITY * msg->axes[AXIS_RIGHT_STICK_X];
    }

    // Axis 4 (Right stick Y) - Joint4
    if (std::abs(msg->axes[AXIS_RIGHT_STICK_Y]) > AXIS_THRESHOLD)
    {
      joint_msg->velocities[3] = JOINT_VELOCITY * msg->axes[AXIS_RIGHT_STICK_Y];
    }

    // Axis 6 (DPAD_X) - Joint5 (incremental)
    if (std::abs(msg->axes[AXIS_DPAD_X]) > AXIS_THRESHOLD)
    {
      joint_msg->velocities[4] = JOINT_VELOCITY * msg->axes[AXIS_DPAD_X];
    }

    // Axis 7 (DPAD_Y) - Joint6 (incremental)
    if (std::abs(msg->axes[AXIS_DPAD_Y]) > AXIS_THRESHOLD)
    {
      joint_msg->velocities[5] = JOINT_VELOCITY * msg->axes[AXIS_DPAD_Y];
    }

    // Button B (1) - Joint7+
    if (msg->buttons[BUTTON_B])
    {
      joint_msg->velocities[6] = JOINT_VELOCITY;
    }

    // Button X (2) - Joint7-
    if (msg->buttons[BUTTON_X])
    {
      joint_msg->velocities[6] = -JOINT_VELOCITY;
    }

    // Always publish (zero velocities when no input = stop)
    joint_msg->header.stamp = now();
    joint_msg->header.frame_id = PLANNING_FRAME_ID;
    joint_pub_->publish(std::move(joint_msg));

    // Gripper control (independent of joint publishing)
    // Button 4 (Left bumper) - Open gripper
    if (msg->buttons[BUTTON_LEFT_BUMPER])
    {
      current_gripper_pos_ = std::min(current_gripper_pos_ + GRIPPER_STEP, GRIPPER_OPEN);
      sendGripperCommand(current_gripper_pos_);
    }
    // Button 5 (Right bumper) - Close gripper
    else if (msg->buttons[BUTTON_RIGHT_BUMPER])
    {
      current_gripper_pos_ = std::max(current_gripper_pos_ - GRIPPER_STEP, GRIPPER_CLOSE);
      sendGripperCommand(current_gripper_pos_);
    }
  }

  void sendStopCommand()
  {
    // Send zero twist
    auto stop_twist = std::make_unique<geometry_msgs::msg::TwistStamped>();
    stop_twist->header.stamp = now();
    stop_twist->header.frame_id = command_frame_;
    twist_pub_->publish(std::move(stop_twist));

    // Send zero joint velocities
    auto stop_joint = std::make_unique<control_msgs::msg::JointJog>();
    stop_joint->joint_names = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                                "panda_joint5", "panda_joint6", "panda_joint7" };
    stop_joint->velocities = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    stop_joint->header.stamp = now();
    stop_joint->header.frame_id = PLANNING_FRAME_ID;
    joint_pub_->publish(std::move(stop_joint));
  }

  void sendGripperCommand(double position)
  {
    if (!gripper_client_->wait_for_action_server(std::chrono::milliseconds(100)))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Gripper action server not available");
      return;
    }

    auto goal = GripperCommand::Goal();
    goal.command.position = position;
    goal.command.max_effort = 0.0;  // Use default effort

    gripper_client_->async_send_goal(goal);
  }

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // Service client
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_input_;

  // Timer for initialization
  rclcpp::TimerBase::SharedPtr init_timer_;

  // Action client
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;

  // State
  bool enabled_;
  ControlMode mode_;
  std::string command_frame_;
  double current_gripper_pos_;
  bool prev_start_;
  bool prev_select_;
  bool prev_button_a_;
  std::vector<int> prev_buttons_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyServoController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
