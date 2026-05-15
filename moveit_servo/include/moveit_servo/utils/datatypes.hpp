
#pragma once

#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <unordered_map>

namespace moveit_servo
{

enum class StatusCode : int8_t
{
  INVALID = -1,
  NO_WARNING = 0,
  DECELERATE_FOR_APPROACHING_SINGULARITY = 1,
  HALT_FOR_SINGULARITY = 2,
  DECELERATE_FOR_LEAVING_SINGULARITY = 3,
  DECELERATE_FOR_COLLISION = 4,
  HALT_FOR_COLLISION = 5,
  JOINT_BOUND = 6
};

const std::unordered_map<StatusCode, std::string> SERVO_STATUS_CODE_MAP(
    { { StatusCode::INVALID, "Invalid" },
      { StatusCode::NO_WARNING, "No warnings" },
      { StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY, "Moving closer to a singularity, decelerating" },
      { StatusCode::HALT_FOR_SINGULARITY, "Very close to a singularity, emergency stop" },
      { StatusCode::DECELERATE_FOR_LEAVING_SINGULARITY, "Moving away from a singularity, decelerating" },
      { StatusCode::DECELERATE_FOR_COLLISION, "Close to a collision, decelerating" },
      { StatusCode::HALT_FOR_COLLISION, "Collision detected, emergency stop" },
      { StatusCode::JOINT_BOUND, "Close to a joint bound (position or velocity), halting" } });

// The datatype that specifies the type of command that servo should expect.
enum class CommandType : int8_t
{
  JOINT_JOG = 0,
  TWIST = 1,
  POSE = 2,

  // Range of allowed values used for validation.
  MIN = JOINT_JOG,
  MAX = POSE
};

typedef std::pair<StatusCode, Eigen::VectorXd> JointDeltaResult;

// The joint jog command, this will be vector of length equal to the number of joints of the robot.
struct JointJogCommand
{
  std::vector<std::string> names;
  std::vector<double> velocities;
};

// The twist command,  frame_id is the name of the frame in which the command is specified in.
// frame_id must always be specified.
struct TwistCommand
{
  std::string frame_id;
  Eigen::Vector<double, 6> velocities;
};

// The Pose command,  frame_id is the name of the frame in which the command is specified in.
// frame_id must always be specified.
struct PoseCommand
{
  std::string frame_id;
  Eigen::Isometry3d pose;
};

// The generic input type for servo that can be JointJog, Twist or Pose.
typedef std::variant<JointJogCommand, TwistCommand, PoseCommand> ServoInput;

// The output datatype of servo, this structure contains the names of the joints along with their positions, velocities and accelerations.
struct KinematicState
{
  std::vector<std::string> joint_names;// 1. 关节名称列表
  Eigen::VectorXd positions, velocities, accelerations; // 2. 关节位置、速度和加速度向量
  rclcpp::Time time_stamp;

  KinematicState(const int num_joints)
  {
    joint_names.resize(num_joints);
    positions = Eigen::VectorXd::Zero(num_joints);
    velocities = Eigen::VectorXd::Zero(num_joints);
    accelerations = Eigen::VectorXd::Zero(num_joints);
  }

  KinematicState()
  {
  }
};

// Mapping joint names and their position in the move group vector
typedef std::unordered_map<std::string, std::size_t> JointNameToMoveGroupIndexMap;

}  // namespace moveit_servo
