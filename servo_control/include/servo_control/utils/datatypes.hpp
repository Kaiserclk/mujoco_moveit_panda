
#pragma once

#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <unordered_map>

namespace servo_control
{

enum class StatusCode : int8_t  
{
  INVALID = -1,                              // 无效状态
  NO_WARNING = 0,                            // 正常，无警告
  DECELERATE_FOR_APPROACHING_SINGULARITY = 1, // 靠近奇异点，减速
  HALT_FOR_SINGULARITY = 2,                  // 非常接近奇异点，紧急停止
  DECELERATE_FOR_LEAVING_SINGULARITY = 3,    // 离开奇异点，减速
  HALT_FOR_EMERGENCY = 4,                    // 紧急停止
  JOINT_BOUND = 5,                           // 接近关节限位（位置或速度），停止
  DECELERATE_FOR_COLLISION = 6,              // 碰撞风险，减速
  HALT_FOR_COLLISION = 7                     // 碰撞风险，紧急停止
};

const std::unordered_map<StatusCode, std::string> SERVO_STATUS_CODE_MAP(
    { { StatusCode::INVALID, "Invalid" },
      { StatusCode::NO_WARNING, "No warnings" },
      { StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY, "Moving closer to a singularity, decelerating" },
      { StatusCode::HALT_FOR_SINGULARITY, "Very close to a singularity, emergency stop" },
      { StatusCode::DECELERATE_FOR_LEAVING_SINGULARITY, "Moving away from a singularity, decelerating" },
      { StatusCode::HALT_FOR_EMERGENCY, "Emergency stop" },
      { StatusCode::JOINT_BOUND, "Close to a joint bound (position or velocity), halting" },
      { StatusCode::DECELERATE_FOR_COLLISION, "Close to a collision, decelerating" },
      { StatusCode::HALT_FOR_COLLISION, "Collision detected, emergency stop" } });

// The datatype that specifies the type of command that servo should expect.
enum class CommandType : int8_t  
{
  JOINT_JOG = 0,    // 关节空间点动命令
  TWIST = 1,        // 笛卡尔空间速度命令
  POSE = 2,         // 笛卡尔空间位姿命令

  // 允许值的范围，用于参数验证
  MIN = JOINT_JOG,  // 最小值 = 0
  MAX = POSE        // 最大值 = 2
};

typedef std::pair<StatusCode, Eigen::VectorXd> JointDeltaResult;

// 关节点动指令，该指令为向量形式，向量长度与机器人关节数量一致。
struct JointJogCommand
{
  std::vector<std::string> names; // 关节名称列表
  std::vector<double> velocities;  // 对应的关节速度
};

// The twist command,  frame_id is the name of the frame in which the command is specified in.
// frame_id must always be specified.
struct TwistCommand
{
  std::string frame_id; // 坐标系名称
  Eigen::Vector<double, 6> velocities; // 速度向量
};

// The Pose command,  frame_id is the name of the frame in which the command is specified in.
// frame_id must always be specified.
struct PoseCommand
{
  std::string frame_id; // 坐标系名称
  Eigen::Isometry3d pose; // 姿态矩阵
};

// The generic input type for servo that can be JointJog, Twist or Pose.
typedef std::variant<JointJogCommand, TwistCommand, PoseCommand> ServoInput;

// The output datatype of servo, this structure contains the names of the joints along with their positions, velocities and accelerations.

struct KinematicState
{
  std::vector<std::string> joint_names;// 关节名称列表
  Eigen::VectorXd positions, velocities, accelerations; // 关节位置、速度和加速度向量
  rclcpp::Time time_stamp;// 时间戳

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

}  // namespace servo_control
