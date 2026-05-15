
#pragma once

#include <moveit_servo/collision_monitor.hpp>
#include <moveit_servo/moveit_servo_lib_parameters.hpp>
#include <moveit_servo/utils/command.hpp>
#include <moveit_servo/utils/datatypes.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/online_signal_smoothing/smoothing_base_class.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <variant>
#include <rclcpp/logger.hpp>
#include <queue>

namespace moveit_servo
{

class Servo
{
public:
  Servo(const rclcpp::Node::SharedPtr& node, std::shared_ptr<const servo::ParamListener> servo_param_listener,
        const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  ~Servo();

  // 禁用拷贝构造
  Servo(const Servo&) = delete;

  // 禁用拷贝赋值
  Servo& operator=(Servo&) = delete;

  /**
   * \brief 计算遵循给定命令所需的关节状态
   * @param robot_state 用于计算下一个关节状态的 RobotStatePtr 实例
   * @param command 要遵循的命令，std::variant 类型，可处理 JointJog、Twist 和 Pose
   * @return 所需的关节状态
   */
  KinematicState getNextJointState(const moveit::core::RobotStatePtr& robot_state, const ServoInput& command);

  /**
   * \brief 设置传入伺服命令的类型
   * @param command_type 伺服应该期望的命令类型
   */
  void setCommandType(const CommandType& command_type);

  /**
   * \brief 获取伺服当前期望的命令类型
   * @return 命令类型
   */
  CommandType getCommandType() const;

  /**
   * \brief 获取伺服的当前状态
   * @return 当前状态
   */
  StatusCode getStatus() const;

  /**
   * \brief 获取与当前伺服状态关联的消息
   * @return 状态消息
   */
  std::string getStatusMessage() const;

  /**
   * \brief 启动/停止碰撞检查线程
   * @param check_collision 为 false 时停止碰撞检查线程，为 true 时启动
   */
  void setCollisionChecking(const bool check_collision);

  /**
   * \brief 获取最新的伺服参数
   * @return 伺服参数
   */
  servo::Params& getParams();

  /**
   * \brief 获取规划场景监视器给出的机器人当前状态
   * 如果当前机器人状态立即可用，此方法可能会阻塞
   * @param block_for_current_state 如果为 true，则显式等待新的机器人状态
   * @return 机器人的当前状态
   */
  KinematicState getCurrentRobotState(bool block_for_current_state) const;

  /**
   * \brief 当命令过期时，在 commanded 状态下平滑停止
   * @param halt_state 期望的停止状态
   * @return 一个 pair，第一个元素是布尔值，表示机器人是否已停止，第二个是向期望停止状态步进的状态
   */
  std::pair<bool, KinematicState> smoothHalt(const KinematicState& halt_state);

  /**
   * \brief 如果设置了平滑插件，则对输入状态应用平滑处理
   * @param state 要被平滑插件更新的状态
   */
  void doSmoothing(KinematicState& state);

  /**
   * \brief 将平滑插件重置到指定状态（如果已设置）
   * @param state 期望将平滑插件重置到的状态
   */
  void resetSmoothing(const KinematicState& state);

private:
  /**
   * \brief 查找从规划帧到指定命令帧的变换
   * 如果命令帧是机器人模型的一部分，直接使用机器人模型查找变换
   * 否则，回退到使用 TF 查找变换
   * @param command_frame 命令帧名称
   * @param planning_frame 规划帧名称
   * @return 规划帧和命令帧之间的变换，如果查找变换失败则返回 std::nullopt
   */
  std::optional<Eigen::Isometry3d> getPlanningToCommandFrameTransform(const std::string& command_frame,
                                                                      const std::string& planning_frame) const;

  /**
   * \brief 将给定的速度命令转换到规划帧
   * `command.frame_id` 指定的命令帧应该是静止帧或末端执行器帧
   * 参考资料:
   * https://core.ac.uk/download/pdf/154240607.pdf
   * https://www.seas.upenn.edu/~meam520/notes02/Forces8.pdf
   * @param command 要转换的速度命令
   * @param planning_frame 规划帧的名称
   * @return 转换后的速度命令
   */
  std::optional<TwistCommand> toPlanningFrame(const TwistCommand& command, const std::string& planning_frame) const;

  /**
   * \brief 将给定的位姿命令转换到规划帧
   * @param command 要转换的位姿命令
   * @param planning_frame 规划帧的名称
   * @return 转换后的位姿命令
   */
  std::optional<PoseCommand> toPlanningFrame(const PoseCommand& command, const std::string& planning_frame) const;

  /**
   * \brief 计算遵循接收到的命令所需的关节位置变化
   * @param command 传入的伺服命令
   * @param robot_state 用于计算命令的 RobotStatePtr 实例
   * @return 所需的关节位置变化（增量）
   */
  Eigen::VectorXd jointDeltaFromCommand(const ServoInput& command, const moveit::core::RobotStatePtr& robot_state);

  /**
   * \brief 验证伺服参数
   * @param servo_params 伺服参数
   * @return 如果参数有效则返回 true，否则返回 false
   */
  bool validateParams(const servo::Params& servo_params);

  /**
   * \brief 更新伺服参数并执行验证
   */
  bool updateParams();

  /**
   * \brief 创建并初始化伺服要使用的平滑插件
   */
  void setSmoothingPlugin();

  /**
   * \brief 对指定关节应用停止逻辑
   * @param joints_to_halt 要停止的关节索引
   * @param current_state 当前运动学状态
   * @param target_state 目标运动学状态
   * @return 边界约束后的运动学状态
   */
  KinematicState haltJoints(const std::vector<size_t>& joints_to_halt, const KinematicState& current_state,
                            const KinematicState& target_state) const;

  // 变量
  StatusCode servo_status_;
  // 此变量需要线程安全，以便可以实时更新
  std::atomic<CommandType> expected_command_type_;

  servo::Params servo_params_;
  const rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::shared_ptr<const servo::ParamListener> servo_param_listener_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // 此值将由 CollisionMonitor 在单独的线程中更新
  std::atomic<double> collision_velocity_scale_ = 1.0;
  std::unique_ptr<CollisionMonitor> collision_monitor_;

  // 插件加载器
  std::unique_ptr<pluginlib::ClassLoader<online_signal_smoothing::SmoothingBaseClass>> smoother_loader_;

  // 指向（可选）平滑插件的指针
  pluginlib::UniquePtr<online_signal_smoothing::SmoothingBaseClass> smoother_ = nullptr;

  // 关节子组名称与对应关节名称 - move group 索引映射之间的关系
  std::unordered_map<std::string, JointNameToMoveGroupIndexMap> joint_name_to_index_maps_;

  // 每个活动关节位置变量的当前关节限制安全边界
  std::vector<double> joint_limit_margins_;
};

}  // namespace moveit_servo
