#pragma once

#include <moveit_servo/utils/common.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace moveit_servo
{

/**
 * @brief 根据给定的关节点动指令计算关节位置偏移量
 * @param command 关节点动指令
 * @param robot_state_ 从规划场景监视器获取的机器人当前状态
 * @param servo_params 伺服控制参数
 * @param joint_name_group_index_map 关节子组名称与运动组关节向量位置的映射关系
 * @return 执行状态及所需的关节位置变化量（增量值）
 */
JointDeltaResult jointDeltaFromJointJog(const JointJogCommand& command, const moveit::core::RobotStatePtr& robot_state,
                                        const servo::Params& servo_params,
                                        const JointNameToMoveGroupIndexMap& joint_name_group_index_map);


/**
 * @brief 根据给定的 twist 动指令计算关节位置偏移量
 * @param command twist 动指令
 * @param robot_state_ 从规划场景监视器获取的机器人当前状态
 * @param servo_params 伺服控制参数
 * @param planning_frame 计划坐标系名称
 * @param joint_name_group_index_map 关节子组名称与运动组关节向量位置的映射关系
 * @return 执行状态及所需的关节位置变化量（增量值）
 */
JointDeltaResult jointDeltaFromTwist(const TwistCommand& command, const moveit::core::RobotStatePtr& robot_state,
                                     const servo::Params& servo_params, const std::string& planning_frame,
                                     const JointNameToMoveGroupIndexMap& joint_name_group_index_map);

/**
 * @brief 根据给定的位姿动指令计算关节位置偏移量
 * @param command 位姿动指令
 * @param robot_state_ 从规划场景监视器获取的机器人当前状态
 * @param servo_params 伺服控制参数
 * @param planning_frame 计划坐标系名称
 * @param ee_frame 末端执行器坐标系名称
 * @param joint_name_group_index_map 关节子组名称与运动组关节向量位置的映射关系
 * @return 执行状态及所需的关节位置变化量（增量值）
 */
JointDeltaResult jointDeltaFromPose(const PoseCommand& command, const moveit::core::RobotStatePtr& robot_state,
                                    const servo::Params& servo_params, const std::string& planning_frame,
                                    const std::string& ee_frame,
                                    const JointNameToMoveGroupIndexMap& joint_name_group_index_map);


/**
 * @brief 根据给定的笛卡尔坐标偏移量计算关节位置偏移量
 * @param cartesian_position_delta 笛卡尔坐标偏移量
 * @param robot_state_ 从规划场景监视器获取的机器人当前状态
 * @param servo_params 伺服控制参数
 * @param joint_name_group_index_map 关节子组名称与运动组关节向量位置的映射关系
 * @return 执行状态及所需的关节位置变化量（增量值）
 */
JointDeltaResult jointDeltaFromIK(const Eigen::VectorXd& cartesian_position_delta,
                                  const moveit::core::RobotStatePtr& robot_state, const servo::Params& servo_params,
                                  const JointNameToMoveGroupIndexMap& joint_name_group_index_map);

}  // namespace moveit_servo
