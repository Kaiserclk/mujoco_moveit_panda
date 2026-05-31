
#include <servo_control/utils/command.hpp>
#include <moveit/utils/logger.hpp>

namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.ros.servo");
}
}  // namespace

namespace servo_control
{

/**
 * @brief 从关节 jogging 命令生成关节增量
 * @param command 关节 jogging 命令
 * @param servo_params 舵机参数
 * @return 关节增量
 */
JointDeltaResult jointDeltaFromJointJog(const JointJogCommand& command, const servo_control::Params& servo_params)
{

  const auto joint_names = servo_params.joint_names;
  Eigen::VectorXd joint_position_delta(joint_names.size());
  Eigen::VectorXd velocities(joint_names.size());

  velocities.setZero();
  if (command.velocities.size() != command.names.size())
  {
    RCLCPP_WARN_STREAM(getLogger(), "Invalid joint jog command. Each joint name must have one corresponding "
                                    "velocity command. Received "
                                        << command.names.size() << " joints with " << command.velocities.size()
                                        << " commands.");
    return std::make_pair(StatusCode::INVALID, joint_position_delta);
  }

  for (size_t i = 0; i < command.names.size(); ++i)
  {
    auto it = std::find(joint_names.begin(), joint_names.end(), command.names[i]);
    if (it != std::end(joint_names))
    {
      velocities[std::distance(joint_names.begin(), it)] = command.velocities[i];
    }
    else
    {
      RCLCPP_WARN_STREAM(getLogger(), "Invalid joint name: " << command.names[i]
                                                             << "Either you're sending commands for a joint "
                                                                "that is not part of the certain joints "
                                                                "cannot be moved because a ");
      return std::make_pair(StatusCode::INVALID, joint_position_delta);
    }
  }

  if (!isValidCommand(velocities))
  {
    RCLCPP_WARN_STREAM(getLogger(), "Invalid velocity values in joint jog command");
    return std::make_pair(StatusCode::INVALID, joint_position_delta);
  }

  joint_position_delta = velocities * servo_params.publish_period;
  if (servo_params.command_in_type == "unitless")
  {
    joint_position_delta *= servo_params.scale.joint;
  }

  return std::make_pair(StatusCode::NO_WARNING, joint_position_delta);
}

JointDeltaResult jointDeltaFromTwist(const TwistCommand& command, const moveit::core::RobotStatePtr& robot_state,
                                     const servo_control::Params& servo_params, const std::string& planning_frame)
{
  StatusCode status = StatusCode::NO_WARNING;
  Eigen::VectorXd joint_position_delta(servo_params.joint_names.size());
  Eigen::Vector<double, 6> cartesian_position_delta;

  if (command.frame_id != planning_frame)
  {
    RCLCPP_WARN_STREAM(getLogger(), "Command frame is: " << command.frame_id << ", expected: " << planning_frame);
    return std::make_pair(StatusCode::INVALID, joint_position_delta);
  }

  if (!isValidCommand(command))
  {
    RCLCPP_WARN_STREAM(getLogger(), "Invalid twist command.");
    return std::make_pair(StatusCode::INVALID, joint_position_delta);
  }

  if (command.velocities.isZero())
  {
    joint_position_delta.setZero();
  }
  else
  {
    // Compute the Cartesian position delta based on incoming twist command.
    cartesian_position_delta = command.velocities * servo_params.publish_period;
    if (servo_params.command_in_type == "unitless")
    {
      // 无单位指令
      cartesian_position_delta.head<3>() *= servo_params.scale.linear;
      cartesian_position_delta.tail<3>() *= servo_params.scale.rotational;
    }
    else if (servo_params.command_in_type == "speed_units")
    {
      // 速度单位指令
      if (servo_params.scale.linear > 0.0)
      {
        const auto linear_speed_scale = command.velocities.head<3>().norm() / servo_params.scale.linear;
        if (linear_speed_scale > 1.0)
        {
          cartesian_position_delta.head<3>() /= linear_speed_scale;
        }
      }
      if (servo_params.scale.rotational > 0.0)
      {
        const auto angular_speed_scale = command.velocities.tail<3>().norm() / servo_params.scale.rotational;
        if (angular_speed_scale > 1.0)
        {
          cartesian_position_delta.tail<3>() /= angular_speed_scale;
        }
      }
    }

    // Compute the required change in joint angles.
    const auto delta_result =
        jointDeltaFromIK(cartesian_position_delta, robot_state, servo_params);
    status = delta_result.first;
    if (status != StatusCode::INVALID)
    {
      joint_position_delta = delta_result.second;
      // Get velocity scaling information for singularity.
      const auto singularity_scaling_info =
          velocityScalingFactorForSingularity(robot_state, cartesian_position_delta, servo_params);
      // Apply velocity scaling for singularity, if there was any scaling.
      if (singularity_scaling_info.second != StatusCode::NO_WARNING)
      {
        status = singularity_scaling_info.second;
        RCLCPP_WARN_STREAM(getLogger(), SERVO_STATUS_CODE_MAP.at(status));
        joint_position_delta *= singularity_scaling_info.first;
      }
    }
  }

  return std::make_pair(status, joint_position_delta);
}

JointDeltaResult jointDeltaFromPose(const PoseCommand& command, const moveit::core::RobotStatePtr& robot_state,
                                    const servo_control::Params& servo_params, const std::string& planning_frame,
                                    const std::string& ee_frame,
                                    const JointNameToMoveGroupIndexMap& joint_name_group_index_map)
{
  StatusCode status = StatusCode::NO_WARNING;
  Eigen::VectorXd joint_position_delta(servo_params.joint_names.size());

  if (!isValidCommand(command))
  {
    RCLCPP_WARN_STREAM(getLogger(), "Invalid pose command.");
    return std::make_pair(StatusCode::INVALID, joint_position_delta);
  }

  if (command.frame_id != planning_frame)
  {
    RCLCPP_WARN_STREAM(getLogger(), "Command frame is: " << command.frame_id << " expected: " << planning_frame);
    return std::make_pair(StatusCode::INVALID, joint_position_delta);
  }

  Eigen::Vector<double, 6> cartesian_position_delta;
  // Compute linear and angular change needed.
  const Eigen::Isometry3d ee_pose{ robot_state->getGlobalLinkTransform(planning_frame).inverse() *
                                   robot_state->getGlobalLinkTransform(ee_frame) };
  const Eigen::Quaterniond q_current(ee_pose.rotation());
  Eigen::Quaterniond q_target(command.pose.rotation());
  Eigen::Vector3d translation_error = command.pose.translation() - ee_pose.translation();//计算目标位置与当前位置的差值

  // Limit the commands by the maximum linear and angular speeds provided.
  if (servo_params.scale.linear > 0.0)
  {
    const auto linear_speed_scale =
        (translation_error.norm() / servo_params.publish_period) / servo_params.scale.linear;
    if (linear_speed_scale > 1.0)
    {
      translation_error /= linear_speed_scale;
    }
  }
  if (servo_params.scale.rotational > 0.0)
  {
    //计算两个四元数之间的角度差（最短旋转角度）
    const auto angular_speed_scale =
        (std::abs(q_target.angularDistance(q_current)) / servo_params.publish_period) / servo_params.scale.rotational;
    if (angular_speed_scale > 1.0)
    {
      //球面线性插值
      q_target = q_current.slerp(1.0 / angular_speed_scale, q_target);
    }
  }

  // Compute the Cartesian deltas from the velocity-scaled values.
  const auto angle_axis_error = Eigen::AngleAxisd(q_target * q_current.inverse());
  cartesian_position_delta.head<3>() = translation_error;
  cartesian_position_delta.tail<3>() = angle_axis_error.axis() * angle_axis_error.angle();

  // Compute the required change in joint angles.
  const auto delta_result =
      jointDeltaFromIK(cartesian_position_delta, robot_state, servo_params);
  status = delta_result.first;
  if (status != StatusCode::INVALID)
  {
    joint_position_delta = delta_result.second;
    // Get velocity scaling information for singularity.
    const auto singularity_scaling_info =
        velocityScalingFactorForSingularity(robot_state, cartesian_position_delta, servo_params);
    // Apply velocity scaling for singularity, if there was any scaling.
    if (singularity_scaling_info.second != StatusCode::NO_WARNING)
    {
      status = singularity_scaling_info.second;
      RCLCPP_WARN_STREAM(getLogger(), SERVO_STATUS_CODE_MAP.at(status));
      joint_position_delta *= singularity_scaling_info.first;
    }
  }
  return std::make_pair(status, joint_position_delta);
}

JointDeltaResult jointDeltaFromIK(const Eigen::VectorXd& cartesian_position_delta,
                                  const moveit::core::RobotStatePtr& robot_state, 
                                  const servo_control::Params& servo_params)
{
  const auto& group_name =servo_params.move_group_name;
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(group_name);

  std::vector<double> current_joint_positions;
  robot_state->copyJointGroupPositions(joint_model_group, current_joint_positions);

  Eigen::VectorXd delta_theta(servo_params.joint_names.size());
  StatusCode status = StatusCode::NO_WARNING;

  const kinematics::KinematicsBaseConstPtr ik_solver = joint_model_group->getSolverInstance();
  bool ik_solver_supports_group = true;
  if (ik_solver)
  {
    ik_solver_supports_group = ik_solver->supportsGroup(joint_model_group);
    if (!ik_solver_supports_group)
    {
      status = StatusCode::INVALID;
      RCLCPP_ERROR_STREAM(getLogger(), "Loaded IK plugin does not support group " << joint_model_group->getName());
    }
  }

  if (ik_solver && ik_solver_supports_group)
  {
    const Eigen::Isometry3d base_to_tip_frame_transform =
        robot_state->getGlobalLinkTransform(ik_solver->getBaseFrame()).inverse() *
        robot_state->getGlobalLinkTransform(ik_solver->getTipFrame());

    const geometry_msgs::msg::Pose next_pose =
        poseFromCartesianDelta(cartesian_position_delta, base_to_tip_frame_transform);

    // setup for IK call
    std::vector<double> solution;
    solution.reserve(current_joint_positions.size());
    moveit_msgs::msg::MoveItErrorCodes err;
    kinematics::KinematicsQueryOptions opts;
    opts.return_approximate_solution = true;
    if (ik_solver->searchPositionIK(next_pose, // 目标位姿
                                    current_joint_positions,// 初始猜测（当前关节角度）
                                    servo_params.publish_period / 2.0, // 采样时间
                                    solution,     // 输出：求解得到的关节角度
                                    err, opts))
    {
      // find the difference in joint positions that will get us to the desired pose
      for (size_t i = 0; i < current_joint_positions.size(); ++i)
      {
        delta_theta[i] = solution.at(i) - current_joint_positions.at(i);
      }
    }
    else
    {
      status = StatusCode::INVALID;
      RCLCPP_WARN_STREAM(getLogger(), "Could not find IK solution for requested motion, got error code " << err.val);
    }
  }
  else
  {
    //先略过此部分，后续再了解雅可比的解法
    // Robot does not have an IK solver, use inverse Jacobian to compute IK.
    const Eigen::MatrixXd jacobian = robot_state->getJacobian(joint_model_group);
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd =
        Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
    const Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();

    delta_theta = pseudo_inverse * cartesian_position_delta;
  }


  return std::make_pair(status, delta_theta);
}

}  // namespace moveit_servo
