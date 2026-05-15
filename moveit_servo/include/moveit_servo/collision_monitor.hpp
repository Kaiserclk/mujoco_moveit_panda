#pragma once
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_servo/moveit_servo_lib_parameters.hpp>

namespace moveit_servo
{
class CollisionMonitor
{
public:
  CollisionMonitor(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                   const servo::Params& servo_params, std::atomic<double>& collision_velocity_scale);

  void start();

  void stop();

private:
  /**
   * \brief The collision checking function, this will run in a separate thread.
   */
  void checkCollisions();

  // Variables

  const servo::Params& servo_params_;

  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  moveit::core::RobotState robot_state_;

  // The collision monitor thread.
  std::thread monitor_thread_;
  // The flag used for stopping the collision monitor thread.
  std::atomic<bool> stop_requested_;

  // The scaling factor when approaching a collision.
  std::atomic<double>& collision_velocity_scale_;

  // The data structures used to get information about robot self collisions.
  collision_detection::CollisionRequest self_collision_request_;
  collision_detection::CollisionResult self_collision_result_;
  // The data structures used to get information about robot collision with other objects in the collision scene.
  collision_detection::CollisionRequest scene_collision_request_;
  collision_detection::CollisionResult scene_collision_result_;
};

}  // namespace moveit_servo
