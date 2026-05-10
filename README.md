# mujoco_moveit_panda
使用mujoco仿真panda机械臂，并通过moveit运动控制

##环境准备
- 从源码编译mujoco_ros2_control
git clone https://github.com/ros-controls/mujoco_ros2_control.git

colcon build --symlink-install --packages-select mujoco_ros2_control* --cmake-args -DCMAKE_BUILD_TYPE=Release



# 在mujuco中加载panda
ros2 launch panda_bringup mujoco_panda.launch.py