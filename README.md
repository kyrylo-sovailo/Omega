# Omega

### Dependencies
```
git clone git@git.sim.informatik.tu-darmstadt.de:TurtleBot/turtlebot3_simulation.git --branch logitech_c930e --depth 1
git clone git@git.sim.informatik.tu-darmstadt.de:TurtleBot/tuda_turtlebot3_description.git --branch logitech_c930e --depth 1
git clone git@git.sim.informatik.tu-darmstadt.de:TurtleBot/tuda_turtlebot3_camera_description.git --branch melodic-devel --depth 1
git clone git@git.sim.informatik.tu-darmstadt.de:TurtleBot/turtlebot3_launch.git --branch master --depth 1
git clone git@git.sim.informatik.tu-darmstadt.de:TurtleBot/tuda_turtlebot3_ros_control.git --branch master --depth 1
git clone https://github.com/team-vigir/vigir_lidar_proc.git --branch master --depth 1
git clone https://github.com/team-vigir/robot_self_filter.git --branch noetic-devel --depth 1
```

### Notes
Names and types of topics of working environment:
```
$ for t in $(rostopic list); do echo $t is $(rostopic type $t); done
/clock is rosgraph_msgs/Clock
/gazebo/link_states is gazebo_msgs/LinkStates
/gazebo/model_states is gazebo_msgs/ModelStates
/gazebo/omega/joints/arm_joint_1_position/command is std_msgs/Float64
/gazebo/omega/joints/arm_joint_1_position/pid/parameter_descriptions is dynamic_reconfigure/ConfigDescription
/gazebo/omega/joints/arm_joint_1_position/pid/parameter_updates is dynamic_reconfigure/Config
/gazebo/omega/joints/arm_joint_1_position/state is control_msgs/JointControllerState
/gazebo/omega/joints/arm_joint_2_position/command is std_msgs/Float64
/gazebo/omega/joints/arm_joint_2_position/pid/parameter_descriptions is dynamic_reconfigure/ConfigDescription
/gazebo/omega/joints/arm_joint_2_position/pid/parameter_updates is dynamic_reconfigure/Config
/gazebo/omega/joints/arm_joint_2_position/state is control_msgs/JointControllerState
/gazebo/omega/joints/arm_joint_3_position/command is std_msgs/Float64
/gazebo/omega/joints/arm_joint_3_position/pid/parameter_descriptions is dynamic_reconfigure/ConfigDescription
/gazebo/omega/joints/arm_joint_3_position/pid/parameter_updates is dynamic_reconfigure/Config
/gazebo/omega/joints/arm_joint_3_position/state is control_msgs/JointControllerState
/gazebo/omega/joints/arm_joint_4_position/command is std_msgs/Float64
/gazebo/omega/joints/arm_joint_4_position/pid/parameter_descriptions is dynamic_reconfigure/ConfigDescription
/gazebo/omega/joints/arm_joint_4_position/pid/parameter_updates is dynamic_reconfigure/Config
/gazebo/omega/joints/arm_joint_4_position/state is control_msgs/JointControllerState
/gazebo/omega/joints/gripper_servo_joint_position/command is std_msgs/Float64
/gazebo/omega/joints/gripper_servo_joint_position/pid/parameter_descriptions is dynamic_reconfigure/ConfigDescription
/gazebo/omega/joints/gripper_servo_joint_position/pid/parameter_updates is dynamic_reconfigure/Config
/gazebo/omega/joints/gripper_servo_joint_position/state is control_msgs/JointControllerState
/gazebo/omega/joints/joint_states is sensor_msgs/JointState
/gazebo/omega/joints/wheel_left_controller/command is std_msgs/Float64
/gazebo/omega/joints/wheel_left_controller/pid/parameter_descriptions is dynamic_reconfigure/ConfigDescription
/gazebo/omega/joints/wheel_left_controller/pid/parameter_updates is dynamic_reconfigure/Config
/gazebo/omega/joints/wheel_left_controller/state is control_msgs/JointControllerState
/gazebo/omega/joints/wheel_left_joint_velocity/command is std_msgs/Float64
/gazebo/omega/joints/wheel_right_controller/command is std_msgs/Float64
/gazebo/omega/joints/wheel_right_controller/pid/parameter_descriptions is dynamic_reconfigure/ConfigDescription
/gazebo/omega/joints/wheel_right_controller/pid/parameter_updates is dynamic_reconfigure/Config
/gazebo/omega/joints/wheel_right_controller/state is control_msgs/JointControllerState
/gazebo/omega/joints/wheel_right_joint_velocity/command is std_msgs/Float64
/gazebo/parameter_descriptions is dynamic_reconfigure/ConfigDescription
/gazebo/parameter_updates is dynamic_reconfigure/Config
/gazebo/performance_metrics is gazebo_msgs/PerformanceMetrics
/gazebo/set_link_state is gazebo_msgs/LinkState
/gazebo/set_model_state is gazebo_msgs/ModelState
/grasp_state/manipulator_arm/object_grasped is std_msgs/Bool
/omega/camera/parameter_descriptions is dynamic_reconfigure/ConfigDescription
/omega/camera/parameter_updates is dynamic_reconfigure/Config
/omega/camera/rgb/camera_info is sensor_msgs/CameraInfo
/omega/camera/rgb/image_raw is sensor_msgs/Image
/omega/camera/rgb/image_raw/compressed is sensor_msgs/CompressedImage
/omega/camera/rgb/image_raw/compressed/parameter_descriptions is dynamic_reconfigure/ConfigDescription
/omega/camera/rgb/image_raw/compressed/parameter_updates is dynamic_reconfigure/Config
/omega/client_count is std_msgs/Int32
/omega/connected_clients is rosbridge_msgs/ConnectedClients
/omega/joints/arm_trajectory_controller/command is trajectory_msgs/JointTrajectory
/omega/joints/arm_trajectory_controller/follow_joint_trajectory/cancel is actionlib_msgs/GoalID
/omega/joints/arm_trajectory_controller/follow_joint_trajectory/feedback is control_msgs/FollowJointTrajectoryActionFeedback
/omega/joints/arm_trajectory_controller/follow_joint_trajectory/goal is control_msgs/FollowJointTrajectoryActionGoal
/omega/joints/arm_trajectory_controller/follow_joint_trajectory/result is control_msgs/FollowJointTrajectoryActionResult
/omega/joints/arm_trajectory_controller/follow_joint_trajectory/status is actionlib_msgs/GoalStatusArray
/omega/joints/arm_trajectory_controller/state is control_msgs/JointTrajectoryControllerState
/omega/joints/gripper_grasp_controller/command is turtlebot3_msgs/GraspState
/omega/joints/gripper_grasp_controller/state is turtlebot3_msgs/GraspState
/omega/joints/gripper_trajectory_controller/command is trajectory_msgs/JointTrajectory
/omega/joints/gripper_trajectory_controller/follow_joint_trajectory/cancel is actionlib_msgs/GoalID
/omega/joints/gripper_trajectory_controller/follow_joint_trajectory/feedback is control_msgs/FollowJointTrajectoryActionFeedback
/omega/joints/gripper_trajectory_controller/follow_joint_trajectory/goal is control_msgs/FollowJointTrajectoryActionGoal
/omega/joints/gripper_trajectory_controller/follow_joint_trajectory/result is control_msgs/FollowJointTrajectoryActionResult
/omega/joints/gripper_trajectory_controller/follow_joint_trajectory/status is actionlib_msgs/GoalStatusArray
/omega/joints/gripper_trajectory_controller/state is control_msgs/JointTrajectoryControllerState
/omega/joints/joint_states is sensor_msgs/JointState
/omega/joints/stop_write is std_msgs/Bool
/omega/joints/wheel_left_controller/command is std_msgs/Float64
/omega/joints/wheel_left_controller/state is control_msgs/JointControllerState
/omega/joints/wheel_right_controller/command is std_msgs/Float64
/omega/joints/wheel_right_controller/state is control_msgs/JointControllerState
/omega/laserscan_filtered is sensor_msgs/LaserScan
/omega/odom is nav_msgs/Odometry
/omega/odom_to_tf/euler is geometry_msgs/Vector3Stamped
/omega/odom_to_tf/pose is geometry_msgs/PoseStamped
/omega/scan_cloud is sensor_msgs/PointCloud2
/omega/scan_cloud_filtered is sensor_msgs/PointCloud2
/omega/scan_cloud_filtered_containment is sensor_msgs/PointCloud2
/omega/self_filter_nodelet_manager/bond is bond/Status
/omega/sensor/camera/camera_info is sensor_msgs/CameraInfo
/omega/sensor/camera/image_rect_color is sensor_msgs/Image
/omega/sensor/imu_filtered is sensor_msgs/Imu
/omega/sensor/imu_raw is sensor_msgs/Imu
/omega/sensor/scan_raw is sensor_msgs/LaserScan
/rosout is rosgraph_msgs/Log
/rosout_agg is rosgraph_msgs/Log
/tf is tf2_msgs/TFMessage
/tf_static is tf2_msgs/TFMessage
```

GIMP to OpenCV HSV values:
GIMP:
 - H in [0..360]
 - S in [0..100]
 - V in [0..100]
OpenCV:
 - H in [0..179]
 - S in [0..255]
 - V in [0..255]
