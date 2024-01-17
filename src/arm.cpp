#include <omega/arm.h>

omega::Arm::Arm(ros::NodeHandle &node, OmegaHeader *header)
{
    OMEGA_CONFIG_VECTOR("arm/base_position", 3, base_position);
    OMEGA_CONFIG_VECTOR("arm/idle_position", 3, idle_position);
    OMEGA_CONFIG("arm/idle_pitch", idle_pitch);
    OMEGA_CONFIG_VECTOR("arm/l", 4, idle_position);
    OMEGA_CONFIG_VECTOR("arm/q_init", 4, idle_position);
    OMEGA_CONFIG_VECTOR("arm/w_max", 4, idle_position);
    _pub = node.advertise<trajectory_msgs::JointTrajectory>("joints/arm_trajectory_controller/command", 1, true);
}

void omega::Arm::update(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (unsigned int i = 0; i < msg.name.size(); i++)
    {
        if (msg.name[i] == "arm_joint_1") _position(0) = msg.position[i];
        else if (msg.name[i] == "arm_joint_2") _position(1) = msg.position[i];
        else if (msg.name[i] == "arm_joint_3") _position(2) = msg.position[i];
        else if (msg.name[i] == "arm_joint_4") _position(3) = msg.position[i];
    }
}

void omega::Arm::start_move_idle()
{
   
}

void omega::Arm::start_move(const Eigen::Vector3d point, double pitch)
{
    trajectory_msgs::JointTrajectory command;
    command.joint_names = std::vector<std::string>{"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4"};
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = std::vector<double>{ 0.5, 0.5, 0.5, 0.5 }; //TODO: Inverse kinematics, calculate remaining time
    point.time_from_start = ros::Duration(0.5);
    command.points.push_back(point);
    _pub.publish(command);
}

void omega::Arm::abort_move()
{
    trajectory_msgs::JointTrajectory command;
    command.joint_names = std::vector<std::string>{"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4"};
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(4);
    Eigen::Vector4d::Map(point.positions.data()) = _position;
    point.time_from_start = ros::Duration(0.5);
    command.points.push_back(point);
    _pub.publish(command);
}

ros::Duration omega::Arm::get_time_remaining() const
{
    //TODO
}

double omega::Arm::get_cartesian_error() const
{
    //TODO: Forward kinematics
}

double omega::Arm::get_angular_error() const
{
    //TODO: Forward kinematics
}