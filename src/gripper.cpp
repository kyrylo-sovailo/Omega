#include <omega/omega.h>

omega::Gripper::Gripper(ros::NodeHandle &node, OmegaHeader *header)
{
    _sub = image_transfer.subscribe("joints/gripper_grasp_controller/state", 1, &OmegaHeader::grasp_state_callback, header);
    _pub = node.advertise<turtlebot3_msgs::GraspState>("joints/gripper_grasp_controller/command", 1, true);
}

void omega::Gripper::update(sensor_msgs::JointState::ConstPtr msg)
{}

void omega::Gripper::update(turtlebot3_msgs::GraspState::ConstPtr msg)
{
    _grasped = msg->object_grasped;
}

void omega::Gripper::open(double duration)
{
    turtlebot3_msgs::GraspState command;
    command2.grasp_state = turtlebot3_msgs::GraspState::OPEN;
    _gripper_pub.publish(command);
}

void omega::Gripper::close(double duration)
{
    turtlebot3_msgs::GraspState command;
    command2.grasp_state = turtlebot3_msgs::GraspState::CLOSE;
    _gripper_pub.publish(command);
}

bool omega::Gripper::grasped() const
{
    return _grasped;
}