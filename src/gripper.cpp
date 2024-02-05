#include <omega/gripper.h>
#include <omega/omega.h>

void omega::Gripper::_read_state(const turtlebot3_msgs::GraspState::ConstPtr &msg)
{
    _opened = msg->grasp_state == turtlebot3_msgs::GraspState::OPEN;
    _grasped = msg->object_grasped;
}

void omega::Gripper::_write_state(bool open)
{
    turtlebot3_msgs::GraspState command;
    command.grasp_state = open ? turtlebot3_msgs::GraspState::OPEN : turtlebot3_msgs::GraspState::CLOSE;
    _pub.publish(command);
}

omega::Gripper::Gripper(ros::NodeHandle *node, Omega *owner) : _owner(owner)
{
    //Config
    OMEGA_CONFIG("gripper/duration", duration);

    //Technical
    _sub = node->subscribe("joints/gripper_grasp_controller/state", 1, &Omega::grasp_state_update, owner);
    _pub = node->advertise<turtlebot3_msgs::GraspState>("joints/gripper_grasp_controller/command", 1, true);

    //State
    ROS_INFO("omega::Gripper waiting for joints/gripper_grasp_controller/state");
    boost::shared_ptr<const turtlebot3_msgs::GraspState> msg = ros::topic::waitForMessage<turtlebot3_msgs::GraspState>("joints/gripper_grasp_controller/state", ros::Duration(5.0));
    if (msg == nullptr) { const char *error = "Failed to receive gripper state"; ROS_ERROR("%s", error); throw std::runtime_error(error); }
    _read_state(msg);
    ROS_INFO("omega::Gripper received joints/gripper_grasp_controller/state");

    ROS_INFO("omega::Gripper initialized");
}

void omega::Gripper::update(ros::Time now, const turtlebot3_msgs::GraspState::ConstPtr &msg)
{
    update(now);
    _read_state(msg);
}

void omega::Gripper::update(ros::Time now)
{
    if (now >= _finish_time) _move_finished = true;
}

bool omega::Gripper::start_move(ros::Time now, bool open)
{
    _move_started = true;
    _move_finished = false;
    _finish_time = now + ros::Duration(duration);
    _write_state(open);
    return true;
}

bool omega::Gripper::get_move_started() const
{
    return _move_started;
}

bool omega::Gripper::get_move_finished() const
{
    return _move_finished;
}

bool omega::Gripper::get_grasped() const
{
    return _grasped;
}