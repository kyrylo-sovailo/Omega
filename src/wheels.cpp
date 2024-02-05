#include <omega/wheels.h>
#include <omega/timer.h>
#include <omega/omega.h>
#include <std_msgs/Float64.h>

void omega::Wheels::_read_state(const sensor_msgs::JointState::ConstPtr &msg)
{
    double left_speed, right_speed;
    for (unsigned int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == "wheel_left") left_speed = msg->velocity[i];
        else if (msg->name[i] == "wheel_right") right_speed = msg->velocity[i];
    }
    _linear_speed = (right_speed + left_speed) * wheel_radius;
    _angular_speed = (right_speed - left_speed) * wheel_radius / wheel_separation;
}

omega::Wheels::Wheels(ros::NodeHandle *node, Omega *owner) : _owner(owner)
{
    //Config
    OMEGA_CONFIG("wheels/wheel_radius", wheel_radius);
    OMEGA_CONFIG("wheels/wheel_separation", wheel_separation);

    OMEGA_CONFIG("wheels/linear_stddev_linear", linear_stddev_linear);
    OMEGA_CONFIG("wheels/angular_stddev_angular", angular_stddev_angular);

    OMEGA_CONFIG("wheels/max_wheel_vel", max_wheel_vel);
    OMEGA_CONFIG("wheels/max_linear_vel", max_linear_vel);
    OMEGA_CONFIG("wheels/max_linear_acc", max_linear_acc);
    OMEGA_CONFIG("wheels/max_angular_vel", max_angular_vel);
    OMEGA_CONFIG("wheels/max_angular_acc", max_angular_acc);

    //Technical
    _left_pub = node->advertise<std_msgs::Float64>("joints/wheel_left_controller/command", 1, true);
    _right_pub = node->advertise<std_msgs::Float64>("joints/wheel_right_controller/command", 1, true);

    ROS_INFO("omega::Wheels initialized");
}

void omega::Wheels::update(ros::Time now, const sensor_msgs::JointState::ConstPtr &msg)
{
    _read_state(msg);
}

void omega::Wheels::update(ros::Time now)
{
    //Limit both linear and angular goal by factor
    std_msgs::Float64 command_right, command_left;
    double factor = 1;
    command_right.data = (_linear_end_goal + 0.5 * _angular_end_goal * wheel_separation) / wheel_radius;
    command_left.data  = (_linear_end_goal - 0.5 * _angular_end_goal * wheel_separation) / wheel_radius;
    if (std::abs(command_right.data) > max_wheel_vel  ) factor = std::min(factor, max_wheel_vel   / std::abs(command_right.data));
    if (std::abs(command_left.data)  > max_wheel_vel  ) factor = std::min(factor, max_wheel_vel   / std::abs(command_left.data));
    if (std::abs(_linear_end_goal)   > max_linear_vel ) factor = std::min(factor, max_linear_vel  / std::abs(_linear_end_goal));
    if (std::abs(_angular_end_goal)  > max_angular_vel) factor = std::min(factor, max_angular_vel / std::abs(_angular_end_goal));

    //Limit linear and angular goal separately by acceleration
    const double dt = std::min((now - _last_sent).toSec(), 1 / _owner->timer->frequency);
    _linear_goal = std::max(_linear_goal - dt * max_linear_acc, std::min(factor * _linear_end_goal, _linear_goal + dt * max_linear_acc));
    _angular_goal = std::max(_angular_goal - dt * max_angular_acc, std::min(factor * _angular_end_goal, _angular_goal + dt * max_angular_acc));
    command_right.data = (_linear_goal + 0.5 * _angular_goal * wheel_separation) / wheel_radius;
    command_left.data  = (_linear_goal - 0.5 * _angular_goal * wheel_separation) / wheel_radius;

    //Send
    _last_sent = now;
    _right_pub.publish(command_right);
    _left_pub.publish(command_left);
}

void omega::Wheels::set_speed(double linear, double angular)
{
    _linear_end_goal = linear;
    _angular_end_goal = angular;
}

void omega::Wheels::get_speed(double *linear, double *angular)
{
    *linear = _linear_speed;
    *angular = _angular_speed;
}