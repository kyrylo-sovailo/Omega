#include <omega/robot_tracker.h>
#include <omega/omega.h>

omega::RobotTracker::RobotTracker(ros::NodeHandle *node, Omega *owner) : _owner(owner)
{}

void omega::RobotTracker::update(ros::Time time, const cv::Mat &image)
{}

void omega::RobotTracker::update(ros::Time time, double linear, double angular)
{}

void omega::RobotTracker::update(ros::Time time)
{}

Eigen::Vector2d omega::RobotTracker::get_position() const
{
    return Eigen::Vector2d(_state(0), _state(1));
}

double omega::RobotTracker::get_orientation() const
{
    return _state(2);
}

Eigen::Vector2d omega::RobotTracker::get_position_stddev() const
{
    return Eigen::Vector2d(std::sqrt(_variance(0, 0)), std::sqrt(_variance(1, 1)));
}

double omega::RobotTracker::get_orientation_stddev() const
{
    return std::sqrt(_variance(2, 2));
}

Eigen::Matrix3d omega::RobotTracker::get_variance() const
{
    return _variance;
}