#include <omega/ball_tracker.h>
#include <omega/omega.h>
#include <cmath>

Eigen::Vector2d omega::BallTracker::Ball::get_position() const
{
    return Eigen::Vector2d(state(0), state(1));
}

Eigen::Vector2d omega::BallTracker::Ball::get_velocity() const
{
    return Eigen::Vector2d(state(2), state(3));
}

Eigen::Vector2d omega::BallTracker::Ball::get_position_stddev() const
{
    return Eigen::Vector2d(std::sqrt(state(0, 0)), std::sqrt(state(1, 1)));
}

Eigen::Vector2d omega::BallTracker::Ball::get_velocity_stddev() const
{
    return Eigen::Vector2d(std::sqrt(state(2, 2)), std::sqrt(state(3, 3)));
}

omega::BallTracker::BallTracker(ros::NodeHandle *node, Omega *owner) : _owner(owner)
{}

void omega::BallTracker::update(ros::Time time, const cv::Mat &image)
{}

void omega::BallTracker::update(ros::Time time, double linear, double angular)
{}

void omega::BallTracker::update(ros::Time time)
{}

const std::vector<omega::BallTracker::Ball> &omega::BallTracker::get_balls() const
{
    return _balls;
}