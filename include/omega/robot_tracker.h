#pragma once
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>

namespace omega
{
    class Omega;

    class RobotTracker
    {
    private:
        Omega *_owner;
        Eigen::Vector3d _state;
        Eigen::Matrix3d _variance;
    
    public:
        RobotTracker(ros::NodeHandle *node, Omega *owner);
        void update(ros::Time time, const cv::Mat &image);
        void update(ros::Time time, double linear, double angular);
        void update(ros::Time time);

        Eigen::Vector2d get_position() const;
        double get_orientation() const;
        Eigen::Vector2d get_position_stddev() const;
        double get_orientation_stddev() const;
    };
};