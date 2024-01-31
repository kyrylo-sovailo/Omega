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
        bool _find_visible_segment(Eigen::Vector3d *source, Eigen::Vector3d *destination);
        void _project_line();
        Eigen::Vector2d _project_point(Eigen::Vector3d point);
        Eigen::Vector2d _project_line(Eigen::Vector3d source, Eigen::Vector3d destination);
    
    public:
        RobotTracker(ros::NodeHandle *node, Omega *owner);
        void update(ros::Time time, const cv::Mat &image);
        void update(ros::Time time, double linear, double angular);
        void update(ros::Time time);

        Eigen::Vector2d get_position() const;
        double get_orientation() const;
        Eigen::Vector2d get_position_stddev() const;
        double get_orientation_stddev() const;
        Eigen::Matrix3d get_variance() const;
    };
};