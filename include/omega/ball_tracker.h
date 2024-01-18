#pragma once
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>

namespace omega
{
    class Omega;

    ///Detects and tracks balls
    class BallTracker
    {
    public:
        struct Ball
        {
            Eigen::Vector4d state;
            Eigen::Matrix4d var;
            ros::Time last_seen;

            Eigen::Vector2d get_position() const;
            Eigen::Vector2d get_velocity() const;
            Eigen::Vector2d get_position_stddev() const;
            Eigen::Vector2d get_velocity_stddev() const;
        };
    
    private:
        Omega *_owner;
        std::vector<Ball> _balls;

    public:
        double radius;
        double roll_distance;
        double timeout;
        int dilate_size;
        int min_hue, max_hue, min_saturation, max_saturation, min_value, max_value;

        BallTracker(ros::NodeHandle *node, Omega *owner);
        void update(ros::Time time, const cv::Mat &image);
        void update(ros::Time time, double linear, double angular);
        void update(ros::Time time);
        const std::vector<Ball> &get_balls() const;
    };
};