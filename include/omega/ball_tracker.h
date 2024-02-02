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
            Eigen::Vector4d local_state;
            Eigen::Matrix4d local_var;
            Eigen::Vector4d global_state;
            Eigen::Matrix4d global_var;
            ros::Time last_seen;

            Eigen::Vector2d get_position(bool global = false) const;
            Eigen::Vector2d get_velocity(bool global = false) const;
            Eigen::Vector2d get_position_stddev(bool global = false) const;
            Eigen::Vector2d get_velocity_stddev(bool global = false) const;
        };
    
    private:
        struct BallVision
        {
            //Circles
            Eigen::Vector2d image_center;
            double image_radius, image_radius_variance;

            //Balls
            Eigen::Vector2d local_position;
            Eigen::Matrix2d local_position_variance;
            Eigen::Vector2d global_position;
            Eigen::Matrix2d global_position_variance;

            //Matching
            Ball *match = nullptr;
        };

        Omega *_owner;
        std::vector<Ball> _balls;
        bool _last_update_valid = false;
        ros::Time _last_update;

        void _find_countours(const cv::Mat &bgr_image, std::vector<std::vector<cv::Point>> *contours);
        void _find_circles(const std::vector<std::vector<cv::Point>> &contours, std::vector<BallVision> *balls);
        void _find_positions(std::vector<BallVision> *balls);
        void _match(ros::Time now, std::vector<BallVision> *balls);
        void _correct(ros::Time now, const std::vector<BallVision> &balls);

    public:
        //Geometry
        double ball_radius;
        //Probability
        double position_stddev_time, speed_stddev_time;
        //Image processing
        double min_radius, min_area;
        double timeout;
        double match_max_distance;
        int min_hue, max_hue, min_saturation, max_saturation, min_value, max_value;
        int dilate_size;

        BallTracker(ros::NodeHandle *node, Omega *owner);
        void update(ros::Time now, const cv::Mat &bgr_image);
        void update(ros::Time now, double linear, double angular);
        void update(ros::Time now);
        const std::vector<Ball> &get_balls() const;
    };
};