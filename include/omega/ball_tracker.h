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
        struct BallVision
        {
            //Circles
            Eigen::Vector2d image_center;
            double image_radius, image_radius_variance;

            //Balls
            Eigen::Vector2d ball_position;
            Eigen::Matrix2d ball_position_variance;

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
        void _update(ros::Time now, const std::vector<BallVision> &balls);

    public:
        //Physical parameters
        double radius;
        double position_stddev_time, speed_stddev_time;

        //Image processing
        int min_hue, max_hue, min_saturation, max_saturation, min_value, max_value;
        int dilate_size;

        //Detection
        double min_radius, min_area;
        double timeout;

        BallTracker(ros::NodeHandle *node, Omega *owner);
        void update(ros::Time time, const cv::Mat &bgr_image);
        void update(ros::Time time);
        const std::vector<Ball> &get_balls() const;
    };
};