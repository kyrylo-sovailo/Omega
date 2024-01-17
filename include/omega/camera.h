#pragma once
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>

namespace omega
{
    class Omega;

    class Camera
    {
    private:
        Omega *_owner;
        image_transport::Subscriber _sub;
        image_transport::Subscriber _sub_info;
        Eigen::Matrix<double, 3, 4> _matrix;
        void _update(const sensor_msgs::CameraInfo::ConstPtr &msg);
        
    public:
        double blur_strength;
        int blur_size;

        Camera(ros::NodeHandle *node, Omega *owner);
        void update(const sensor_msgs::CameraInfo::ConstPtr &msg, cv::Mat &image);
        Eigen::Matrix<double, 3, 4> get_camera_matrix();
    };
}