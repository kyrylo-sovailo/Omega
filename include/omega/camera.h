#pragma once
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
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
        ros::Subscriber _sub_info;
        
        void _update(const sensor_msgs::CameraInfo::ConstPtr &msg);
        
    public:
        //Geometry
        Eigen::Vector3d base_position;
        //Image processing
        double blur_strength;
        int blur_size;
        //Runtime parameters
        unsigned int width, height;
        Eigen::Matrix<double, 4, 4> matrix, matrix_inv;

        Camera(ros::NodeHandle *node, Omega *owner);
        void update(const sensor_msgs::ImageConstPtr &msg, cv::Mat &hsv_image);
    };
}