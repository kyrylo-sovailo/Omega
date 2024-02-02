#pragma once
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

namespace omega
{
    class Omega;

    class Debugger
    {
    private:
        Omega *_owner;
        cv::Mat _buffer;
        image_transport::Publisher _pub;
    
    public:
        static const cv::Scalar blue;
        static const cv::Scalar green;
        static const cv::Scalar red;

        bool active;
        
        Debugger(ros::NodeHandle *node, Omega *owner);
        void draw_background(const cv::Mat &bgr_image);
        void draw_mask(const cv::Mat &binary_image);
        void draw_rectangle(cv::Rect rect, const cv::Scalar &color);
        void draw_circle(const Eigen::Vector2d &center, double radius, const cv::Scalar &color);
        void draw_line(const Eigen::Vector2d &source, const Eigen::Vector2d &destination, const cv::Scalar &color);
        void draw_line(double r, double theta, const cv::Scalar &color);
        void publish();
    };
}