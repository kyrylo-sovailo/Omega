#pragma once
#include <opencv2/core.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

namespace omega
{
    class Debugger
    {
    private:
        cv::Mat _buffer;
        image_transport::Publisher _pub;
    
    public:
        static const cv::Scalar blue;
        static const cv::Scalar green;
        static const cv::Scalar red;

        bool active;
        
        Debugger(ros::NodeHandle *node);
        void draw_background(const cv::Mat &bgr_image);
        void draw_mask(const cv::Mat &binary_image);
        void draw_circle(cv::Point center, int radius, const cv::Scalar &color);
        void draw_rectangle(cv::Rect rect, const cv::Scalar &color);
        void publish();
    };
}