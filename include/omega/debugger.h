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
        bool active;
        
        Debugger(ros::NodeHandle *node);
        void draw_background(const cv::Mat &bgr_image);
        void draw_mask(const cv::Mat &binary_image);
        void draw_circle();
        void publish();
    };
}