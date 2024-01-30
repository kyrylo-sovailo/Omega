#include <omega/debugger.h>
#include <omega/omega.h>
#include <cv_bridge/cv_bridge.h>

omega::Debugger::Debugger(ros::NodeHandle *node)
{
    OMEGA_CONFIG("debugger/active", active);

    if (active)
    {
        image_transport::ImageTransport image_transfer(*node);
        _pub = image_transfer.advertise("debugger/image", 1);
    }
}

void omega::Debugger::draw_background(const cv::Mat &bgr_image)
{
    if (active)
    {
        _buffer = bgr_image;
    }
}

void omega::Debugger::draw_mask(const cv::Mat &binary_image)
{
    if (active)
    {
        std::vector<cv::Mat> channels(3);
        cv::split(_buffer, channels);
        cv::bitwise_or(channels[0], binary_image, channels[0]);
        cv::bitwise_or(channels[1], binary_image, channels[1]);
        cv::bitwise_or(channels[2], binary_image, channels[2]);
        cv::merge(channels, _buffer);
    }
}

void omega::Debugger::publish()
{
    if (active)
    {
        _pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", _buffer).toImageMsg());
    }
}