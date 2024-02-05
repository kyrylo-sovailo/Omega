#include <omega/debugger.h>
#include <omega/camera.h>
#include <omega/omega.h>
#include <cv_bridge/cv_bridge.h>

const cv::Scalar omega::Debugger::black = cv::Scalar(0, 0, 0);
const cv::Scalar omega::Debugger::white = cv::Scalar(255, 255, 255);
const cv::Scalar omega::Debugger::blue = cv::Scalar(255, 0, 0);
const cv::Scalar omega::Debugger::green = cv::Scalar(0, 255, 0);
const cv::Scalar omega::Debugger::red = cv::Scalar(0, 0, 255);
const cv::Scalar omega::Debugger::yellow = cv::Scalar(0, 255, 255);

omega::Debugger::Debugger(ros::NodeHandle *node, Omega *owner) : _owner(owner)
{
    OMEGA_CONFIG("debugger/active", active);
    if (!active) return;
    
    image_transport::ImageTransport image_transfer(*node);
    _pub = image_transfer.advertise("debugger/image", 1);
    ROS_INFO("omega::Debugger initialized");
}

void omega::Debugger::draw_background(const cv::Mat &bgr_image)
{
    if (!active) return;
    _buffer = bgr_image;
}

void omega::Debugger::draw_mask(const cv::Mat &binary_image, const cv::Scalar &color)
{
    if (!active) return;
    std::vector<cv::Mat> channels(3);
    cv::split(_buffer, channels);
    cv::Mat binary_image_inv;
    if (color[0] == 0 || color[1] == 0 || color[2] == 0) cv::bitwise_not(binary_image, binary_image_inv);
    for (unsigned int i = 0; i < 3; i++)
    {
        if (color[i] == 0) cv::bitwise_and(channels[i], binary_image_inv, channels[i]);
        else cv::bitwise_or(channels[i], binary_image, channels[i]);
    }
    cv::merge(channels, _buffer);
}

void omega::Debugger::draw_circle(const Eigen::Vector2d &center, double radius, const cv::Scalar &color)
{
    if (!active) return;
    cv::circle(_buffer, cv::Point((int)center.x(), (int)center.y()), (int)radius, color, 2);
}

void omega::Debugger::draw_rectangle(cv::Rect rect, const cv::Scalar &color)
{
    if (!active) return;
    cv::rectangle(_buffer, rect, color, 2);
}

void omega::Debugger::draw_line(const Eigen::Vector2d &source, const Eigen::Vector2d &destination, const cv::Scalar &color)
{
    if (!active) return;
    cv::line(_buffer, cv::Point((int)source.x(), (int)source.y()), cv::Point((int)destination.x(), (int)destination.y()), color, 2);
}

void omega::Debugger::draw_line(double r, double theta, const cv::Scalar &color)
{
    if (!active) return;
    const double a = cos(theta);
    const double b = sin(theta);
    const double x0 = a * r;
    const double y0 = b * r;
    double span = std::max(_owner->camera->width, _owner->camera->height);
    cv::Point point1((int)(x0 + span * (-b)), (int)(y0 + span * (a)));
    cv::Point point2((int)(x0 - span * (-b)), (int)(y0 - span * (a)));
    cv::line(_buffer, point1, point2, color, 2);
}

void omega::Debugger::publish()
{
    if (!active) return;
    _pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", _buffer).toImageMsg());
}