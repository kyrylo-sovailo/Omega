#include <omega/omega.h>
#include <cv_bridge/cv_bridge.h>

void omega::Omega::_image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr image_pointer = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat &image = image_pointer->image;

    //Remove white noise
    cv::GaussianBlur(image, image, cv::Size(5, 5), 3);

    //Convert to HSV
    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);

    //Cutoff

    cv::Scalar lower(min_hue, min_sat, min_val);
    cv::Scalar upper(max_hue, max_sat, max_val);
    cv::Mat detection_image = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::inRange(image, lower, upper, detection_image);

    //Erode & dilate
    cv::erode(detection_image, detection_image, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(detection_image, detection_image, cv::Mat(), cv::Point(-1, -1), 2);

    if (_config.debug)
    {
        _debug_image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", detection_image).toImageMsg());
    }
}

void omega::Omega::_joint_state_callback(const sensor_msgs::JointState::ConstPtr msg)
{
}

void omega::Omega::_imu_callback(const sensor_msgs::Imu::ConstPtr msg)
{
}

void omega::Omega::_timer_callback(const ros::TimerEvent &event)
{
    

    // Arm command
    

    //Gripper command
    
}

omega::Omega::Omega(ros::NodeHandle &node) : _config(node)
{
    // Subscribers
    image_transport::ImageTransport image_transfer(node);
    _image_sub = image_transfer.subscribe("camera/rgb/image_raw", 1, &Omega::_image_callback, this);
    _joint_state_sub = node.subscribe("joints/joint_states", 1, &Omega::_joint_state_callback, this);
    _imu_sub = node.subscribe("sensor/imu_raw", 1, &Omega::_imu_callback, this);
    

    // Publishers
    if (_config.debug) _debug_image_pub = image_transfer.advertise("debug/image", 1);
    
    _gripper
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omega_node");
    ros::NodeHandle node;
    omega::Omega omega(node);
    ros::spin();
    return 0;
}