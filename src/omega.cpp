#include <omega/omega.h>
#include <cv_bridge/cv_bridge.h>

void omega::Omega::_image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}

void omega::Omega::_joint_state_callback(const sensor_msgs::JointState::ConstPtr msg)
{
}

void omega::Omega::_imu_callback(const sensor_msgs::Imu::ConstPtr msg)
{
}

void omega::Omega::_timer_callback(const ros::TimerEvent &event)
{
    std_msgs::Float64 msg;
    msg.data = 0.5;
    _left_wheel_pub.publish(msg);
    msg.data = -0.5;
    _right_wheel_pub.publish(msg);
}

omega::Omega::Omega(ros::NodeHandle &node) : _config(node)
{
    // Subscribers
      image_transport::ImageTransport image_transfer(node);
    _image_sub = image_transfer.subscribe("camera/rgb/image_raw", 1, &Omega::_image_callback, this);
    _joint_state_sub = node.subscribe("joints/joint_states", 1, &Omega::_joint_state_callback, this);
    _imu_sub = node.subscribe("sensor/imu_filtered", 1, &Omega::_imu_callback, this);
    _timer = node.createTimer(ros::Duration(0.1), &Omega::_timer_callback, this);

    // Publishers
    _left_wheel_pub = node.advertise<std_msgs::Float64>("joints/wheel_left_controller/command", 1, true);
    _right_wheel_pub = node.advertise<std_msgs::Float64>("joints/wheel_right_controller/command", 1, true);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omega");
    ros::NodeHandle node;
    omega::Omega omega(node);
    ros::spin();
    return 0;
}