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
    const unsigned int min_hue = 6;
    const unsigned int max_hue = 50;
    const unsigned int min_sat = 80;
    const unsigned int max_sat = 255;
    const unsigned int min_val = 110;
    const unsigned int max_val = 255;
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
    std_msgs::Float64 msg;
    msg.data = 0.5;
    _left_wheel_pub.publish(msg);
    msg.data = -0.5;
    _right_wheel_pub.publish(msg);

    // Arm command
    trajectory_msgs::JointTrajectory command;
    command.joint_names = std::vector<std::string>{"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4"};
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = std::vector<double>{ 0.5, 0.5, 0.5, 0.5 };
    point.time_from_start = ros::Duration(0.5);
    command.points.push_back(point);
    _arm_pub.publish(command);

    //Gripper command
    turtlebot3_msgs::GraspState command2;
    command2.grasp_state = turtlebot3_msgs::GraspState::CLOSE;
    _gripper_pub.publish(command2);
}

omega::Omega::Omega(ros::NodeHandle &node) : _config(node)
{
    // Subscribers
    image_transport::ImageTransport image_transfer(node);
    _image_sub = image_transfer.subscribe("camera/rgb/image_raw", 1, &Omega::_image_callback, this);
    _joint_state_sub = node.subscribe("joints/joint_states", 1, &Omega::_joint_state_callback, this);
    _imu_sub = node.subscribe("sensor/imu_raw", 1, &Omega::_imu_callback, this);
    _timer = node.createTimer(ros::Duration(1.0), &Omega::_timer_callback, this);

    // Publishers
    if (_config.debug) _debug_image_pub = image_transfer.advertise("debug/image", 1);
    _left_wheel_pub = node.advertise<std_msgs::Float64>("joints/wheel_left_controller/command", 1, true);
    _right_wheel_pub = node.advertise<std_msgs::Float64>("joints/wheel_right_controller/command", 1, true);
    _arm_pub = node.advertise<trajectory_msgs::JointTrajectory>("joints/arm_trajectory_controller/command", 1, true);
    _gripper_pub = node.advertise<turtlebot3_msgs::GraspState>("joints/gripper_grasp_controller/command", 1, true);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omega_node");
    ros::NodeHandle node;
    omega::Omega omega(node);
    ros::spin();
    return 0;
}