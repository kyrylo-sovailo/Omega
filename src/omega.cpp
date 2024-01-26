#include <omega/omega.h>
#include <omega/arm.h>
#include <omega/ball_tracker.h>
#include <omega/camera.h>
#include <omega/config.h>
#include <omega/gripper.h>
#include <omega/robot_tracker.h>
#include <omega/timer.h>
#include <omega/wheels.h>

omega::Omega::Omega(ros::NodeHandle *node)
{
    ROS_INFO("Omega initialization started");

    //Config
    OMEGA_CONFIG("config/start_delay", start_delay);

    //Technical
    _joint_state_sub = node->subscribe("joints/joint_states", 1, &Omega::joint_state_update, this);
    config = new Config(node);
    timer = new Timer(node, this);
    camera = new Camera(node, this);
    arm = new Arm(node, this);
    gripper = new Gripper(node, this);
    wheels = new Wheels(node, this);
    ball_tracker = new BallTracker(node, this);
    robot_tracker = new RobotTracker(node, this);

    ROS_INFO("Omega initialization finished");
}

void omega::Omega::joint_state_update(const sensor_msgs::JointState::ConstPtr &msg)
{
    ros::Time now = ros::Time::now();
    arm->update(now, msg);
    wheels->update(now, msg);
    
    double linear, angular;
    wheels->get_speed(&linear, &angular);
    robot_tracker->update(now, linear, angular);
}

void omega::Omega::imu_update(const sensor_msgs::Imu::ConstPtr &msg)
{
    //IMU not implemented
}

void omega::Omega::image_update(const sensor_msgs::Image::ConstPtr &msg)
{
    ros::Time now = ros::Time::now();
    cv::Mat image;
    camera->update(msg, image);
    robot_tracker->update(now, image);
    ball_tracker->update(now, image);
}

void omega::Omega::grasp_state_update(const turtlebot3_msgs::GraspState::ConstPtr &msg)
{
    ros::Time now = ros::Time::now();
    gripper->update(now, msg);
}

void omega::Omega::timer_update(const ros::TimerEvent &event)
{
    ros::Time now = event.current_real;
    arm->update(now);
    gripper->update(now);
    wheels->update(now);
}

omega::Omega::~Omega()
{
    if (robot_tracker != nullptr) delete robot_tracker;
    if (ball_tracker != nullptr) delete ball_tracker;
    if (wheels != nullptr) delete wheels;
    if (gripper != nullptr) delete gripper;
    if (arm != nullptr) delete arm;
    if (camera != nullptr) delete camera;
    if (timer != nullptr) delete timer;
    if (config != nullptr) delete config;
}

int _main(int argc, char **argv)
{
    ros::init(argc, argv, "omega_node");
    ros::NodeHandle node;
    omega::Omega omega(&node);
    ros::spin();
    return 0;
}

int main(int argc, char **argv)
{
    return _main(argc, argv);
}