#include <omega/omega.h>
#include <omega/arm.h>
#include <omega/ball_tracker.h>
#include <omega/camera.h>
#include <omega/config.h>
#include <omega/debugger.h>
#include <omega/gripper.h>
#include <omega/robot_tracker.h>
#include <omega/timer.h>
#include <omega/wheels.h>
#include <std_msgs/Bool.h>

omega::Omega::Omega(ros::NodeHandle *node)
{
    ROS_INFO("Omega initialization started");

    //Config
    OMEGA_CONFIG("config/start_delay", start_delay);
    OMEGA_CONFIG("config/arm_start_delay", arm_start_delay);
    OMEGA_CONFIG("config/gripper_start_delay", gripper_start_delay);

    //Components
    config = new Config(node);
    debugger = new Debugger(node, this);
    timer = new Timer(node, this);
    camera = new Camera(node, this);
    arm = new Arm(node, this);
    gripper = new Gripper(node, this);
    wheels = new Wheels(node, this);
    ball_tracker = new BallTracker(node, this);
    robot_tracker = new RobotTracker(node, this);

    //Topics
    _joint_state_sub = node->subscribe("joints/joint_states", 1, &Omega::joint_state_update, this);
    _external_arm_command_sub = node->subscribe("cmd_arm", 1, &Omega::external_arm_command_update, this);
    _external_command_sub = node->subscribe("cmd_vel", 1, &Omega::external_command_update, this);
    _torque_enable_pub = node->advertise<std_msgs::Bool>("joints/torque_enable", 1, true);
    std_msgs::Bool torque_enable_command;
    torque_enable_command.data = false;
    _torque_enable_pub.publish(torque_enable_command);

    ROS_INFO("Omega initialization finished");
}

void omega::Omega::external_arm_command_update(const std_msgs::String::ConstPtr &msg)
{
    ros::Time now = ros::Time::now();
    if (msg->data == "unknown_to_idle") arm->start_move_unknown_to_idle(now);
    else if (msg->data == "idle_to_pick")
    {
        Arm::Pose pose;
        pose.position = Eigen::Vector3d(0.3, 0, 0.05);
        pose.pitch = M_PI;
        arm->start_move_idle_to_pick(now, pose);
    }
    else if (msg->data == "pick_to_idle") arm->start_move_pick_to_idle(now);
    else if (msg->data == "idle_to_throw") arm->start_move_idle_to_throw(now);
    else if (msg->data == "throw_to_idle") arm->start_move_throw_to_idle(now);
    else if (msg->data == "open") gripper->start_move(now, true);
    else if (msg->data == "close") gripper->start_move(now, false);
    else ROS_WARN("Invalid external arm command");
}

void omega::Omega::external_command_update(const geometry_msgs::Twist::ConstPtr &msg)
{
    _external_command_linear = msg->linear.x;
    _external_command_angular = msg->angular.z;
}

void omega::Omega::joint_state_update(const sensor_msgs::JointState::ConstPtr &msg)
{
    ros::Time now = ros::Time::now();
    arm->update(now, msg);
    wheels->update(now, msg);
    
    double linear_speed, angular_speed;
    wheels->get_speed(&linear_speed, &angular_speed);
    robot_tracker->update(now, linear_speed, angular_speed);
    ball_tracker->update(now, linear_speed, angular_speed);
}

void omega::Omega::imu_update(const sensor_msgs::Imu::ConstPtr &msg)
{
    //IMU not implemented
}

void omega::Omega::image_update(const sensor_msgs::Image::ConstPtr &msg)
{
    ros::Time now = ros::Time::now();
    cv::Mat hsv_image;
    camera->update(msg, hsv_image);
    robot_tracker->update(now, hsv_image);
    ball_tracker->update(now, hsv_image);
    debugger->publish();
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

    //Initialization
    if (!_initialized)
    {
        _initialize_time = now;
    }
    if (!_started)
    {
        if ((now - _initialize_time) >= ros::Duration(start_delay)) _started = true;
    }
    if (!_arm_started)
    {
        if ((now - _initialize_time) >= ros::Duration(arm_start_delay)) { _arm_started = true; arm->start_move_unknown_to_idle(now); }
    }
    if (!_gripper_started)
    {
        if ((now - _initialize_time) >= ros::Duration(gripper_start_delay)) { _gripper_started = true; gripper->start_move(now, true); }
    }

    //Normal decision making
    if (_started)
    {
        wheels->set_speed(_external_command_linear, _external_command_angular);
    }
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
    if (debugger != nullptr) delete debugger;
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