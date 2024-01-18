#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <turtlebot3_msgs/GraspState.h>

namespace omega
{
    class Arm;
    class BallTracker;
    class Camera;
    class Config;
    class Gripper;
    class RobotTracker;
    class Timer;
    class Wheels;

    class Omega
    {
    private:
        ros::Subscriber _joint_state_sub;
    
    public:
        Arm *arm = nullptr;
        BallTracker *ball_tracker = nullptr;
        Camera *camera = nullptr;
        Config *config = nullptr;
        Gripper *gripper = nullptr;
        RobotTracker *robot_tracker = nullptr;
        Timer *timer = nullptr;
        Wheels *wheels = nullptr;

        Omega(ros::NodeHandle *node);
        void joint_state_update(const sensor_msgs::JointState::ConstPtr &msg);
        void imu_update(const sensor_msgs::Imu::ConstPtr &msg);
        void image_update(const sensor_msgs::Image::ConstPtr &msg);
        void grasp_state_update(const turtlebot3_msgs::GraspState::ConstPtr &msg);
        void timer_update(const ros::TimerEvent &event);
        ~Omega();
    };
}

#define OMEGA_STRING2(s) #s
#define OMEGA_STRING(s) OMEGA_STRING2(s)

#define OMEGA_CONFIG(s, v) if (!node->getParam(OMEGA_STRING(s), v)) { const char *error = "Failed to get parameter " OMEGA_STRING(s); ROS_ERROR(error); throw std::runtime_error(error); }
#define OMEGA_CONFIG_VECTOR(s, n, v) { std::vector<double> b; if (node->getParam(OMEGA_STRING(s), b) && b.size() == n) v = Eigen::Matrix<double, n, 1>::Map(b.data()); else { const char *error = "Failed to get parameter " OMEGA_STRING(s); ROS_ERROR(error); throw std::runtime_error(error); } }