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
    class Debugger;
    class Gripper;
    class RobotTracker;
    class Timer;
    class Wheels;

    class Omega
    {
    private:
        ros::Subscriber _joint_state_sub;
    
    public:
        enum class State
        {
            
        };

        Arm *arm = nullptr;
        BallTracker *ball_tracker = nullptr;
        Camera *camera = nullptr;
        Config *config = nullptr;
        Debugger *debugger = nullptr;
        Gripper *gripper = nullptr;
        RobotTracker *robot_tracker = nullptr;
        Timer *timer = nullptr;
        Wheels *wheels = nullptr;

        double start_delay;

        Omega(ros::NodeHandle *node);
        void joint_state_update(const sensor_msgs::JointState::ConstPtr &msg);
        void imu_update(const sensor_msgs::Imu::ConstPtr &msg);
        void image_update(const sensor_msgs::Image::ConstPtr &msg);
        void grasp_state_update(const turtlebot3_msgs::GraspState::ConstPtr &msg);
        void timer_update(const ros::TimerEvent &event);
        ~Omega();
    };

    template <typename T> inline constexpr T sqr(T a) { return a * a; }
}

#define OMEGA_CONFIG(s, v) if (!node->getParam(s, v)) { const char *error = "Failed to get parameter " s; ROS_ERROR("%s", error); throw std::runtime_error(error); }
#define OMEGA_CONFIG_VECTOR(s, n, v) { std::vector<double> b; if (node->getParam(s, b) && b.size() == n) v = Eigen::Matrix<double, n, 1>::Map(b.data()); else { const char *error = "Failed to get parameter " s; ROS_ERROR("%s", error); throw std::runtime_error(error); } }