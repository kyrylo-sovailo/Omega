#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
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
        //Technical
        ros::Subscriber _joint_state_sub, _external_arm_command_sub, _external_command_sub;
        ros::Publisher _torque_enable_pub;

        //Initialization
        ros::Time _initialize_time;
        bool _initialized = false, _started = false, _arm_started = false, _gripper_started = false;

        //Logics
        double _external_command_linear = 0, _external_command_angular = 0;

    public:
        //Config
        double arm_start_delay, gripper_start_delay, start_delay;

        //Components
        Arm *arm = nullptr;
        BallTracker *ball_tracker = nullptr;
        Camera *camera = nullptr;
        Config *config = nullptr;
        Debugger *debugger = nullptr;
        Gripper *gripper = nullptr;
        RobotTracker *robot_tracker = nullptr;
        Timer *timer = nullptr;
        Wheels *wheels = nullptr;

        Omega(ros::NodeHandle *node);
        void external_arm_command_update(const std_msgs::String::ConstPtr &msg);
        void external_command_update(const geometry_msgs::Twist::ConstPtr &msg);
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
#define OMEGA_CONFIG_DEGREE(s, v) if (!node->getParam(s, v)) { const char *error = "Failed to get parameter " s; ROS_ERROR("%s", error); throw std::runtime_error(error); } else { v = M_PI * v / 180; }
#define OMEGA_CONFIG_VECTOR(s, n, v) { std::vector<double> b; if (node->getParam(s, b) && b.size() == n) v = Eigen::Matrix<double, n, 1>::Map(b.data()); else { const char *error = "Failed to get parameter " s; ROS_ERROR("%s", error); throw std::runtime_error(error); } }
#define OMEGA_CONFIG_COLOR(s, v) { std::vector<int> b; if (node->getParam(s, b) && b.size() == 3) { for (unsigned int i = 0; i < 3; i++) v[i] = b[i]; } else { const char *error = "Failed to get parameter " s; ROS_ERROR("%s", error); throw std::runtime_error(error); } }