#pragma once
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

namespace omega
{
    class Omega
    {
    public:
        Omega(ros::NodeHandle &node);
        void update(const ros::TimerEvent &event);

    protected:
        // Subscribers
        void _joint_state_callback(const sensor_msgs::JointState::ConstPtr msg);
        void _imu_callback(const sensor_msgs::Imu::ConstPtr msg);
        ros::Subscriber _joint_state_sub;
        ros::Subscriber _imu_sub;
        
        // Publishers
        ros::Publisher _right_wheel_pub;
        ros::Publisher _left_wheel_pub;

        // Timers
        ros::Timer _timer;
    };
}