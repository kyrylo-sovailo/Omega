#pragma once

#include <omega/config.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

namespace omega
{
    class Omega
    {
    private:
        // Components
        Config _config;
        
        // Subscribers
        void _joint_state_callback(const sensor_msgs::JointState::ConstPtr msg);
        void _imu_callback(const sensor_msgs::Imu::ConstPtr msg);
        void _timer_callback(const ros::TimerEvent &event);
        ros::Subscriber _joint_state_sub;
        ros::Subscriber _imu_sub;
        ros::Timer _timer;
        
        // Publishers
        ros::Publisher _right_wheel_pub;
        ros::Publisher _left_wheel_pub;
        
    public:
        Omega(ros::NodeHandle &node);    
    };
}