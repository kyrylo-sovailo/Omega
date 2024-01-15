#pragma once

#include <omega/config.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <turtlebot3_msgs/GraspState.h>
#include <image_transport/image_transport.h>

namespace omega
{
    class Omega
    {
    private:
        // Components
        Config _config;

        // Subscribers
        void _image_callback(const sensor_msgs::Image::ConstPtr &msg);
        void _joint_state_callback(const sensor_msgs::JointState::ConstPtr msg);
        void _imu_callback(const sensor_msgs::Imu::ConstPtr msg);
        void _timer_callback(const ros::TimerEvent &event);
        image_transport::Subscriber _image_sub;
        ros::Subscriber _joint_state_sub;
        ros::Subscriber _imu_sub;
        ros::Timer _timer;
        
        // Publishers
        image_transport::Publisher _debug_image_pub;
        ros::Publisher _right_wheel_pub;
        ros::Publisher _left_wheel_pub;
        ros::Publisher _arm_pub;
        ros::Publisher _gripper_pub;
        
    public:
        Omega(ros::NodeHandle &node);    
    };
}