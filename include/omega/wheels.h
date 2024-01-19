#pragma once
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

namespace omega
{
    class Omega;

    class Wheels
    {
    private:
        Omega *_owner;
        
        //Subscriber
        double _last_left_position = 0, _last_right_position = 0;
        ros::Time _last_received;
        double _last_linear_speed = 0, _last_angular_speed = 0;

        //Publisher
        ros::Publisher _right_pub, _left_pub;
        ros::Time _last_sent;
        double _linear_goal = 0, _linear_end_goal = 0, _angular_goal = 0, _angular_end_goal = 0;
    
    public:
        double radius;
        double separation;
        double max_wheel_vel;
        double max_linear_vel;
        double max_linear_acc;
        double max_angular_vel;
        double max_angular_acc;

        Wheels(ros::NodeHandle *node, Omega *owner);
        void update(ros::Time now, const sensor_msgs::JointState::ConstPtr &msg);
        void update(ros::Time now);

        void set_speed(double linear, double angular);
        void get_speed(double *linear, double *angular);
    };
};