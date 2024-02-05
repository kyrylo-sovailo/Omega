#pragma once
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

namespace omega
{
    class Omega;

    class Wheels
    {
    private:
        //Technical
        Omega *_owner;
        ros::Publisher _right_pub, _left_pub;
        
        //Subscriber
        double _linear_speed = 0, _angular_speed = 0;

        //Publisher
        ros::Time _last_sent;
        double _linear_goal = 0, _linear_end_goal = 0, _angular_goal = 0, _angular_end_goal = 0;

        void _read_state(const sensor_msgs::JointState::ConstPtr &msg);
    
    public:
        //Geometry
        double wheel_radius, wheel_separation;
        //Probability
        double linear_stddev_linear, angular_stddev_angular;
        //Limits
        double max_wheel_vel, max_linear_vel, max_linear_acc, max_angular_vel, max_angular_acc;

        Wheels(ros::NodeHandle *node, Omega *owner);
        void update(ros::Time now, const sensor_msgs::JointState::ConstPtr &msg);
        void update(ros::Time now);

        void set_speed(double linear, double angular);
        void get_speed(double *linear, double *angular);
    };
};