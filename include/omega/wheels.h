#pragma once
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

namespace omega
{
    class Omega;

    class Wheels
    {
    private:
        ros::Publisher _right_pub, _left_pub;
    
    public:
        Wheels(ros::NodeHandle &node, OmegaHeader *header);
        void update(const sensor_msgs::JointState::ConstPtr &msg);
        void update(const ros::TimerEvent &event);

        void set_speed(double linear, double angular);
        void get_movement(double *linear, double *angular);
    };
};