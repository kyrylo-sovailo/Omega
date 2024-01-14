#pragma once
#include <ros/ros.h>

namespace omega
{
    class Config
    {
    public:
        bool debug;
        
        double wheel_radius;
        double wheel_separation;

        double max_wheel_vel;
        double max_linear_vel;
        double max_linear_acc;
        double max_angular_vel;
        double max_angular_acc;

        Config(ros::NodeHandle &node);
    };
}