#pragma once
#include <ros/ros.h>

namespace omega
{
    class Config
    {
    public:
        double distance_to_ball;
        double distance_to_wall;
        double start_delay;

        Config(ros::NodeHandle *node);
    };
}