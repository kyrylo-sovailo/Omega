#pragma once
#include <ros/ros.h>

namespace omega
{
    class Config
    {
    public:
        bool debug;
        double distance_to_ball;
        double distance_to_wall;

        Config(ros::NodeHandle *node);
    };
}