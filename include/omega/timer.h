#pragma once
#include <ros/ros.h>

namespace omega
{
    class Omega;

    class Timer
    {
    public:
        double frequency;

        Timer(ros::NodeHandle *node, OmegaHeader *header);
    };
};