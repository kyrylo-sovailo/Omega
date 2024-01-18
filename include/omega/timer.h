#pragma once
#include <ros/ros.h>

namespace omega
{
    class Omega;

    class Timer
    {
    private:
        ros::Timer _timer;
    
    public:
        double frequency;

        Timer(ros::NodeHandle *node, Omega *owner);
    };
};