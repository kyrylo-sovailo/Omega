#pragma once
#include <ros/ros.h>
#include <omega/omega_header.h>

omega::Wheels::Wheels(ros::NodeHandle &node, OmegaHeader *header)
{
    _left_pub = node.advertise<std_msgs::Float64>("joints/wheel_left_controller/command", 1, true);
    _right_pub = node.advertise<std_msgs::Float64>("joints/wheel_right_controller/command", 1, true);
}

void omega::Wheels::update(const ros::TimerEvent &event)
{
    //Limit acceleration
}

void omega::Wheels::set(double linear, double angular)
{
    std_msgs::Float64 command_right;
    std_msgs::Float64 command_left;

    command_right.data = linear
    command_left.data

    _right_wheel_pub.publish(msg);
    _left_wheel_pub.publish(msg);
}