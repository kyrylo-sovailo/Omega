#pragma once
#include <turtlebot3_msgs/GraspState.h>
#include <ros/ros.h>

namespace omega
{
    class Omega;

    class Gripper
    {
    private:
        Omega *_owner;
        ros::Subscriber _sub;
        ros::Publisher _pub;
        bool _grasped = false;
    
    public:
        double duration;

        Gripper(ros::NodeHandle &node, Omega *omega);
        void update(const turtlebot3_msgs::GraspState::ConstPtr &msg);

        void start_move(bool open);
        ros::Duration get_time_remaining() const;
        bool get_grasped() const;
    };
};