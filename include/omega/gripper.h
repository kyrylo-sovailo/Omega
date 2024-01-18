#pragma once
#include <turtlebot3_msgs/GraspState.h>
#include <ros/ros.h>

namespace omega
{
    class Omega;

    ///Handles gripper
    class Gripper
    {
    private:
        Omega *_owner;
        ros::Subscriber _sub;
        ros::Publisher _pub;
        bool _opened = false, _move_started = false, _move_finished = false, _grasped = false;
        ros::Time _finish_time;
    
    public:
        double duration;

        Gripper(ros::NodeHandle *node, Omega *omega);
        void update(ros::Time time, const turtlebot3_msgs::GraspState::ConstPtr &msg);
        void update(ros::Time time);

        bool start_move(ros::Time now, bool open);
        bool get_move_started() const;
        bool get_move_finished() const;
        bool get_grasped() const;
    };
};