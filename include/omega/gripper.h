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
        //Technical
        Omega *_owner;
        ros::Subscriber _sub;
        ros::Publisher _pub;

        //Initialization
        bool _initialized = false, _started = false;
        ros::Time _initialization_time;

        //Movement
        bool _opened = false, _grasped = false;
        bool _move_started = false, _move_finished = false;
        ros::Time _finish_time;

        void _read_state(const turtlebot3_msgs::GraspState::ConstPtr &msg);
        void _write_state(bool open);
    
    public:
        double duration;
        double start_delay;

        Gripper(ros::NodeHandle *node, Omega *omega);
        void update(ros::Time now, const turtlebot3_msgs::GraspState::ConstPtr &msg);
        void update(ros::Time now);

        bool start_move(ros::Time now, bool open);
        bool get_move_started() const;
        bool get_move_finished() const;
        bool get_opened() const;
        bool get_grasped() const;
    };
};