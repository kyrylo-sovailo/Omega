#pragma once
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <ros/ros.h>

namespace omega
{
    class Omega;
    
    ///Handles arm movements
    class Arm
    {
    private:
        Omega *_owner;
        ros::Publisher _pub;
        Eigen::Vector4d _position = Eigen::Vector4d::Zero();
        Eigen::Vector3d _goal_point = Eigen::Vector3d::Zero();
        double _goal_pitch = 0;
        bool _move_started = false, _move_finished = false;
        ros::Time _finish_time;
    
    public:
        Eigen::Vector3d base_position;
        Eigen::Vector3d idle_position;
        double idle_pitch;
        Eigen::Vector4d l, q_init, q_min, q_max, w_max;

        Arm(ros::NodeHandle *node, Omega *owner);
        void update(ros::Time time, const sensor_msgs::JointState::ConstPtr &msg);
        void update(ros::Time time);

        bool start_move_idle(ros::Time now);
        bool start_move(ros::Time now, Eigen::Vector3d point, double pitch);
        void abort_move();
        bool get_move_started() const;
        bool get_move_finished() const;
        double get_error() const;
    };
};