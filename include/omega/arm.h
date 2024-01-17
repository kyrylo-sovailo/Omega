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
        Eigen::Vector4d _position;
    
    public:
        Eigen::Vector3d base_position;
        Eigen::Vector3d idle_position;
        double idle_pitch;
        Eigen::Vector4d l, q_init, w_max;

        Arm(ros::NodeHandle *node, Omega *owner);
        void update(const sensor_msgs::JointState::ConstPtr &msg);

        void start_move_idle();
        void start_move(const Eigen::Vector3d point, double pitch);
        void abort_move();
        ros::Duration get_time_remaining() const;
        double get_cartesian_error() const;
        double get_angular_error() const;
    };
};