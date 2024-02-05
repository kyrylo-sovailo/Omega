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
    public:
        struct Pose
        {
            Eigen::Vector3d position;
            double pitch;
        };

    private:
        //Technical
        Omega *_owner;
        ros::Publisher _pub;

        //Movement
        Eigen::Vector4d _pose;
        Pose _goal_pose;
        bool _move_started = false, _move_finished = false;
        ros::Time _finish_time;

        void _read_pose(const sensor_msgs::JointState::ConstPtr &msg);
        void _write_trajectory(const std::vector<Eigen::Vector4d> &poses, const std::vector<double> &durations);
    
    public:
        //Geometry
        Eigen::Vector3d base_position;
        Eigen::Vector4d l, q_init;
        //Limits
        Eigen::Vector4d q_min, q_max, w_max;
        //Behavior
        Pose to_idle_pose, idle_pose, to_pick_pose, to_throw_pose, throw_pose;

        Arm(ros::NodeHandle *node, Omega *owner);
        void update(ros::Time now, const sensor_msgs::JointState::ConstPtr &msg);
        void update(ros::Time now);
        
        bool start_move(ros::Time now, const std::vector<Pose> &path);
        bool start_move_unknown_to_idle(ros::Time now);
        bool start_move_idle_to_pick(ros::Time now, const Pose &pick_pose);
        bool start_move_pick_to_idle(ros::Time now);
        bool start_move_idle_to_throw(ros::Time now);
        bool start_move_throw_to_idle(ros::Time now);
        void abort_move();
        bool get_move_started() const;
        bool get_move_finished() const;
        double get_error() const;
    };
};