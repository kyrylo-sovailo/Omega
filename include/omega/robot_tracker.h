#pragma once
#include <omega/differentiable.h>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>

namespace omega
{
    class Omega;

    class RobotTracker
    {
    private:
        struct VerticalEdge
        {
            Eigen::Vector2d coordinate;
        };
        std::vector<VerticalEdge> _vertical_edges;

        struct HorizontalWall
        {
            Eigen::Vector2d source, destination;
        };
        std::vector<HorizontalWall> _horizontal_walls;

        Omega *_owner;
        Eigen::Vector3d _state;
        Eigen::Matrix3d _variance;
        bool _find_visible_segment(Eigen::Vector3d *source, Eigen::Vector3d *destination);
        void _project_line();
        Eigen::Matrix<ddouble, 2, 1> _project_point(const Eigen::Matrix<ddouble3, 3, 1> &point);
        Eigen::Vector2d _project_line(Eigen::Vector3d source, Eigen::Vector3d destination);
    
    public:
        //Geometry
        Eigen::Vector2d init_position;
        double init_angle;
        std::string arena;
        //Probability
        double position_stddev_time;
        double angle_stddev_time;
        double wall_angle_stddev;
        double wall_position_stddev;

        RobotTracker(ros::NodeHandle *node, Omega *owner);
        void update(ros::Time time, const cv::Mat &image);
        void update(ros::Time time, double linear, double angular);
        void update(ros::Time time);

        Eigen::Vector2d get_position() const;
        double get_orientation() const;
        Eigen::Vector2d get_position_stddev() const;
        double get_orientation_stddev() const;
        Eigen::Matrix3d get_variance() const;
    };
};