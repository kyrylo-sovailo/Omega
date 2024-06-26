#pragma once
#include <omega/differentiable.h>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>

namespace omega
{
    class Omega;

    //Class for robot position tracking
    class RobotTracker
    {
    private:
        //Database entry for vertical edge
        struct VerticalEdge
        {
            Eigen::Vector2d coordinate;
            double height;
        };
        std::vector<VerticalEdge> _vertical_edges;

        //Database entry for horizontal wall
        struct HorizontalWall
        {
            Eigen::Vector2d source, destination;
            double height;
        };
        std::vector<HorizontalWall> _horizontal_walls;

        //Perceived line
        struct VisionLine
        {
            double r, theta;
            bool horizontal;
            double a, b;
            bool matched = false;
        };

        //Expected vertical edge
        struct ExpectedVerticalEdge
        {
            ddouble3 b;
            VisionLine *match;
        };

        //Expected horizontal wall
        struct ExpectedHorizontalWall
        {
            ddouble3 a_upper, b_upper;
            ddouble3 a_lower, b_lower;
            VisionLine *match_upper = nullptr;
            VisionLine *match_lower = nullptr;
        };

        Omega *_owner;
        Eigen::Vector3d _state;
        Eigen::Matrix3d _variance;
        bool _last_update_valid;
        ros::Time _last_update;
        
        static Eigen::Vector4d _extend(const Eigen::Vector3d &point);
        static Eigen::Matrix<ddouble3, 4, 1> _extend(const Eigen::Matrix<ddouble3, 3, 1> &point);

        //Find point coordinates in local space
        Eigen::Matrix<ddouble3, 3, 1> _world_to_local(const Eigen::Vector3d &point) const;
        ///Find point coordinates in camera plane
        Eigen::Vector2d _local_to_screen(const Eigen::Vector3d &point) const;
        ///Find point coordinates in camera plane
        Eigen::Matrix<ddouble3, 2, 1> _local_to_screen(const Eigen::Matrix<ddouble3, 3, 1> &point) const;
        ///Find intersection point with screen boundaries
        bool _is_intersection_on_screen(const Eigen::Vector3d &source, const Eigen::Vector3d &destination, bool horizontal, bool upper, double &t) const;
        ///Find if a point is displayed
        bool _is_point_on_screen(const Eigen::Vector2d &point) const;
        //Find visible line segment
        bool _is_line_on_screen(const Eigen::Vector3d &source, const Eigen::Vector3d &destination, double &t_source, double &t_destination) const;
        
        //Find expected parameters of horizontal line
        bool _get_expected_horizontal_line(const Eigen::Matrix<ddouble3, 3, 1> &source, const Eigen::Matrix<ddouble3, 3, 1> &destination, ddouble3 &a, ddouble3 &b) const;
        //Find expected parameters of vertical line
        bool _get_expected_vertical_line(const Eigen::Matrix<ddouble3, 3, 1> &source, const Eigen::Matrix<ddouble3, 3, 1> &destination, ddouble3 &b) const;
        
        ///Find expected horizontal walls
        void _get_expected_horizontal_walls(std::vector<ExpectedHorizontalWall> *walls) const;
        ///Find expected vertical edges
        void _get_expected_vertical_edges(std::vector<ExpectedVerticalEdge> *edges) const;
        //Find lines from image
        void _get_lines(const cv::Mat &bgr_image, std::vector<VisionLine> *lines) const;
        //Find lines for vertical edges
        void _match_vertical_lines(std::vector<ExpectedVerticalEdge> *edges, std::vector<VisionLine> *lines) const;
        //Find lines for horizontal walls
        void _match_horizontal_lines(std::vector<ExpectedHorizontalWall> *walls, std::vector<VisionLine> *lines) const;
        //Calculate dimension of update
        unsigned int _construct_dimension(const std::vector<ExpectedHorizontalWall> &walls, const std::vector<ExpectedVerticalEdge> &edges) const;
        //Construct vector of expected values
        void _construct_expectation(const std::vector<ExpectedHorizontalWall> &walls, const std::vector<ExpectedVerticalEdge> &edges,
            Eigen::Matrix<double, Eigen::Dynamic, 1> &expectation) const;
        //Construct derivative matrix of expected values
        void _construct_expectation_derivative(const std::vector<ExpectedHorizontalWall> &walls, const std::vector<ExpectedVerticalEdge> &edges,
            Eigen::Matrix<double, Eigen::Dynamic, 3> &derivative) const;
        //Construct vector of measured values
        void _construct_measurement(const std::vector<ExpectedHorizontalWall> &walls, const std::vector<ExpectedVerticalEdge> &edges,
            Eigen::Matrix<double, Eigen::Dynamic, 1> &measurement) const;
        //Construct variance matrix of measured values
        void _construct_measurement_variance(const std::vector<ExpectedHorizontalWall> &walls, const std::vector<ExpectedVerticalEdge> &edges,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &variance) const;
    
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
        //Image processing
        int wall_color_min[3], wall_color_max[3];
        int grass_color_min[3], grass_color_max[3];
        int dilate_size;
        double canny_threshold1, canny_threshold2;
        int canny_aperture;
        double hough_distance_resolution, hough_angle_resolution;
        int horizontal_hough_threshold, vertical_hough_threshold;
        double horizontal_max_angle, vertical_max_angle;
        int horizontal_max_lines, vertical_max_lines;
        //Tracking
        double match_max_a, match_max_b;

        RobotTracker(ros::NodeHandle *node, Omega *owner);
        void update(ros::Time time, const cv::Mat &bgr_image);
        void update(ros::Time time, double linear, double angular);
        void update(ros::Time time);

        Eigen::Vector2d get_position() const;
        double get_orientation() const;
        Eigen::Vector2d get_position_stddev() const;
        double get_orientation_stddev() const;
        Eigen::Matrix3d get_variance() const;
    };
};