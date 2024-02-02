#include <omega/robot_tracker.h>
#include <omega/camera.h>
#include <omega/debugger.h>
#include <omega/omega.h>
#include <opencv2/imgproc.hpp>

/*
Eigen::Matrix<omega::ddouble3, 3, 1> omega::RobotTracker::_get_point_local(const Eigen::Vector3d &point) const
{
    ddouble3 x = _state.x(); x.derivative[0] = 1;
    ddouble3 y = _state.y(); y.derivative[1] = 1;
    ddouble3 theta = _state.z(); theta.derivative[2] = 1;
    return Eigen::AngleAxis<ddouble3>(-theta, Eigen::Matrix<ddouble3, 3, 1>::UnitZ()) * Eigen::Translation<ddouble3, 3>(-x, -y, 0) * point;
}

bool omega::RobotTracker::_get_line_visible(Eigen::Matrix<ddouble3, 3, 1> &source, Eigen::Matrix<ddouble3, 3, 1> &destination) const
{
    
}

Eigen::Matrix<omega::ddouble3, 2, 1> omega::RobotTracker::_project_point(const Eigen::Matrix<ddouble3, 3, 1> &point) const
{
    Eigen::Matrix<ddouble3, 4, 1> point_ex;
    point_ex.segment<3>(0) = point;
    point_ex(3) = ddouble3(1);
    point_ex = _owner->camera->matrix * point_ex;
    Eigen::Matrix<ddouble3, 2, 1> projection(point_ex(0) / point_ex(2), point_ex(1) / point_ex(2));
    return projection;
}

void omega::RobotTracker::_get_horizontal_line_expectation(const Eigen::Matrix<ddouble3, 3, 1> &source, const Eigen::Matrix<ddouble3, 3, 1> &destination, ddouble3 &a, ddouble3 &b) const
{
    const unsigned int points = 5;
    ddouble3 x_sum = 0, x2_sum = 0, xy_sum = 0, y_sum = 0;
    for (int i = 0; i < points; i++)
    {
        Eigen::Matrix<ddouble3, 2, 1> point = _project_point(((points - i - 1) * source + i * destination) / (points - 1));
        x_sum += point.x();
        x2_sum += point.x() * point.x();
        xy_sum += point.x() * point.y();
        y_sum += point.y();
    }
    a = ((double)points * xy_sum - x_sum * y_sum) / ((double)points * x2_sum) - x_sum * x_sum;
    b = (y_sum - a * x_sum) / (double)points;
}

void omega::RobotTracker::_get_vertical_line_expectation(const Eigen::Matrix<ddouble3, 3, 1> &source, const Eigen::Matrix<ddouble3, 3, 1> &destination, ddouble3 &b) const
{
    b = (_project_point(source + destination)).x();
}
*/
void omega::RobotTracker::_get_lines(const cv::Mat &bgr_image, std::vector<VisionLine> *lines) const
{
    //Image processing
    cv::Mat gray_image;
    cv::cvtColor(bgr_image, gray_image, cv::COLOR_BGR2GRAY);
    cv::Canny(gray_image, gray_image, canny_threshold1, canny_threshold2, canny_aperture);

    //Find horizontal lines
    std::vector<cv::Vec2f> hough_lines1;
    cv::Mat gray_copy = gray_image.clone();
    cv::HoughLines(gray_copy, hough_lines1, hough_distance_resolution, hough_angle_resolution, hough_threshold, 0, 0,
        M_PI / 2 - horizontal_max_angle, M_PI / 2 + horizontal_max_angle);
    for (unsigned int i = 0; i < hough_lines1.size(); i++)
    {
        VisionLine line;
        line.horizontal = true;
        line.r = hough_lines1[i][0];
        line.theta = hough_lines1[i][1];
        line.a = -std::cos(line.theta) / std::sin(line.theta);
        line.b = line.r / std::sin(line.theta);
        _owner->debugger->draw_line(line.r, line.theta, Debugger::red);
        lines->push_back(line);
    }
    
    //Find vertical lines
    hough_lines1.resize(0);
    gray_copy = gray_image.clone();
    cv::HoughLines(gray_copy, hough_lines1, hough_distance_resolution, hough_angle_resolution, hough_threshold, 0, 0,
        0, vertical_max_angle);
    
    std::vector<cv::Vec2f> hough_lines2;
    gray_copy = gray_image.clone();
    cv::HoughLines(gray_copy, hough_lines2, hough_distance_resolution, hough_angle_resolution, hough_threshold, 0, 0,
        M_PI - vertical_max_angle, M_PI);

    hough_lines1.insert(hough_lines1.end(), hough_lines2.cbegin(), hough_lines2.cend());
    for(unsigned int i = 0; i < hough_lines1.size(); i++)
    {
        VisionLine line;
        line.horizontal = false;
        line.r = hough_lines1[i][0];
        line.theta = hough_lines1[i][1];
        line.a = -std::sin(line.theta) / std::cos(line.theta);
        line.b = line.r / std::cos(line.theta);
        lines->push_back(line);
        _owner->debugger->draw_line(line.r, line.theta, Debugger::red);
    }
}
/*
void omega::RobotTracker::_match_vertical_lines(std::vector<ExpectedVerticalEdge> *edges, std::vector<VisionLine> *lines) const
{
    for (auto edge = edges->begin(); edge != edges->end(); edge++)
    {
        double min_distance = std::numeric_limits<double>::infinity();
        VisionLine *min_distance_line;
        for (auto line = lines->begin(); line != lines->end(); line++)
        {
            if (line->matched || line->horizontal) continue;
            if (std::abs(edge->a.value - line->a) > match_max_a) continue;
            if (std::abs(edge->b.value - line->b) > match_max_b) continue;
            const double distance = std::hypot((edge->a.value - line->a) / match_max_a, (edge->b.value - line->b) / match_max_b);
            if (distance < min_distance) { min_distance = distance; min_distance_line = &(*line); }
        }

        if (!std::isinf(min_distance))
        {
            edge->match = min_distance_line;
            min_distance_line->matched = true;
        }
    }
}

void omega::RobotTracker::_match_horizontal_lines(std::vector<ExpectedHorizontalWall> *walls, std::vector<VisionLine> *lines) const
{
    for (auto wall = walls->begin(); wall != walls->end(); wall++)
    {
        //Upper
        double min_distance = std::numeric_limits<double>::infinity();
        VisionLine *min_distance_line;
        for (auto line = lines->begin(); line != lines->end(); line++)
        {
            if (line->matched || !line->horizontal) continue;
            if (std::abs(wall->a_upper.value - line->a) > match_max_a) continue;
            if (std::abs(wall->b_upper.value - line->b) > match_max_b) continue;
            const double distance = std::hypot((wall->a_upper.value - line->a) / match_max_a, (wall->b_upper.value - line->b) / match_max_b);
            if (distance < min_distance) { min_distance = distance; min_distance_line = &(*line); }
        }
        if (!std::isinf(min_distance))
        {
            wall->match_upper = min_distance_line;
            min_distance_line->matched = true;
        }

        //Lower
        min_distance = std::numeric_limits<double>::infinity();
        for (auto line = lines->begin(); line != lines->end(); line++)
        {
            if (line->matched || !line->horizontal) continue;
            if (std::abs(wall->a_lower.value - line->a) > match_max_a) continue;
            if (std::abs(wall->b_lower.value - line->b) > match_max_b) continue;
            const double distance = std::hypot((wall->a_lower.value - line->a) / match_max_a, (wall->b_lower.value - line->b) / match_max_b);
            if (distance < min_distance) { min_distance = distance; min_distance_line = &(*line); }
        }
        if (!std::isinf(min_distance))
        {
            wall->match_lower = min_distance_line;
            min_distance_line->matched = true;
        }
    }
}

unsigned int omega::RobotTracker::_construct_dimension(const std::vector<ExpectedHorizontalWall> &walls, const std::vector<ExpectedVerticalEdge> &edges) const
{
    unsigned int dimension = 0;
    for (auto wall = walls.cbegin(); wall != walls.cend(); wall++)
    {
        if (wall->match_lower != nullptr && wall->match_upper != nullptr) dimension += 2;
    }
    for (auto edge = edges.cbegin(); edge != edges.cend(); edge++)
    {
        if (edge->match != nullptr) dimension += 1;
    }
    return dimension;
}

void omega::RobotTracker::_construct_expectation(const std::vector<ExpectedHorizontalWall> &walls, const std::vector<ExpectedVerticalEdge> &edges,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &expectation) const
{
    expectation.resize(_construct_dimension(walls, edges));
    unsigned int dimension = 0;
    for (auto wall = walls.cbegin(); wall != walls.cend(); wall++)
    {
        if (wall->match_lower != nullptr && wall->match_upper != nullptr)
        {
            expectation(dimension + 0) = wall->a_upper.value - wall->a_lower.value;
            expectation(dimension + 1) = wall->b_upper.value - wall->b_lower.value;
            dimension += 2;
        }
    }
    for (auto edge = edges.cbegin(); edge != edges.cend(); edge++)
    {
        if (edge->match != nullptr)
        {
            expectation(dimension) = edge->b.value;
            dimension += 1;
        }
    }
}

void omega::RobotTracker::_construct_expectation_derivative(const std::vector<ExpectedHorizontalWall> &walls, const std::vector<ExpectedVerticalEdge> &edges,
    Eigen::Matrix<double, Eigen::Dynamic, 3> &derivative) const
{
    derivative.resize(_construct_dimension(walls, edges));
    unsigned int dimension = 0;
    for (auto wall = walls.cbegin(); wall != walls.cend(); wall++)
    {
        if (wall->match_lower != nullptr && wall->match_upper != nullptr)
        {
            for (unsigned int i = 0; i < 3; i++)
            {
                derivative(dimension + 0, i) = wall->a_upper.derivative[i] - wall->a_lower.derivative[i];
                derivative(dimension + 1, i) = wall->b_upper.derivative[i] - wall->b_lower.derivative[i];
            }
            dimension += 2;
        }
    }
    for (auto edge = edges.cbegin(); edge != edges.cend(); edge++)
    {
        if (edge->match != nullptr)
        {
            for (unsigned int i = 0; i < 3; i++)
            {
                derivative(dimension, i) = edge->b.derivative[i];
            }
            dimension += 1;
        }
    }
}

void omega::RobotTracker::_construct_measurement(const std::vector<ExpectedHorizontalWall> &walls, const std::vector<ExpectedVerticalEdge> &edges,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &measurement) const
{
    measurement.resize(_construct_dimension(walls, edges));
    unsigned int dimension = 0;
    for (auto wall = walls.cbegin(); wall != walls.cend(); wall++)
    {
        if (wall->match_lower != nullptr && wall->match_upper != nullptr)
        {
            measurement(dimension + 0) = wall->match_upper->a - wall->match_lower->a;
            measurement(dimension + 1) = wall->match_upper->b - wall->match_lower->b;
            dimension += 2;
        }
    }
    for (auto edge = edges.cbegin(); edge != edges.cend(); edge++)
    {
        if (edge->match != nullptr)
        {
            measurement(dimension) = edge->match->b;
            dimension += 1;
        }
    }
}

void omega::RobotTracker::_construct_measurement_variance(const std::vector<ExpectedHorizontalWall> &walls, const std::vector<ExpectedVerticalEdge> &edges,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &variance) const
{
    //Assume standard deviations to be three times resolution
    variance.setZero(_construct_dimension(walls, edges), _construct_dimension(walls, edges));
    unsigned int dimension = 0;
    for (auto wall = walls.cbegin(); wall != walls.cend(); wall++)
    {
        if (wall->match_lower != nullptr && wall->match_upper != nullptr)
        {
            variance(dimension + 0, dimension + 0) = sqr(3 * hough_angle_resolution);
            variance(dimension + 1, dimension + 1) = sqr(3 * hough_distance_resolution);
            dimension += 2;
        }
    }
    for (auto edge = edges.cbegin(); edge != edges.cend(); edge++)
    {
        if (edge->match != nullptr)
        {
            variance(dimension, dimension) = sqr(3 * hough_distance_resolution);
            dimension += 1;
        }
    }
}
*/

omega::RobotTracker::RobotTracker(ros::NodeHandle *node, Omega *owner) : _owner(owner)
{
    //Config
    OMEGA_CONFIG_VECTOR("robot_tracker/init_position", 2, init_position);
    OMEGA_CONFIG_DEGREE("robot_tracker/init_angle", init_angle);
    OMEGA_CONFIG("robot_tracker/arena", arena);
    
    OMEGA_CONFIG("robot_tracker/position_stddev_time", position_stddev_time);
    OMEGA_CONFIG("robot_tracker/angle_stddev_time", angle_stddev_time);
    OMEGA_CONFIG("robot_tracker/wall_angle_stddev", wall_angle_stddev);
    OMEGA_CONFIG("robot_tracker/wall_position_stddev", wall_position_stddev);
    
    OMEGA_CONFIG("robot_tracker/canny_threshold1", canny_threshold1);
    OMEGA_CONFIG("robot_tracker/canny_threshold2", canny_threshold2);
    OMEGA_CONFIG("robot_tracker/canny_aperture", canny_aperture);
    OMEGA_CONFIG("robot_tracker/hough_distance_resolution", hough_distance_resolution);
    OMEGA_CONFIG_DEGREE("robot_tracker/hough_angle_resolution", hough_angle_resolution);
    OMEGA_CONFIG("robot_tracker/hough_threshold", hough_threshold);
    OMEGA_CONFIG_DEGREE("robot_tracker/horizontal_max_angle", horizontal_max_angle);
    OMEGA_CONFIG_DEGREE("robot_tracker/vertical_max_angle", vertical_max_angle);
    OMEGA_CONFIG("robot_tracker/match_max_a", match_max_a);
    OMEGA_CONFIG("robot_tracker/match_max_b", match_max_b);

    //Arena
    if (arena == "arena2018")
    {}
    else if (arena == "arena2024")
    {}
    else throw std::runtime_error("Unknown arena");

    //State
    _state.segment<2>(0) = init_position;
    _state(2) = init_angle;

    ROS_INFO("omega::RobotTracker initialized");
}

void omega::RobotTracker::update(ros::Time time, const cv::Mat &bgr_image)
{
    std::vector<VisionLine> lines;
    _get_lines(bgr_image, &lines);
}

void omega::RobotTracker::update(ros::Time time, double linear, double angular)
{}

void omega::RobotTracker::update(ros::Time time)
{}

Eigen::Vector2d omega::RobotTracker::get_position() const
{
    return Eigen::Vector2d(_state(0), _state(1));
}

double omega::RobotTracker::get_orientation() const
{
    return _state(2);
}

Eigen::Vector2d omega::RobotTracker::get_position_stddev() const
{
    return Eigen::Vector2d(std::sqrt(_variance(0, 0)), std::sqrt(_variance(1, 1)));
}

double omega::RobotTracker::get_orientation_stddev() const
{
    return std::sqrt(_variance(2, 2));
}

Eigen::Matrix3d omega::RobotTracker::get_variance() const
{
    return _variance;
}