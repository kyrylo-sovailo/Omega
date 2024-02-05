#include <omega/robot_tracker.h>
#include <omega/camera.h>
#include <omega/debugger.h>
#include <omega/kalman.h>
#include <omega/omega.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>

Eigen::Vector4d omega::RobotTracker::_extend(const Eigen::Vector3d &point)
{
    Eigen::Vector4d ex;
    ex.segment<3>(0) = point;
    ex(3) = 1;
    return ex;
}

Eigen::Matrix<omega::ddouble3, 4, 1> omega::RobotTracker::_extend(const Eigen::Matrix<ddouble3, 3, 1> &point)
{
    Eigen::Matrix<ddouble3, 4, 1> ex;
    ex.segment<3>(0) = point;
    ex(3) = ddouble3(1);
    return ex;
}

Eigen::Matrix<omega::ddouble3, 3, 1> omega::RobotTracker::_world_to_local(const Eigen::Vector3d &point) const
{
    ddouble3 x = _state.x(); x.derivative[0] = 1;
    ddouble3 y = _state.y(); y.derivative[1] = 1;
    ddouble3 theta = _state.z(); theta.derivative[2] = 1;
    return Eigen::Translation<ddouble3, 3>(-_owner->camera->base_position.cast<ddouble3>()) *
        Eigen::AngleAxis<ddouble3>(-theta, Eigen::Matrix<ddouble3, 3, 1>::UnitZ()) *
        Eigen::Translation<ddouble3, 3>(-x, -y, 0) *
        point.cast<ddouble3>();
}
Eigen::Vector2d omega::RobotTracker::_local_to_screen(const Eigen::Vector3d &point) const
{
    Eigen::Vector4d uvw = _owner->camera->matrix * _extend(point);
    return Eigen::Vector2d(uvw(0) / uvw(2), uvw(1) / uvw(2));
}

Eigen::Matrix<omega::ddouble3, 2, 1> omega::RobotTracker::_local_to_screen(const Eigen::Matrix<ddouble3, 3, 1> &point) const
{
    Eigen::Matrix<omega::ddouble3, 4, 1> uvw = _owner->camera->matrix.cast<ddouble3>() * _extend(point);
    return Eigen::Matrix<omega::ddouble3, 2, 1>(uvw(0) / uvw(2), uvw(1) / uvw(2));
}

bool omega::RobotTracker::_is_intersection_on_screen(const Eigen::Vector3d &source, const Eigen::Vector3d &destination, bool horizontal, bool upper, double &t) const
{
    const Eigen::Vector4d source_ex = _extend(source);
    const Eigen::Vector4d destination_ex = _extend(destination);
    const Eigen::Matrix<double, 1, 4> Kuv = _owner->camera->matrix.row(horizontal ? 0 : 1);
    const Eigen::Matrix<double, 1, 4> Kw = _owner->camera->matrix.row(2);
    const double k = 0.5 * (upper ? 1 : -1) * (horizontal ? _owner->camera->width : _owner->camera->height);
    t = ((double)(-Kuv * source_ex) + (double)(k * Kw * source_ex)) /
        ((double)(-Kuv * source_ex) + (double)(Kuv * destination_ex) + (double)(k * Kw * source_ex) - (double)(k * Kw * destination_ex));
    if (t > 0 && t < 1)
    {
        const Eigen::Matrix<double, 1, 4> Kuv_alt = _owner->camera->matrix.row(horizontal ? 1 : 0);
        const double k_alt = 0.5 * (horizontal ? _owner->camera->height : _owner->camera->width);
        const Eigen::Vector4d point_ex = _extend(Eigen::Vector3d(destination * t + source * (1 - t)));
        const double uv_alt = ((double)(Kuv_alt * point_ex)) / ((double)(Kw * point_ex));
        if (uv_alt > -k && uv_alt < k) return true;
    }
    return false;
}

bool omega::RobotTracker::_is_point_on_screen(const Eigen::Vector2d &point) const
{
    double ku = 0.5 * _owner->camera->width;
    double kv = 0.5 * _owner->camera->height;
    return point.x() >= -ku && point.x() <= ku && point.y() >= -kv && point.y() <= kv;
}

bool omega::RobotTracker::_is_line_on_screen(const Eigen::Vector3d &source, const Eigen::Vector3d &destination, double &t_source, double &t_destination) const
{
    //Find all valid on-screen intersections
    unsigned int intersections_count = 0;
    double intersections[4];
    for (unsigned int i = 0; i < 4; i++)
    {
        double t;
        if (_is_intersection_on_screen(source, destination, i == 0 || i == 1, i == 0 || i == 3, t))
        {
            Eigen::Vector3d point = t * destination + (1 - t) * source;
            if (_is_point_on_screen(_local_to_screen(point)))
            {
                intersections[intersections_count] = t;
                intersections_count++;
            }
        }
    }

    //Sort intersections
    std::sort(intersections, intersections + intersections_count);

    //Return line segment whose middle is on screen
    double running_source = 0.0;
    for (unsigned int i = 0; i < intersections_count + 1; i++)
    {
        double running_destination = (i <= intersections_count) ? intersections[i] : 1.0;
        double running_middle = (running_source + running_destination) / 2;
        Eigen::Vector3d running_middle_point = running_middle * destination + (1 - running_middle) * source;
        if (_is_point_on_screen(_local_to_screen(running_middle_point)))
        {
            t_source = running_source;
            t_destination = running_destination;
            return true;
        }
        running_source = running_destination;
    }
    return false;
}

bool omega::RobotTracker::_get_expected_horizontal_line(const Eigen::Matrix<ddouble3, 3, 1> &source, const Eigen::Matrix<ddouble3, 3, 1> &destination, ddouble3 &a, ddouble3 &b) const
{
    double t_source, t_destination;
    if (_is_line_on_screen(source.cast<double>(), destination.cast<double>(), t_source, t_destination))
    {
        Eigen::Matrix<ddouble3, 3, 1> minisource = t_source * destination + (1 - t_source) * source;
        Eigen::Matrix<ddouble3, 3, 1> minidestination = t_destination * destination + (1 - t_destination) * source;
        Eigen::Matrix<ddouble3, 2, 1> screen_source = _local_to_screen(minisource);
        Eigen::Matrix<ddouble3, 2, 1> screen_destination = _local_to_screen(minidestination);
        a = (screen_destination.y() - screen_source.y()) / (screen_destination.x() - screen_source.x());
        b = screen_source.y() - a * screen_source.x();
        return true;
    }
    return false;
}

bool omega::RobotTracker::_get_expected_vertical_line(const Eigen::Matrix<ddouble3, 3, 1> &source, const Eigen::Matrix<ddouble3, 3, 1> &destination, ddouble3 &b) const
{
    double t_source, t_destination;
    if (_is_line_on_screen(source.cast<double>(), destination.cast<double>(), t_source, t_destination))
    {
        Eigen::Matrix<ddouble3, 3, 1> minisource = t_source * destination + (1 - t_source) * source;
        Eigen::Matrix<ddouble3, 3, 1> minidestination = t_destination * destination + (1 - t_destination) * source;
        Eigen::Matrix<ddouble3, 2, 1> screen_source = _local_to_screen(minisource);
        Eigen::Matrix<ddouble3, 2, 1> screen_destination = _local_to_screen(minidestination);
        ddouble3 a = (screen_destination.x() - screen_source.x()) / (screen_destination.y() - screen_source.y());
        b = screen_source.x() - a * screen_source.y();
        return true;
    }
    return false;
}

void omega::RobotTracker::_get_lines(const cv::Mat &hsv_image, std::vector<VisionLine> *lines) const
{
    //Color recognition
    cv::Scalar wall_lower(wall_color_min[0], wall_color_min[1], wall_color_min[2]);
    cv::Scalar wall_upper(wall_color_max[0], wall_color_max[1], wall_color_max[2]);
    cv::Scalar grass_lower(grass_color_min[0], grass_color_min[1], grass_color_min[2]);
    cv::Scalar grass_upper(grass_color_max[0], grass_color_max[1], grass_color_max[2]);
    cv::Mat wall_binary_image, grass_binary_image;
    cv::inRange(hsv_image, wall_lower, wall_upper, wall_binary_image);
    cv::inRange(hsv_image, grass_lower, grass_upper, grass_binary_image);
    cv::bitwise_or(wall_binary_image, grass_binary_image, wall_binary_image);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_size, dilate_size));
    cv::dilate(wall_binary_image, wall_binary_image, element);
    _owner->debugger->draw_mask(wall_binary_image, Debugger::white);
    
    //Edge detection
    cv::Mat channels[3];
    cv::split(hsv_image, channels);
    cv::Mat edge_binary_image;
    cv::Canny(channels[2], edge_binary_image, canny_threshold1, canny_threshold2, canny_aperture); //Use only value for edge detection
    cv::bitwise_and(edge_binary_image, wall_binary_image, edge_binary_image);
    _owner->debugger->draw_mask(edge_binary_image, Debugger::blue);
    
    //Find horizontal lines
    std::function<bool(const cv::Vec3f &a, const cv::Vec3f &b)> sort = [](const cv::Vec3f &a, const cv::Vec3f &b) -> bool { return a[2] > b[2]; };
    std::vector<cv::Vec3f> hough_lines;
    cv::Mat edge_copy = edge_binary_image.clone();
    cv::HoughLines(edge_copy, hough_lines, hough_distance_resolution, hough_angle_resolution, horizontal_hough_threshold, 0, 0,
        M_PI / 2 - horizontal_max_angle, M_PI / 2 + horizontal_max_angle);
    std::sort(hough_lines.begin(), hough_lines.end(), sort);
    for (unsigned int i = 0; i < hough_lines.size() && i < horizontal_max_lines; i++)
    {
        VisionLine line;
        line.horizontal = true;
        line.r = hough_lines[i][0];
        line.theta = hough_lines[i][1];
        line.a = -std::cos(line.theta) / std::sin(line.theta);
        line.b = line.r / std::sin(line.theta);
        _owner->debugger->draw_line(line.r, line.theta, Debugger::red);
        lines->push_back(line);
    }
    
    //Find vertical lines
    hough_lines.resize(0);
    edge_copy = edge_binary_image.clone();
    cv::HoughLines(edge_copy, hough_lines, hough_distance_resolution, hough_angle_resolution, vertical_hough_threshold, 0, 0,
        0, vertical_max_angle);
    {
        std::vector<cv::Vec3f> hough_lines2;
        edge_copy = edge_binary_image;
        cv::HoughLines(edge_copy, hough_lines2, hough_distance_resolution, hough_angle_resolution, vertical_hough_threshold, 0, 0,
            M_PI - vertical_max_angle, M_PI);
        hough_lines.insert(hough_lines.end(), hough_lines2.cbegin(), hough_lines2.cend());
    }
    std::sort(hough_lines.begin(), hough_lines.end(), sort);
    for(unsigned int i = 0; i < hough_lines.size() && i < vertical_max_lines; i++)
    {
        VisionLine line;
        line.horizontal = false;
        line.r = hough_lines[i][0];
        line.theta = hough_lines[i][1];
        line.a = -std::sin(line.theta) / std::cos(line.theta);
        line.b = line.r / std::cos(line.theta);
        lines->push_back(line);
        _owner->debugger->draw_line(line.r, line.theta, Debugger::red);
    }
}

void omega::RobotTracker::_match_vertical_lines(std::vector<ExpectedVerticalEdge> *edges, std::vector<VisionLine> *lines) const
{
    for (auto edge = edges->begin(); edge != edges->end(); edge++)
    {
        double min_distance = std::numeric_limits<double>::infinity();
        VisionLine *min_distance_line;
        for (auto line = lines->begin(); line != lines->end(); line++)
        {
            if (line->matched || line->horizontal) continue;
            if (std::abs(edge->b.value - line->b) > match_max_b) continue;
            const double distance = std::abs((edge->b.value - line->b) / match_max_b);
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

void omega::RobotTracker::_get_expected_horizontal_walls(std::vector<ExpectedHorizontalWall> *walls) const
{
    for (auto wall = _horizontal_walls.cbegin(); wall != _horizontal_walls.cend(); wall++)
    {
        Eigen::Matrix<ddouble3, 3, 1> source_upper = _world_to_local(Eigen::Vector3d(wall->source.x(), wall->source.y(), wall->height));
        Eigen::Matrix<ddouble3, 3, 1> source_lower = _world_to_local(Eigen::Vector3d(wall->source.x(), wall->source.y(), 0));
        Eigen::Matrix<ddouble3, 3, 1> destination_upper = _world_to_local(Eigen::Vector3d(wall->destination.x(), wall->destination.y(), wall->height));
        Eigen::Matrix<ddouble3, 3, 1> destination_lower = _world_to_local(Eigen::Vector3d(wall->destination.x(), wall->destination.y(), 0));
        ExpectedHorizontalWall expected;
        if (_get_expected_horizontal_line(source_lower, destination_lower, expected.a_lower, expected.b_lower)
        && _get_expected_horizontal_line(source_upper, destination_upper, expected.a_upper, expected.b_upper))
        {
            Eigen::Vector2d source_upper_screen = _local_to_screen(Eigen::Vector3d(source_upper.cast<double>()));
            Eigen::Vector2d source_lower_screen = _local_to_screen(Eigen::Vector3d(source_lower.cast<double>()));
            Eigen::Vector2d destination_upper_screen = _local_to_screen(Eigen::Vector3d(destination_upper.cast<double>()));
            Eigen::Vector2d destination_lower_screen = _local_to_screen(Eigen::Vector3d(destination_lower.cast<double>()));
            _owner->debugger->draw_line(source_upper_screen, destination_upper_screen, Debugger::green);
            _owner->debugger->draw_line(source_lower_screen, destination_lower_screen, Debugger::green);
            walls->push_back(expected);
        }
    }
}

void omega::RobotTracker::_get_expected_vertical_edges(std::vector<ExpectedVerticalEdge> *edges) const
{
    for (auto edge = _vertical_edges.cbegin(); edge != _vertical_edges.cend(); edge++)
    {
        Eigen::Matrix<ddouble3, 3, 1> source = _world_to_local(Eigen::Vector3d(edge->coordinate.x(), edge->coordinate.y(), 0.0));
        Eigen::Matrix<ddouble3, 3, 1> destination = _world_to_local(Eigen::Vector3d(edge->coordinate.x(), edge->coordinate.y(), edge->height));
        ExpectedVerticalEdge expected;
        if (_get_expected_vertical_line(source, destination, expected.b))
        {
            Eigen::Vector2d source_screen = _local_to_screen(Eigen::Vector3d(source.cast<double>()));
            Eigen::Vector2d destination_screen = _local_to_screen(Eigen::Vector3d(destination.cast<double>()));
            _owner->debugger->draw_line(source_screen, destination_screen, Debugger::green);
            edges->push_back(expected);
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
    derivative.resize(_construct_dimension(walls, edges), 3);
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
    
    OMEGA_CONFIG_COLOR("robot_tracker/wall_color_min", wall_color_min);
    OMEGA_CONFIG_COLOR("robot_tracker/wall_color_max", wall_color_max);
    OMEGA_CONFIG_COLOR("robot_tracker/grass_color_min", grass_color_min);
    OMEGA_CONFIG_COLOR("robot_tracker/grass_color_max", grass_color_max);
    OMEGA_CONFIG("robot_tracker/dilate_size", dilate_size);
    OMEGA_CONFIG("robot_tracker/canny_threshold1", canny_threshold1);
    OMEGA_CONFIG("robot_tracker/canny_threshold2", canny_threshold2);
    OMEGA_CONFIG("robot_tracker/canny_aperture", canny_aperture);
    OMEGA_CONFIG("robot_tracker/hough_distance_resolution", hough_distance_resolution);
    OMEGA_CONFIG_DEGREE("robot_tracker/hough_angle_resolution", hough_angle_resolution);
    OMEGA_CONFIG("robot_tracker/horizontal_hough_threshold", horizontal_hough_threshold);
    OMEGA_CONFIG_DEGREE("robot_tracker/horizontal_max_angle", horizontal_max_angle);
    OMEGA_CONFIG("robot_tracker/horizontal_max_lines", horizontal_max_lines);
    OMEGA_CONFIG("robot_tracker/vertical_hough_threshold", vertical_hough_threshold);
    OMEGA_CONFIG_DEGREE("robot_tracker/vertical_max_angle", vertical_max_angle);
    OMEGA_CONFIG("robot_tracker/vertical_max_lines", vertical_max_lines);

    OMEGA_CONFIG("robot_tracker/match_max_a", match_max_a);
    OMEGA_CONFIG("robot_tracker/match_max_b", match_max_b);

    //Arena
    if (arena == "arena2018" || arena == "arena2024")
    {
        _vertical_edges.resize(6);
        for (unsigned int i = 0; i < 6; i++) _vertical_edges[i].height = 0.250;
        _vertical_edges[0].coordinate = Eigen::Vector2d(0.010, 0.950);
        _vertical_edges[1].coordinate = Eigen::Vector2d(1.280, 0.950);
        _vertical_edges[2].coordinate = Eigen::Vector2d(1.510, 0.722);
        for (unsigned int i = 0; i < 3; i++) { _vertical_edges[3 + i].coordinate = _vertical_edges[2 - i].coordinate; _vertical_edges[3 + i].coordinate.y() *= -1; }
        _horizontal_walls.resize(6);
        for (unsigned int i = 0; i < 6; i++)
        {
            _horizontal_walls[i].source = _vertical_edges[i].coordinate;
            _horizontal_walls[i].destination = _vertical_edges[(i + 1) % 6].coordinate;
            _horizontal_walls[i].height = 0.250;
        }
    }
    else throw std::runtime_error("Unknown arena");

    //State
    _state.segment<2>(0) = init_position;
    _state(2) = init_angle;

    ROS_INFO("omega::RobotTracker initialized");
}

void omega::RobotTracker::update(ros::Time now, const cv::Mat &hsv_image)
{
    std::vector<VisionLine> lines;
    std::vector<ExpectedHorizontalWall> walls;
    std::vector<ExpectedVerticalEdge> edges;
    _get_lines(hsv_image, &lines);
    _get_expected_horizontal_walls(&walls);
    _get_expected_vertical_edges(&edges);
    _match_vertical_lines(&edges, &lines);
    _match_horizontal_lines(&walls, &lines);

    Eigen::Matrix<double, Eigen::Dynamic, 1> expectation, measurement;
    Eigen::Matrix<double, Eigen::Dynamic, 3> derivative;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> variance;
    _construct_expectation(walls, edges, expectation);
    _construct_expectation_derivative(walls, edges, derivative);
    _construct_measurement(walls, edges, measurement);
    _construct_measurement_variance(walls, edges, variance);
    //kalman_correct<double, 3, Eigen::Dynamic>(); //TODO: implement
}

void omega::RobotTracker::update(ros::Time now, double linear_speed, double angular_speed)
{
    //Initialize last_update
    if (!_last_update_valid) { _last_update = now; _last_update_valid = true; return; }

    //TODO: add deviation pro distance
    const double dt = (now - _last_update).toSec();
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q.block<2, 2>(0, 0) = dt * sqr(position_stddev_time) * Eigen::Matrix2d::Identity();
    Q.block<2, 2>(2, 2) = dt * sqr(angle_stddev_time) * Eigen::Matrix2d::Identity();
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    Eigen::Vector3d Bu(linear_speed * std::cos(_state(2)), linear_speed * std::sin(_state(2)), angular_speed); //TODO: trapezoidal integration

    //Run Kalman
    //kalman_update<double, 3>(_state, _variance, A, Bu, Q); //TODO: implement

    //Remember update time
    _last_update = now;
}

void omega::RobotTracker::update(ros::Time now)
{
    //Do nothing
}

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