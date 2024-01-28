#include <omega/ball_tracker.h>
#include <omega/kalman.h>
#include <omega/camera.h>
#include <omega/robot_tracker.h>
#include <omega/omega.h>
#include <opencv2/imgproc.hpp>
#include <cmath>

Eigen::Vector2d omega::BallTracker::Ball::get_position() const
{
    return Eigen::Vector2d(state(0), state(1));
}

Eigen::Vector2d omega::BallTracker::Ball::get_velocity() const
{
    return Eigen::Vector2d(state(2), state(3));
}

Eigen::Vector2d omega::BallTracker::Ball::get_position_stddev() const
{
    return Eigen::Vector2d(std::sqrt(state(0, 0)), std::sqrt(state(1, 1)));
}

Eigen::Vector2d omega::BallTracker::Ball::get_velocity_stddev() const
{
    return Eigen::Vector2d(std::sqrt(state(2, 2)), std::sqrt(state(3, 3)));
}

void omega::BallTracker::_find_countours(const cv::Mat &bgr_image, std::vector<std::vector<cv::Point>> *contours)
{
    //Convert to HSV
    cv::Mat hsv_image;
    cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

    //Cutoff
    cv::Scalar lower(min_hue, min_saturation, min_value);
    cv::Scalar upper(max_hue, max_saturation, max_value);
    cv::Mat binary_image = cv::Mat::zeros(bgr_image.size(), CV_8UC1);
    cv::inRange(bgr_image, lower, upper, binary_image);

    //Erode & dilate
    cv::erode(binary_image, binary_image, cv::Mat(), cv::Point(-1, -1), dilate_size);
    cv::dilate(binary_image, binary_image, cv::Mat(), cv::Point(-1, -1), dilate_size);

    //Find contours
    cv::findContours(binary_image, *contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
}

void omega::BallTracker::_find_circles(const std::vector<std::vector<cv::Point>> &contours, std::vector<BallVision> *balls)
{
    for (auto contour = contours.cbegin(); contour != contours.cend(); contour++)
    {
        //Discard too wide, too tall and too small
        cv::Rect image_box = cv::boundingRect(*contour);
        if (0.8 * image_box.width > image_box.height) continue;
        if (0.8 * image_box.height > image_box.width) continue;
        const double image_radius_estimate = std::sqrt(image_box.width/2 * image_box.height/2);
        if (image_radius_estimate < min_radius) continue;

        //Calculate area and center
        BallVision ball;
        Eigen::Vector2d image_point0((*contour)[0].x, (*contour)[0].y);
        ball.image_center = Eigen::Vector2d::Zero();
        double image_area = 0;
        for (auto point = contour->cbegin() + 2; point != contour->cend(); point++)
        {
            const Eigen::Vector2d image_point1((point - 1)->x, (point - 1)->y);
            const Eigen::Vector2d image_point2(point->x, point->y);
            const Eigen::Vector2d v1 = image_point1 - image_point2;
            const Eigen::Vector2d v2 = image_point1 - image_point0;
            const Eigen::Vector2d minicenter = (image_point0 + image_point1 + image_point2) / 3;
            const double miniarea = std::abs((v1.x() * v2.y() - v2.x() * v1.y()) / 2);
            ball.image_center += miniarea * minicenter;
            image_area += miniarea;
        }
        ball.image_center /= image_area;
        if (image_area < min_area * M_PI * image_radius_estimate * image_radius_estimate) continue; //area too small

        //Calculate radius
        ball.image_radius = 0;
        for (auto point = contour->cbegin(); point != contour->cend(); point++)
        {
            const Eigen::Vector2d image_point1(point->x, point->y);
            const double miniradius = (image_point1 - ball.image_center).norm();
            ball.image_radius += miniradius;
        }
        ball.image_radius /= contour->size();
        if (ball.image_radius < min_radius) continue;

        //Calculare variance
        ball.image_radius_variance = 0;
        for (auto point = contour->cbegin(); point != contour->cend(); point++)
        {
            const Eigen::Vector2d image_point1(point->x, point->y);
            const double miniradius = (image_point1 - ball.image_center).norm();
            ball.image_radius_variance += (miniradius - ball.image_radius) * (miniradius - ball.image_radius);
        }
        ball.image_radius_variance /= contour->size();

        balls->push_back(ball);
    }
}

void omega::BallTracker::_find_positions(std::vector<BallVision> *balls)
{
    const double p1 = std::hypot(_owner->camera->matrix(0, 0), _owner->camera->matrix(0, 1));
    const double p2 = std::hypot(_owner->camera->matrix(1, 0), _owner->camera->matrix(1, 1));
    const double p1p2r = std::sqrt(p1 * p2) * radius;

    for (auto ball = balls->begin(); ball != balls->end(); ball++)
    {    
        //Calculate w and w variance
        const double w = p1p2r / ball->image_radius;
        const double w_var = ball->image_radius_variance * sqr(p1p2r / ball->image_radius);

        //Calculate local coordunates and their variance
        const Eigen::Vector4d uvw(ball->image_center.x() * w, ball->image_center.y() * w, w, 1);
        const Eigen::Matrix4d uvw_var = Eigen::Vector4d(ball->image_radius_variance * w, ball->image_radius_variance * w, w_var, 1).asDiagonal();
        Eigen::Vector4d local = _owner->camera->matrix_inverse * uvw; //TODO: Rotation???
        local.segment<3>(0) += _owner->camera->base_position;
        const Eigen::Matrix4d local_var = _owner->camera->matrix_inverse * uvw_var * _owner->camera->matrix_inverse.transpose();

        //Calculate ball coordinates and their variance
        const Eigen::Vector2d robot_position = _owner->robot_tracker->get_position();
        const double robot_orientation = _owner->robot_tracker->get_orientation();
        const Eigen::Matrix3d robot_variance = _owner->robot_tracker->get_variance();
        Eigen::Matrix4d tr = (
            Eigen::Translation3d(robot_position.x(), robot_position.y(), 0) *
            Eigen::AngleAxisd(robot_orientation, Eigen::Vector3d::UnitX())
        ).matrix();
        Eigen::Matrix<double, 4, 3> tr_j = Eigen::Matrix<double, 4, 3>::Zero();
        tr_j(0, 0) = 1;
        tr_j(1, 1) = 1;
        tr_j(0, 2) = -std::sin(robot_orientation) * local.x() - std::cos(robot_orientation) * local.y();
        tr_j(1, 2) =  std::cos(robot_orientation) * local.x() - std::sin(robot_orientation) * local.y();
        const Eigen::Vector4d global = tr * local;
        const Eigen::Matrix4d global_var = tr * local_var * tr.transpose() + tr_j * robot_variance * tr_j.transpose();

        ball->ball_position = global.segment<2>(0);
        ball->ball_position_variance = global_var.block<2, 2>(0, 0);
    }
}

void omega::BallTracker::_match(ros::Time now, std::vector<BallVision> *balls)
{
    //TODO: distance based algorithm, needs to be probability based

    //For every seen ball
    std::vector<bool> database_ball_matches(false, _balls.size());
    for (auto ball = balls->begin(); ball != balls->end(); ball++)
    {
        if (ball->match != nullptr) continue;
        
        //Search for nearest database ball
        double min_distance = std::numeric_limits<double>::quiet_NaN();
        unsigned int min_distance_ball;
        for (unsigned int i = 0; i < _balls.size(); i++)
        {
            if (database_ball_matches[i]) continue;

            const double distance = (ball->ball_position - _balls[i].get_position()).norm();
            if (distance < 0.1 && distance < min_distance) { min_distance = distance; min_distance_ball = i; }
        }
        
        if (min_distance == min_distance)
        {
            //If found
            database_ball_matches[min_distance_ball] = true;
            ball->match = &_balls[min_distance_ball];
        }
        else
        {
            //If not found
            Ball new_ball;
            new_ball.state.setZero();
            new_ball.var.setZero();
            new_ball.state.segment<2>(0) = ball->ball_position;
            new_ball.var.block<2, 2>(0, 0) = ball->ball_position_variance;
            _balls.push_back(new_ball);

            database_ball_matches.resize(_balls.size(), false);
            ball->match = &_balls.back();

            _balls[min_distance_ball].last_seen = now;
        }
    }
}

void omega::BallTracker::_update(ros::Time now, const std::vector<BallVision> &balls)
{
    Eigen::Matrix<double, 2, 4> C;
    C.setZero();
    C(0, 0) = 1;
    C(1, 1) = 1;

    for (auto ball = balls.begin(); ball != balls.end(); ball++)
    {
        if (ball->match == nullptr || ball->match->last_seen == now) continue;
        
        kalman_correct<double, 4, 2>(ball->match->state, ball->match->var, ball->ball_position, C, ball->ball_position_variance);
        ball->match->last_seen = now;
    }
}

omega::BallTracker::BallTracker(ros::NodeHandle *node, Omega *owner) : _owner(owner)
{
    OMEGA_CONFIG("ball_tracker/radius", radius);
    OMEGA_CONFIG("ball_tracker/position_stddev_time", position_stddev_time);
    OMEGA_CONFIG("ball_tracker/speed_stddev_time", speed_stddev_time);

    OMEGA_CONFIG("ball_tracker/min_hue", min_hue);
    OMEGA_CONFIG("ball_tracker/max_hue", max_hue);
    OMEGA_CONFIG("ball_tracker/min_saturation", min_saturation);
    OMEGA_CONFIG("ball_tracker/max_saturation", max_saturation);
    OMEGA_CONFIG("ball_tracker/min_value", min_value);
    OMEGA_CONFIG("ball_tracker/max_value", max_value);
    OMEGA_CONFIG("ball_tracker/dilate_size", dilate_size);

    OMEGA_CONFIG("ball_tracker/min_radius", min_radius);
    OMEGA_CONFIG("ball_tracker/min_area", min_area);
    OMEGA_CONFIG("ball_tracker/timeout", timeout);
}

void omega::BallTracker::update(ros::Time now, const cv::Mat &bgr_image)
{
    std::vector<std::vector<cv::Point>> contours;
    _find_countours(bgr_image, &contours);

    //std::vector<BallVision> balls;
    //_find_circles(contours, &balls);

    //_find_positions(&balls);
    //_match(now, &balls);
    //_update(now, balls);
}

void omega::BallTracker::update(ros::Time now)
{
    if (!_last_update_valid) { _last_update = now; _last_update_valid = true; return; }

    for (int i = _balls.size() - 1; i >= 0; i--)
    {
        if (now - _balls[i].last_seen < ros::Duration(timeout)) _balls.erase(_balls.begin() + i);
    }

    /*
    Eigen::Matrix4d A;
    A.setZero();
    A(0, 2) = 1;
    A(1, 3) = 1;
    Eigen::Vector4d Bu;
    Bu.setZero();

    for (auto ball = _balls.begin(); ball != _balls.end(); ball++)
    {
        const double dt = (now - _last_update).toSec();
        Eigen::Matrix4d Q = Eigen::Vector4d(
            dt * sqr(position_stddev_time),
            dt * sqr(position_stddev_time),
            dt * sqr(speed_stddev_time),
            dt * sqr(speed_stddev_time)).asDiagonal();

        kalman_update<double, 4, 2>(ball->state, ball->var, A, Bu, Q);
    }

    _last_update = now;
    */
}

const std::vector<omega::BallTracker::Ball> &omega::BallTracker::get_balls() const
{
    return _balls;
}