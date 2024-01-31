#include <omega/ball_tracker.h>
#include <omega/kalman.h>
#include <omega/camera.h>
#include <omega/robot_tracker.h>
#include <omega/debugger.h>
#include <omega/omega.h>
#include <opencv2/imgproc.hpp>
#include <cmath>

Eigen::Vector2d omega::BallTracker::Ball::get_position(bool global) const
{
    const Eigen::Vector4d &state = global ? global_state : local_state;
    return state.segment<2>(0);
}

Eigen::Vector2d omega::BallTracker::Ball::get_velocity(bool global) const
{
    const Eigen::Vector4d &state = global ? global_state : local_state;
    return state.segment<2>(2);
}

Eigen::Vector2d omega::BallTracker::Ball::get_position_stddev(bool global) const
{
    const Eigen::Matrix4d &var = global ? local_var : local_var;
    return Eigen::Vector2d(std::sqrt(var(0, 0)), std::sqrt(var(1, 1)));
}

Eigen::Vector2d omega::BallTracker::Ball::get_velocity_stddev(bool global) const
{
    const Eigen::Matrix4d &var = global ? local_var : local_var;
    return Eigen::Vector2d(std::sqrt(var(2, 2)), std::sqrt(var(3, 3)));
}

void omega::BallTracker::_find_countours(const cv::Mat &bgr_image, std::vector<std::vector<cv::Point>> *contours)
{
    //Convert to HSV
    cv::Mat hsv_image;
    cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

    //Cutoff
    cv::Scalar lower(min_hue, min_saturation, min_value);
    cv::Scalar upper(max_hue, max_saturation, max_value);
    cv::Mat binary_image = cv::Mat::zeros(hsv_image.size(), CV_8UC1);
    cv::inRange(hsv_image, lower, upper, binary_image);

    //Erode & dilate
    cv::dilate(binary_image, binary_image, cv::Mat(), cv::Point(-1, -1), dilate_size);
    cv::erode(binary_image, binary_image, cv::Mat(), cv::Point(-1, -1), dilate_size);

    //Publish
    _owner->debugger->draw_mask(binary_image);

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
        _owner->debugger->draw_rectangle(image_box, Debugger::red);

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
        _owner->debugger->draw_circle(cv::Point((int)ball.image_center.x(), (int)ball.image_center.y()), (int)ball.image_radius, Debugger::blue);

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
        const double w_var = ball->image_radius_variance * sqr(p1p2r / sqr(ball->image_radius));

        //Calculate local coordunates and their variance
        const Eigen::Vector4d uvw(ball->image_center.x() * w, ball->image_center.y() * w, w, 1);
        const Eigen::Matrix4d uvw_var = Eigen::Vector4d(ball->image_radius_variance * w, ball->image_radius_variance * w, w_var, 1).asDiagonal();
        const Eigen::Matrix4d T = (Eigen::Translation3d(_owner->camera->base_position) *
            Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ())
            ).matrix() * _owner->camera->matrix_inv;
        const Eigen::Vector4d local = T * uvw;
        const Eigen::Matrix4d local_var = T * uvw_var * T.transpose();
        ball->local_position = local.segment<2>(0);
        ball->local_position_variance = local.block<2, 2>(0, 0);

        //Calculate ball coordinates and their variance
        const Eigen::Vector2d robot_position = _owner->robot_tracker->get_position();
        const double robot_orientation = _owner->robot_tracker->get_orientation();
        const Eigen::Matrix3d robot_variance = _owner->robot_tracker->get_variance();
        Eigen::Matrix4d G = (
            Eigen::Translation3d(robot_position.x(), robot_position.y(), 0) *
            Eigen::AngleAxisd(robot_orientation, Eigen::Vector3d::UnitZ())
        ).matrix();
        Eigen::Matrix<double, 4, 3> G_J = Eigen::Matrix<double, 4, 3>::Zero();
        G_J(0, 0) = 1;
        G_J(1, 1) = 1;
        G_J(0, 2) = -std::sin(robot_orientation) * local.x() - std::cos(robot_orientation) * local.y();
        G_J(1, 2) =  std::cos(robot_orientation) * local.x() - std::sin(robot_orientation) * local.y();
        const Eigen::Vector4d global = G * local;
        const Eigen::Matrix4d global_var = G * local_var * G.transpose() + G_J * robot_variance * G_J.transpose();
        ball->global_position = global.segment<2>(0);
        ball->global_position_variance = global_var.block<2, 2>(0, 0);
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

            const double distance = (ball->local_position - _balls[i].get_position()).norm();
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
            new_ball.local_state.setZero();
            new_ball.local_state.segment<2>(0) = ball->local_position;
            new_ball.local_var.setZero();
            new_ball.local_var.block<2, 2>(0, 0) = ball->local_position_variance;
            new_ball.global_state.setZero();
            new_ball.global_state.segment<2>(0) = ball->global_position;
            new_ball.global_var.setZero();
            new_ball.global_var.block<2, 2>(0, 0) = ball->global_position_variance;
            _balls.push_back(new_ball);

            database_ball_matches.push_back(true);
            ball->match = &_balls.back();
        }
        ball->match->last_seen = now;
    }
}

void omega::BallTracker::_correct(ros::Time now, const std::vector<BallVision> &balls)
{
    Eigen::Matrix<double, 2, 4> C;
    C.setZero();
    C(0, 0) = 1;
    C(1, 1) = 1;

    for (auto ball = balls.begin(); ball != balls.end(); ball++)
    {
        if (ball->match == nullptr || ball->match->last_seen == now) continue;
        
        kalman_correct<double, 4, 2>(ball->match->local_state, ball->match->local_var, ball->local_position, C, ball->local_position_variance);
        kalman_correct<double, 4, 2>(ball->match->global_state, ball->match->global_var, ball->global_position, C, ball->global_position_variance);
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

    ROS_INFO("omega::BallTracker initialized");
}

void omega::BallTracker::update(ros::Time now, const cv::Mat &bgr_image)
{
    std::vector<std::vector<cv::Point>> contours;
    _find_countours(bgr_image, &contours);

    std::vector<BallVision> balls;
    _find_circles(contours, &balls);

    _find_positions(&balls);
    //_match(now, &balls);
    //_update(now, balls);
}

void omega::BallTracker::update(ros::Time now, double linear, double angular)
{
    //Initialize last_update
    if (!_last_update_valid) { _last_update = now; _last_update_valid = true; return; }

    //Delete long unseen balls
    for (int i = _balls.size() - 1; i >= 0; i--)
    {
        if (now - _balls[i].last_seen < ros::Duration(timeout)) _balls.erase(_balls.begin() + i);
    }

    const double dt = (now - _last_update).toSec();
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q.block<2, 2>(0, 0) = dt * sqr(position_stddev_time) * Eigen::Matrix2d::Identity();
    Q.block<2, 2>(2, 2) = dt * sqr(speed_stddev_time) * Eigen::Matrix2d::Identity();

    Eigen::Matrix4d A_global = Eigen::Matrix4d::Zero();
    A_global.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();
    A_global.block<2, 2>(0, 2) = dt * Eigen::Matrix2d::Identity();
    A_global.block<2, 2>(2, 2) = Eigen::Matrix2d::Identity();
    Eigen::Vector4d Bu_global = Eigen::Vector4d::Zero();

    Eigen::Matrix4d A_local = Eigen::Matrix4d::Zero();
    A_local.block<2, 2>(0, 0) = Eigen::Rotation2Dd(-angular).toRotationMatrix();
    A_local.block<2, 2>(0, 2) = dt * Eigen::Rotation2Dd(-angular / 2).toRotationMatrix();
    A_local.block<2, 2>(2, 2) = Eigen::Rotation2Dd(-angular).toRotationMatrix();
    Eigen::Vector4d Bu_local = Eigen::Vector4d::Zero();
    Bu_local.segment<2>(0) = Eigen::Rotation2Dd(-angular / 2).toRotationMatrix() * Eigen::Vector2d(-linear, 0);

    for (auto ball = _balls.begin(); ball != _balls.end(); ball++)
    {
        kalman_update<double, 4, 2>(ball->local_state, ball->local_var, A_local, Bu_local, Q);
        kalman_update<double, 4, 2>(ball->global_state, ball->global_var, A_global, Bu_global, Q);
    }

    _last_update = now;
}

void omega::BallTracker::update(ros::Time now)
{
    update(now, 0.0, 0.0);
}

const std::vector<omega::BallTracker::Ball> &omega::BallTracker::get_balls() const
{
    return _balls;
}