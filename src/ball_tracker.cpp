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

omega::BallTracker::BallTracker(ros::NodeHandle *node, Omega *owner) : _owner(owner)
{
    OMEGA_CONFIG("ball_tracker/radius", radius);
    OMEGA_CONFIG("ball_tracker/roll_distance", roll_distance);

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

void omega::BallTracker::update(ros::Time now, const cv::Mat &const_image)
{
    //Convert to HSV
    cv::Mat image;
    cv::cvtColor(const_image, image, cv::COLOR_BGR2HSV);

    //Cutoff
    cv::Scalar lower(min_hue, min_saturation, min_value);
    cv::Scalar upper(max_hue, max_saturation, max_value);
    cv::Mat binary_image = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::inRange(image, lower, upper, binary_image);

    //Erode & dilate
    cv::erode(binary_image, binary_image, cv::Mat(), cv::Point(-1, -1), dilate_size);
    cv::dilate(binary_image, binary_image, cv::Mat(), cv::Point(-1, -1), dilate_size);

    //Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    //Search for circles
    std::vector<bool> seen(_balls.size(), false);
    for (auto contour = contours.cbegin(); contour != contours.cend(); contour++)
    {
        //Discard too wide, too tall and too small
        cv::Rect image_box = cv::boundingRect(*contour);
        if (0.8 * image_box.width > image_box.height) continue;
        if (0.8 * image_box.height > image_box.width) continue;
        const double image_radius_estimate = std::sqrt(image_box.width/2 * image_box.height/2);
        if (image_radius_estimate < min_radius) continue;

        //Calculate area and center
        Eigen::Vector2d image_point0((*contour)[0].x, (*contour)[0].y);
        Eigen::Vector2d image_center(0, 0);
        double image_area = 0;
        for (auto point = contour->cbegin() + 2; point != contour->cend(); point++)
        {
            const Eigen::Vector2d image_point1((point - 1)->x, (point - 1)->y);
            const Eigen::Vector2d image_point2(point->x, point->y);
            const Eigen::Vector2d v1 = image_point1 - image_point2;
            const Eigen::Vector2d v2 = image_point1 - image_point0;
            const Eigen::Vector2d minicenter = (image_point0 + image_point1 + image_point2) / 3;
            const double miniarea = std::abs((v1.x() * v2.y() - v2.x() * v1.y()) / 2);
            image_center += miniarea * minicenter;
            image_area += miniarea;
        }
        image_center /= image_area;
        if (image_area < min_area * M_PI * image_radius_estimate * image_radius_estimate) continue; //area too small

        //Calculate radius and variance
        double image_radius = 0, image_radius_variance = 0;
        for (auto point = contour->cbegin(); point != contour->cend(); point++)
        {
            const Eigen::Vector2d image_point1(point->x, point->y);
            const double miniradius = (image_point1 - image_center).norm();
            image_radius += miniradius;
        }
        image_radius /= contour->size();
        if (image_radius < min_radius) continue;
        for (auto point = contour->cbegin(); point != contour->cend(); point++)
        {
            const Eigen::Vector2d image_point1(point->x, point->y);
            const double miniradius = (image_point1 - image_center).norm();
            image_radius_variance += (miniradius - image_radius) * (miniradius - image_radius);
        }
        image_radius_variance /= contour->size();

        //Calculate coordinates
        const double p1 = std::hypot(_owner->camera->matrix(0, 0), _owner->camera->matrix(0, 1));
        const double p2 = std::hypot(_owner->camera->matrix(1, 0), _owner->camera->matrix(1, 1));
        const double w = std::sqrt(p1 * p2) * radius / image_radius;
        Eigen::Vector4d uvw(image_center.x() * w, image_center.y() * w, w, 1);
        Eigen::Vector4d center4 = _owner->camera->matrix_inverse * uvw;
        Eigen::Vector3d center = center4.segment<3>(0);
        center = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()) * center;
        center += _owner->camera->base_position;
        center = Eigen::Translation3d(_owner->robot_tracker->get_position().x(), _owner->robot_tracker->get_position().y(), 0) *
            Eigen::AngleAxisd(_owner->robot_tracker->get_orientation(), Eigen::Vector3d::UnitZ()) *
            center;
        const Eigen::Vector2d center2 = center.segment<2>(0);

        //Search ball
        double min_distance = std::numeric_limits<double>::quiet_NaN();
        unsigned int min_distance_ball;
        for (unsigned int i = 0; i < _balls.size(); i++)
        {
            const double distance = (center2 - _balls[i].get_position()).norm();
            if (distance < min_distance) { min_distance = distance; min_distance_ball = i; }
        }

        //Update ball
        if (min_distance == min_distance)
        {
            seen[min_distance_ball] = true;
            _balls[min_distance_ball].last_seen = now;
            //Update
        }
    }
}

void omega::BallTracker::update(ros::Time now)
{
    for (int i = _balls.size() - 1; i >= 0; i--)
    {
        if (now - _balls[i].last_seen < ros::Duration(timeout)) _balls.erase(_balls.begin() + i);
    }
}

const std::vector<omega::BallTracker::Ball> &omega::BallTracker::get_balls() const
{
    return _balls;
}