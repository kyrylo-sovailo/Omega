#include <omega/config.h>
#include <omega/omega.h>

omega::Config::Config(ros::NodeHandle *node)
{
    OMEGA_CONFIG("config/distance_to_ball", distance_to_ball);
    OMEGA_CONFIG("config/distance_to_wall", distance_to_wall);
    ROS_INFO("omega::Config initialized");
}