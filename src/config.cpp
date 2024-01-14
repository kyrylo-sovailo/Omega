#include <omega/omega.h>

#define STRING2(s) #s
#define STRING(s) STRING2(s)

#define OMEGA_CONFIG(s) if (!node.getParam("config/" STRING(s), s)) ROS_ERROR("Failed to get parameter " STRING(s))

omega::Config::Config(ros::NodeHandle &node)
{
    OMEGA_CONFIG(wheel_radius);
    OMEGA_CONFIG(wheel_separation);
    OMEGA_CONFIG(max_wheel_vel);
    OMEGA_CONFIG(max_linear_vel);
    OMEGA_CONFIG(max_linear_acc);
    OMEGA_CONFIG(max_angular_vel);
    OMEGA_CONFIG(max_angular_acc);
}