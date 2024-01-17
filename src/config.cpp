#include <omega/omega.h>

omega::Config::Config(ros::NodeHandle &node)
{
    OMEGA_CONFIG(debug);
    OMEGA_CONFIG(wheel_radius);
    OMEGA_CONFIG(wheel_separation);
    OMEGA_CONFIG(max_wheel_vel);
    OMEGA_CONFIG(max_linear_vel);
    OMEGA_CONFIG(max_linear_acc);
    OMEGA_CONFIG(max_angular_vel);
    OMEGA_CONFIG(max_angular_acc);
}