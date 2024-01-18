#include <omega/timer.h>
#include <omega/omega.h>

omega::Timer::Timer(ros::NodeHandle *node, Omega *owner)
{
    //Config
    OMEGA_CONFIG("timer/frequency", frequency);

    //Technical
    _timer = node->createTimer(ros::Duration(1 / frequency), &Omega::timer_update, owner);
}