#include <omega/timer.h>

omega::Timer::Timer(ros::NodeHandle &node, OmegaHeader *header)
{
    _timer = node.createTimer(ros::Duration(1.0), &OmegaHeader::timer_callback, header);
}