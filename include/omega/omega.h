#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

namespace omega
{
    class Omega
    {
    private:
        //Subscribers
        ros::Subscriber _joint_state_sub;
        ros::Subscriber _imu_sub;

        //Publishers
        ros::Publisher _wheel_l_vel_cmd_pub;
        ros::Publisher _wheel_r_vel_cmd_pub;

        //Timer
        ros::Timer _timer;

    public:
        Omega(ros::NodeHandle &handle);
        void _joint_state_callback(const sensor_msgs::JointState::ConstPtr msg);
        void _imu_callback(const sensor_msgs::Imu::ConstPtr msg);
        void _timer_callback(const ros::TimerEvent &msg);
    };
};