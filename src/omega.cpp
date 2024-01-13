#include <omega/omega.h>

omega::Omega::Omega(ros::NodeHandle &handle)
{
    //Read parameters
    double speed_left = handle.param("config/speed_left", 0.0);
    double speed_right = handle.param("config/speed_right", 0.0);

    //Initialize subscribers
    _joint_state_sub = handle.subscribe("joints/joint_states", 1, &Omega::_joint_state_callback, this);
    _imu_sub = handle.subscribe("sensor/imu_filtered", 1, &Omega::_imu_callback, this);
    
    //Initialize publishers
    _wheel_l_vel_cmd_pub = handle.advertise<std_msgs::Float64>("joints/wheel_left_controller/command", 1, true);
    _wheel_r_vel_cmd_pub = handle.advertise<std_msgs::Float64>("joints/wheel_right_controller/command", 1, true);
    
    //Intialize timers
    _timer = handle.createTimer(ros::Duration(0, 100 * 1000000), &Omega::_timer_callback, this);

    //Write speed
    std_msgs::Float64 msg;
    msg.data = speed_left;  _wheel_l_vel_cmd_pub.publish(msg);
    msg.data = speed_right; _wheel_r_vel_cmd_pub.publish(msg);
}

void omega::Omega::_joint_state_callback(const sensor_msgs::JointState::ConstPtr msg)
{
    ROS_INFO("_joint_state_callback");
}

void omega::Omega::_imu_callback(const sensor_msgs::Imu::ConstPtr msg)
{
    ROS_INFO("_imu_callback");
}

void omega::Omega::_timer_callback(const ros::TimerEvent &msg)
{
    ROS_INFO("_timer_callback");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omega");
    ros::NodeHandle handle;
    omega::Omega omega(handle);
    ros::spin();
    return 0;
}
