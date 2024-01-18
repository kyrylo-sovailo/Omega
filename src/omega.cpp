#include <omega/omega.h>
#include <omega/arm.h>
#include <omega/ball_tracker.h>
#include <omega/camera.h>
#include <omega/config.h>
#include <omega/gripper.h>
#include <omega/robot_tracker.h>
#include <omega/timer.h>
#include <omega/wheels.h>

omega::Omega::Omega(ros::NodeHandle *node)
{
    _joint_state_sub = node->subscribe("joints/joint_states", 1, &Omega::joint_state_update, this);
    config = new Config(node);
    timer = new Timer(node, this);
    camera = new Camera(node, this);
    arm = new Arm(node, this);
    gripper = new Gripper(node, this);
    wheels = new Wheels(node, this);
    ball_tracker = new BallTracker(node, this);
    robot_tracker = new RobotTracker(node, this);
}

void omega::Omega::joint_state_update(const sensor_msgs::JointState::ConstPtr &msg)
{}

void omega::Omega::imu_update(const sensor_msgs::Imu::ConstPtr &msg)
{}

void omega::Omega::image_update(const sensor_msgs::Image::ConstPtr &msg)
{}

void omega::Omega::grasp_state_update(const turtlebot3_msgs::GraspState::ConstPtr &msg)
{}

void omega::Omega::timer_update(const ros::TimerEvent &event)
{}

omega::Omega::~Omega()
{}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omega_node");
    ros::NodeHandle node;
    omega::Omega omega(&node);
    ros::spin();
    return 0;
}