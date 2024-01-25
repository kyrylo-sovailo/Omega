#include <omega/arm.h>
#include <omega/omega.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <cmath>

omega::Arm::Arm(ros::NodeHandle *node, Omega *owner) : _owner(owner)
{
    //Config
    OMEGA_CONFIG_VECTOR("arm/base_position", 3, base_position);
    OMEGA_CONFIG_VECTOR("arm/idle_position", 3, idle_position);
    OMEGA_CONFIG("arm/idle_pitch", idle_pitch);
    OMEGA_CONFIG_VECTOR("arm/l", 4, l);
    OMEGA_CONFIG_VECTOR("arm/q_init", 4, q_init);
    OMEGA_CONFIG_VECTOR("arm/q_min", 4, q_min);
    OMEGA_CONFIG_VECTOR("arm/q_max", 4, q_max);
    OMEGA_CONFIG_VECTOR("arm/w_max", 4, w_max);

    //Technical
    _pub = node->advertise<trajectory_msgs::JointTrajectory>("joints/arm_trajectory_controller/command", 1, true);

    //State
    boost::shared_ptr<const sensor_msgs::JointState> msg = ros::topic::waitForMessage<sensor_msgs::JointState>("joints/joint_states", ros::Duration(5.0));
    if (msg == nullptr) { const char *error = "Failed to receive robot state"; ROS_ERROR(error); throw std::runtime_error(error); }
    update(ros::Time::now(), msg);

    start_move_idle(ros::Time::now());

    ROS_INFO("omega::Arm initialized");
}

void omega::Arm::update(ros::Time time, const sensor_msgs::JointState::ConstPtr &msg)
{
    update(time);
    for (unsigned int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == "arm_joint_1") _position(0) = msg->position[i];
        else if (msg->name[i] == "arm_joint_2") _position(1) = msg->position[i];
        else if (msg->name[i] == "arm_joint_3") _position(2) = msg->position[i];
        else if (msg->name[i] == "arm_joint_4") _position(3) = msg->position[i];
    }
}

void omega::Arm::update(ros::Time time)
{
    if (time >= _finish_time) _move_finished = true;
}

bool omega::Arm::start_move_idle(ros::Time now)
{
    return start_move(now, idle_position, idle_pitch);
}

bool omega::Arm::start_move(ros::Time now, Eigen::Vector3d point, double pitch)
{
    _move_started = _move_finished = false;

    //Inverse kinematics
    point -= base_position;
    Eigen::Vector4d goal;
    goal(0) = std::atan2(point.y(), point.x());
    point = Eigen::AngleAxisd(-goal(0), Eigen::Vector3d::UnitZ()) * (point - Eigen::Vector3d(0, 0, l(0)));
    const Eigen::Vector2d point2(point.x() - l(3) * std::cos(pitch), point.z() + l(3) * std::sin(pitch));
    const double length2 = point2.norm();
    const double alpha = std::atan2(point2.y(), point2.x());
    const double cos_beta = (l(1) * l(1) + length2 * length2 - l(2) * l(2)) / (2 * l(1) * length2);
    if (cos_beta < -1.0 || cos_beta > 1.0) return false;
    const double beta = std::acos(cos_beta);
    goal(1) = alpha + beta; //positive solution
    Eigen::Vector2d point1(l(1) * std::cos(goal(1)), l(1) * std::sin(goal(1)));
    goal(2) = std::atan2((point2 - point1).y(), (point2 - point1).x()) - goal(1);
    goal(3) = pitch - goal(1) - goal(2);
    goal -= q_init;
    if ((goal.array() > q_max.array()).any() || (goal.array() < q_min.array()).any()) return false;

    //Logics
    const double duration = ((goal.array() - _position.array()).abs() / w_max.array()).maxCoeff();
    _move_started = true;
    _finish_time = now + ros::Duration(duration);
    _goal_point = point;
    _goal_pitch = pitch;
    
    //Publishing
    trajectory_msgs::JointTrajectory command;
    command.joint_names = std::vector<std::string>{"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4"};
    trajectory_msgs::JointTrajectoryPoint command_point;
    command_point.positions.resize(4);
    Eigen::Vector4d::Map(command_point.positions.data()) = goal;
    command_point.time_from_start = ros::Duration(duration);
    command.points.push_back(command_point);
    _pub.publish(command); ROS_INFO("Arm command sent");
    return true;
}

void omega::Arm::abort_move()
{
    _move_started = _move_finished = false;
    trajectory_msgs::JointTrajectory command;
    command.joint_names = std::vector<std::string>{"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4"};
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(4);
    Eigen::Vector4d::Map(point.positions.data()) = _position;
    point.time_from_start = ros::Duration(0.5);
    command.points.push_back(point);
    _pub.publish(command);
}

bool omega::Arm::get_move_started() const
{
    return _move_started;
}

bool omega::Arm::get_move_finished() const
{
    return _move_finished;
}

double omega::Arm::get_error() const
{
    const Eigen::Affine3d t1 = Eigen::AngleAxisd(_position(0), Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(0, 0, l(0)) * Eigen::AngleAxisd(_position(0) + q_init(0), Eigen::Vector3d::UnitX());
    const Eigen::Affine3d t2 = Eigen::Translation3d(l(1), 0, 0) * Eigen::AngleAxisd(_position(1) + q_init(1), Eigen::Vector3d::UnitX());
    const Eigen::Affine3d t3 = Eigen::Translation3d(l(2), 0, 0) * Eigen::AngleAxisd(_position(2) + q_init(2), Eigen::Vector3d::UnitX());
    const Eigen::Affine3d t4 = Eigen::Translation3d(l(3), 0, 0) * Eigen::AngleAxisd(_position(3) + q_init(3), Eigen::Vector3d::UnitX());
    const Eigen::Affine3d t = t1 * t2 * t3 * t4;
    Eigen::Vector3d point = t * Eigen::Vector3d::Zero();
    point += base_position;
    return (_goal_point - point).norm();
}