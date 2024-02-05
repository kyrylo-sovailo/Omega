#include <omega/arm.h>
#include <omega/omega.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <cmath>

void omega::Arm::_read_pose(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (unsigned int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == "arm_joint_1") _pose(0) = msg->position[i];
        else if (msg->name[i] == "arm_joint_2") _pose(1) = msg->position[i];
        else if (msg->name[i] == "arm_joint_3") _pose(2) = msg->position[i];
        else if (msg->name[i] == "arm_joint_4") _pose(3) = msg->position[i];
    }
}

void omega::Arm::_write_trajectory(const std::vector<Eigen::Vector4d> &poses, const std::vector<double> &durations)
{
    trajectory_msgs::JointTrajectory command;
    command.joint_names = std::vector<std::string>{"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4"};
    command.points.reserve(poses.size());
    for (unsigned int i = 0; i < poses.size(); i++)
    {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(4);
        Eigen::Vector4d::Map(point.positions.data()) = poses[i];
        point.time_from_start = ros::Duration(durations[i]);
        command.points.push_back(point);
    }
    _pub.publish(command);
}

omega::Arm::Arm(ros::NodeHandle *node, Omega *owner) : _owner(owner)
{
    //Config
    OMEGA_CONFIG_VECTOR("arm/base_position", 3, base_position);
    OMEGA_CONFIG_VECTOR("arm/l", 4, l);
    OMEGA_CONFIG_VECTOR("arm/q_init", 4, q_init);

    OMEGA_CONFIG_VECTOR("arm/q_min", 4, q_min);
    OMEGA_CONFIG_VECTOR("arm/q_max", 4, q_max);
    OMEGA_CONFIG_VECTOR("arm/w_max", 4, w_max);
    
    OMEGA_CONFIG_VECTOR("arm/to_idle_position", 3, to_idle_pose.position);
    OMEGA_CONFIG("arm/to_idle_pitch", to_idle_pose.pitch);
    OMEGA_CONFIG_VECTOR("arm/idle_position", 3, idle_pose.position);
    OMEGA_CONFIG("arm/idle_pitch", idle_pose.pitch);
    OMEGA_CONFIG_VECTOR("arm/to_pick_position", 3, to_pick_pose.position);
    OMEGA_CONFIG("arm/to_pick_pitch", to_pick_pose.pitch);
    OMEGA_CONFIG_VECTOR("arm/to_throw_position", 3, to_throw_pose.position);
    OMEGA_CONFIG("arm/to_throw_pitch", to_throw_pose.pitch);
    OMEGA_CONFIG_VECTOR("arm/throw_position", 3, throw_pose.position);
    OMEGA_CONFIG("arm/throw_pitch", throw_pose.pitch);

    //Technical
    _pub = node->advertise<trajectory_msgs::JointTrajectory>("joints/arm_trajectory_controller/command", 1, true);

    //State
    ROS_INFO("omega::Arm waiting for joints/joint_states");
    boost::shared_ptr<const sensor_msgs::JointState> msg = ros::topic::waitForMessage<sensor_msgs::JointState>("joints/joint_states", *node, ros::Duration(5.0));
    if (msg == nullptr) { const char *error = "Failed to receive robot state"; ROS_ERROR("%s", error); throw std::runtime_error(error); }
    _read_pose(msg);
    ROS_INFO("omega::Arm received joints/joint_states");

    ROS_INFO("omega::Arm initialized");
}

void omega::Arm::update(ros::Time now, const sensor_msgs::JointState::ConstPtr &msg)
{
    update(now);
    _read_pose(msg);
}

void omega::Arm::update(ros::Time now)
{
    if (now >= _finish_time) _move_finished = true;
}

bool omega::Arm::start_move(ros::Time now, const std::vector<Pose> &path)
{
    _move_started = _move_finished = false;

    std::vector<Eigen::Vector4d> poses;
    std::vector<double> durations;
    Eigen::Vector4d last_pose = _pose;
    double last_duration = 0;

    //Inverse kinematics
    for (auto i = path.cbegin(); i != path.cend(); i++)
    {
        Eigen::Vector3d position = i->position - base_position;
        Eigen::Vector4d goal_pose;
        goal_pose(0) = std::atan2(position.y(), position.x());
        position = Eigen::AngleAxisd(-goal_pose(0), Eigen::Vector3d::UnitZ()) * (position - Eigen::Vector3d(0, 0, l(0)));
        const Eigen::Vector2d position2(position.x() - l(3) * std::cos(i->pitch), position.z() + l(3) * std::sin(i->pitch));
        const double length2 = position2.norm();
        const double alpha = std::atan2(position2.y(), position2.x());
        const double cos_beta = (l(1) * l(1) + length2 * length2 - l(2) * l(2)) / (2 * l(1) * length2);
        if (cos_beta < -1.0 || cos_beta > 1.0) return false;
        const double beta = std::acos(cos_beta);
        goal_pose(1) = alpha + beta; //positive solution
        Eigen::Vector2d position1(l(1) * std::cos(goal_pose(1)), l(1) * std::sin(goal_pose(1)));
        goal_pose(2) = std::atan2((position2 - position1).y(), (position2 - position1).x()) - goal_pose(1);
        goal_pose(3) = i->pitch - goal_pose(1) - goal_pose(2);
        goal_pose -= q_init;
        if ((goal_pose.array() > q_max.array()).any() || (goal_pose.array() < q_min.array()).any()) return false;
        
        const double duration = ((goal_pose.array() - last_pose.array()).abs() / w_max.array()).maxCoeff();
        last_pose = goal_pose;
        last_duration += duration;
        poses.push_back(goal_pose);
        durations.push_back(last_duration);
    }

    //Logics
    _move_started = true;
    _finish_time = now + ros::Duration(durations.back());
    _goal_pose = path.back();
    
    //Publishing
    _write_trajectory(poses, durations);
    return true;
}

bool omega::Arm::start_move_unknown_to_idle(ros::Time now)
{
    std::vector<Pose> poses { to_idle_pose, idle_pose };
    start_move(now, poses);
}

bool omega::Arm::start_move_idle_to_pick(ros::Time now, const Pose &pick_pose)
{
    std::vector<Pose> poses { to_pick_pose, pick_pose };
    start_move(now, poses);
}

bool omega::Arm::start_move_pick_to_idle(ros::Time now)
{
    std::vector<Pose> poses { to_pick_pose, idle_pose };
    start_move(now, poses);
}

bool omega::Arm::start_move_idle_to_throw(ros::Time now)
{
    std::vector<Pose> poses { to_throw_pose, throw_pose };
    start_move(now, poses);
}

bool omega::Arm::start_move_throw_to_idle(ros::Time now)
{
    std::vector<Pose> poses { to_throw_pose, idle_pose };
    start_move(now, poses);
}

void omega::Arm::abort_move()
{
    std::vector<Eigen::Vector4d> poses { _pose };
    std::vector<double> durations { 0 };
    _write_trajectory(poses, durations);
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
    const Eigen::Affine3d t1 = Eigen::AngleAxisd(_pose(0) + q_init(0), Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(0, 0, l(0)) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX());
    const Eigen::Affine3d t2 = Eigen::AngleAxisd(_pose(1) + q_init(1), Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(l(1), 0, 0);
    const Eigen::Affine3d t3 = Eigen::AngleAxisd(_pose(2) + q_init(2), Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(l(2), 0, 0);
    const Eigen::Affine3d t4 = Eigen::AngleAxisd(_pose(3) + q_init(3), Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(l(3), 0, 0);
    const Eigen::Affine3d t = t1 * t2 * t3 * t4;
    Eigen::Vector3d position = t * Eigen::Vector3d::Zero();
    position += base_position;
    return (_goal_pose.position - position).norm();
}