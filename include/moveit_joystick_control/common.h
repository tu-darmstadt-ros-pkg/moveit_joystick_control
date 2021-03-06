#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Eigen>
#include <sensor_msgs/JointState.h>

#include <moveit/robot_state/robot_state.h>

namespace moveit_joystick_control {

Eigen::Quaterniond rpyToRot(const Eigen::Vector3d& rpy);
Eigen::Vector3d rotToRpy(const Eigen::Matrix3d &rot);
std::vector<double> stateFromList(const sensor_msgs::JointState& last_state, const std::vector<std::string>& joint_names);

void updateRobotStatePose(robot_state::RobotState& state, const Eigen::Affine3d& pose);

}

#endif
