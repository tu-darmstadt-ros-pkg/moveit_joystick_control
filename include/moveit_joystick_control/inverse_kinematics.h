#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include<ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <sensor_msgs/JointState.h>

namespace moveit_joystick_control {

class InverseKinematics {
public:
  bool init(std::string group_name);
  bool calcInvKin(const Eigen::Affine3d &pose, const std::vector<double>& seed, std::vector<double> &solution);
  Eigen::Affine3d getEndEffectorPose(const std::vector<double>& joint_positions);

  std::vector<std::string> getJointNames();
  std::string getBaseFrame();
private:
  std::string moveitErrCodeToString(int32_t code);

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;

  std::string tip_frame_;
  std::vector<std::string> joint_names_;
};

}
#endif
