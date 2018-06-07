#include <moveit_joystick_control/common.h>

#include <kdl/frames.hpp>
#include <ros/ros.h>

namespace moveit_joystick_control {

Eigen::Quaterniond rpyToRot(const Eigen::Vector3d& rpy) {
  KDL::Rotation kdl_rot = KDL::Rotation::RPY(rpy(0), rpy(1), rpy(2));
  Eigen::Quaterniond quat;
  kdl_rot.GetQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
  return quat;
}

Eigen::Vector3d rotToRpy(const Eigen::Matrix3d &rot) {
  Eigen::Quaterniond rot_quad(rot);
  KDL::Rotation kdl_rot = KDL::Rotation::Quaternion(rot_quad.x(), rot_quad.y(), rot_quad.z(), rot_quad.w());
  double roll, pitch, yaw;
  kdl_rot.GetRPY(roll, pitch, yaw);
  return Eigen::Vector3d(roll, pitch, yaw);
}

std::vector<double> stateFromList(const sensor_msgs::JointState& last_state, const std::vector<std::string>& joint_names) {
  std::vector<double> state(joint_names.size(), 0);
  for (unsigned int name_idx = 0; name_idx < joint_names.size(); name_idx++) {
    std::vector<std::string>::const_iterator it = std::find(last_state.name.begin(), last_state.name.end(), joint_names[name_idx]);
    if (it != last_state.name.end()) {
      unsigned int state_idx = it - last_state.name.begin();
      state[name_idx] = last_state.position[state_idx];
    } else {
      ROS_WARN_STREAM("Joint '" << joint_names[name_idx] << "' wasn't found in received joint state.");
    }
  }
  return state;
}

}
