#ifndef JOYSTICK_CONTROL_H
#define JOYSTICK_CONTROL_H

#include <sensor_msgs/Joy.h>
#include <Eigen/Eigen>

#include <moveit_joystick_control/controller_configuration.h>
#include <moveit_joystick_control/inverse_kinematics.h>
#include <urdf/model.h>

namespace moveit_joystick_control {

struct Twist {
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

class JoystickControl {
public:
  JoystickControl(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  void starting();
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping();
private:
  void joyCb(const sensor_msgs::JoyConstPtr& joy_ptr);
  void jointStateCb(const sensor_msgs::JointStateConstPtr& joint_state_msg);
  Twist joyToTwist(const sensor_msgs::Joy& joy);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ControllerConfig config_;
  double max_speed_linear_;
  double max_speed_angular_;
  double max_speed_gripper_;

  Eigen::Affine3d goal_pose_;
  std::vector<double> goal_state_;

  Twist twist_; // Current command
  double gripper_pos_;
  double gripper_speed_;

  std::string gripper_joint_name_;
  std::vector<std::string> joint_names_;
  bool joint_state_received_;
  sensor_msgs::JointState last_state_;

  InverseKinematics ik_;
  urdf::Model urdf_model_;
  double gripper_upper_limit_;
  double gripper_lower_limit_;

  ros::Subscriber joy_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher gripper_cmd_pub_;
  ros::Publisher goal_pose_pub_;
};

}

#endif
