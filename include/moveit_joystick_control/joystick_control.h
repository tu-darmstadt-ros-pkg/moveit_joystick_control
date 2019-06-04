#ifndef JOYSTICK_CONTROL_H
#define JOYSTICK_CONTROL_H

#include <Eigen/Eigen>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <urdf/model.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Joy.h>

#include <moveit_joystick_control/inverse_kinematics.h>
#include <moveit_joystick_control/controller_mapping/controller_mapper_factory.h>

namespace moveit_joystick_control {

struct Twist {
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

class JoystickControl : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
public:
  JoystickControl();

  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh) override;

  void starting(const ros::Time&) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time&) override;
private:
  bool loadGripperJointLimits();
  void updateArm(const ros::Time& time, const ros::Duration& period);
  /// Updates the goal pose
  /// Returns true if the goal pose has changed
  bool computeNewGoalPose(const ros::Duration& period);
  void updateGripper(const ros::Time& time, const ros::Duration& period);
  void loadJoystickConfig(const ros::NodeHandle& nh);
  void joyCb(const sensor_msgs::JoyConstPtr& joy_ptr);
  void jointStateCb(const sensor_msgs::JointStateConstPtr& joint_state_msg);
  Twist joyToTwist(const sensor_msgs::Joy& joy);
  void publishRobotState(const std::vector<double>& arm_joint_states, const collision_detection::CollisionResult::ContactMap& contact_map_);
  /// Transforms pose to desired frame
  /// Pose has to be relative to base frame
  geometry_msgs::PoseStamped getPoseInFrame(const Eigen::Affine3d& pose, std::string frame);

  ros::NodeHandle pnh_;

  bool initialized_;
  bool enabled_;

//  ControllerConfig config_;
  std::map<std::string, std::shared_ptr<ControllerMapperBase>> config_;
  bool reset_pose_;
  bool reset_tool_center_;

  bool move_tool_center_;


  double max_speed_linear_;
  double max_speed_angular_;
  double max_speed_gripper_;

  int free_angle_;

  Eigen::Affine3d tool_center_offset_; // Offset relative to end-effector
  Eigen::Affine3d tool_goal_pose_; // Goal pose of tool
  Eigen::Affine3d ee_goal_pose_; // Goal pose of the end-effector
  std::vector<double> goal_state_;
  std::vector<double> previous_goal_state_;

  collision_detection::CollisionResult::ContactMap contact_map_;

  Twist twist_; // Current command, transform relative to tool center
  double gripper_pos_;
  double gripper_speed_;

  std::vector<std::string> joint_names_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  bool joint_state_received_;
  sensor_msgs::JointState last_state_;
  std::vector<double> current_joint_angles_;

  std::string gripper_joint_name_;
  hardware_interface::JointHandle gripper_handle_;
  double gripper_upper_limit_;
  double gripper_lower_limit_;

  InverseKinematics ik_;


  bool hold_pose_;
  bool hold_pose_pressed_;
  geometry_msgs::PoseStamped hold_goal_pose_;

  ros::Subscriber enable_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher goal_pose_pub_;
  ros::Publisher robot_state_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}

#endif
