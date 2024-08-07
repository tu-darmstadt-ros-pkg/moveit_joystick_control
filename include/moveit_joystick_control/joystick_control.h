#ifndef JOYSTICK_CONTROL_H
#define JOYSTICK_CONTROL_H

#include <Eigen/Eigen>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <urdf/model.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include <moveit_joystick_control/inverse_kinematics.h>

namespace moveit_joystick_control {

struct Twist {
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

class JoystickControl : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
public:
  JoystickControl();

  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh, ros::NodeHandle& pnh) override;

  void starting(const ros::Time&) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time&) override;
private:
  void publishStatus() const;
  bool loadGripperJointLimits();
  void updateArm(const ros::Time& time, const ros::Duration& period);
  /// Updates the goal pose
  /// Returns true if the goal pose has changed
  bool computeNewGoalPose(const ros::Duration& period);
  void updateGripper(const ros::Time& time, const ros::Duration& period);

  void twistCmdCb(const geometry_msgs::TwistConstPtr& twist_msg);
  void gripperCmdCb(const std_msgs::Float64ConstPtr& float_ptr);
  void jointStateCb(const sensor_msgs::JointStateConstPtr& joint_state_msg);
  bool resetPoseCb(std_srvs::EmptyRequest& /*request*/, std_srvs::EmptyResponse& /*response*/);
  bool resetToolCenterCb(std_srvs::EmptyRequest& /*request*/, std_srvs::EmptyResponse& /*response*/);
  bool holdPoseCb(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response);
  bool moveToolCenterCb(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response);

  void publishRobotState(const std::vector<double>& arm_joint_states, const collision_detection::CollisionResult::ContactMap& contact_map_);
  void hideRobotState();
  /// Transforms pose to desired frame
  /// Pose has to be relative to base frame
  geometry_msgs::PoseStamped getPoseInFrame(const Eigen::Affine3d& pose, std::string frame);

  ros::NodeHandle pnh_;

  bool initialized_;
  bool enabled_;

  bool reset_pose_;
  bool reset_tool_center_;

  bool move_tool_center_;

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
  geometry_msgs::PoseStamped hold_goal_pose_;

  ros::Subscriber twist_cmd_sub_;
  ros::Subscriber gripper_cmd_sub_;
  ros::Subscriber move_tool_twist_sub_;
  ros::Subscriber enable_sub_;
  ros::Subscriber joint_state_sub_;
  ros::ServiceServer reset_pose_server_;
  ros::ServiceServer reset_tool_center_server_;
  ros::ServiceServer hold_pose_server_;
  ros::ServiceServer move_tool_center_server_;

  ros::Publisher goal_pose_pub_;
  ros::Publisher robot_state_pub_;
  ros::Publisher enabled_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}

#endif
