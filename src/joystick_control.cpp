#include <moveit_joystick_control/joystick_control.h>

#include <moveit_joystick_control/common.h>
#include <eigen_conversions/eigen_msg.h>

namespace moveit_joystick_control {

JoystickControl::JoystickControl(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), joint_state_received_(false), gripper_pos_(0.0), gripper_speed_(0.0)
{
  // Load parameters
  pnh.param("max_speed_linear", max_speed_linear_, 0.05);
  pnh.param("max_speed_angular", max_speed_angular_, 0.01);
  pnh.param("max_speed_gripper", max_speed_gripper_, 0.05);

  pnh.param<std::string>("gripper_joint_name", gripper_joint_name_, "gripper_servo_joint");

  std::string group_name;
  pnh.param<std::string>("group_name", group_name, "arm_group");
  ik_.init(group_name);
  ros::NodeHandle controller_config_nh(pnh, "controller_configuration");
  config_ = ControllerConfig::createFromServer(controller_config_nh);

  joint_names_ = ik_.getJointNames();
  goal_state_.resize(joint_names_.size());

  std::string robot_description;
  if(!nh.getParam("/robot_description", robot_description)) {
    ROS_ERROR_STREAM("Failed to load /robot_description.");
  }
  // Loading urdf
  if (!urdf_model_.initString(robot_description)) {
    ROS_ERROR_STREAM("Failed to parse urdf.");
  }
  // Load gripper limits
  urdf::JointConstSharedPtr gripper_joint = urdf_model_.getJoint(gripper_joint_name_);
  if (!gripper_joint) {
    ROS_ERROR_STREAM("Could not find gripper joint '" << gripper_joint_name_ << "'.");
  }
  gripper_upper_limit_ = gripper_joint->limits->upper;
  gripper_lower_limit_ = gripper_joint->limits->lower;

  // Subscribers and publishers
  goal_pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("goal_pose", 10);
  cmd_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/command", 10);
  gripper_cmd_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_command", 10);
  joint_state_sub_ = nh_.subscribe("/joint_states", 10, &JoystickControl::jointStateCb, this);
  joy_sub_ = nh_.subscribe("/joy", 10, &JoystickControl::joyCb, this);
}

void JoystickControl::starting()
{
  // Set initial endeffector pose
  ROS_INFO_STREAM("Waiting for joint states..");
  while (!joint_state_received_) {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  goal_pose_ = ik_.getEndEffectorPose(stateFromList(last_state_, joint_names_));
  ROS_INFO_STREAM("Joystick Control started.");

  std::vector<double> gripper_state = stateFromList(last_state_, std::vector<std::string>(1, gripper_joint_name_));
  if (gripper_state.empty()) {
    ROS_WARN_STREAM("Could not retrieve gripper position. Initializing with 0");
    gripper_pos_ = 0;
  } else {
    gripper_pos_ = gripper_state[0];
  }
  gripper_speed_ = 0.0;
}

void JoystickControl::update(const ros::Time& time, const ros::Duration& period)
{
  // Update endeffector pose with current command
  Eigen::Affine3d twist_transform(rpyToRot(period.toSec() * twist_.angular));
  twist_transform.translation() = period.toSec() * twist_.linear;
  goal_pose_ = goal_pose_ * twist_transform;

  // Publish new goal pose
  geometry_msgs::PoseStamped goal_pose_msg;
  goal_pose_msg.header.frame_id = ik_.getBaseFrame();
//  goal_pose_msg.header.stamp
  tf::poseEigenToMsg(goal_pose_, goal_pose_msg.pose);
  goal_pose_pub_.publish(goal_pose_msg);

  // Compute ik
  if (!ik_.calcInvKin(goal_pose_, stateFromList(last_state_, joint_names_), goal_state_)) {
    return;
  }

  // Send new goal to trajectory controllers
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = goal_state_;
  point.time_from_start = ros::Duration(0.100);
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = joint_names_;
  trajectory.points.push_back(point);

  cmd_pub_.publish(trajectory);

  // Update gripper position
  gripper_pos_ += period.toSec() * gripper_speed_;
  gripper_pos_ = std::min(gripper_upper_limit_, std::max(gripper_lower_limit_, gripper_pos_));
//  ROS_INFO_STREAM("[" << gripper_lower_limit_ << " < " << gripper_pos_ << " < " << gripper_upper_limit_ << "]");

  // Send new gripper command
  point.positions = std::vector<double>(1, gripper_pos_);
  trajectory.joint_names = std::vector<std::string>(1, gripper_joint_name_);
  trajectory.points[0] = point;

  gripper_cmd_pub_.publish(trajectory);
}

void JoystickControl::stopping()
{
  ROS_INFO_STREAM("Joystick Control stopped.");
}

void JoystickControl::joyCb(const sensor_msgs::JoyConstPtr& joy_ptr)
{
  // Update command
  twist_ = joyToTwist(*joy_ptr);
  gripper_speed_ = max_speed_gripper_ * (joy_ptr->buttons[config_.btn_close_gripper] - joy_ptr->buttons[config_.btn_open_gripper]);
//  ROS_INFO_STREAM("linear: [" << twist_.linear.x() << ", " << twist_.linear.y() << ", " << twist_.linear.z() << "]");
//  ROS_INFO_STREAM("angular: [" << twist_.angular.x() << ", " << twist_.angular.y() << ", " << twist_.angular.z() << "]");
}

void JoystickControl::jointStateCb(const sensor_msgs::JointStateConstPtr& joint_state_msg)
{
  if (!joint_state_received_) {
    joint_state_received_ = true;
    last_state_ = *joint_state_msg;
  } else {
    for (unsigned int joint_idx = 0; joint_idx < joint_state_msg->name.size(); joint_idx++) {
      const std::string& joint_name = joint_state_msg->name[joint_idx];
      std::vector<std::string>::iterator it = std::find(last_state_.name.begin(), last_state_.name.end(), joint_name);
      if (it != last_state_.name.end()) {
        // we know this joint already, update position
        unsigned int idx = it - last_state_.name.begin();
        last_state_.position[idx] = joint_state_msg->position[joint_idx];
        last_state_.velocity[idx] = joint_state_msg->velocity[joint_idx];
      } else {
        // new joint
        last_state_.name.push_back(joint_name);
        last_state_.position.push_back(joint_state_msg->position[joint_idx]);
        last_state_.velocity.push_back(joint_state_msg->velocity[joint_idx]);
      }
    }
  }
}

Twist JoystickControl::joyToTwist(const sensor_msgs::Joy& joy)
{
  Twist twist;
  twist.linear.x() = max_speed_linear_ * -joy.axes[config_.axis_linear_x];
  twist.linear.y() = max_speed_linear_ * joy.axes[config_.axis_linear_y];
  twist.linear.z() = (max_speed_linear_ * (joy.buttons[config_.axis_linear_z_inc] - joy.buttons[config_.axis_linear_z_dec]));

  twist.angular.x() = max_speed_angular_ * (joy.buttons[config_.btn_angular_roll_inc] - joy.buttons[config_.btn_angular_roll_dec]);
  twist.angular.y() = max_speed_angular_ * joy.axes[config_.axis_angular_pitch];
  twist.angular.z() = max_speed_angular_ * joy.axes[config_.axis_angular_yaw];

  return twist;
}

}
