#include <moveit_joystick_control/joystick_control.h>

#include <moveit_joystick_control/common.h>
#include <eigen_conversions/eigen_msg.h>

namespace moveit_joystick_control {

JoystickControl::JoystickControl(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), joint_state_received_(false), gripper_pos_(0.0), gripper_speed_(0.0), free_angle_(-1), enabled_(false)
{
  // Load parameters
  pnh.param("max_speed_linear", max_speed_linear_, 0.05);
  pnh.param("max_speed_angular", max_speed_angular_, 0.01);
  pnh.param("max_speed_gripper", max_speed_gripper_, 0.05);
  pnh.param("reset_button_idx", reset_button_idx_, 0);

  std::string free_angle_str;
  pnh.param<std::string>("free_angle", free_angle_str, "");
  std::transform(free_angle_str.begin(), free_angle_str.end(), free_angle_str.begin(), ::tolower);
  if (free_angle_str == "x") {
    free_angle_ = 0;
  } else if (free_angle_str == "y") {
    free_angle_ = 1;
  } else if (free_angle_str == "z") {
    free_angle_ = 2;
  } else {
    free_angle_ = -1;
    ROS_WARN_STREAM("Invalid free angle '" << free_angle_str << "'.");
  }

  pnh.param<std::string>("gripper_joint_name", gripper_joint_name_, "gripper_servo_joint");

  std::string group_name;
  pnh.param<std::string>("group_name", group_name, "arm_group");
  ik_.init(group_name);
  loadControllerConfig(pnh);

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
  if (enabled_) return;
  // Set initial endeffector pose
  ROS_INFO_STREAM("Waiting for joint states..");
  while (!joint_state_received_) {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  twist_.linear = Eigen::Vector3d::Zero();
  twist_.angular = Eigen::Vector3d::Zero();
  goal_pose_ = ik_.getEndEffectorPose(stateFromList(last_state_, joint_names_));
  reset_pose_ = false;
  ROS_INFO_STREAM("Joystick Control started.");

  std::vector<double> gripper_state = stateFromList(last_state_, std::vector<std::string>(1, gripper_joint_name_));
  if (gripper_state.empty()) {
    ROS_WARN_STREAM("Could not retrieve gripper position. Initializing with 0");
    gripper_pos_ = 0;
  } else {
    gripper_pos_ = gripper_state[0];
  }
  gripper_speed_ = 0.0;
  enabled_ = true;
}

void JoystickControl::update(const ros::Time& time, const ros::Duration& period)
{
  if (!enabled_) return;
  updateArm(time, period);
  updateGripper(time, period);
}

void JoystickControl::stopping()
{
  if (!enabled_) return;
  ROS_INFO_STREAM("Joystick Control stopped.");
  enabled_ = false;
}

void JoystickControl::updateArm(const ros::Time& time, const ros::Duration& period)
{
  if (twist_.linear == Eigen::Vector3d::Zero() && twist_.angular == Eigen::Vector3d::Zero()) {
    return;
  }

  Eigen::Affine3d old_goal_ = goal_pose_;

  if (!reset_pose_) {
    // Update endeffector pose with current command
    Eigen::Affine3d twist_transform(rpyToRot(period.toSec() * twist_.angular));
    twist_transform.translation() = period.toSec() * twist_.linear;
    goal_pose_ = goal_pose_ * twist_transform;

    // Update free angles
    if (free_angle_ != -1) {
      Eigen::Affine3d current_pose = ik_.getEndEffectorPose(stateFromList(last_state_, joint_names_));
      Eigen::Affine3d goal_to_current = goal_pose_.inverse() * current_pose;
      Eigen::Vector3d goal_to_current_rpy = rotToRpy(goal_to_current.linear());
  //    ROS_INFO_STREAM("current_rpy: [" << current_rpy[0] << ", " << current_rpy[1] << ", " << current_rpy[2] << "]");

      goal_pose_ = goal_pose_ * Eigen::AngleAxisd(goal_to_current_rpy[free_angle_], Eigen::Vector3d::UnitX());
    }
  } else {
    goal_pose_ = ik_.getEndEffectorPose(stateFromList(last_state_, joint_names_));
    reset_pose_ = false;
  }


  // Visualization: Publish new goal pose
  geometry_msgs::PoseStamped goal_pose_msg;
  goal_pose_msg.header.frame_id = ik_.getBaseFrame();
  tf::poseEigenToMsg(goal_pose_, goal_pose_msg.pose);
  goal_pose_pub_.publish(goal_pose_msg);

  // Compute ik
  if (ik_.calcInvKin(goal_pose_, stateFromList(last_state_, joint_names_), goal_state_)) {
    // Check if solution is collision free
    bool collision_free = ik_.isCollisionFree(last_state_, goal_state_);
    if (!collision_free) {
//      ROS_WARN_STREAM("Solution is in collision.");
      goal_pose_ = old_goal_;
    } else {
      // Send new goal to trajectory controllers
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = goal_state_;
      point.time_from_start = ros::Duration(0.100);
      trajectory_msgs::JointTrajectory trajectory;
      trajectory.joint_names = joint_names_;
      trajectory.points.push_back(point);

      cmd_pub_.publish(trajectory);
    }
  } else {
    goal_pose_ = old_goal_;
  }
}

void JoystickControl::updateGripper(const ros::Time& time, const ros::Duration& period)
{
  if (gripper_speed_ == 0.0) {
    return;
  }
  gripper_pos_ += period.toSec() * gripper_speed_;
  gripper_pos_ = std::min(gripper_upper_limit_, std::max(gripper_lower_limit_, gripper_pos_));
//  ROS_INFO_STREAM("[" << gripper_lower_limit_ << " < " << gripper_pos_ << " < " << gripper_upper_limit_ << "]");

  // Send new gripper command
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = std::vector<double>(1, gripper_pos_);  point.time_from_start = ros::Duration(0.100);
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = std::vector<std::string>(1, gripper_joint_name_);
  trajectory.points.push_back(point);

  gripper_cmd_pub_.publish(trajectory);
}

void JoystickControl::loadControllerConfig(const ros::NodeHandle& nh)
{
  ROS_INFO_STREAM("Loading controller config from namespace " << nh.getNamespace() + "/controller_configuration");
  XmlRpc::XmlRpcValue mapping;
  nh.getParam("controller_configuration", mapping);
  ROS_ASSERT(mapping.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = mapping.begin(); it != mapping.end(); ++it)
  {
    std::string action_name = (std::string)(it->first);
    ros::NodeHandle action_nh(nh, "controller_configuration/" + action_name);
    AxisMapper mapping = AxisMapper::createFromParameterServer(action_nh);
    config_.emplace(action_name, mapping);
    ROS_INFO_STREAM("Added action: " << action_name);
  }
  if (config_.empty()) {
    ROS_WARN_STREAM("No controller configuration defined");
  }
}

void JoystickControl::joyCb(const sensor_msgs::JoyConstPtr& joy_ptr)
{
  // Update command
  if (reset_button_idx_ >= 0 && reset_button_idx_ < joy_ptr->buttons.size() && joy_ptr->buttons[reset_button_idx_]) {
    reset_pose_ = true;
  }
  twist_ = joyToTwist(*joy_ptr);
  gripper_speed_ = max_speed_gripper_ * config_["gripper"].computeCommand(*joy_ptr);
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
  twist.linear.x() = max_speed_linear_ * config_["translate_x"].computeCommand(joy);
  twist.linear.y() = max_speed_linear_ * config_["translate_y"].computeCommand(joy);
  twist.linear.z() = max_speed_linear_ * config_["translate_z"].computeCommand(joy);

  twist.angular.x() = max_speed_angular_ * config_["rotate_roll"].computeCommand(joy);
  twist.angular.y() = max_speed_angular_ * config_["rotate_pitch"].computeCommand(joy);
  twist.angular.z() = max_speed_angular_ * config_["rotate_yaw"].computeCommand(joy);

  return twist;
}

}
