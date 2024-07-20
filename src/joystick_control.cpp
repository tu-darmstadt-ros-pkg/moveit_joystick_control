#include <moveit_joystick_control/joystick_control.h>

#include <moveit_joystick_control/common.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Bool.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>

namespace moveit_joystick_control {

JoystickControl::JoystickControl()
  : initialized_(false),
    enabled_(false),
    free_angle_(-1),
    tool_center_offset_(Eigen::Affine3d::Identity()),
    gripper_pos_(0.0),
    gripper_speed_(0.0),
    joint_state_received_(false),
    hold_pose_(false),
    tf_listener_(tf_buffer_)
{}

bool JoystickControl::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  pnh_ = pnh;
  // Load parameters
  std::string free_angle_str;
  if (pnh_.getParam("free_angle", free_angle_str) && free_angle_str != "") {
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
  }

  // **** Arm group ****
  std::string group_name;
  pnh_.param<std::string>("group_name", group_name, "arm_group");
  ik_.init(group_name);

  // Get joint handles
  joint_names_ = ik_.getJointNames();
  for (const std::string& joint: joint_names_) {
    try {
      joint_handles_.push_back(hw->getHandle(joint));
    } catch (hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(e.what());
      return false;
    }
  }
  goal_state_.resize(joint_names_.size());
  current_joint_angles_.resize(joint_names_.size());
  previous_goal_state_.resize(joint_names_.size());

  // **** Gripper ****
  pnh_.param<std::string>("gripper_joint_name", gripper_joint_name_, "gripper_servo_joint");
  // Get handle
  try {
    gripper_handle_ = hw->getHandle(gripper_joint_name_);
  } catch (hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(e.what());
    return false;
  }
  // Get limits
  if (!loadGripperJointLimits()) {
    return false;
  }

  // Subscribers and publishers
  goal_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 10);
  robot_state_pub_ = nh.advertise<moveit_msgs::DisplayRobotState>("robot_state", 10);
  enabled_pub_ = pnh.advertise<std_msgs::Bool>("enabled", 10, true);
  publishStatus();

  joint_state_sub_ = nh.subscribe("/joint_states", 10, &JoystickControl::jointStateCb, this);

  std::string twist_topic = pnh_.param("twist_cmd_topic", std::string("twist_cmd"));
  twist_cmd_sub_ = pnh.subscribe(twist_topic, 10, &JoystickControl::twistCmdCb, this);

  std::string gripper_topic = pnh_.param("gripper_cmd_topic", std::string("gripper_cmd"));
  gripper_cmd_sub_ = pnh.subscribe(gripper_topic, 10, &JoystickControl::gripperCmdCb, this);

  reset_pose_server_ = pnh_.advertiseService("reset_pose", &JoystickControl::resetPoseCb, this);
  reset_tool_center_server_ = pnh_.advertiseService("reset_tool_center", &JoystickControl::resetToolCenterCb, this);
  hold_pose_server_ = pnh_.advertiseService("hold_pose", &JoystickControl::holdPoseCb, this);
  move_tool_center_ = pnh_.advertiseService("move_tool_center", &JoystickControl::moveToolCenterCb, this);
  return true;
}

void JoystickControl::starting(const ros::Time&)
{
  if (enabled_) return;
  // Set initial endeffector pose
  twist_.linear = Eigen::Vector3d::Zero();
  twist_.angular = Eigen::Vector3d::Zero();
  reset_pose_ = true;
  reset_tool_center_ = false;
  move_tool_center_ = false;
  hold_pose_ = false;
  gripper_pos_ = gripper_handle_.getPosition();
  gripper_speed_ = 0.0;
  initialized_ = true;
  enabled_ = true;
  publishStatus();

  ROS_INFO_STREAM("Joystick Control started.");
}

void JoystickControl::update(const ros::Time& time, const ros::Duration& period)
{
  if (!initialized_) {
    return;
  }
  // Get current state
  for (unsigned int i = 0; i < joint_names_.size(); i++) {
    current_joint_angles_[i] = joint_handles_[i].getPosition();
  }

  // Compute next state
  updateArm(time, period);
  updateGripper(time, period);

  // Visualization:
  // TODO: Not real-time safe
  // Publish desired robot state
  publishRobotState(goal_state_, contact_map_);

  // Publish desired goal pose
  geometry_msgs::PoseStamped goal_pose_msg;
  goal_pose_msg.header.frame_id = ik_.getBaseFrame();
  tf::poseEigenToMsg(tool_goal_pose_, goal_pose_msg.pose);
  goal_pose_pub_.publish(goal_pose_msg);
}

void JoystickControl::stopping(const ros::Time&)
{
  if (!enabled_) return;
  ROS_INFO_STREAM("Joystick Control stopped.");
  enabled_ = false;
  publishStatus();
  hideRobotState();
}

bool JoystickControl::loadGripperJointLimits()
{
  std::string robot_description;
  if(!pnh_.getParam("/robot_description", robot_description)) {
    ROS_ERROR_STREAM("Failed to load /robot_description.");
    return false;
  }
  // Loading urdf
  urdf::Model urdf_model;
  if (!urdf_model.initString(robot_description)) {
    ROS_ERROR_STREAM("Failed to parse urdf.");
    return false;
  }
  // Load gripper limits
  urdf::JointConstSharedPtr gripper_joint = urdf_model.getJoint(gripper_joint_name_);
  if (!gripper_joint) {
    ROS_ERROR_STREAM("Could not find gripper joint '" << gripper_joint_name_ << "'.");
    return false;
  }
  gripper_upper_limit_ = gripper_joint->limits->upper;
  gripper_lower_limit_ = gripper_joint->limits->lower;
  return true;
}

void JoystickControl::updateArm(const ros::Time& /*time*/, const ros::Duration& period)
{
  Eigen::Affine3d old_goal = ee_goal_pose_;
  previous_goal_state_ = goal_state_;

  // Compute new goal pose
  computeNewGoalPose(period);

  // Compute ik
  if (ik_.calcInvKin(ee_goal_pose_, previous_goal_state_, goal_state_)) { // Init solution with previous goal state
    // Check if solution is collision free
    contact_map_.clear();
    bool collision_free = ik_.isCollisionFree(last_state_, goal_state_, contact_map_);
    if (!collision_free) {
      ee_goal_pose_ = old_goal;
      goal_state_ = previous_goal_state_;
    }
  } else {
    // IK failed
    ee_goal_pose_ = old_goal;
    goal_state_ = previous_goal_state_;
  }

  // Write next goal state
  for (unsigned int i = 0; i < joint_names_.size(); i++) {
    joint_handles_[i].setCommand(goal_state_[i]);
  }
}

bool JoystickControl::computeNewGoalPose(const ros::Duration& period)
{
  if (reset_pose_) {
    // Reset end-effector goal
    previous_goal_state_ = current_joint_angles_;
    ee_goal_pose_ = ik_.getEndEffectorPose(current_joint_angles_);
    tool_goal_pose_ = ee_goal_pose_ * tool_center_offset_;
    reset_pose_ = false;
    return true;
  }

  if (reset_tool_center_) {
    // Reset tool center to end-effector goal
    tool_center_offset_ = Eigen::Affine3d::Identity();
    tool_goal_pose_ = ee_goal_pose_;
    reset_tool_center_ = false;
    return false;
  }

  if (hold_pose_) {
    geometry_msgs::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(ik_.getBaseFrame(), hold_goal_pose_.header.frame_id, ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      return false;
    }
    Eigen::Affine3d base_to_frame;
    tf::transformMsgToEigen(transform_stamped.transform, base_to_frame);
    Eigen::Affine3d frame_to_ee;
    tf::poseMsgToEigen(hold_goal_pose_.pose, frame_to_ee);
    ee_goal_pose_ = base_to_frame * frame_to_ee;
    tool_goal_pose_ = ee_goal_pose_ * tool_center_offset_;
    return true;
  }

  // Update endeffector pose with current command
  if (twist_.linear == Eigen::Vector3d::Zero() && twist_.angular == Eigen::Vector3d::Zero()) {
    return false;
  }
  Eigen::Affine3d twist_transform(rpyToRot(period.toSec() * twist_.angular));
  twist_transform.translation() = period.toSec() * twist_.linear;

  Eigen::Affine3d tool_center_movement = tool_center_offset_ * twist_transform;
  tool_goal_pose_ = ee_goal_pose_ * tool_center_movement;
  if (!move_tool_center_) {
    // Move end-effector
    ee_goal_pose_ = tool_goal_pose_ * tool_center_offset_.inverse(Eigen::Isometry);
    return true;
  } else {
    // Move tool frame
    tool_center_offset_ = tool_center_movement;
    return false;
  }

  // Update free angles
//  if (free_angle_ != -1) {
//    Eigen::Affine3d current_pose = ik_.getEndEffectorPose(current_joint_angles_);
//    Eigen::Affine3d goal_to_current = ee_goal_pose_.inverse(Eigen::Isometry) * current_pose;
//    Eigen::Vector3d goal_to_current_rpy = rotToRpy(goal_to_current.linear());
////    ROS_INFO_STREAM("current_rpy: [" << current_rpy[0] << ", " << current_rpy[1] << ", " << current_rpy[2] << "]");

//    ee_goal_pose_ = ee_goal_pose_ * Eigen::AngleAxisd(goal_to_current_rpy[free_angle_], Eigen::Vector3d::UnitX());
//  }
}

void JoystickControl::updateGripper(const ros::Time& /*time*/, const ros::Duration& period)
{
  gripper_pos_ += period.toSec() * gripper_speed_;
  gripper_pos_ = std::min(gripper_upper_limit_, std::max(gripper_lower_limit_, gripper_pos_));
  gripper_handle_.setCommand(gripper_pos_);
}

void JoystickControl::twistCmdCb(const geometry_msgs::TwistConstPtr& twist_msg)
{
  twist_.linear.x() = twist_msg->linear.x;
  twist_.linear.y() = twist_msg->linear.y;
  twist_.linear.z() = twist_msg->linear.z;
  twist_.angular.x() = twist_msg->angular.x;
  twist_.angular.y() = twist_msg->angular.y;
  twist_.angular.z() = twist_msg->angular.z;
}

void JoystickControl::gripperCmdCb(const std_msgs::Float64ConstPtr& float_ptr)
{
  gripper_speed_ = float_ptr->data;
}

void JoystickControl::jointStateCb(const sensor_msgs::JointStateConstPtr& joint_state_msg)
{
  if (!joint_state_received_) {
    joint_state_received_ = true;
    last_state_ = *joint_state_msg;
  } else {
    for (unsigned int joint_idx = 0; joint_idx < joint_state_msg->name.size(); joint_idx++) {
      const std::string& joint_name = joint_state_msg->name[joint_idx];
      auto it = std::find(last_state_.name.begin(), last_state_.name.end(), joint_name);
      if (it != last_state_.name.end()) {
        // we know this joint already, update position
        size_t idx = static_cast<size_t>(it - last_state_.name.begin()); // save, because begin() is always <= it
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

bool JoystickControl::resetPoseCb(std_srvs::EmptyRequest& /*request*/, std_srvs::EmptyResponse& /*response*/)
{
  reset_pose_ = true;
  return true;
}

bool JoystickControl::resetToolCenterCb(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
{
  reset_tool_center_ = true;
  return true;
}

bool JoystickControl::holdPoseCb(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response)
{
  if (hold_pose_ != request.data) {
    hold_pose_ = request.data;
    ROS_INFO_STREAM("Hold pose " << (hold_pose_ ? "enabled" : "disabled"));
    if (hold_pose_) {
      hold_goal_pose_ = getPoseInFrame(ee_goal_pose_, "odom");
    }
  }
  response.success = true;
  return true;
}

bool JoystickControl::moveToolCenterCb(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response)
{
  move_tool_center_ = request.data;
  response.success = true;
  return true;
}

void JoystickControl::publishRobotState(const std::vector<double>& arm_joint_states, const collision_detection::CollisionResult::ContactMap& contact_map)
{
  // Get robot state with current positions + IK solutionn
  robot_state::RobotState robot_state = ik_.getAsRobotState(last_state_, arm_joint_states);

  // Add world pose of robot
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform("world", ik_.getBaseFrame(), ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return;
  }
  Eigen::Affine3d pose;
  tf::transformMsgToEigen(transform_stamped.transform, pose);
  updateRobotStatePose(robot_state, pose);

  // Convert to msg
  moveit_msgs::DisplayRobotState display_robot_state;
  moveit::core::robotStateToRobotStateMsg(robot_state, display_robot_state.state);

  // Highlight links in collision
  std_msgs::ColorRGBA color;
  color.a = color.r = 1.0;
  color.b = color.g = 0.0;
  for (const auto& it : contact_map) {
    const std::string& link1 = it.first.first;
    const std::string& link2 = it.first.second;
    moveit_msgs::ObjectColor object_color;
    object_color.id = link1;
    object_color.color = color;
    display_robot_state.highlight_links.push_back(object_color);
    object_color.id = link2;
    display_robot_state.highlight_links.push_back(object_color);
  }

  robot_state_pub_.publish(display_robot_state);
}

geometry_msgs::PoseStamped JoystickControl::getPoseInFrame(const Eigen::Affine3d& pose, std::string frame)
{
  // Get transform from base to desired frame
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(frame, ik_.getBaseFrame(), ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }
  Eigen::Affine3d frame_to_base;
  tf::transformMsgToEigen(transform_stamped.transform, frame_to_base);

  // Transform given pose to desired frame
  Eigen::Affine3d frame_to_pose = frame_to_base * pose;

  // Build message
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = frame;
  pose_stamped.header.stamp = transform_stamped.header.stamp;
  tf::poseEigenToMsg(frame_to_pose, pose_stamped.pose);
  return pose_stamped;
}
void JoystickControl::publishStatus() const {
  std_msgs::Bool bool_msg;
  bool_msg.data = enabled_;
  enabled_pub_.publish(bool_msg);
}
void JoystickControl::hideRobotState() {
  // Convert to msg
  moveit_msgs::DisplayRobotState display_robot_state;
  display_robot_state.hide = true;
  robot_state_pub_.publish(display_robot_state);
}

}

PLUGINLIB_EXPORT_CLASS(moveit_joystick_control::JoystickControl, controller_interface::ControllerBase);
