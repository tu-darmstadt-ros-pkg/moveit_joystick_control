#include <ros/ros.h>
#include <moveit_joystick_control/joystick_control.h>
#include <std_msgs/Bool.h>

bool enabled_;
bool start_controller_;
bool stop_controller_;

void setEnabled(bool enable) {
  if (enable != enabled_) {
    enabled_ = enable;
    if (enabled_) {
      start_controller_ = true;
    } else {
      stop_controller_ = true;
    }
  }
}

void enabledCb(const std_msgs::BoolConstPtr& bool_ptr) {
  setEnabled(bool_ptr->data);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "moveit_joystick_control");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  bool enable;
  nh.param("enable", enable, false);
  start_controller_ = false;
  stop_controller_ = false;
  setEnabled(enable);

  ros::Subscriber enabled_sub = pnh.subscribe<std_msgs::Bool>("enable", 10, enabledCb);

  double control_rate;
  pnh.param("control_rate", control_rate, 25.0);
  ros::Rate rate(control_rate);

  moveit_joystick_control::JoystickControl control(nh, pnh);
  control.init();
  ros::Time time = ros::Time::now();
  while (ros::ok()) {
    ros::spinOnce();

    if (start_controller_) {
      control.starting();
      time = ros::Time::now();
      start_controller_ = false;
    }

    ros::Time current = ros::Time::now();
    ros::Duration period = time - current;
    control.update(current, period);
    time = current;

    if (stop_controller_) {
      control.stopping();
      stop_controller_ = false;
    }
    if (!rate.sleep()) {
      ROS_WARN_STREAM("Desired control rate of " << control_rate << " hz was not met in this cycle. Current: " << 1.0/rate.cycleTime().toSec() << " hz");
    }
  }
  return 0;
}
