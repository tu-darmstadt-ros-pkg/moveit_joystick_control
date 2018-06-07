#include <ros/ros.h>
#include <moveit_joystick_control/joystick_control.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "moveit_joystick_control");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  double control_rate;
  pnh.param("control_rate", control_rate, 25.0);
  ros::Rate rate(control_rate);

  moveit_joystick_control::JoystickControl control(nh, pnh);
  control.starting();
  ros::Time time = ros::Time::now();
  while (ros::ok()) {
    ros::spinOnce();
    ros::Time current = ros::Time::now();
    ros::Duration period = time - current;
    control.update(current, period);
    time = current;
    rate.sleep();
  }
  control.stopping();
  return 0;
}
