#ifndef CONTROLLER_CONFIGURATION_H
#define CONTROLLER_CONFIGURATION_H

#include <ros/ros.h>

namespace moveit_joystick_control {

struct ControllerConfig {
  int axis_linear_y; // Id of axis to control linear movement in y direction
  int axis_linear_z; // Id of axis to control linear movement in z direction
  int axis_linear_x_inc; // Id of axis to control linear movement in x direction
  int axis_linear_x_dec; // Id of axis to control linear movement in x direction

  int axis_angular_roll; // Id of axis to control roll
  int axis_angular_pitch; // Id of axis to control pitch

  int btn_close_gripper; // Id of button to close gripper
  int btn_open_gripper; // Id of button to open gripper

  int btn_angular_yaw_inc; // Id of button to increase third component of angular movement
  int btn_angular_yaw_dec; // Id of button to decrease third component of angular movement

  static ControllerConfig createFromServer(const ros::NodeHandle& nh) {
    ControllerConfig config;
    nh.param<int>("axis_linear_x_inc", config.axis_linear_x_inc, 4);
    nh.param<int>("axis_linear_x_dec", config.axis_linear_x_dec, 5);
    nh.param<int>("axis_linear_y", config.axis_linear_y, 0);
    nh.param<int>("axis_linear_z", config.axis_linear_z, 1);

    nh.param<int>("axis_angular_roll", config.axis_angular_roll, 2);
    nh.param<int>("axis_angular_pitch", config.axis_angular_pitch, 3);
    nh.param<int>("btn_angular_yaw_inc", config.btn_angular_yaw_inc, 0);
    nh.param<int>("btn_angular_yaw_dec", config.btn_angular_yaw_dec, 1);

    nh.param<int>("btn_close_gripper", config.btn_close_gripper, 2);
    nh.param<int>("btn_open_gripper", config.btn_open_gripper, 3);

    return config;
  }
};

}

#endif
