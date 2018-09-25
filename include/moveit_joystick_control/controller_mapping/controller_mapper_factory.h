#ifndef MOVEIT_JOYSTICK_CONTROL_CONTROLLER_MAPPER_FACTORY
#define MOVEIT_JOYSTICK_CONTROL_CONTROLLER_MAPPER_FACTORY

#include <ros/ros.h>

#include <moveit_joystick_control/controller_mapping/axis_mapper.h>
#include <moveit_joystick_control/controller_mapping/button_axis_mapper.h>
#include <moveit_joystick_control/controller_mapping/button_mapper.h>
#include <moveit_joystick_control/controller_mapping/dual_axis_mapper.h>

namespace moveit_joystick_control {

std::shared_ptr<ControllerMapperBase> switchController(const ros::NodeHandle& nh) {
  int axis_index;
  if (nh.getParam("axis_index", axis_index)) {
    return std::make_shared<AxisMapper>(axis_index);
  }

  int button_index;
  if (nh.getParam("button_index", button_index)) {
    return std::make_shared<ButtonMapper>(button_index);
  }

  int button_inc_index, button_dec_index;
  if (nh.getParam("button_inc_index", button_inc_index) && nh.getParam("button_dec_index", button_dec_index)) {
    return std::make_shared<ButtonAxisMapper>(button_inc_index, button_dec_index);
  }

  int axis_inc_index, axis_dec_index;
  if (nh.getParam("axis_inc_index", axis_inc_index) && nh.getParam("axis_dec_index", axis_dec_index)) {
    return std::make_shared<DualAxisMapper>(axis_inc_index, axis_dec_index);
  }

  ROS_ERROR_STREAM("None of the required parameters found to create a controller mapper.");
  return std::shared_ptr<ControllerMapperBase>();
}

std::shared_ptr<ControllerMapperBase> createControllerMapper(const ros::NodeHandle& nh) {
  std::shared_ptr<ControllerMapperBase> controller_mapper = switchController(nh);

  if (!controller_mapper) {
    return controller_mapper;
  }

  double scale;
  nh.param<double>("scale", scale, 1.0);
  controller_mapper->setScale(scale);

  return controller_mapper;
}



}

#endif
