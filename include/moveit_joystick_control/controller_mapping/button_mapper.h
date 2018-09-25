#ifndef MOVEIT_JOYSTICK_CONTROL_BUTTON_MAPPER
#define MOVEIT_JOYSTICK_CONTROL_BUTTON_MAPPER

#include <moveit_joystick_control/controller_mapping/controller_mapper_base.h>

#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace moveit_joystick_control {

class ButtonMapper : public ControllerMapperBase {
public:
  ButtonMapper(size_t button_index);
  double computeCommand(const sensor_msgs::Joy& joy);

private:
  size_t button_index_;
};

}

#endif
