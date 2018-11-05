#ifndef MOVEIT_JOYSTICK_CONTROL_BUTTON_AXIS_MAPPER
#define MOVEIT_JOYSTICK_CONTROL_BUTTON_AXIS_MAPPER

#include <moveit_joystick_control/controller_mapping/controller_mapper_base.h>

#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace moveit_joystick_control {

class ButtonAxisMapper : public ControllerMapperBase {
public:
  ButtonAxisMapper(size_t button_inc_index, size_t button_dec_index);
  double computeCommand(const sensor_msgs::Joy& joy) const override;

private:
  size_t button_inc_index_;
  size_t button_dec_index_;
};

}

#endif
