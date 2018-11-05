#ifndef MOVEIT_JOYSTICK_CONTROL_DUAL_AXIS_MAPPER
#define MOVEIT_JOYSTICK_CONTROL_DUAL_AXIS_MAPPER

#include <moveit_joystick_control/controller_mapping/controller_mapper_base.h>

#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace moveit_joystick_control {

class DualAxisMapper : public ControllerMapperBase {
public:
  DualAxisMapper(size_t axis_inc_index, size_t axis_dec_index);
  double computeCommand(const sensor_msgs::Joy& joy);

private:
  size_t axis_inc_index_;
  size_t axis_dec_index_;
};

}

#endif
