#include <moveit_joystick_control/controller_mapping/button_mapper.h>

namespace moveit_joystick_control {

ButtonMapper::ButtonMapper(size_t button_index)
  : ControllerMapperBase(), button_index_(button_index)
{}

double ButtonMapper::computeCommand(const sensor_msgs::Joy& joy) const
{
  if (button_index_ < joy.buttons.size()) {
    return static_cast<double>(joy.buttons[button_index_]);
  } else {
    ROS_ERROR_STREAM("Button index " << button_index_ << " is out of bounds (Size: " << joy.buttons.size() << ").");
    return 0.0;
  }
}

}
