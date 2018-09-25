#include <moveit_joystick_control/controller_mapping/button_mapper.h>

namespace moveit_joystick_control {

ButtonMapper::ButtonMapper(size_t button_index)
  : ControllerMapperBase(), button_index_(button_index)
{}

double ButtonMapper::computeCommand(const sensor_msgs::Joy& joy)
{
    return static_cast<double>(joy.buttons[button_index_]);
}

}
