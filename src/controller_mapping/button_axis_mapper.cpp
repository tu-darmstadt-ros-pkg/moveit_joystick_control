#include <moveit_joystick_control/controller_mapping/button_axis_mapper.h>

namespace moveit_joystick_control {

ButtonAxisMapper::ButtonAxisMapper(size_t button_inc_index, size_t button_dec_index)
  : button_inc_index_(button_inc_index), button_dec_index_(button_dec_index)
{}

double ButtonAxisMapper::computeCommand(const sensor_msgs::Joy& joy)
{
  return scale() * static_cast<double>(joy.buttons[button_inc_index_] - joy.buttons[button_dec_index_]);
}

}
