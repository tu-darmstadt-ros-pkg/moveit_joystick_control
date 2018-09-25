#include <moveit_joystick_control/controller_mapping/axis_mapper.h>

namespace moveit_joystick_control {

AxisMapper::AxisMapper(size_t axis_index)
  : ControllerMapperBase(), axis_index_(axis_index)
{}

double AxisMapper::computeCommand(const sensor_msgs::Joy& joy)
{
    return scale() * static_cast<double>(joy.axes[axis_index_]);
}

}
