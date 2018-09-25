#include <moveit_joystick_control/controller_mapping/dual_axis_mapper.h>

namespace moveit_joystick_control {

DualAxisMapper::DualAxisMapper(size_t axis_inc_index, size_t axis_dec_index)
  : ControllerMapperBase(), axis_inc_index_(axis_inc_index), axis_dec_index_(axis_dec_index)
{}

double DualAxisMapper::computeCommand(const sensor_msgs::Joy& joy)
{
    return scale() * static_cast<double>(joy.axes[axis_inc_index_] - joy.axes[axis_dec_index_]);
}

}
