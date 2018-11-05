#include <moveit_joystick_control/controller_mapping/axis_mapper.h>

namespace moveit_joystick_control {

AxisMapper::AxisMapper(size_t axis_index)
  : ControllerMapperBase(), axis_index_(axis_index)
{}

double AxisMapper::computeCommand(const sensor_msgs::Joy& joy)
{
  if (axis_index_ < joy.axes.size()) {
    return scale() * static_cast<double>(joy.axes[axis_index_]);
  } else {
    ROS_ERROR_STREAM("Axis index " << axis_index_ << " is out of bounds (Size: " << joy.axes.size() << ").");
    return 0.0;
  }
}

}
