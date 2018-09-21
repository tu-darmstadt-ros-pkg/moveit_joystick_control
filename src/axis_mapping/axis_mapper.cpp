#include <moveit_joystick_control/axis_mapper.h>

namespace moveit_joystick_control {

AxisMapper::AxisMapper()
  : AxisMapper(0)
{}

AxisMapper::AxisMapper(int axis_index)
  : axis_index_(axis_index), emulate_axis_with_buttons_(false), scale_(1.0)
{

}

AxisMapper::AxisMapper(int button_inc_index, int button_dec_index)
  : button_inc_index_(button_inc_index), button_dec_index_(button_dec_index), emulate_axis_with_buttons_(true), scale_(1.0)
{

}

double AxisMapper::computeCommand(const sensor_msgs::Joy& joy)
{
  if (!emulate_axis_with_buttons_) {
    return scale_ * joy.axes[axis_index_];
  } else {
    return scale_ * (joy.buttons[button_inc_index_] - joy.buttons[button_dec_index_]);
  }
}

AxisMapper AxisMapper::createFromParameterServer(const ros::NodeHandle& nh)
{
  double scale;
  nh.param("scale", scale, 1.0);

  AxisMapper mapper;

  int axis_index;
  if (!nh.getParam("axis_index", axis_index)) {
    int button_inc_index, button_dec_index;
    if (!nh.getParam("button_inc_index", button_inc_index) || !nh.getParam("button_dec_index", button_dec_index)) {
      ROS_ERROR_STREAM("Defining a controller mapping requires either the parameter 'axis_index' or 'button_inc_index' and 'button_dec_index'.");
    } else {
      mapper = AxisMapper(button_inc_index, button_dec_index);
    }
  } else {
    mapper = AxisMapper(axis_index);
  }

  mapper.setScale(scale);
  return mapper;
}

double AxisMapper::scale() const
{
  return scale_;
}

void AxisMapper::setScale(double scale)
{
  scale_ = scale;
}

}
