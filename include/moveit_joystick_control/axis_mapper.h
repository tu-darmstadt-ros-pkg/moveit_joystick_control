#ifndef MOVEIT_JOYSTICK_CONTROL_AXIS_MAPPER
#define MOVEIT_JOYSTICK_CONTROL_AXIS_MAPPER

#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace moveit_joystick_control {

class AxisMapper {
public:
  AxisMapper();
  AxisMapper(int axis_index);
  AxisMapper(int button_inc_index, int button_dec_index);
  double computeCommand(const sensor_msgs::Joy& joy);

  static AxisMapper createFromParameterServer(const ros::NodeHandle& nh);
  double scale() const;
  void setScale(double scale);

private:
  bool emulate_axis_with_buttons_;
  double scale_;

  int axis_index_;
  int button_inc_index_;
  int button_dec_index_;
};

}

#endif
