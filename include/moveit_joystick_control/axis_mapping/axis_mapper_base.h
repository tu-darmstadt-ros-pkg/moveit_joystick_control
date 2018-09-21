#ifndef MOVEIT_JOYSTICK_CONTROL_AXIS_MAPPER_BASE
#define MOVEIT_JOYSTICK_CONTROL_AXIS_MAPPER_BASE

#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace moveit_joystick_control {

class AxisMapper {
public:
  virtual ~AxisMapper() {}
  virtual double computeCommand(const sensor_msgs::Joy& joy) = 0;

  virtual void initFromParameterServer(const ros::NodeHandle& nh) = 0;
  double scale() const {
    return scale_;
  }
  void setScale(double scale) {
    scale_ = scale;
  }

private:
  double scale_;
};

}

#endif
