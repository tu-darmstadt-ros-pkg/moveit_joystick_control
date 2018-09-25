#ifndef MOVEIT_JOYSTICK_CONTROL_CONTROLLER_MAPPER_BASE
#define MOVEIT_JOYSTICK_CONTROL_CONTROLLER_MAPPER_BASE

#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace moveit_joystick_control {

class ControllerMapperBase {
public:
  virtual ~ControllerMapperBase();
  virtual double computeCommand(const sensor_msgs::Joy& joy) = 0;

  double scale() const;
  void setScale(double scale);

private:
  double scale_;
};

}

#endif
