#ifndef MOVEIT_JOYSTICK_CONTROL_AXIS_MAPPER
#define MOVEIT_JOYSTICK_CONTROL_AXIS_MAPPER

#include <moveit_joystick_control/controller_mapping/controller_mapper_base.h>

#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace moveit_joystick_control {

class AxisMapper : public ControllerMapperBase {
public:
  AxisMapper(size_t axis_index);
  virtual double computeCommand(const sensor_msgs::Joy& joy) override;
  virtual void initFromParameterServer(const ros::NodeHandle& nh) override;

private:
  size_t axis_index_;
};

}

#endif
