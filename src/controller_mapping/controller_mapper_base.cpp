#include <moveit_joystick_control/controller_mapping/controller_mapper_base.h>

namespace moveit_joystick_control {

ControllerMapperBase::~ControllerMapperBase() {}

double ControllerMapperBase::scale() const {
  return scale_;
}

void ControllerMapperBase::setScale(double scale) {
  scale_ = scale;
}

}
