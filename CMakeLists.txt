cmake_minimum_required(VERSION 2.8.3)
project(moveit_joystick_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  sensor_msgs
  kdl_parser
  urdf
  eigen_conversions
  moveit_msgs
  tf2_ros
  controller_interface
  hardware_interface
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moveit_joystick_control
  CATKIN_DEPENDS moveit_core moveit_ros_planning sensor_msgs kdl_parser urdf eigen_conversions moveit_msgs tf2_ros controller_interface hardware_interface
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/joystick_control.h
  include/${PROJECT_NAME}/controller_mapping/controller_mapper_factory.h
  include/${PROJECT_NAME}/controller_mapping/controller_mapper_base.h
  include/${PROJECT_NAME}/controller_mapping/axis_mapper.h
  include/${PROJECT_NAME}/controller_mapping/button_axis_mapper.h
  include/${PROJECT_NAME}/controller_mapping/dual_axis_mapper.h
  include/${PROJECT_NAME}/controller_mapping/button_mapper.h
  include/${PROJECT_NAME}/inverse_kinematics.h
  include/${PROJECT_NAME}/common.h

  src/joystick_control.cpp
  src/inverse_kinematics.cpp
  src/common.cpp

  src/controller_mapping/controller_mapper_base.cpp
  src/controller_mapping/axis_mapper.cpp
  src/controller_mapping/button_axis_mapper.cpp
  src/controller_mapping/dual_axis_mapper.cpp
  src/controller_mapping/button_mapper.cpp

)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
#add_executable(joystick_control_node src/joystick_control_node.cpp)

## Add cmake target dependencies of the executable
#add_dependencies(joystick_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(joystick_control_node
#  ${PROJECT_NAME}
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES ros_control_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

