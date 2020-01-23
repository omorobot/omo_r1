################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(omo_r1_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  #diagnostic_msgs
  #omo_r1_msgs
  message_generation
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

add_message_files(
   FILES
   R1MotorStatus.msg
   R1MotorStatusLR.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)


################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(omo_r1_diagnostics src/omo_r1_diagnostics.cpp)
#add_dependencies(omo_r1_diagnostics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(omo_r1_diagnostics ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
#install(TARGETS omo_r1_diagnostics
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(PROGRAMS 
#  scripts/create_udev_rules
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(FILES 99-omo_r1-cdc.rules
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

################################################################################
# Test
################################################################################
