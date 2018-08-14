cmake_minimum_required(VERSION 2.8.3)
project(dyn_ef_moveit)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    apriltags2_ros
    tf2_ros
    tf2
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamic_end_effector
#  CATKIN_DEPENDS other_catkin_pkg
  CATKIN_DEPENDS
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
  apriltags2_ros
  tf2_ros
  tf2
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(move_linear_trajectory src/move_linear_trajectory.cpp)
add_executable(apriltag_pos_control src/apriltag_pos_control.cpp)
add_executable(static_tf2_broadcaster src/static_tf2_broadcaster.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(move_linear_trajectory
  ${catkin_LIBRARIES}
)
target_link_libraries(apriltag_pos_control
  ${catkin_LIBRARIES}
)
target_link_libraries(static_tf2_broadcaster
  ${catkin_LIBRARIES}
)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
