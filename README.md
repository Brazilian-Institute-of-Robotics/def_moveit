# def_moveit
MoveIt! package for dynamic_end_effector

Package is created using the MoveIt! Setup Assistant. 

### Prerequisites

This package runs on ROS kinetic, Ubuntu 16.04

### How to use?

You need a loaded robot description for dynamic_end_effector and a running FollowJointTrajectory controller which matches
the controller description in ```conf/controllers.yaml```. MoveIt! will look for the controller described there.
You need to run either ```roslaunch dynamic_end_effector dyn_ef_robot_bringup_gazebo.launch``` to use simulation, or ```roslaunch def_actuation dyn_ef_robot_bringup_hardware.launch```
to use the real robot.

Then execute ```moveit_planning_execution.launch``` or ```dyn_ef_robot_bringup_rviz.launch``` before apriltag_detection.launch.
The Rviz part gives you more information about MoveIt! plugin and its planned path. You can also manually specify a position and
move the arm to it.

### Thank you!

You are always welcome ;-)
