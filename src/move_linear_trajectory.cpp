#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_state/conversions.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "linear_movement_tut");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "dyn_ef_arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());


  sleep(1.0);


  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 1.0;
  target_pose1.orientation.w = 0.5;
  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0.2;
  target_pose1.position.z = 1.2;
  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
  move_group.move();


  sleep(1.0);

  
  target_pose1.position.y = 0.3;
  move_group.setPoseTarget(target_pose1);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
  move_group.move();
  sleep(1.0);

  
  target_pose1.position.x = 0.2;
  move_group.setPoseTarget(target_pose1);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "SUCCESS" : "FAILED");
  move_group.move();
  sleep(1.0);

  target_pose1.position.y = 0.2;
  move_group.setPoseTarget(target_pose1);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success ? "SUCCESS" : "FAILED");
  move_group.move();
  sleep(1.0);
  

  target_pose1.position.x = 0.25;
  move_group.setPoseTarget(target_pose1);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (pose goal) %s", success ? "SUCCESS" : "FAILED");
  move_group.move();
  sleep(1.0);
  


  while(ros::ok())
  {
    // You can plan a Cartesian path directly by specifying a list of waypoints
    // for the end-effector to go through. Note that we are starting
    // from the new start state above.  The initial pose (start state) does not
    // need to be added to the waypoint list but adding it can help with visualizations
    move_group.setStartStateToCurrentState();

    // ****************    getting current RobotState  *************************************************8
    moveit::planning_interface::MoveGroupInterface::Plan trajectory_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    // moveit::core::RobotState start_state_core = *(current_state.get());
    moveit_msgs::RobotState start_state_msg;
    moveit::core::robotStateToRobotStateMsg(*(current_state.get()), start_state_msg, false);

    // set waypoint for the path
    std::vector<geometry_msgs::Pose> waypoints;
    // geometry_msgs::Pose target_pose_cartesian = move_group.getCurrentPose().pose;

    target_pose1.position.y = 0.3;
    waypoints.push_back(target_pose1);

    target_pose1.position.x = 0.2;
    waypoints.push_back(target_pose1);  

    target_pose1.position.y = 0.2;
    waypoints.push_back(target_pose1); 

    target_pose1.position.x = 0.3;
    waypoints.push_back(target_pose1);

    // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
    // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
    // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setPlanningTime(20.0);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::MoveItErrorCodes errCode;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true, &errCode);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    ROS_INFO_STREAM("Error Code of Cartesian Path: " << errCode );

    sleep(5.0);

    trajectory_plan.trajectory_ = trajectory;
    trajectory_plan.start_state_ = start_state_msg;
    trajectory_plan.planning_time_ = 0.666;
    success = (move_group.execute(trajectory_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Executing plan 5 (Cartesian path) %s", success ? "Succeeded" : "FAILED");

    sleep(3.0);
  }


  ros::shutdown();
  return 0;
}



