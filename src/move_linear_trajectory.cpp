#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

//#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
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
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.w = 1.0;
  // target_pose1.position.x = 0.28;
  // target_pose1.position.y = -0.2;
  // target_pose1.position.z = 0.5;
  // move_group.setPoseTarget(target_pose1);
  move_group.setStartStateToCurrentState();

  geometry_msgs::Pose start_pose1;
  start_pose1.orientation.w = 1.0;
  start_pose1.orientation.x = 0.3;
  start_pose1.orientation.y = 0.2;
  start_pose1.orientation.z = 1.0;

  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.2;
  start_pose2.position.y = 0.4;
  start_pose2.position.z = 1.3;


  move_group.setPoseTarget(start_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan fooPlan;
  bool success1 = (move_group.plan(fooPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_STREAM("planned foo: " << success1);
  success1 = (move_group.execute(fooPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_STREAM("executed foo: " << success1);
  sleep(5.0);

  move_group.setPoseTarget(start_pose2);
  success1 = (move_group.plan(fooPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_STREAM("planned foo: " << success1);
  success1 = (move_group.execute(fooPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_STREAM("executed foo: " << success1);
  sleep(10.0);
  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;

  target_pose3.position.z += 0.1;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.1;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z -= 0.1;
  target_pose3.position.y += 0.1;
  // target_pose3.position.x -= 0.1;
  waypoints.push_back(target_pose3);  // up left
  move_group.setMaxVelocityScalingFactor(0.1);

  ROS_INFO_STREAM("Set up Waypoints");

  moveit::planning_interface::MoveGroupInterface::Plan trajectory_plan;

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  trajectory_plan.trajectory_ = trajectory;
  bool success = (move_group.execute(trajectory_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  ros::shutdown();
  return 0;
}
