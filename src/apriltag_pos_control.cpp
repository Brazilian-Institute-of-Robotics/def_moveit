#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_state/conversions.h>

#include <apriltags2_ros/AprilTagDetectionArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_pos_control");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // send static reference of frame camera to frame camera_link
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "camera_link";
    static_transformStamped.child_frame_id = "camera";
    static_transformStamped.transform.translation.x = 0.0;
    static_transformStamped.transform.translation.y = 0.0;
    static_transformStamped.transform.translation.z = 0.0;
    tf::Quaternion qtrans;
    qtrans = tf::createQuaternionFromRPY(-M_PI_2, 0, -M_PI_2);
    static_transformStamped.transform.rotation.x = qtrans[0];
    static_transformStamped.transform.rotation.y = qtrans[1];
    static_transformStamped.transform.rotation.z = qtrans[2];
    static_transformStamped.transform.rotation.w = qtrans[3];
    static_broadcaster.sendTransform(static_transformStamped);
    ROS_INFO("Spinning until killed publishing to world");

    // setup move_group interface for position and trajectory control
    static const std::string PLANNING_GROUP = "dyn_ef_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // moving the camera to an initial position to see the apriltag
    tf::Quaternion q;
    geometry_msgs::Quaternion q_msg;
    // q = tf::createQuaternionFromRPY(-M_PI_2,M_PI_2,0);
    q = tf::createQuaternionFromRPY(-M_PI_2 - 0.2, 0, -M_PI_2);
    tf::quaternionTFToMsg(q, q_msg);

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = q_msg;
    // target_pose1.orientation.x = -1.0;
    // target_pose1.orientation.y = 1.0;
    // target_pose1.orientation.z = 1.0;
    // target_pose1.orientation.w = M_PI/2.0;
    target_pose1.position.x = 0.0;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 1.2;
    move_group.setPoseTarget(target_pose1);

    // planning and executing the arm's movement
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("going to initial position" << success ? "SUCCESS" : "FAILED");
    move_group.move();

    // buffer to listen to tf transforms
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while (ros::ok())
    {
        bool trExists = false;
        geometry_msgs::TransformStamped transformStamped;

        if (tfBuffer._frameExists("tag_0"))
        {
            trExists = true;
            // get transformation of detected apriltag
            try
            {
                transformStamped = tfBuffer.lookupTransform("world", "tag_0", ros::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                trExists = false;
                continue;
            }
        }

        // if no recent position data of tag0 is received, move the camera around
        if (!trExists || ros::Time::now() - transformStamped.header.stamp > ros::Duration(10.0))
        {
            ROS_INFO_STREAM("found no known transform for tag_0, moving robot to find tag_0!");

            // look around and find apriltag
            geometry_msgs::PoseStamped pCurr = move_group.getCurrentPose();
            tf::Quaternion q_rot, q_orig, q_new;
            geometry_msgs::Quaternion q_msg_new;
            q_rot = tf::createQuaternionFromRPY(0, 0, M_PI / 6.0);
            quaternionMsgToTF(pCurr.pose.orientation, q_orig);
            q_new = q_rot * q_orig;
            q_new.normalize();
            quaternionTFToMsg(q_new, q_msg_new);
            pCurr.pose.orientation = q_msg_new;
            pCurr.header.stamp = ros::Time::now();

            // set new target pose
            move_group.setPoseTarget(pCurr);

            // planning and executing the arm's movement
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                move_group.move();
                ROS_INFO_STREAM("success - turned camera");
            }
            else
            {
                ROS_INFO_STREAM("error - move_group.plan() failed!");
            }
        }
        else
        {
            ROS_INFO_STREAM("found transform to tag_0! trying to approach..");

            // declare pose in apriltag-coordinate-system
            geometry_msgs::PoseStamped closeUp, closeUpWorld;
            closeUp.pose.position.x = 0.0;
            closeUp.pose.position.y = 0.0;
            closeUp.pose.position.z = 0.4;
            geometry_msgs::Quaternion q_msg;
            tf::Quaternion qt = tf::createQuaternionFromRPY(M_PI, 0, 0);
            quaternionTFToMsg(qt, q_msg);
            closeUp.pose.orientation = q_msg;
            closeUp.header.stamp = ros::Time::now();
            closeUp.header.frame_id = "tag_0";

            // transform to world coordinates
            tf2::doTransform(closeUp, closeUpWorld, transformStamped);

            // check if we are already close enough to the target
            // if( == move_group.getCurrentPose()) {

            // }
            
            // set new target pose
            move_group.setPoseTarget(closeUpWorld);

            // planning and executing the arm's movement
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                move_group.move();
                ROS_INFO_STREAM("success - moved to approaching point");
            }
            else
            {
                ROS_INFO_STREAM("error - move_group.plan() failed!");
            }
        }

        // ROS_INFO_STREAM("TF is: \n" << transformStamped);
        sleep(1.0);

        // You can plan a Cartesian path directly by specifying a list of waypoints
        // for the end-effector to go through. Note that we are starting
        // from the new start state above.  The initial pose (start state) does not
        // need to be added to the waypoint list but adding it can help with visualizations
        // move_group.setStartStateToCurrentState();

        // // ****************    getting current RobotState  *************************************************8
        // moveit::planning_interface::MoveGroupInterface::Plan trajectory_plan;
        // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        // // moveit::core::RobotState start_state_core = *(current_state.get());
        // moveit_msgs::RobotState start_state_msg;
        // moveit::core::robotStateToRobotStateMsg(*(current_state.get()), start_state_msg, false);

        // // set waypoint for the path
        // std::vector<geometry_msgs::Pose> waypoints;
        // geometry_msgs::Pose target_pose_cartesian = move_group.getCurrentPose().pose;

        // target_pose_cartesian.position.y = 0.4;
        // waypoints.push_back(target_pose_cartesian);

        // target_pose_cartesian.position.x = 0.2;
        // waypoints.push_back(target_pose_cartesian);

        // target_pose_cartesian.position.y = 0.2;
        // waypoints.push_back(target_pose_cartesian);

        // target_pose_cartesian.position.x = 0.4;
        // waypoints.push_back(target_pose_cartesian);

        /* old
    target_pose1.position.y = 0.3;
    waypoints.push_back(target_pose1);

    target_pose1.position.x = 0.2;
    waypoints.push_back(target_pose1);  

    target_pose1.position.y = 0.2;
    waypoints.push_back(target_pose1); 

    target_pose1.position.x = 0.3;
    waypoints.push_back(target_pose1);
    */

        // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
        // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
        // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
        // move_group.setMaxVelocityScalingFactor(0.1);
        // move_group.setPlanningTime(30.0);

        // // We want the Cartesian path to be interpolated at a resolution of 1 cm
        // // which is why we will specify 0.01 as the max step in Cartesian
        // // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
        // // Warning - disabling the jump threshold while operating real hardware can cause
        // // large unpredictable motions of redundant joints and could be a safety issue
        // moveit_msgs::RobotTrajectory trajectory;
        // moveit_msgs::MoveItErrorCodes errCode;
        // const double jump_threshold = 0.05;
        // const double eef_step = 0.01;

        // double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true, &errCode);
        // ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
        // ROS_INFO_STREAM("Error Code of Cartesian Path: " << errCode );

        // trajectory_plan.trajectory_ = trajectory;
        // trajectory_plan.start_state_ = start_state_msg;
        // trajectory_plan.planning_time_ = 0.666;
        // success = (move_group.execute(trajectory_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // ROS_INFO_NAMED("tutorial", "Executing plan 5 (Cartesian path) %s", success ? "Succeeded" : "FAILED");

        // sleep(3.0);
    }

    ros::shutdown();
    return 0;
}
