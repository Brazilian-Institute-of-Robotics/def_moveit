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

    // setup move_group interface for position and trajectory control
    static const std::string PLANNING_GROUP = "dyn_ef_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // create a ground plane for the motion planning
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "our_ground_plane";
    // define an box as ground plane
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2.0;
    primitive.dimensions[1] = 2.0;
    primitive.dimensions[2] = 0.01;
    // define the pose of the groundplane in the world
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.005;
    // create the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    // add the collision object to the world
    ROS_INFO("Add the ground plane into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

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
    target_pose1.position.z = 0.5;
    move_group.setPoseTarget(target_pose1);

    // planning and executing the arm's movement
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("going to initial position" << success ? "SUCCESS" : "FAILED");
    move_group.move();

    // buffer to listen to tf transforms
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    bool goal_reached = false;

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
                ROS_INFO_STREAM("error - move_group.plan() failed! Could not move to look for camera");
            }
        }
        else if (goal_reached == true)
        {
            ROS_INFO("Goal already reached!");
        }
        else
        {
            ROS_INFO("found transform to tag_0! trying to approach..");

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

            // set new target pose
            move_group.setPoseTarget(closeUpWorld);

            // planning and executing the arm's movement
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            if ((move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) == true && goal_reached == false)
            {
                move_group.move();
                ROS_INFO("success - moved to approaching point");

                moveit::planning_interface::MoveGroupInterface::Plan trajectory_plan;
                moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
                // moveit::core::RobotState start_state_core = *(current_state.get());
                moveit_msgs::RobotState start_state_msg;
                moveit::core::robotStateToRobotStateMsg(*(current_state.get()), start_state_msg, false);

                // set waypoint for the path
                std::vector<geometry_msgs::Pose> waypoints;

                // new pose
                closeUp.pose.position.z = 0.02;
                tf2::doTransform(closeUp, closeUpWorld, transformStamped);
                waypoints.push_back(closeUpWorld.pose);

                // set movement speed to 1/10 to carefully press the button
                move_group.setMaxVelocityScalingFactor(0.1);
                move_group.setPlanningTime(30.0);

                // computing the cartesian path
                moveit_msgs::RobotTrajectory trajectory;
                moveit_msgs::MoveItErrorCodes errCode;
                const double jump_threshold = 0.0;
                const double eef_step = 0.01;

                double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true, &errCode);
                ROS_INFO("Calculated %.2f%% of the Cartesian Path", fraction * 100.0);
                ROS_INFO_STREAM("Error Code of Cartesian Path: " << errCode);

                // filling trajectory with information and trying to execute
                trajectory_plan.trajectory_ = trajectory;
                trajectory_plan.start_state_ = start_state_msg;
                trajectory_plan.planning_time_ = 0.666;
                success = (move_group.execute(trajectory_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                ROS_INFO("Executing Cartesian path %s", success ? "Succeeded" : "FAILED");
                // set flag if cartesian path was executed successfully
                if (success)
                {
                    goal_reached = true;
                    ROS_INFO("Goal reached!");
                }

                sleep(3.0);

                // go back to old pose
                std::vector<geometry_msgs::Pose> waypoints_back;
                closeUp.pose.position.z = 0.2;
                tf2::doTransform(closeUp, closeUpWorld, transformStamped);
                waypoints_back.push_back(closeUpWorld.pose);

                // get current pose
                // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
                // moveit::core::RobotState start_state_core = *(current_state.get());
                moveit::core::robotStateToRobotStateMsg(*(move_group.getCurrentState().get()), start_state_msg, false);

                // compute cartesian path
                fraction = move_group.computeCartesianPath(waypoints_back, eef_step, jump_threshold, trajectory, true, &errCode);
                ROS_INFO("Calculated %.2f%% of the Cartesian Path back to old pose", fraction * 100.0);
                ROS_INFO_STREAM("Error Code of Cartesian Path: " << errCode);

                // filling trajectory with information and trying to execute
                trajectory_plan.trajectory_ = trajectory;
                trajectory_plan.start_state_ = start_state_msg;
                trajectory_plan.planning_time_ = 0.666;
                success = (move_group.execute(trajectory_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                ROS_INFO("Executing Cartesian path back to old pose %s", success ? "Succeeded" : "FAILED");
            }
            else
            {
                ROS_INFO("error - move_group.plan() failed!");
            }
        }

        sleep(1.0);
    }

    ros::shutdown();
    return 0;
}
