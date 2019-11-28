#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

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

    // tool to step through the code in rviz
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the code");


    // set velocity scaling to 1
    move_group.setMaxVelocityScalingFactor(0.6);

    // moving the camera to an initial position to see the apriltag
    tf::Quaternion q;
    geometry_msgs::Quaternion q_msg;
    // q = tf::createQuaternionFromRPY(-M_PI_2,M_PI_2,0);
    q = tf::createQuaternionFromRPY(-M_PI_2 - 0.78, 0, -M_PI);
    tf::quaternionTFToMsg(q, q_msg);

    // first of all got to home position, in case the arm is lying on the bumper
    if (move_group.setNamedTarget("home_position")) {
        move_group.setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            // wait for user
            visual_tools.prompt("Press 'next' to go to home position..");
            visual_tools.trigger();
            move_group.move();
        }
    }


    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = q_msg;
    // target_pose1.orientation.x = -1.0;
    // target_pose1.orientation.y = 1.0;
    // target_pose1.orientation.z = 1.0;
    // target_pose1.orientation.w = M_PI/2.0;
    target_pose1.position.x = 0.0;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.5;
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose1);
    move_group.setPlanningTime(40.0);

    // planning and executing the arm's movement
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("going to initial position " << success ? "SUCCESS" : "FAILED");
    // wait for user
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to start looking around for the apriltag");
    // move when user confirmed movement
    move_group.move();
    ROS_INFO_STREAM("finished moving to initial pose!");

    // buffer to listen to tf transforms
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // let tfBuffer listen to transform
    sleep(1.0);

    // pose for going back after pressing button (because the apriltag is not visible while pressing)
    geometry_msgs::PoseStamped prePressPose;

    bool goal_reached = false;
    bool try_again = false;

    geometry_msgs::TransformStamped transformStamped;
    // set header.stamp to current time to wait at least 10 seconds to find the apriltag
    transformStamped.header.stamp = ros::Time::now();

    while (ros::ok())
    {
        bool trExists = false;

        if (goal_reached)
        {
            ROS_INFO("Goal already reached! Going to home pos.. bye!");
            if (move_group.setNamedTarget("home_position")) {
                move_group.setStartStateToCurrentState();
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                    // wait for user
                    visual_tools.prompt("Press 'next' to go home and chill out..");
                    visual_tools.trigger();
                    move_group.move();
                }
            }
            ROS_INFO("Should I go to the rest position, so you can turn off the controller?");
            if (move_group.setNamedTarget("rest_position")) {
                move_group.setStartStateToCurrentState();
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                    // wait for user
                    visual_tools.prompt("Press 'next' to rest in peace :-P");
                    visual_tools.trigger();
                    move_group.move();
                    return 0;
                }
            }
        }

        if (tfBuffer._frameExists("tag_0"))
        {
            trExists = true;
            // get transformation of detected apriltag
            try
            {
                transformStamped = tfBuffer.lookupTransform("world", "tag_0", ros::Time(0));
                ROS_INFO_STREAM("looked up transform!");
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                trExists = false;
                //continue;
            }
        }

        // if no recent position data of tag0 is received, move the camera around
        if (!trExists || ros::Time::now() - transformStamped.header.stamp > ros::Duration(10.0))
        {
            ROS_WARN("found no known transform for tag_0, moving robot to find tag_0!");

            // look around and find apriltag
            geometry_msgs::PoseStamped pCurr = move_group.getCurrentPose();
            tf::Quaternion q_rot, q_orig, q_new;
            geometry_msgs::Quaternion q_msg_new;
            q_rot = tf::createQuaternionFromRPY(0, 0, M_PI / 8.0);
            quaternionMsgToTF(pCurr.pose.orientation, q_orig);
            q_new = q_rot * q_orig;
            q_new.normalize();
            quaternionTFToMsg(q_new, q_msg_new);
            pCurr.pose.orientation = q_msg_new;
            pCurr.header.stamp = ros::Time::now();

            // set new target pose
            move_group.setStartStateToCurrentState();
            move_group.setPoseTarget(pCurr);

            // planning and executing the arm's movement
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                // wait for user
                visual_tools.trigger();
                visual_tools.prompt("Press 'next' to continue looking around for the apriltag");
                move_group.move();
                ROS_INFO_STREAM("success - turned camera");
            }
            else
            {
                ROS_WARN("error - move_group.plan() failed! Could not move to look for camera");
            }
        }
        else if (goal_reached)
        {
            continue;
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

            // save pose to go back later when the apriltag is not visible
            prePressPose = closeUpWorld;

            // set new target pose
            move_group.setStartStateToCurrentState();
            move_group.setPoseTarget(closeUpWorld);

            // planning and executing the arm's movement to the apriltag
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;


            if ((move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) == true && goal_reached == false)
            {
                // wait for user
                visual_tools.trigger();
                visual_tools.prompt("Press 'next' to start aproaching the apriltag");
                move_group.move();
                ROS_INFO("success - moved to approaching point");
                sleep(1.0);

                // now we are closer, check again to get more precise transform
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
                } else {
                    // go a bit back to hopefully find apriltag
                    // closeUp.pose.position.z -= 0.07;
                    // tf2::doTransform(closeUp, closeUpWorld, transformStamped);
                    // move_group.setStartStateToCurrentState();
                    // move_group.setPoseTarget(closeUpWorld);
                    // moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
                    // if (move_group.plan(my_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                    //     move_group.move();
                    // }
                    // // check another time
                    // if (tfBuffer._frameExists("tag_0"))
                    // {
                    //     trExists = true;
                    //     // get transformation of detected apriltag
                    //     try
                    //     {
                    //         transformStamped = tfBuffer.lookupTransform("world", "tag_0", ros::Time(0));
                    //     }
                    //     catch (tf2::TransformException &ex)
                    //     {
                    //         ROS_WARN("%s", ex.what());
                    //         ros::Duration(1.0).sleep();
                    //         trExists = false;
                    //         continue;
                    //     }
                    // }
                }

                moveit::planning_interface::MoveGroupInterface::Plan trajectory_plan;
                moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
                // moveit::core::RobotState start_state_core = *(current_state.get());
                moveit_msgs::RobotState start_state_msg;
                moveit::core::robotStateToRobotStateMsg(*(current_state.get()), start_state_msg, false);

                // set waypoint for the path
                std::vector<geometry_msgs::Pose> waypoints;

                // compute cartesian path to button
                closeUp.pose.position.x += 0.055;
                tf2::doTransform(closeUp, closeUpWorld, transformStamped);
                waypoints.push_back(closeUpWorld.pose);

                closeUp.pose.position.y = 0.013;
                tf2::doTransform(closeUp, closeUpWorld, transformStamped);
                waypoints.push_back(closeUpWorld.pose);

                closeUp.pose.position.z = 0.052+0.031-0.016;
                tf2::doTransform(closeUp, closeUpWorld, transformStamped);
                waypoints.push_back(closeUpWorld.pose);

                closeUp.pose.position.z += 0.05;
                tf2::doTransform(closeUp, closeUpWorld, transformStamped);
                waypoints.push_back(closeUpWorld.pose);

                // set movement speed to 1/10 to carefully press the button
                //move_group.setMaxVelocityScalingFactor(0.1);
                move_group.setPlanningTime(30.0);

                // computing the cartesian path
                moveit_msgs::RobotTrajectory trajectory;
                moveit_msgs::MoveItErrorCodes errCode;
                const double jump_threshold = 0.0;
                const double eef_step = 0.01;

                double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true, &errCode);
                ROS_INFO("Calculated %.2f%% of the Cartesian Path", fraction * 100.0);
                ROS_INFO_STREAM("Error Code of Cartesian Path: " << errCode);

                // wait for user
                visual_tools.trigger();
                visual_tools.prompt("Press 'next' to start the cartesian path");

                // filling trajectory with information and trying to execute
                trajectory_plan.trajectory_ = trajectory;
                trajectory_plan.start_state_ = start_state_msg;
                trajectory_plan.planning_time_ = 0.1337;
                success = (move_group.execute(trajectory_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                ROS_INFO("Executing Cartesian path %s", success ? "Succeeded" : "FAILED");
                // set flag if cartesian path was executed successfully
                if (success && fraction >= 1)
                {
                    goal_reached = true;
                    ROS_INFO("Goal reached!");
                } else {
                    ROS_INFO("SORRY! Could not compute cartesian path, trying to start over from prePressPose ;-)");
                    try_again == true;
                    continue;
                }

                sleep(3.0);

                // go back to old pose
                std::vector<geometry_msgs::Pose> waypoints_back;
                // closeUp.pose.position.z = 0.2;
                // tf2::doTransform(closeUp, closeUpWorld, transformStamped);
                waypoints_back.push_back(prePressPose.pose);

                // get current pose
                // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
                // moveit::core::RobotState start_state_core = *(current_state.get());
                moveit::core::robotStateToRobotStateMsg(*(move_group.getCurrentState().get()), start_state_msg, false);

                // compute cartesian path
                fraction = move_group.computeCartesianPath(waypoints_back, eef_step, jump_threshold, trajectory, true, &errCode);
                ROS_INFO("Calculated %.2f%% of the Cartesian Path back to old pose", fraction * 100.0);
                ROS_INFO_STREAM("Error Code of Cartesian Path: " << errCode);

                // wait for user
                visual_tools.trigger();
                visual_tools.prompt("Press 'next' to return after pressing the button");
                ROS_INFO("path might be not shown in rviz");

                // filling trajectory with information and trying to execute
                trajectory_plan.trajectory_ = trajectory;
                trajectory_plan.start_state_ = start_state_msg;
                trajectory_plan.planning_time_ = 0.666;
                success = (move_group.execute(trajectory_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                ROS_INFO("Executing Cartesian path back to old pose %s", success ? "Succeeded" : "FAILED");
            }
            else
            {
                ROS_WARN("error - move_group.plan() failed!");
            }
        }

        //sleep(1.0);
    }

    ros::shutdown();
    return 0;
}
