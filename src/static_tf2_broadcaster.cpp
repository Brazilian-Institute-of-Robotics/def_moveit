#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>


int main(int argc, char **argv)
{
  ros::init(argc,argv, "static_tf2_broadcast");
  
  // send static reference of frame camera to frame camera_link
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "camera_link";
    static_transformStamped.child_frame_id = "camera1";
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

  ros::spin();
  return 0;
};