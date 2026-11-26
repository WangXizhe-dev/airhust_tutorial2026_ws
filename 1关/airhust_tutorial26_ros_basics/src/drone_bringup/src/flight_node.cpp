#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "flight_node");
  ros::NodeHandle nh;

  constexpr double kTakeoffHeight = 1.5;   // metres
  constexpr double kLandingDistance = 0.5; // metres
  ros::Rate rate(20.0);

  auto setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone/setpoint", 10);
  geometry_msgs::PoseStamped setpoint;
  setpoint.header.frame_id = "map";
  setpoint.pose.orientation.w = 1.0;
  setpoint.pose.position.z = kTakeoffHeight;

  auto obstacle_sub = nh.subscribe<std_msgs::Float32>(
      "/drone/sensor/obstacle_distance", 10,
      [&](const std_msgs::Float32::ConstPtr& msg) {
        if (msg->data < kLandingDistance && setpoint.pose.position.z > 0.0) {
          setpoint.pose.position.z = 0.0;
          ROS_INFO("Landing at obstacle distance %.3f m", msg->data);
        }
      });

  ROS_INFO("flight_node ready");
  while (ros::ok()) {
    ros::spinOnce();
    setpoint.header.stamp = ros::Time::now();
    setpoint_pub.publish(setpoint);
    rate.sleep();
  }
  return 0;
}
