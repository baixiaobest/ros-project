#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "ball_chaser/DriveToTarget.h"

float MAX_LINEAR_X = 1;
float MAX_ANGULAR_Z = 2;

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res) {
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::Vector3 linear, angular;
  
  linear.x = std::min((float)std::max((float)req.linear_x, -MAX_LINEAR_X), MAX_LINEAR_X);
  linear.y = 0;
  linear.z = 0;
  
  angular.x = 0;
  angular.y = 0;
  angular.z = std::min((float)std::max((float)req.angular_z, -MAX_ANGULAR_Z), MAX_ANGULAR_Z);
  
  cmd_vel.linear = linear;
  cmd_vel.angular = angular;
  
  motor_command_publisher.publish(cmd_vel);
  
  std::string resStr = "Sent cmd_vel: linear_x: " + std::to_string(linear.x) 
    + " angular: " + std::to_string(angular.z);
  ROS_INFO_STREAM(resStr);
  res.msg_feedback = resStr;
  
  return true;
}
int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}