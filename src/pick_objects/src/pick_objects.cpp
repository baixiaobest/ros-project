#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <std_msgs/String.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int numGoals = 2;
double goal_x[2] = {3.78, -4.65};
double goal_y[2] = {9.75, 5.73};

move_base_msgs::MoveBaseGoal getGoal(int i) {
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = goal_x[i];
  goal.target_pose.pose.position.y = goal_y[i];
  goal.target_pose.pose.orientation.w = 1;
  
  return goal;
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle n;
  
  ros::Publisher markerStatePub = n.advertise<std_msgs::String>("/marker_state", 10);
  
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("/move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  for (int i=0; i<numGoals; i++) {
    move_base_msgs::MoveBaseGoal goal = getGoal(i);
    
    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, the base moved to goal successfully.");
      if (i == 0) {
        std_msgs::String msg;
        msg.data = "TRANSPORTED";
      	markerStatePub.publish(msg);
      }
      else if (i == 1) {
        std_msgs::String msg;
        msg.data = "END";
      	markerStatePub.publish(msg);
      }
      ros::Duration(5.0).sleep();
    }
    else {
      ROS_INFO("The base failed to move to the goal.");
    }
  }

  return 0;
}