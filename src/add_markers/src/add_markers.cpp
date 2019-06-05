#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <string>

double goal_x[2] = {3.78, -4.65};
double goal_y[2] = {9.75, 5.73};
std::string markerState = "START";

visualization_msgs::Marker getMarker(double x, double y) {
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "marker";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side   
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  
  marker.action = visualization_msgs::Marker::ADD;

  marker.lifetime = ros::Duration();
  return marker;
}

void updateMarkerState(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Receive marker state update");
  markerState = msg->data.c_str();
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  
  ros::Subscriber updateStateSub = n.subscribe("/marker_state", 10, updateMarkerState);
  
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  visualization_msgs::Marker marker;
  
  while (ros::ok())
  {
    if (markerState == "START") {
      marker = getMarker(goal_x[0], goal_y[0]);
    }
    else if (markerState == "TRANSPORTED") {
      marker.action = visualization_msgs::Marker::DELETE;
    }
    else if (markerState == "END") {
      marker = getMarker(goal_x[1], goal_y[1]);
    }
  
    marker_pub.publish(marker);

    ros::spinOnce();
    r.sleep();
  }
}