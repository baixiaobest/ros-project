# Home Service Robot
The robot performs localization given a map, and is able to navigate to a goal position when instructed.

## Pick up objects
The robot is given navigation goal through ROS actionlib. This is similar to server client architecture of the ROS.
The goal is sent from pick_objects node to move_base node through MoveBaseAction.
Once the move_base node receives the node, it calculates the path to the goal based on the map given to the node.

## Marker
Marker is the object that the robot picks up. The add_markers node manages the marker in the rviz.
This node publishes to visualization_marker topic, with information about the location, orientation and shape of
the marker. The rviz subscribes to this topic, and displays this marker.

## Home service function
To let the robot pick up the marker, the pick_objects node and the add_markers node should be communicating.
In my implementation, the pick_objects publishes string message to add_markers node, telling it
what state the marker should be in. There are three states that the marker can be in: "START", "TRANSPORTED" 
and "END". These three states corresponds to marker being in start position, hidden and being in end position.
These state change messages will be sent when robot is navigating to different goals.