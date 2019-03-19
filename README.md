# ros-project
This is a series of robotic projects based on Robotic Operating System (ROS). \
The goal of the project is to create a robot that can safely navigate in a enclosed environment with unpredictable placement
of objects. The robots uses two sensors, a lidar and a camera, to perform simultaneous localization and mapping (SLAM).

## Architecture
### Ball Chaser
Ball chaser is a simple robot exists in Gazebo. It performs a very simple task: find a ball and chase it, thus its name.
There are two nodes that operate the robot, a drive_robot node and a process_image node. 

**drive_robot node:** It provides a service /ball_chaser/command_robot defined in "DriveToTarget.srv". It accepts two float
corresponding to the linear velocity and angular velocity and actuate the robot simulated in the Gazebo.\
**process_image:** This node subscribes a the image published by the camera, and analyze the image for the position of a 
white ball. Then it invokes the service in drive_robot node to chase after the ball. This node uses a proportional controller
that takes the angle between the vector to white ball and robot forward direction as input, and the linear velocity as the output.
