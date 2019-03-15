#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;
// Debug publisher.
ros::Publisher img_debug_pub;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;
  client.call(srv);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
  
  	// Proportional constants for angular z proportional controller.
  	float angularKp = 0.003;
  	float maximumLinearX = 0.5;
  
  	int numWhitePixels = 0;
  	int totalX = 0;
  
  	// Loop through the image pixels.
  	for(int y=0; y<img.height; y++) {
    	for (int x=0; x<img.width; x++) {
          	// Index to the first channel R of pixel at position (x, y).
          	int pixel_idx = y*img.step + x*3;
        	if (img.data[pixel_idx] == white_pixel
               && img.data[pixel_idx+1] == white_pixel
               && img.data[pixel_idx+2] == white_pixel)
            {
              totalX += x;
              numWhitePixels++;
            }
        }
    }
  	// No ball is found, stop the robot.
  	if (numWhitePixels == 0) {
      	drive_robot(0, 0);
    	return;
    }
  
  	// Get the center of the mass of the white ball.
  	int centerOfMassX = totalX / numWhitePixels;
  	int halfWidth = img.width/2;
  	
  	// Position of the ball relative to the center of the camera.
  	int offsetX = halfWidth - centerOfMassX;
  
  	float angularZ(0), linearX(0);
  
  	// Proportional controller for angular z.
  	angularZ = angularKp * offsetX;
  
  	// Larger the offset, lower the linearX and vice versa
    linearX = (1.0 - std::abs(offsetX)/halfWidth) * maximumLinearX;
  	
  	drive_robot(linearX, angularZ);
  	ROS_INFO_STREAM("offset: " + std::to_string(offsetX) + " linear: " + std::to_string(linearX) +" angular: " + std::to_string(angularZ));
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
  
  	// Publisher for image debug. Processed image will be published, and rviz can see it.
  	img_debug_pub = n.advertise<sensor_msgs::Image>("/ball_chaser/img_debug", 10);

    // Handle ROS communication events
    ros::spin();

    return 0;
}