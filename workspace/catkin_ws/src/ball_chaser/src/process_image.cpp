#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  // Request a service and pass the velocities to it to drive the robot
  
  ROS_INFO_STREAM("Driving the robot in the specific direction");
  
  // Request wheel velocities
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;
  
  // Call the commandProbot service and pass the requested wheel velocities
  if (!client.call(srv))
    ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
  int white_pixel = 255;
  bool white_ball = false;
  int n = 0;
  ROS_INFO("D");
  // Loop through each pixel in the image and check if there's a bright white one
  // Then, identify if this pixel falls in the left, mid, or right side of the image 
  // Depending on the white ball position, call the drive_bot function and pass velocities to it
  // Request a stop when there's no white ball seen by the camera
//   for (int i = 0; i < img.height; i++) {
//     for (int j = 0; j < img.step; j++) {
//       if (img.data[i * j] == white_pixel) {
//         n = j;
//         white_ball = true;
//         break;
//       }
//     }
//   }
  
  for (int i = 0; i < img.height; i++) {
    for (int j = 0; j < img.step; j = j + 3) {
      if (img.data[i*img.step+j] == white_pixel && img.data[i*img.step+j+1] == white_pixel && img.data[i*img.step+j+2] == white_pixel) {
        n = j;
        white_ball = true;
        break;
      }
    } 
  }
  ROS_INFO("E");
  
  if (white_ball == true) {
    if (n < (int)(img.step / 3)) {
      // drive to left
      drive_robot(0.5, 0.5);

    }
    else if (n > (int)(2 * img.step / 3)) {
      // drive to right
      drive_robot(0.5, -0.5);
    }
    else {
      // drive straight
      drive_robot(0.5, 0.0);
    }
  }
  else {
    // request a stop
    drive_robot(0.0, 0.0);
  }
  ROS_INFO("F");
}


int main(int argc, char** argv)
{
  // Initialize the process_image and create a handle to it 
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;
  ROS_INFO("A");
  
  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
  ROS_INFO("B");
  
  // Subscribe to /camera/image-raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
  ROS_INFO("C");
  // Handle ROS communication events
  ros::spin();
  
  return 0;
}
