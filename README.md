# Ball-Chaser-Robot
In this project, a ball-chaser mobile robot is build with ROS. This robot is first designed and then programmed to chase white colored balls in the world. There are 2 nodes in the project. `
1. drive_bot
2. process_image
## ROS Node
### drive_bot
This server node provides a ball_chaser/command_robot service to drive the robot around by setting its linear x and angular z velocities. The service server publishes messages containing the velocities for the wheel joints.
### process_image
This client node will subscribe to the robot’s camera images and analyze them to determine the position of the white ball. Once the ball position is determined, the client node will request a service from the `drive_bot` server node to drive the robot toward the ball. The robot can drive either left, right or forward, depending on the robot position inside the image. 

Here, a simple approach is used. First, search for white pixels inside the array image. Since the ball is the only object in the world that is white, white pixels indicate the ball’s presence. Then, once you find that the ball, identify its position with respect to the camera - either the left, middle, or right side of the image.
## Project Environment
To get start with the project environment, run the following:
```
$ cd RoboND-Ball-Chaser-Robot/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
Then, setup RViz to visualize the sensor readings. On the left side of RViz, under `Displays`:
- Select `odom` for **fixed frame**
- Click the **Add** button and
  - add `RobotModel` and your robot model should load up in RViz.
  - add `Camera` and select the **Image topic** that was defined in the camera Gazebo plugin.
  - add `LaserScan` and select the **topic** that was defined in the Hokuyo Gazebo plugin.
  
Open a new terminal and run the following: 
```
$ cd RoboND-Ball-Chaser-Robot/catkin_ws
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```
Put the white ball inside the robot view.

  
