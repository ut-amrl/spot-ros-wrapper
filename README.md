		
# Introduction 
Enable programmatic control of the Boston Dynamic Spot robot through the [Robot Operating System](https://www.ros.org/) (ROS) interface. 

# Overview

The [spot_ros_interface](./spot_ros_interface/) ROS package provides a ROS interface to the Spot API converting ROS messages to API/gRPC calls to the Spot robot.

## Architecture Overview
![System Architecture](./docs/SystemArchitecture.png)

# Build and Run
1. Set up your catkin workspace (e.g. `~/catkin_ws`), and source the developer init script:
    ```
    source ~/catkin_ws/devel/setup.bash
    ```
1. Clone this repo to the `src` directory in your Catkin workspace
    ```
    cd ~/catkin_ws/src/
    git clone https://github.com/ut-amrl/spot-ros-wrapper.git
    ```
1. Build using `catkin_make`:
   ```
   cd ~/catkin_ws
   catkin build
   ```
1. Run:
	```
	rosrun spot_ros_interface  spot_ros_interface.py --username USERNAME --password PASSWORD  192.168.80.3
	```
	*Note:* Spot's default IP address is 192.168.80.3 over WiFi, 10.0.0.3 over the back ethernet port, and 192.168.50.3 for the front GXP port.

### Controlling Spot from your keyboard

With the ROS wrapper running, open a new terminal and run `keyboard_teleop.py` in the `spot_ros_interface` ROS package:
```
rosrun spot_ros_interface keyboard_teleop.py
```
and follow the instructions on screen.

Ensure `spot_ros_interface.py` is running.

*Note:* If Spot is in a faulty state and/or upside down, make sure to call the self-right command first (from the keyboard_teleop application, press "r").

### To run occupancy grid visualizer

```
roslaunch spot_urdf rviz_display.launch
```

Potential Issues:

- Spot model RViz visualization is white and not oriented properly
    - Solution: Ensure spot_ros_interface and robot_state_publisher nodes are running (the latter should have started by the rvis_display.launch file).

- The occupancy grid is not being displayed:
    - Solution: Ensure that RViz is set to display a Marker type msg (on the left-hand panel), and that it is subscribed to the `occupancy_grid` topic.

### Helpful CLI commands

1. Stand up:
    ```
    rosservice call /stand_cmd '{body_pose: {translation: {x: 0, y: 0, z: 0}, rotation: {x: 0, y: 0, z: 0, w: 1} } }'
    ```
1. Drive forward slowly:
    ```
    rostopic pub /cmd_vel geometry_msgs/Twist -r 10 '{linear: {x: 0.15, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
    ```
1. Sit
    ```
    rosservice call /sit_cmd True
    ```

# ROS Package guidelines
[ROS package guidelines](https://github.com/ethz-asl/mav_tools_public/wiki/How-to-Write-a-ROS-Package)

Contribute

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/). For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.

