# interbotix_xslocobot_nav

## Overview
This package configures the ROS Navigation Stack needed to give any X-Series Interbotix Locobot platform the ability to perform simultaneous localization and mapping (a.k.a SLAM), navigation, or just localization. It can be used with just the [Intel RealSense D435](https://www.intelrealsense.com/depth-camera-d435/) camera or with both the camera and the [A2M8 RPLidar](https://www.slamtec.com/en/Lidar/A2) laser scanner. The localization and mapping part is done using the [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) ROS package while the navigation part is accomplished via the [move_base](http://wiki.ros.org/move_base) ROS package. For best results, this package should be run with the robot in an indoor, uncluttered environment that does not contain too much sunlight and has minimal reflective surfaces.

## Structure
![xslocobot_nav_flowchart](images/xslocobot_nav_flowchart.png)
As shown above, this package builds on top of the *interbotix_xslocobot_control* package (which starts the **xs_sdk** node), and is used in conjunction with the *rtabmap_ros* and *move_base* ROS packages. A short description of the nodes needed from those packages can be found below:
- **controller_manager** - responsible for loading and starting a set of controllers at once, as well as automatically stopping and unloading those same controllers
- **xs_hardware_interface** - receives joint commands from the ROS controllers and publishes them to the correct topics (subscribed to by the **xs_sdk** node) at the appropiate times

## Usage
This package is not meant to be used by itself but included in a launch file within your custom ROS package (which should expose a FollowJointTrajectoryAction interface).
To run this package, type the line below in a terminal (assuming a Locobot with a PincherX 100 is being launched).
```
$ roslaunch interbotix_xslocobot_nav xslocobot_nav.launch robot_model:=locobot_wx200
```
This is the bare minimum needed to get up and running. Take a look at the table below to see how to further customize with other launch file arguments.

| Argument | Description | Default Value |
| -------- | ----------- | :-----------: |
| robot_model | model type of the Interbotix Locobot such as 'locobot_base' or 'locobot_wx250s' | "" |
| robot_name | name of the robot (typically equal to `robot_model`, but could be anything) | "$(arg robot_model)" |
| show_lidar | set to true if the lidar is installed on the robot; this will load the lidar related links to the 'robot_description' parameter | $(arg use_lidar) |
| external_urdf_loc | the file path to the custom urdf.xacro file that you would like to include in the Interbotix robot's urdf.xacro file| "" |
| use_rviz | launches Rviz | false |
| use_camera | if true, the RealSense D435 camera nodes are launched | false |
| mode_configs | the file path to the 'mode config' YAML file | refer to [xslocobot_nav.launch](launch/xslocobot_nav.launch) |
| dof | the degrees of freedom of the arm | 5 |
