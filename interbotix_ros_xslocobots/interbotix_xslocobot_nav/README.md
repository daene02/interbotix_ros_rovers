# interbotix_xslocobot_nav

## Overview
This package configures the ROS Navigation Stack needed to give any X-Series Interbotix Locobot platform the ability to perform simultaneous localization and mapping (a.k.a SLAM), navigation, or just localization. It can be used with just the [Intel RealSense D435](https://www.intelrealsense.com/depth-camera-d435/) camera or with both the camera and the [A2M8 RPLidar](https://www.slamtec.com/en/Lidar/A2) laser scanner. The localization and mapping part is done using the [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) ROS package while the navigation part is accomplished via the [move_base](http://wiki.ros.org/move_base) ROS package. For best results, this package should be run with the robot in an indoor, uncluttered environment that does not contain too much sunlight and has minimal reflective surfaces.

## Structure
![xslocobot_nav_flowchart](images/xslocobot_nav_flowchart.png)
As shown above, this package builds on top of the *interbotix_xslocobot_control* package (which starts the **xs_sdk** node), and is used in conjunction with the *rtabmap_ros* and *move_base* ROS packages. A short description of the nodes needed from those packages can be found below:
- **controller_manager** - responsible for loading and starting a set of controllers at once, as well as automatically stopping and unloading those same controllers
- **xs_hardware_interface** - receives joint commands from the ROS controllers and publishes them to the correct topics (subscribed to by the **xs_sdk** node) at the appropiate times

## Usage
There are three ways this package can be used. One way is to build a map from scratch and have the robot perform SLAM. Assuming you have the Locobot WX200 robot with the lidar add-on, run the following in a terminal via SSH:
```
$ roslaunch interbotix_xslocobot_nav xslocobot_nav.launch robot_model:=locobot_wx200 use_lidar:=true rtabmap_args:=-d
```

The above command will delete the current database (by default - stored at `~/.ros/rtabmap.db`) and create a new one. The database stores map cloud and graph data (see the ROS Wiki for info on these concepts) which it then uses at run time to create a 2D map that move_base can use.

Once the Nav Stack is running, you should see info messages appearing in the terminal once a second saying something similar to...
```
rtabmap (3): Rate=1.00s, Limit=0.000s, RTAB-Map=0.0697s, Maps update=0.0099s pub=0.0000s (local map=1, WM=1)
```

Now, to visualize the robot in Rviz, run the following on your personal Linux computer (note that you should first run the remote installation script on your personal computer if you haven't done so already):
```
$ roslaunch interbotix_xslocobot_descriptions remote_view.launch robot_name:=locobot_wx200 rviz_frame:=map
```

Rviz should now open up looking like the picture below:
<p align="center">
  <img width="70%" height="auto" src="images/rviz_start.png">
</p>

To visualize the map being created, just click the checkbox by the **Map** display. To see a live color feed as well as filtered point cloud data from the RealSense camera, click the **Camera** display. Note that move_base uses this filtered point cloud data to detect obstacles in the robot's path. It is filtered to reduce bandwidth and to segment out the floor so that the robot doesn't think the 'floor is lava' so-to-speak :). On the other hand, rtabmap_ros uses both the live feed and an aligned depth feed (not displayed) to perform mapping and localization. Next, click the **LaserScan** display to show a 360 degree view of where it thinks there are obstacles. This is used both by move_base for obstacle dectection and rtabmap_ros for mapping and localization refinement. Moving on, the **RtabmapRos** display can be used to show a point-cloud representation of the robot's environment built in real-time as the robot moves. See the picture below for a visualization of all these displays in Rviz.

<p align="center">
  <img width="70%" height="auto" src="images/map_building.png">
</p>



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
