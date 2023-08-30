# turtle_ws

Welcome to the **turtle_ws** ROS workspace, designed to showcase a solution for the robotics challenge held at the FH Aachen ROS Summer School of 2023.

The challenge encompassed both manual and autonomous navigation of a robot within an arena and the mapping of AprilTags locations during the process.

Additionally, the robot can be ordered to drive to their location after the detection process is finished.


# TurtleBot3

To make topics avaliable on the physical robot:
```
ros2 launch robot_bringup robot.launch.yaml
```

#### URDF
The **turtlebot3** package is built from source, as the URDF had to be adjusted to correctly detect AprilTags. The *camera_link* frame is rotated, so the z-axis corresponds to the camera axis.

To ensure proper functioning, include `export TURTLEBOT3_MODEL=waffle_pi` in your shell configuration file.

#### Teleoperation

Take control of the robot's movement using a USB controller:
```
ros2 run joy joy_node
ros2 run xbox xbox
```

#### Mapping

Generate a map using the SLAM toolbox:
```
ros2 launch my_robot_slam slam_toolbox.launch.yaml use_sim_time:=false
```

# Autonomous navigation

## Nav2 and Localization

Localize the robot within a map and launch Nav2 servers. Specify the file path of the map in the localization launch file.

```
ros2 launch my_robot_navigation robot_nav.launch.py
ros2 launch my_robot_slam localization.launch.py
```
## AprilTag

Create the /image_rect topic and initiate detection for 41h12 AprilTags:
```
ros2 launch tag_detection 41h12.launch.yaml
```

## Autoexplorer

The autoexplorer node continuously generates random points within the maps' boundaries and gives them to Nav2 as goal poses.
```
ros2 run my_robot_navigation autoexplorer
```

## Listener

The listener node maintains persistent knowledge of detected AprilTag positions by establishing a static transform between the map frame and the TagFrame:ID frame upon detection. After identifying a specific number of tags as defined in the code (currently set at 3), the node publishes to **/all_tags_detected**, resulting in the shutdown of the autoexplorer node.

```
ros2 run listener listener
```

## Drive to tag

After AprilTag-frames have been set by *listener*, they can be given to Nav2 as a goal pose.
```
ros2 run listener navigate_to_tag
```
To drive to a frame, publish its name to **/choose_tag**:
```
ros2 topic pub --once /choose_tag std_msgs/String "data: 'name'"
```
