---
title: '6 SLAM navigation'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 6
---

# SLAM navigation #

## Introduction ##

SLAM (simultaneous localization and mapping) is a technique for creating
a map of environment and determining robot position at the same time. It
is widely used in robotics.

While moving, current measurements and localization are changing, in
order to create map it is necessary to merge measurements from previous
positions.

For definition of SLAM problem we use given values (1,2) and expected values (3,4):

<div>
<p>1.  Robot control </p>
	<table border="0" width="100%" cellspacing="0" cellpadding="0">
	    <tr>
		<td><center><i>u<sub>{1:t}</sub> = { u<sub>1</sub>, u<sub>2</sub>, u<sub>3</sub>, ..., u<sub>t</sub>}</i></center></td>
		<td width="60">(1)</td>
	    </tr>
	</table>
</div>


<div>
2.  Observations
	<table border="0" width="100%" cellspacing="0" cellpadding="0">
	    <tr>
		<td><center><i>z<sub>{1:t}</sub> = { z<sub>1</sub>, z<sub>2</sub>, z<sub>3</sub>, ..., z<sub>t</sub>}</i></center></td>
		<td width="60">(2)</td>
	    </tr>
	</table>
</div>


<div>
3.  Environment map
	<table border="0" width="100%" cellspacing="0" cellpadding="0">
	    <tr>
		<td><center><i>m</i></center></td>
		<td width="60">(3)</td>
	    </tr>
	</table>
</div>
 

<div>
4.  Robot trajectory
	<table border="0" width="100%" cellspacing="0" cellpadding="0">
	    <tr>
		<td><center><i>x<sub>{1:t}</sub> = { x<sub>1</sub>, x<sub>2</sub>, x<sub>3</sub>, ..., x<sub>t</sub>}</i></center></td>
		<td width="60">(4)</td>
	    </tr>
	</table>
</div>


<div>
    Robot trajectory estimates are determined with:

	<table border="0" width="100%" cellspacing="0" cellpadding="0">
	    <tr>
		<td><center><i>p(x<sub>0:t</sub>,m|z<sub>1:t</sub>, u<sub>1:t</sub>)</i></center></td>
		<td width="60">(5)</td>
	    </tr>
	</table>
</div>
 
## Coordinate frames tracking ##

ROS can help you with keeping track of coordinate frames over time.
Package for it is `tf2` - the transform library, it comes with a
specific message type: `tf/Transform` and it is always bound to one
topic: `/tf`. Message `tf/Transform` consist of transformation
(translation and rotation) between two coordinate frames, names of both
frames and timestamp. Publishing to `/tf` is done in different way than
to any other topic, we will write `tf` publisher in the example.

### Publishing transformation ###


We will make a node that subscribe to `/pose` topic with message type
`geometry_msgs/PoseStamped` and publish transformation between objects
mentioned in `/pose`. This node is required only on ROSbot, Gazebo is publishing necessary `tf` frames itself.

Begin with headers:

``` cpp
    #include <ros/ros.h>
    #include <geometry_msgs/PoseStamped.h>
    #include <tf/transform_broadcaster.h>
``` 

Publisher for transformation:

``` cpp
    static tf::TransformBroadcaster br;
``` 

Transformation message:

``` cpp
    tf::Transform transform;
``` 

Quaternion for storing rotation data:

``` cpp
    tf::Quaternion q;
``` 

Function for handling incoming `/PoseStamped` messages, extract
quaternion parameters from message and put it to quaternion structure,
put translation parameters into transform message, put quaternion
structure into transform message, publish transform:

``` cpp
    void pose_callback(const geometry_msgs::PoseStampedPtr &pose) {
       q.setX(pose->pose.orientation.x);
       q.setY(pose->pose.orientation.y);
       q.setZ(pose->pose.orientation.z);
       q.setW(pose->pose.orientation.w);

       transform.setOrigin( tf::Vector3(pose->pose.position.x, pose->pose.position.y, 0.0) );
       transform.setRotation(q);

       br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
    }
``` 

Publishing of transform is done with `sendTransform` function which
parameter is `StampedTransform` object. This object parameters are:

-   `transform` - transformation data

-   `ros::Time::now()` - timestamp for current transformation

-   `odom` - transformation parent frame - the one that is static

-   `base_link` - transformation child frame - the one that is
    transformed

In main function just initialize node and subscribe to `/pose` topic:

``` cpp
    int main(int argc, char **argv) {
       ros::init(argc, argv, "drive_controller");
       ros::NodeHandle n("~");
       ros::Subscriber pose_sub = n.subscribe("/pose", 1, pose_callback);
       ros::Rate loop_rate(100);
       while (ros::ok()) {
          ros::spinOnce();
          loop_rate.sleep();
       }
    }
``` 

Your final file should look like this:

``` cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>


tf::Transform transform;
tf::Quaternion q;

void pose_callback(const geometry_msgs::PoseStampedPtr &pose)
{
    static tf::TransformBroadcaster br;
    q.setX(pose->pose.orientation.x);
    q.setY(pose->pose.orientation.y);
    q.setZ(pose->pose.orientation.z);
    q.setW(pose->pose.orientation.w);

    transform.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, 0.0));
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drive_controller");
    ros::NodeHandle n("~");
    ros::Subscriber pose_sub = n.subscribe("/pose", 1, pose_callback);
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
```

Save it as `drive_controller.cpp`.

Now in `CMakeLists.txt` file find line:

    find_package(catkin REQUIRED COMPONENTS
        roscpp
    )

and change it to:

    find_package(catkin REQUIRED COMPONENTS
        roscpp tf
    )

then declare executable:

    add_executable(drive_controller_node src/drive_controller.cpp)

And specify libraries to link:

    target_link_libraries(drive_controller_node
            ${catkin_LIBRARIES}
            )

### Publisher for static transformations ###

If we want to publish transformation which is not changing in time, we
can use `static_transform_publisher` from `tf` package. We will use it
for publishing relation between robot base and laser scanner.

We can run it from command line:

    $ rosrun tf static_transform_publisher 0 0 0 3.14 0 0 base_link laser_frame 100

Arguments are consecutively:

-   `0 0 0` - x y z axes of translation, values are in meters

-   `3.14 0 0 ` - z y x axes of rotation, values are in radians

-   `base_link` - transformation parent frame - the one that is static

-   `laser_frame` - transformation child frame - the one that is
    transformed

-   `100` - delay between consecutive messages

You can use it to adjust position of your laser scanner relative to
robot. The best would be, if you place scanner in such a way that its
rotation axis is coaxial with robot rotation axis and front of laser
scanner base should face the same direction as robot front. Most
probably your laser scanner will be attached above robot base. To set
scanner 10 centimeters above robot you should use:

    $ rosrun tf static_transform_publisher 0 0 0.1 0 0 0 base_link laser_frame 100

And if your scanner is also rotated by 180º around z-axis it should be:

    $ rosrun tf static_transform_publisher 0 0 0.1 3.14 0 0 base_link laser_frame 100

Remember that if you have improperly mounted scanner or its position is
not set correctly, your map will be generated with errors or it will be
not generated at all.

### Visualizing transformation with rviz ###

You can visualize all transformations that are published in the system.
To test it run:

-   `CORE2` bridge node -
    `/opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2 `

-   `drive_controller_node` - publisher that you just created

Or instead ot these two start `Gazebo`:

- `roslaunch rosbot_gazebo maze_world.launch`

You will also need `rviz` and keyboard controller:

-   `teleop_twist_keyboard` - keyboard controller

-   `rviz` - visualization tool


You may also add some `static_transform_publisher` nodes.

Now click **Add** from object manipulation buttons, in new window select
**`By display type`** and from the list select `Tf`. You can also add
`Pose` visualization. You can observe as `robot_base` frame moves
accordingly to robot movement.

![image](/assets/img/ros/man_6_1.png)

## SLAM implementation in ROS ##

To perform accurate and precise SLAM, the best is to use laser scanner
and odometry system with high resolution encoders.

### Running the laser scanner ###

In this example we will use **`rpLidar`** laser scanner. Place it on
your robot, main rotation axis should pass the centre of robot. Front of
the **`rpLidar`** should face the same direction as front of the robot.

As a driver for the **`rpLidar`** we will use `rplidarNode` from
`rplidar_ros` package. This node communicate with device and publish its
scans to `/scan` topic with message type `sensor_msgs/LaserScan`. We do
not need more configuration for it now.

To test it you can run only this one node:

```bash
    $ rosrun rplidar_ros rplidarNode
```

For **Gazebo** you do not need any additional nodes, just start simulator and laser scans will be already published to appropriate topic.

You should have `/scan` topic in your system. You can examine it with
`rostopic info` but better do not try to echo it, it is possible but you
will get lots of output that is hard to read. Better method for checking
the `/scan` topic is to use rviz. Run rviz and click **Add** from object
manipulation buttons, in new window select **`By topic`** and from the
list select `/scan`. In visualized items list find position
`Fixed Frame` and change it to `laser_frame`. To improve visibility of
scanned shape, you may need to adjust one of visualized object options,
set value of `Style` to `Points`. You should see many points which
resemble shape of obstacles surrounding your robot.

![image](/assets/img/ros/man_6_2.png)

Shut down `rplidarNode` and run it again, but with some other nodes:

-   `CORE2` bridge node -
    `/opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2 `

-   `rplidarNode` - driver for rpLidar laser scanner

-   `drive_controller_node` - publisher that you just created

Or instead of these three, start `Gazebo`:

- `roslaunch rosbot_gazebo maze_world.launch`

You will also need:

-   `static_transform_publisher` - `tf` publisher for transformation of
    laser scanner relative to robot

-   `teleop_twist_keyboard` - keyboard controller


-   `rviz` - visualization tool

You can use below `launch` file:

``` launch
<launch>

    <arg name="use_rosbot" default="true"/>
    <arg name="use_gazebo" default="false"/>

    <include if="$(arg use_gazebo)" file="$(find rosbot_gazebo)/launch/maze_world.launch"/>
    <include if="$(arg use_gazebo)" file="$(find rosbot_gazebo)/launch/rosbot.launch"/>

    <node if="$(arg use_rosbot)" pkg="rplidar_ros" type="rplidarNode" name="rplidar">
        <param name="angle_compensate" type="bool" value="true"/>
    </node>
    
    <node if="$(arg use_rosbot)" pkg="tutorial_pkg" type="drive_controller_node" name="drive_controller"/>

    <node pkg="tf" type="static_transform_publisher" name="laser_broadcaster" args="0 0 0 3.14 0 0 base_link laser_frame 100" />

    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop_twist_keyboard" output="screen"/>

    <node pkg="rviz" type="rviz" name="rviz"/>

</launch>
```

In `rviz` add `Tf` and `/scan`. This time set `Fixed Frame` to `odom`.

Try to move around your robot, you should see as laser scans change its
shape accordingly to obstacles passed by robot.

![image](/assets/img/ros/man_6_3.png)

### Navigation and map building ###

For building a map and localizing robot relative to it, we will use
`slam_gmapping` node from `gmapping` package.

This node subscribes `/tf` topic to obtain robot pose relative to
starting point and laser scanner pose relative to robot and also
subscribe `/scan` topic to obtain laser scanner messages. Node publishes
to `/map` topic with message type `nav_msgs/OccupancyGrid`, this contain
the actual map data.

We need to set few parameters:

-   `base_frame` - name of frame related with robot, in our case it will
    be `base_link`

-   `odom_frame` - name of frame related with starting point, in our
    case it will be `odom`

-   `delta` - map resolution expressed as size of every pixel in meters

You can use below `launch` file:

``` launch
<launch>

    <arg name="use_rosbot" default="true"/>
    <arg name="use_gazebo" default="false"/>

    <include if="$(arg use_gazebo)" file="$(find rosbot_gazebo)/launch/maze_world.launch"/>
    <include if="$(arg use_gazebo)" file="$(find rosbot_gazebo)/launch/rosbot.launch"/>

    <node if="$(arg use_rosbot)" pkg="rplidar_ros" type="rplidarNode" name="rplidar">
        <param name="angle_compensate" type="bool" value="true"/>
    </node>
    
    <node if="$(arg use_rosbot)" pkg="tutorial_pkg" type="drive_controller_node" name="drive_controller"/>

    <node pkg="tf" type="static_transform_publisher" name="laser_broadcaster" args="0 0 0 3.14 0 0 base_link laser_frame 100" />

    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop_twist_keyboard" output="screen"/>

    <node pkg="rviz" type="rviz" name="rviz"/>

    <node pkg="gmapping" type="slam_gmapping" name="gmapping">
        <param name="base_frame" value="base_link"/>
        <param name="odom_frame" value="odom" />
        <param name="delta" value="0.1" />
    </node>

</launch>
```

In `rviz` add `Tf` and `/scan`, again open object adding window, select
**`By topic`** and from the list select `/map`.

At the beginning there could be no map, you may need to wait few second
until it is generated. Starting state should be similar to the one on
picture:

![image](/assets/img/ros/man_6_4.png)

Now drive your robot around and observe as new parts of map are added,
continue until all places are explored. Final map should look like
below:

![image](/assets/img/ros/man_6_5.png)

## Summary ##

After completing this tutorial you should be able to publish
transformations between various frames, connect laser scanner to the
system, set up `slam_gmapping` node to perform SLAM task and visualize
map, robot position and all related frames.

---------

*by Łukasz Mitka, Husarion*

*Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com*
