---
title: '10 UWB localization'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 10
---

# UWB localization #

## Introduction ##

Localization can be performed with use of many methods. In prevoius tutorials we used SLAM based on laser scanner. This methd performs best in indoor areas with lots of distinguishable landmarks. For outdoors uses or large indoor areas with few landmarks this can be faulty. One solution could be to use GPS, but its accuracy is usually not enough for robotics.
 
Appropriate solution for precise navigation in outdoor conditions could be Ultra Wideband (UWB) technology. It allow for precise navigation with use of fixed-position markers (anchors) and receiver (tag) mounted to mobile base. This solution require a map to be known in advance and position of all anchors realative to this map.

## UWB localization in ROS ##

In this tutorail we will use UWB anchors and tag from [Pozyx Labs](https://www.pozyx.io/). It comes with Arduino and Python libraries. We will use Python version which allow tag to communicate with Linux, there are also some simple [examples](https://github.com/pozyxLabs/pozyx_ros) of using the library in ROS. We will prepare a bit more sophisticated node for our use.


### Requirements regarding robot ###

To preform localization with UWB, robot does not need to comply with any specific requirements. We will need only a USB socket, most of ROS compatible robots have it. 
 
### Requirements regarding environment ###

Before we start with programming, we need to make some arrangements. First of all we need a map of environment, this could be any map source that is publishing `nav_msgs/OccupancyGrid`. For this purpose we will use `map_server` node from `map_server` package. Moreover we need to know position of each anchor relative to this map.

### Map setup

Create `maps` folder under `tutorial_pkg`, here you will store your maps and their configuration files.

The source of our map will be a graphic file containing layout of obstacles and unoccupied areas, white color means free cells, black means occupied. Default configuration assumes that lower left corner is the beggining of map coordinate system with x axis pointing towards right side of image and y axis pointing towards top of the image. We will use UWB for a small area example, thus we can let it be quite detaied, one pixel will represent squre of 1x1 cm. Draw the map using any image edior you want and save it as `map.jpg` inside `maps` folder. 

Now we can prepare configuration file for the map, create `map.yaml` file in `maps` folder and paste:

```
image: map.jpg
resolution: 0.01
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.95
free_thresh: 0.05
negate: 0
```

Parameter explanation :

-   `image` - path to a graphic file cotaning the actual map, could be absolute or relative
-   `resolution` - pixel size in meters
-   `origin` - position of the first pixel
-   `occupied_thresh` - percentage of maximum color value, every pixel above this value will be marked as occupied
-   `free_thresh` - percentage of maximum color value, every pixel below this value will be marked as free
-   `negate` - indicates if color values should be inverted

The `map_server` node accepts images as grayscale, images could be in colour, but they will be converted to grayscale automatically, then obstacles will be determined with use of `occupied_thresh` and `free_thresh` parameters.

Above two files are required by the `map_server` node to publish map as `nav_msgs/OccupancyGrid`.

### Anchors setup

Now we can set up the localization system, for 2D localization at least 3 anchors are required, for 3D position 4 anchors are needed. In tutorial we will place 4 anchors and will perform 2D localization.

Place your anchors following below guidelines:
-   Anchors must be positioned with the same orientation
-   Anchors must be spreaded evenly along workspace
-   Each of the anchors need to be placed at unique height
-   Each anchor need to be in line-of-sight with other anchors
-   Position of each anchor need to be precisely measured (up to milimiter accuracy), values need to be expressed in map reference frame

Above guidelines allow for most accurate localization, if you do not keep with them, the system may still work, but you will get less accurate measurements. Each anchor has its unique ID printed on casing in form `0xYYYY` where `YYYY` is the hexadecimal number. Note the anchors IDs along with their positions, you will need them later.

### Tag setup

Tag need to be mounted to a mobile base, preferably on top o the robot with antenna pointing up. Please note the tag axis orientation relative to mobile base. To remind, ROS convention assumes that `x` axis is pointing in front of the robot, `y` axis is pointing to the left of the robot and `z` axis is pointing up.

### The localization node

In localization node we are going to use Pozyx python library, install it with:

```
pip install pypozyx
```

Inside the `tutorial_pkg` create folder `scripts` and inside it create file `uwb_localize.py`.

Below is the code for `uwb_localize` node with each section explained.


Import required libraries:

```
#!/usr/bin/env python
import pypozyx
import rospy
from geometry_msgs.msg import TransformStamped
import std_msgs.msg
import tf
```

Define method to get number of known devices:

```
def get_list_size(pozyx):
    list_size = pypozyx.SingleRegister()
    pozyx.getDeviceListSize(list_size)
    return list_size
```

Define method to get config of known devices:

```
def get_device_list(pozyx):
    list_size = get_list_size(pozyx)
    device_list = pypozyx.DeviceList(list_size=list_size[0])
    pozyx.getDeviceIds(device_list)
    return device_list
```

Define method to set new coordinates for selected anchor:

```
def set_anchor_pos(pozyx, x, y, z, id):
    new_pos = pypozyx.Coordinates()
    new_pos.x = x
    new_pos.y = y
    new_pos.z = z
    status = pozyx.changeDeviceCoordinates(id, new_pos)
    if status == pypozyx.POZYX_FAILURE:
        rospy.logerr("Can not set position for 0x%x" % id)
        return False
    else:
        rospy.logdebug("Position for anchor 0x%x set succesfully" % id)
        return True
```

This method takes arguments `x`, `y`, `z` which are anchor coordinates in milimeters. Last argument is anchor ID.

Define method for setting up all anchors:

```
def set_up_anchors(pozyx):
    rospy.logdebug("Clearing device list")
    pozyx.clearDevices()
    rospy.logdebug("Discovering anchors")
    pozyx.doDiscovery()
    list_size = get_list_size(pozyx)
    rospy.logdebug("Discovery finished, found %d devices" % list_size[0])

    if list_size[0] == 0:
        rospy.logerr("No anchord found, abort operation")
        return False
    if(list_size[0] < 4):
        rospy.logerr("%d anchors is not enough to localize" % list_size[0])
        return False

    rospy.loginfo("Seting new anchor positions")
    set_anchor_pos(pozyx, x1, y1, z1, id1)
    set_anchor_pos(pozyx, x2, y2, z2, id2)
    set_anchor_pos(pozyx, x3, y3, z3, id3)
    set_anchor_pos(pozyx, x4, y4, z4, id4)
```

This method clears tag memory in case any other anchors were saved. Then anchors discovery is executed, this step requires all anchrs to be powered up and within detection range.
Method `set_anchor_pos` is called wih arguments which are anchor coordinates `x`, `y`, `z` and anchor identifier `id`. Plese replace them with the values that you noted in `Anchors setup` step.

Define the main method and init ROS node:

```
def pozyx_main():
    rospy.init_node('pozyx_pose_node')
```

Retrieve parameters set by user:

```
    port_name = '/dev/ttyACM0'
    if rospy.has_param('~port'):
        port_name = rospy.get_param('~port')
    rospy.logdebug("Param port: %s" % port_name)

    world_frame = 'pozyx'
    if rospy.has_param('~frame_name'):
        world_frame = rospy.get_param('~frame_name')
    rospy.logdebug("Param frame name: %s" % world_frame)

    tag_frame = 'tag'
    if rospy.has_param('~tag_frame'):
        tag_frame = rospy.get_param('~tag_frame')
    rospy.logdebug("Param frame name: %s" % tag_frame)

    init_anchors = False
    if rospy.has_param('~setup_anchors'):
        init_anchors = rospy.get_param('~setup_anchors')
    rospy.logdebug("Param frame name: %s" % init_anchors)
```

Establish connection with tag: 

```
    try:
        pozyx = pypozyx.PozyxSerial(port_name)
    except:
        rospy.logerr("Pozyx not connected, shutdown node")
        return
```

Value `pozyx` returned by constructor `pypozyx.PozyxSerial` is the handle for issuing varoius UWB commands.


Call anchors initialization method dependant on user parameter:

```
    if init_anchors:
        set_up_anchors(pozyx)
```

Retrieve known devices, this will be needed to publish their `tf` frames:

```
    device_list = get_device_list(pozyx)
```

Set positioning parameters:

```
    pozyx.setRangingProtocolPrecision()
    pozyx.setPositionAlgorithmTracking()
    pozyx.setPositioningFilterFIR(2)
```

Set rate of position updates:

```
    r = rospy.Rate(10) # 10hz
```

Prepare message for anchors `tf` publications, anchor rotation will never be changed in script as they need to be mounted in the same orientation:

```
    br = tf.TransformBroadcaster()
    transformStampedMsg = TransformStamped()
    transformStampedMsg.header.frame_id = world_frame
    transformStampedMsg.header.stamp = rospy.Time.now()
    transformStampedMsg.child_frame_id = tag_frame
    transformStampedMsg.transform.translation.x = 0.0
    transformStampedMsg.transform.translation.y = 0.0
    transformStampedMsg.transform.translation.z = 0.0
    transformStampedMsg.transform.rotation.x = 0.0
    transformStampedMsg.transform.rotation.y = 0.0
    transformStampedMsg.transform.rotation.z = 0.0    
    transformStampedMsg.transform.rotation.w = 1.0
```

Retrieve anchor positions and store them in `list()` object:

```
    anchor_list = list()
    for i in device_list:
        anchor_coordinates = pypozyx.Coordinates()
        pozyx.getDeviceCoordinates(i, anchor_coordinates)
        anchor_list.append(anchor_coordinates)
```

Declare counter for pubished messages and begin node infinite loop:

```
    published_transforms = 0
    while not rospy.is_shutdown():
```

Cover the positioning functions with `try - except` clause in case of communication error:

```
        try:
            coords = pypozyx.Coordinates()
            quat = pypozyx.Quaternion()
            pozyx.doPositioning(coords, pypozyx.POZYX_2D)
            pozyx.getQuaternion(quat)
```

In case that positioning returned all zeroes, skip that valuee. All zeroes occures when device gets resetted or during startup. Otherwise publish position:

```
            if coords.x==0 and coords.y==0 and coords.z==0:
                rospy.logdebug("Ommitted one publishing loop out of %d due to values reset", published_transforms)
                published_transforms = 0
            else:
                transformStampedMsg.header.stamp = rospy.Time.now()
                transformStampedMsg.transform.translation.x = float(coords.x)/1000
                transformStampedMsg.transform.translation.y = float(coords.y)/1000
                transformStampedMsg.transform.translation.z = float(coords.z)/1000
                transformStampedMsg.transform.rotation.x = quat.x
                transformStampedMsg.transform.rotation.y = quat.y
                transformStampedMsg.transform.rotation.z = quat.z
                transformStampedMsg.transform.rotation.w = quat.w
                br.sendTransformMessage(transformStampedMsg)
                published_transforms += 1
```

We will also publish anchors positions, here we utilise the anchor list obtained earlier:

```
            for i in range(len(anchor_list)):
                br.sendTransform((float(anchor_list[i].x)/1000, float(anchor_list[i].y)/1000, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "anchor_" + hex(device_list[i]), world_frame)
```

Handle exception if they occured and sleep for the loop duration:

```
        except Exception as e:
            rospy.logerr("Error: %s" % str(e))
            rospy.loginfo("Pozyx read error")
            del pozyx
            try:
                pozyx = pypozyx.PozyxSerial(port_name)
                rospy.loginfo("Pozyx reconnected")
            except:
                rospy.loginfo("Pozyx not connected")
                return
        r.sleep()
```

Script entry point, call the main method:

```
if __name__ == '__main__':
    try:
        pozyx_main()
    except rospy.ROSInterruptException:
        pass
```

Make sure that you copied all code snippets and save the `uwb_localize.py` file.

### Launching the system

We will need three nodes for complete localization system:

-   `map_server` from `map_server` package to publish a map, as an argument we need to provide `map.yaml` configuration file.
-   `uwb_localize.py` that we just prepared, we need to provide parameters: `port`, `frame_name`, `tag_frame` and `setup_anchors`.
-   `static_transform_publisher` from `tf` package to publish transform from tag frame to robot base frame, arguments for this node may depend on tag orientation.

You can use below `launch` file:

```
<launch>
    <node pkg="map_server" type="map_server" name="map_server" args="$(find tutorial_pkg)/maps/map.yaml"/>

    <node pkg="htutorial_10" type="uwb_localize.py" name="uwb" output="screen" respawn="true">
        <param name="port" value="/dev/ttyACM0"/>
        <param name="frame_name" value="map"/>
        <param name="tag_frame" value="uwb_tag"/>
        <param name="setup_anchors" value="true"/>
    </node>

    <node pkg="tf" type="static_transform_publisher" name="base_pub" args="0 0 0 0 0 -1.5707 uwb_tag base_frame 10" />

</launch>
```

To test and verify localization you may want to drive your robot, you can use serial bridge and `teleop_twist_keyboard` for this:

```
/opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2
```

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```


## Summary ##

After completing this tutorial you should be familiar with Ultra Wideband localization technology.

---------

*by Łukasz Mitka, Husarion*

*Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com*
