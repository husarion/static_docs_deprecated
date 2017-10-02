---
title: '1 ROS introduction'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 1
---

# ROS introduction

## System structure

ROS is a software suite that allow for quick and easy building of
autonomous robotic systems. ROS should be considered as set of tools for
creating new solutions or adjusting already existing ones. Major
advantage of this system is great set of drivers and implemented
algorithms widely used in robotics.

### Nodes

Base unit in ROS is node. Nodes are in charge of handling devices or
computing algorithms, each node for separate task. Nodes can communicate
with each other with use of topics or services. ROS software is
distributed in packages. Single package is usually developed for
performing one type of task and can contain one or multiple nodes.

### Topics

In ROS, topic is a data stream used to exchange information between
topics. The are used to send frequent messages of one type. This could
be sensor readout or motor goal speed. Each topic is registered under
the unique name and with defined message type. Nodes can connect with it
to publish messages or subscribe them. For a given topic, one node can
not publish and subscribe to it at the same time, but there is no
restrictions in the number of different nodes publishing or subscribing.

### Services

Communication by services resemble client-server model. In this mode one
node (the server) registers service in system. Later any other node can
ask that service and get response. In contrast to topics, services allow
for two way communication, as request can also contain some data.

## Basic tools

During work with ROS there are some tools that are most useful. They are
intended to examining nodes and topics.

### rosnode

Rosnode is command line application for examining which nodes are
registered in the system and also check their statuses. Use of
application is as follow:

``` bash
     $ rosnode command [node_name]
``` 

Command could be:

- `list` - display list of running nodes

- `info` - display info regarding selected node

- `kill` - stop selected node

Detailed info could be found in
[ROS documentation](http://wiki.ros.org/rosnode).

### rostopic

Rostopic is command line application for examining which topics are
already being published and subscribed, check details of selected topic
or read messages being sent on it. Use of application is as follow:

``` bash
     $ rostopic command [topic_name]
``` 

Command could be:

- `list` - display list of topics

- `info` - display info regarding selected topic

- `echo` - display messages published in the topic

Detailed info could be found in
[ROS documentation](http://wiki.ros.org/rostopic).

### rqt\_graph

`rqt_graph` is graphical tool for visualization of data flow across nodes
in system. To run application type in terminal:

``` bash
     $ rqt_graph
```

## Robot platform

This tutorial is created for ROSbot, open-source robot platform. You can read more about this here: [Hackaday.io](https://hackaday.io/project/21885-rosbot-autonomous-robot-platform) . 

The platform contains:
- 1 × Husarion CORE2-ROS (version with Raspberry Pi 3 or ASUS Tinker Board)
- 1 × RPLIDAR A2 360°
- 1 × Digital camera
- 1 × MPU9250
- 4 × SHARP GP2Y0A41SK0F

And this is how it looks like:
<div>
<center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/rosbot_front.png" /></center>
</div>

<div>
<center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/rosbot_back.png" /></center>
</div>


## ROS and CORE2 Work flow

Before you start working with ROS at CORE2 platform, you need to connect
to it. You can establish connection in two ways: using `ssh` or by
remote desktop. For both methods you need an IP address of your CORE2,
it could be found on cloud.husarion.com after clicking “more” button
next to robot name.

To connect by ssh type in terminal:

    $ ssh husarion@xxx.xxx.xxx.xxx

password is:  `husarion`

To connect by remote desktop you need a remote desktop client
application, depending on your system, you may have various clients.
Parameters for connection:

<table>
    <tr>
        <td>protocol</td><td>`rdp`</td>
    </tr>
    <tr>
        <td>server</td><td>`xxx.xxx.xxx.xxx`</td>
    </tr>
    <tr>
        <td>username</td><td>`husarion`</td>
    </tr>
    <tr>
        <td>password</td><td>`husarion`</td>
    </tr>
</table>


If you are working on Windows, press `WinKey + r` then type `mstsc`.
You will see window:

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_1_1a.png" /></center></div>

Type your device IP address and click `connect`. If you are working on
Mac, you can use **Microsoft Remote Desktop** available at **AppStore**.
If you are working on Ubuntu, you can use **Remmina**.

![image](https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_1_0.png)

First step in work with ROS is to run master process called roscore.
This node handle registration of other nodes, topics and services. After
this you can run your nodes. To start master you can use command:

    $ roscore

### Starting system step by step

You can start ROS by typing each name of each node manually, you can do
this with command:

    $ rosrun package_name node_type [options]

Package\_name and node\_type are names of package and node that you want
to run.

#### Defining node name

If you want to bind specific identifier to node, at the end of the
command, add:

    __name:=new_node_name

Note that there are two underscores before the name.

#### Setting parameter

You can also set parameter value by adding:

    _param:=value

Note that there is an underscore before the parameter name.

To find what kind of parameters you can set, and what type of data it
accepts, check documentation for exact node.

#### Remapping topic name

If you want to change name of topic subscribed or published by node you
can use remapping option. To do this, at the end of the command, add:

    old_topic_name:=new_topic_name

Note that there is no underscore before the old name.

### Starting system step by step - Example

In this section, we will set up ROS system that is equipped with a USB
camera and show image from camera on display. We are going to work only
on PC, you do not need to plug CORE2, Raspberry Pi or any other device
beside USB camera, if you are using laptop, integrated camera will be
OK.

#### Starting master

We will begin with master, to do it type in command line:

    $ roscore

You should see something like this:

![image](https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_1_1.png)

Now you can use tools from chapter 2 in order to examine your system,
don’t worry that you didn’t started any node yet.

#### Examining nodes

Let’s begin with checking the list of existing nodes, in new terminal
type:

    $ rosnode list

As the output you should get:

    husarion@core2-ros:~$ rosnode list 
    /rosout

This means, that you have now one node running, it’s name is `/rosout`
and it is responsible for handling console log mechanism. Next you can
check some info of this node

    $ rosnode info /rosout

And as the output you should get:

    husarion@core2-ros:~$ rosnode info /rosout 
    -------------------------------------
    Node [/rosout]
    Publications: 
     * /rosout_agg [rosgraph_msgs/Log]
     
    Subscriptions: 
     * /rosout [unknown type]
     
    Services: 
     * /rosout/set_logger_level
     * /rosout/get_loggers
     
    contacting node http://core2-ros:48067/ ...
    Pid: 4594

You can see here that node `/rosout` is publishing to topic
`/rosout_agg`, subscribing topic `/rosout` and offer two services:
`/rosout/set_logger_level` and `/rosout/get_loggers`.

#### Examining topics

Now we will check what topics are registered in the system and get some
info about them. In new console type:

    $ rostopic list

You should get in the output:

    husarion@core2-ros:~$ rostopic list 
    /rosout
    /rosout_agg

This means that you have two topics registered in the system. Let’s get
some info about first of them:

    $ rostopic info /rosout

As the output you should get:

    husarion@core2-ros:~$ rostopic info /rosout
    Type: rosgraph_msgs/Log
    
    Publishers: None
    
    Subscribers: 
     * /rosout (http://core2-ros:33119/)

From this you can read, that on topic `/rosout` can be transmitted only
messages of type `rosgraph_msgs/Log`, there is no node that publishes to
this topic and node `/rosout` subscribes it. Now, try to get analogous
info about second topic:

    $ rostopic info /rosout_agg

As the output you should get:

    husarion@core2-ros:~$ rostopic info /rosout_agg 
    Type: rosgraph_msgs/Log
    
    Publishers: 
     * /rosout (http://core2-ros:33119/)
     
    Subscribers: None

From this you can read, that on topic `/rosout_agg` can be transmitted
only messages of type `rosgraph_msgs/Log`, node `/rosout` publishes to
this topic and there is no node that subscribes it.

#### Starting camera node

Now you will run node for handling USB camera device. For this you will
use node `usb_cam_node` from package `usb_cam`. To use this node you may
need to set parameter `video_device` that define which camera you want
to use. You can find list of cameras available in your system by typing
in terminal:

    $ ls /dev/video*

Output should be similar to this:

    husarion@core2-ros:~$ ls /dev/video*
    /dev/video0  /dev/video1

Each entry is for one camera. Above example means that we have two cameras on our system. 
Number of cameras may vary depending on your configuration. This node also supports 
Raspberry Pi camera, even though it is not an USB camera. In this example we will use 
USB camera and its name is `/dev/video0`. Having required information you can start
node by typing into terminal:

    $ rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _image_height:=480 _image_width:=640 
    _pixel_format:=yuyv _framerate:=10

Remember that value of `video_device` and other parameters may vary
depending on which camera you want to use. For a standard USB camera you
can try with default values (just do not set `_image_height`,
`_image_width` and `_pixel_format`) or try to run your camera with
different values. For `_pixel_format`, possible values are `mjpeg`,
`rgb24` and `uyvy`. For `_image_height` and `_image_width` use your
camera resolution.

As output you should get something like below:

![image](https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_1_2.png)


**Task 1** 

Use `rosnode` and `rostopic` tools to check if new nodes or
topic appeared in the system. Next find some info regarding new node and
topics.

#### Starting image view node

Now you have camera node running, but can not see yet image from it. You
will use node `image_view` from `image_view` package. This node by
default subscribe to topic `image`. You need to remap this name to topic
published by camera node. If you performed task 1, you should know that
camera node is publishing to topic `/usb_cam/image_raw`. To run image
view node with remapping topic name type in terminal:

    $ rosrun image_view image_view image:=/usb_cam/image_raw

As the output you should get:

![image](https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_1_3.png)


**Task 2** 

Use `rosnode` and `rostopic` tools to check what changed in
the system after running another node. Notice what changed in properties
of `/usb_cam/image_raw` topic.

#### Examining system with rqt\_graph

Now you will use `rqt_graph` tool in order to get graph of data flow in
the system. In new terminal type:

    $ rqt_graph

There will be no response in terminal, but new window will appear:

![image](https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_1_4.png)

Interpretation of the graph is as follows:

-   Ovals represent node

-   Rectangles represent topics

-   Arrows pointing from node to topic represent publication to this
    topic

-   Arrows pointing from topic to node represent subscription to this
    topic

### Starting system with `roslaunch`

`Roslaunch` is a tool that simplifies running of multiple nodes at one
time. This tool uses `.launch` files that contain configuration of all
nodes to be run. Usage of `rosluanch` is simple, in new terminal type:

    roslaunch package file.launch

or

    roslaunch file.launch

The first one is for the case when you use launch file provided with
package, you can run it from any folder. The second option is when you
use standalone launch file, you must run it in folder where the launch
file is located or give the path to it.

#### Structure of .launch file 

Structure of `.launch` file is defined as a markup language, similar to
HTML. Every `.launch` file must begin with starting marker: `<launch>`
and end with closing one: `<\launch>`. Between them should be placed
markers defining nodes. You can define node by `node` marker, it’s
structure is as follows:

    <node pkg="package_name" type="node" name="id" required="true" output="screen">
    </node>

Where fields `pkg`, `type` and `name` are required, rest are optional.
Meaning of fields:

-   `pkg` - name of package

-   `type` - node to be run

-   `name` - id which will be binded to node

-   `required` - if true, all nodes in the `.launch` file will be
    stopped if this node stops or fails, default value is false.

-   `output` - if value is `screen` node output will be directed to
    screen, if value is `log` output will be directed to log file,
    default is log.

For each node can be set parameters or topics could be remapped.

For setting parameters use marker `param`:

    <param name="name" value="value"/>

Meaning of fields:

-   `name` - name of parameter

-   `value` - desired value of the parameter

For remapping topic names use marker `remap`:

    <remap from="/old" to="/new"/>

Meaning of fields:

-   `from` - old topic name

-   `to` - desired topic name

### Starting system with `roslaunch` - Example

In this section we will start the same nodes as in previous example, but
this time with use of `roslaunch` tool.

At first you will need a `.launch` file.

     <launch>

         <node pkg="usb_cam" type="usb_cam_node" name="usb_cam">
             <param name="video_device" value="/dev/video0"/>
               <param name="image_width" value="640"/>
               <param name="image_height" value="480"/>
               <param name="pixel_format" value="yuyv"/>
               <param name="framerate" value="10"/>
         </node>

         <node pkg="image_view" type="image_view" name="image_view">
             <remap from="/image" to="/usb_cam/image_raw"/>
         </node>

     </launch>

Copy the above code to text editor, adjust value of `video_device` and
other parameters if needed and save it to file `tutorial.launch` in your
home directory.

Next close all consoles and nodes that are already running, go to new
terminal and type:

    $ roslaunch tutorial.launch

You should get output like this:

![image](https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_1_5.png)

Notice that you do not need to run `roscore` before using `roslaunch`,
if `roscore` is not running already, `roslaunch` will run it before
starting nodes. 

**Task 3** 

Use `rosnode`, `rostopic` and `rqt_graph`
tools to examine system started with use of `roslaunch`, there should be
no difference in comparison to system started step by step.

## Summary

After completing this tutorial you should know what are the basic
components and tools of ROS. You should know two methods for starting
node, setting parameters and remapping topic names. You should be able
to check what nodes are running in system and to which topics they are
publishing and subscribing.


---------

*by Łukasz Mitka, AGH Krakow*

*Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com*

