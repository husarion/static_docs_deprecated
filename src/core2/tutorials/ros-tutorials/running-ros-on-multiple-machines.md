---
title: 'Running ROS on multiple machines'
platform_title: 'CORE2'
core2: true
autotoc: true
layout: layout.hbs
order: 5
---

# Running ROS on multiple machines #

## Introduction ##

In this manual you will learn how to configure ROS to work on multiple
computers.You will use this configuration to set up system consisting of
two robots, which perform task of searching an object.

In this manual you will need two robots based on CORE2 with the same
equipment as in previous manual.

## Network setup ##

To run ROS on multiple machines, all of them must be in the same local
network, if necessary, use `hConfig` app to connect all devices to one
network. You will need IP address of every device.

While working on multiple machines, you need only one `roscore` running,
choose one device for it, we will call it `master`.

Now you will set two environmental variables for each device. On
`master` set:

-   **`ROS_MASTER_URI`** with value `http://xxx.xxx.xxx.xxx:11311`

-   **`ROS_IP`** with value `xxx.xxx.xxx.xxx`

Value `xxx.xxx.xxx.xxx` should be IP address of `master`.

On second device (and any further if you want use more machines) set:

-   **`ROS_MASTER_URI`** with value `http://xxx.xxx.xxx.xxx:11311`

-   **`ROS_IP`** with value `yyy.yyy.yyy.yyy`

Value `yyy.yyy.yyy.yyy` should be IP address of device.

Note that **`ROS_MASTER_URI`** has the same value for all machines while
**`ROS_IP`** changes value for each device.

You can set that variables with command `export`:

    $ export ROS_MASTER_URI=http://xxx.xxx.xxx.xxx:11311

    $ export ROS_IP=yyy.yyy.yyy.yyy

Remember that you need to set these variables in each terminal window or
you can use `.bashrc` file to automate it.

With above configuration, nodes running on different machines should be
able to communicate.

**Task 1**

Set configuration for working on multiple machines at two devices. On
one device run only `roscore`, on second run `usb_cam_node` and
`image_view` nodes. Now run `rqt_graph` on both of them, you should see
the same graph.

Next run `image_view` node on machine with `roscore`, you should see
there image from camera mounted on device with `usb_cam_node` running.
Again use `rqt_graph` examine what changed in the system.

## Performing a task with multiple machines ##

In this section we will program two robots for searching objects one
after another. For this we will need one node that manages the search
process, this will run only on one machine. We will also need node that
searches for object, this will run on each device. For recognizing
objects we will use `find_object_2d` node. If you have saved objects
from previous tutorials you can use them, in other case you will need to
teach at least four objects. Remember that both robots need to have the
same image database, you should copy saved images from one device to
another.

### `mission_controller` node ###

In package `tutorial_pkg` in `src` folder create file
`mission_controller.cpp` and open it with text editor.

Begin with headers:

    #include <ros/ros.h>
    #include <std_msgs/Char.h>
    #include <std_srvs/Empty.h>

Publisher for current task:

    ros::Publisher task_pub;

Constants with IDs of objects to be found:

    #define OBJECT_1_ID 1
    #define OBJECT_2_ID 2
    #define HOME_1_ID 3
    #define HOME_2_ID 4

Vector to store sequence of searched objects:

    std::vector<int> objects;

Number of currently searched object:

    uint8_t current_object = 0;

Message for sending id of currently searched object:

    std_msgs::Char task;

Service callback function for reporting found objects:

    bool object_found(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
       current_object++;
    }

In `main` function, define sequence of searched objects:

    objects.push_back(OBJECT_1_ID);
    objects.push_back(OBJECT_2_ID);
    objects.push_back(HOME_1_ID);
    objects.push_back(HOME_2_ID);

Node initialization:

    ros::init(argc, argv, "mission_controller");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);

Register service, here will be reported objects that are found:

    ros::ServiceServer service = n.advertiseService("/object_found", object_found);

Publish topic with ID of currently searched object:

    task_pub = n.advertise<std_msgs::Char>("/task", 1);

In infinite while loop, trigger incoming messages, check ID of currently
searched object and publish it:

    ros::spinOnce();
    loop_rate.sleep();
    if (current_object < objects.size()) {
       task.data = objects[current_object];
    } else {
       // mission finished
       task.data = 0;
    }
    task_pub.publish(task);

Your final file should look like this:

```
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_srvs/Empty.h>

ros::Publisher task_pub;

#define OBJECT_1_ID 8
#define OBJECT_2_ID 9
#define HOME_1_ID 4
#define HOME_2_ID 3

std::vector<int> objects;

uint8_t current_object = 0;
std_msgs::Char task;

bool object_found(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
   current_object++;
}

int main(int argc, char **argv) {
   objects.push_back(OBJECT_1_ID);
   objects.push_back(OBJECT_2_ID);
   objects.push_back(HOME_1_ID);
   objects.push_back(HOME_2_ID);
   ros::init(argc, argv, "mission_controller");
   ros::NodeHandle n("~");
   ros::Rate loop_rate(50);
   ros::ServiceServer service = n.advertiseService("/object_found", object_found);
   task_pub = n.advertise<std_msgs::Char>("/task", 1);
   while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
      if (current_object < objects.size()) {
         task.data = objects[current_object];
      } else {
         // mission finished
         task.data = 0;
      }
      task_pub.publish(task);
   }
}
```

### `search_controller` node ###

In the same package create another file named `search_controller.cpp`
and open it with text editor.

Begin with header files:

    #include <ros/ros.h>
    #include <std_msgs/Float32MultiArray.h>
    #include <geometry_msgs/Twist.h>
    #include <sensor_msgs/Range.h>
    #include <std_msgs/Char.h>
    #include <std_srvs/Empty.h>

Publisher and message for desired velocity:

    ros::Publisher action_pub;
    geometry_msgs::Twist set_vel;

Variables for distance measured by sensors:

    float distL = 0;
    float distR = 0;

Variable for currently searched object ID:

    u_char search_obj;

IDs of objects to be searched by this node:

    int objectID;
    int homeID;

Client for found object reporting service:

    ros::ServiceClient client;

Callbacks for updating distances and object ID:

    void distL_callback(const sensor_msgs::Range &range) {
       distL = range.range;
    }

    void distR_callback(const sensor_msgs::Range &range) {
       distR = range.range;
    }

    void task_callback(const std_msgs::Char &task) {
       search_obj = task.data;
    }

Callback for handling recognized objects, if ID is in accordance with
searched object, report it to service:

    void objectCallback(const std_msgs::Float32MultiArrayPtr &object) {
       if (object->data.size() > 0) {
          if (search_obj == object->data[0]) {
             //object found
             std_srvs::Empty srv;
             client.call(srv);
          }
       }
    }

In main function, node initialization:

    ros::init(argc, argv, "action_controller");
    ros::NodeHandle n("~");

Subscribe to topics:

    ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);
    ros::Subscriber distL_sub = n.subscribe("/rangeL", 1, distL_callback);
    ros::Subscriber distR_sub = n.subscribe("/rangeR", 1, distR_callback);
    ros::Subscriber task_sub = n.subscribe("/task", 1, task_callback);

Get `objectID` and `homeID` params, robot will search only for objects
with these IDs:

    n.param<int>("objectID", objectID, 0);
    n.param<int>("homeID", homeID, 0);

Instantiate client for service:

    client = n.serviceClient<std_srvs::Empty>("/object_found");

Set loop rate:

    ros::Rate loop_rate(10);

Instantiate velocity publisher:

    action_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

Set default values for velocity:

    set_vel.linear.x = 0;
    set_vel.linear.y = 0;
    set_vel.linear.z = 0;
    set_vel.angular.x = 0;
    set_vel.angular.y = 0;
    set_vel.angular.z = 0;

In infinite while loop, trigger incoming messages:

    ros::spinOnce();
    loop_rate.sleep();

If searched object ID comply with this node’s ID, set desired robot
velocity based on sensor measurements:

    if (search_obj == objectID || search_obj == homeID) {
             if (distL > 1) {
                distL = 1;
             }
             if (distR > 1) {
                distR = 1;
             }
             if (distL > 0 && distR > 0) {
                set_vel.angular.z = (distL - distR) * 2;
                set_vel.linear.x = (distL + distR);
             } else if (distL > 0) {
                set_vel.angular.z = -0.5;
                set_vel.linear.x = -0.25;
             } else if (distR > 0) {
                set_vel.angular.z = 0.5;
                set_vel.linear.x = -0.25;
             } else {
                set_vel.linear.x = 1.5;
                set_vel.angular.z = 0;
             }
          } else {
             set_vel.linear.x = 0;
             set_vel.angular.z = 0;
          }
          action_pub.publish(set_vel);

Your final file should look like this:

```
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Char.h>
#include <std_srvs/Empty.h>

ros::Publisher action_pub;
geometry_msgs::Twist set_vel;

float distL = 0;
float distR = 0;

u_char search_obj;
int objectID;
int homeID;

ros::ServiceClient client;

void distL_callback(const sensor_msgs::Range &range) {
   distL = range.range;
}

void distR_callback(const sensor_msgs::Range &range) {
   distR = range.range;
}

void task_callback(const std_msgs::Char &task) {
   search_obj = task.data;
}

void objectCallback(const std_msgs::Float32MultiArrayPtr &object) {
   if (object->data.size() > 0) {
      if (search_obj == object->data[0]) {
         //object found
         std_srvs::Empty srv;
         client.call(srv);
      }
   }
}

int main(int argc, char **argv) {

   ros::init(argc, argv, "action_controller");
   ros::NodeHandle n("~");
   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);
   ros::Subscriber distL_sub = n.subscribe("/rangeL", 1, distL_callback);
   ros::Subscriber distR_sub = n.subscribe("/rangeR", 1, distR_callback);
   ros::Subscriber task_sub = n.subscribe("/task", 1, task_callback);

   n.param<int>("objectID", objectID, 0);
   n.param<int>("homeID", homeID, 0);
   client = n.serviceClient<std_srvs::Empty>("/object_found");

   ros::Rate loop_rate(10);
   action_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
   set_vel.linear.x = 0;
   set_vel.linear.y = 0;
   set_vel.linear.z = 0;
   set_vel.angular.x = 0;
   set_vel.angular.y = 0;
   set_vel.angular.z = 0;
   while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
      if (search_obj == objectID || search_obj == homeID) {
         if (distL > 1) {
            distL = 1;
         }
         if (distR > 1) {
            distR = 1;
         }
         if (distL > 0 && distR > 0) {
            set_vel.angular.z = (distL - distR) * 2;
            set_vel.linear.x = (distL + distR);
         } else if (distL > 0) {
            set_vel.angular.z = -0.5;
            set_vel.linear.x = -0.25;
         } else if (distR > 0) {
            set_vel.angular.z = 0.5;
            set_vel.linear.x = -0.25;
         } else {
            set_vel.linear.x = 1.5;
            set_vel.angular.z = 0;
         }
      } else {
         set_vel.linear.x = 0;
         set_vel.angular.z = 0;
      }
      action_pub.publish(set_vel);
   }
}
```

### Building the nodes ###

To build your nodes you need to edit the `CMakeLists.txt` file. Find
line:

    add_executable(action_controller_node src/action_controller.cpp)

And add two more lines:

    add_executable(mission_controller_node src/mission_controller.cpp)
    add_executable(search_controller_node src/search_controller.cpp)

Then after:

    target_link_libraries(action_controller_node
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            )

Add:

    target_link_libraries(mission_controller_node
            ${catkin_LIBRARIES}
            )

    target_link_libraries(search_controller_node
            ${catkin_LIBRARIES}
            )

Now you can build your nodes, but before you run them, `launch` file for
them will be required.

### Running the nodes ###

To run your nodes you need two `launch` files, one for each robot. First
will be running on one robot:

```
<launch>

    <node pkg="usb_cam" type="usb_cam_node" name="usb_cam">
        <param name="video_device" value="/dev/video0"/>
        <param name="image_width" value="640"/>
        <param name="image_height" value="480"/>
        <param name="pixel_format" value="yuyv"/>
    </node>

    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/usb_cam/image_raw"/>
        <param name="gui" value="false"/>
        <param name="objects_path" value="/home/pi/ros_workspace/find_obj"/>
    </node>

    <node pkg="tutorial_pkg" type="mission_controller_node" name="mission_controller">
    </node>

    <node pkg="tutorial_pkg" type="search_controller_node" name="search_controller">
        <param name="objectID" value="3"/>
        <param name="homeID" value="4"/>
    </node>

</launch>
```

Here you run `usb_cam_node` and `find_object_2d` nodes as in previous
tutorials, you also add nodes that you just created,
`mission_controller_node` without any parameters and
`search_controller_node` with `objectID` and `homeID` parameters that
identify objects which will be searched by robot.

Second `launch` file will be running on another robot:

```
<launch>

    <node pkg="usb_cam" type="usb_cam_node" name="usb_cam_2">
        <param name="video_device" value="/dev/video0"/>
        <param name="image_width" value="640"/>
        <param name="image_height" value="480"/>
        <param name="pixel_format" value="yuyv"/>
    </node>

    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d_2">
        <remap from="image" to="/usb_cam_2/image_raw"/>
        <param name="gui" value="false"/>
        <remap from="objects" to="objects_2"/>
        <param name="objects_path" value="/home/pi/ros_workspace/find_obj"/>
    </node>

    <node pkg="tutorial_pkg" type="search_controller_node" name="search_controller_2">
        <remap from="objects" to="objects_2"/>
        <remap from="cmd_vel" to="cmd_vel_2"/>
        <remap from="rangeL" to="rangeL_2"/>
        <remap from="rangeR" to="rangeR_2"/>        
        <param name="objectID" value="8"/>
        <param name="homeID" value="9"/>
    </node>

</launch>
```

Here you also run `usb_cam_node` and `find_object_2d` nodes, but all
names are remapped with suffix `_2` to distinguish them between both
robots. For `search_controller_node` you will also remap all names and
additionally you should change values of `objectID` and `homeID`
parameters to match different objects from image database.

Remember that for `CORE2` bridge node on second robot you also need to
remap all topic names.

**Task 2** On device that you have chosen for `master` run `roscore`.
Then on one of the robots run first `launch` file with `CORE2` bridge
node:

    $ /opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2

On another robot run second `launch` file with `CORE2` bridge node:

    $ /opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2 __name:=serial_node_2 
    cmd_vel:=cmd_vel_2 rangeL:=rangeL_2 rangeR:=rangeR_2 pose:=pose_2

Observe as one of your robots move avoiding obstacles, when it finds
object, second robot start to search, they should move sequentially
until all objects are recognized.

## Summary ##

After completing this tutorial you should be able to configure your
CORE2 devices to work together and exchange data with each other. You
should also know how to program robots for performing tasks in
cooperation.

*by Łukasz Mitka, AGH Krakow*
