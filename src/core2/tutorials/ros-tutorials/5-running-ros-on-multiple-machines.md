---
title: '5 Running ROS on multiple machines'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 5
---

# Running ROS on multiple machines #

## Introduction ##

In this manual you will learn how to configure ROS to work on multiple
computers.You will use this configuration to set up system consisting of
two robots, which perform task of searching an object.

In this manual you will need two robots based on CORE2 with the same equipment as in the previous manual. 

In case you are working on **Gazebo** simulator, it is not possible to setup system to work on multiple computers, altough you can simulate many ROSbots in simulator running on one machine. Just skip the section **Network setup** and proceed to **Performing a task with multiple machines**.

## Network setup ##

To run ROS on multiple machines, all of them must be in the same local
network- if necessary, use `hConfig` app to connect all devices to one
network. You will need IP address of every device.

While working on multiple machines, you need only one `roscore` running.
Choose one device for it- we will call it `master`.

To connect our devices in one network we will use Husarnet. Full instructions 
for the use of Husarnet are described in the [husanet tutorial](https://husarion.com/tutorials/husarnet/following-object-using-your-smartphone).

If you already connected both your robots to Husarion cloud you can go to the next step, if not, do it before proceeding.

Now log in to the husarion cloud and look at the bottom of the page. You should see heading `Unassigned device` and at least two device.

Create Husarnet network by clicking `Link to virtual robot` and choosing `Create new virtual robot` and type in a name for your robot. This will create a new Husarnet network.
Next enter your virtual robot and click `Add component`, then choose correct device from the drop down list and click `Add device`. 

Now you just have to choose which one will be your ROS master. With above configuration, nodes running on different machines will be able to communicate.

TIP! Remember that `roscore` must be running on the device indicated as ROS master!!!

**Task 1**

Set configuration for working on multiple machines on two devices. On
the first device run only `roscore`, on the second run `astra.launch` and
`image_view` nodes. Now run `rqt_graph` on both of them, you should see
the same graph.

Next run `image_view` node on machine with `roscore`. You should see
the image from camera mounted on the device with `astra.launch` running.
Again use `rqt_graph` to examine what changed in the system.

## Performing a task with multiple machines ##

In this section we will program two robots to search objects one
after another. To do that we will need one node which manages the search
process- it will run only on one machine. We will also need a node that
searches for the object- it will run on both devices. For recognizing
objects we will use `find_object_2d` node. If you have saved objects
from previous tutorials you can use them, in other case you will need to
teach at least four objects. Remember that both robots need to have the
same image database, you should copy saved images from one device to
another.

### `mission_controller` node ###

In `tutorial_pkg` package in `src` folder create file
`mission_controller.cpp` and open it with a text editor.

Begin with the headers:

``` cpp
    #include <ros/ros.h>
    #include <std_msgs/Char.h>
    #include <std_srvs/Empty.h>
``` 

Publisher for current task:

``` cpp
    ros::Publisher task_pub;
``` 

Constants with IDs of objects to be found:

``` cpp
    #define OBJECT_1_ID 8
    #define OBJECT_2_ID 6
    #define HOME_1_ID 12
    #define HOME_2_ID 11
``` 

Vector to store sequence of searched objects:

``` cpp
    std::vector<int> objects;
``` 
    
Number of currently searched object:

``` cpp
    uint8_t current_object = 0;
``` 

Message for sending id of currently searched object:

``` cpp
    std_msgs::Char task;
``` 

Service callback function for reporting found objects:

``` cpp
    bool object_found(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        current_object++;
        ROS_INFO("Current object: %d", current_object);
        return true;
    }
``` 

In `main` function, definining sequence of searched objects:

``` cpp
    objects.push_back(OBJECT_1_ID);
    objects.push_back(OBJECT_2_ID);
    objects.push_back(HOME_1_ID);
    objects.push_back(HOME_2_ID);
``` 

Node initialization:

``` cpp
    ros::init(argc, argv, "mission_controller");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);
``` 

Registering the service- objects that are found will be reported here:

``` cpp
    ros::ServiceServer service = n.advertiseService("/object_found", object_found);
``` 

Publishing topic with ID of currently searched object:

``` cpp
    task_pub = n.advertise<std_msgs::Char>("/task", 1);
``` 

In infinite while loop- triggering incoming messages, checking ID of currently
searched object and publishing it:

``` cpp
    ros::spinOnce();
    loop_rate.sleep();
    if (current_object < objects.size()) {
       task.data = objects[current_object];
    } else {
       // mission finished
       task.data = 0;
    }
    task_pub.publish(task);
``` 

Your final file should look like this:

``` cpp
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_srvs/Empty.h>

ros::Publisher task_pub;

#define OBJECT_1_ID 8
#define OBJECT_2_ID 6
#define HOME_1_ID 12
#define HOME_2_ID 11

std::vector<int> objects;

uint8_t current_object = 0;
std_msgs::Char task;

bool object_found(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    current_object++;
    ROS_INFO("Current object: %d", current_object);
    return true;
}

int main(int argc, char **argv)
{
    objects.push_back(OBJECT_1_ID);
    objects.push_back(OBJECT_2_ID);
    objects.push_back(HOME_1_ID);
    objects.push_back(HOME_2_ID);
    ros::init(argc, argv, "mission_controller");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);
    ros::ServiceServer service = n.advertiseService("/object_found", object_found);
    task_pub = n.advertise<std_msgs::Char>("/task", 1);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        if (current_object < objects.size())
        {
            task.data = objects[current_object];
        }
        else
        {
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

Begin with the header files:

``` cpp
    #include <ros/ros.h>
    #include <std_msgs/Float32MultiArray.h>
    #include <geometry_msgs/Twist.h>
    #include <sensor_msgs/Range.h>
    #include <std_msgs/Char.h>
    #include <std_srvs/Empty.h>
``` 

Publisher and message for desired velocity:

``` cpp
    ros::Publisher action_pub;
    geometry_msgs::Twist set_vel;
``` 

Variables for distance measured by sensors:

``` cpp
    float distL = 0;
    float distR = 0;
``` 

Variables for storing maximum sensor range:

```cpp
    float sensorL_max = 0;
    float sensorR_max = 0;
```

Variable for currently searched object ID:

``` cpp
    u_char search_obj;
``` 

IDs of objects to be searched by this node:

``` cpp
    int objectID;
    int homeID;
``` 

Client for found object reporting service:

``` cpp
    ros::ServiceClient client;
``` 

Callbacks for updating distances and object ID:

``` cpp
    void distL_callback(const sensor_msgs::Range &range) {
       distL = range.range;
       sensorL_max = range.max_range;
    }

    void distR_callback(const sensor_msgs::Range &range) {
       distR = range.range;
       sensorR_max = range.max_range;
    }

    void task_callback(const std_msgs::Char &task) {
       search_obj = task.data;
    }
```

Callback for handling recognized objects, if ID is in accordance with
searched object, reporting it to service:

``` cpp
    void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
    {
        if (object->data.size() > 0)
        {
            if (search_obj == object->data[0])
            {
                ROS_INFO("Object found, call service %s", client.getService().c_str());
                std_srvs::Empty srv;
                client.call(srv);
            }
        }
    }
``` 

In main function, node initialization:

``` cpp
    ros::init(argc, argv, "action_controller");
    ros::NodeHandle n("~");
``` 

Subscribing to topics:

``` cpp
    ros::Subscriber sub = n.subscribe("objects", 1, objectCallback);
    ros::Subscriber distL_sub = n.subscribe("range/fl", 1, distL_callback);
    ros::Subscriber distR_sub = n.subscribe("range/fr", 1, distR_callback);
    ros::Subscriber task_sub = n.subscribe("/task", 1, task_callback);
``` 

Getting `objectID` and `homeID` params, robot will search only for objects
with these IDs:

``` cpp
    n.param<int>("objectID", objectID, 0);
    n.param<int>("homeID", homeID, 0);
``` 

Initiating client for the service:

``` cpp
    client = n.serviceClient<std_srvs::Empty>("/object_found");
``` 

Setting loop rate:

``` cpp
    ros::Rate loop_rate(10);
``` 

Initiating velocity publisher:

``` cpp
    action_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
``` 

Setting default values for velocity:

``` cpp
    set_vel.linear.x = 0;
    set_vel.linear.y = 0;
    set_vel.linear.z = 0;
    set_vel.angular.x = 0;
    set_vel.angular.y = 0;
    set_vel.angular.z = 0;
``` 

In infinite while loop, triggering incoming messages:

``` cpp
    ros::spinOnce();
    loop_rate.sleep();
``` 

If searched object ID complies with this node’s ID, setting desired robot
velocity based on sensor measurements:

``` cpp
    if (search_obj == objectID || search_obj == homeID)
    {
        if (distL > 1)
        {
            distL = 1;
        }
        if (distR > 1)
        {
            distR = 1;
        }
        if (distL > 0 && distR > 0)
        {
            set_vel.angular.z = (distL - distR) * 2;
            set_vel.linear.x = ((distL + distR) / 2) - ((sensorL_max +sensorR_max) / 4);
        }
        else if (distL > 0)
        {
            set_vel.angular.z = -0.25;
            set_vel.linear.x = -0.125;
        }
        else if (distR > 0)
        {
            set_vel.angular.z = 0.25;
            set_vel.linear.x = -0.125;
        }
        else
        {
            set_vel.linear.x = 0.25;
            set_vel.angular.z = 0;
        }
    }
    else
    {
        set_vel.linear.x = 0;
        set_vel.angular.z = 0;
    }
    action_pub.publish(set_vel);
``` 

Your final file should look like this:

``` cpp
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
float sensorL_max = 0;
float sensorR_max = 0;

u_char search_obj;
int objectID;
int homeID;

ros::ServiceClient client;

void distL_callback(const sensor_msgs::Range &range)
{
    distL = range.range;
    sensorL_max = range.max_range;
}

void distR_callback(const sensor_msgs::Range &range)
{
    distR = range.range;
    sensorR_max = range.max_range;
}

void task_callback(const std_msgs::Char &task)
{
    search_obj = task.data;
}

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
    if (object->data.size() > 0)
    {
        if (search_obj == object->data[0])
        {
            ROS_INFO("Object found, call service %s", client.getService().c_str());
            std_srvs::Empty srv;
            client.call(srv);
        }
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "action_controller");
    ros::NodeHandle n("~");
    ros::Subscriber sub = n.subscribe("objects", 1, objectCallback);
    ros::Subscriber distL_sub = n.subscribe("range/fl", 1, distL_callback);
    ros::Subscriber distR_sub = n.subscribe("range/fr", 1, distR_callback);
    ros::Subscriber task_sub = n.subscribe("/task", 1, task_callback);

    n.param<int>("objectID", objectID, 0);
    n.param<int>("homeID", homeID, 0);
    client = n.serviceClient<std_srvs::Empty>("/object_found");

    ros::Rate loop_rate(10);
    action_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    set_vel.linear.x = 0;
    set_vel.linear.y = 0;
    set_vel.linear.z = 0;
    set_vel.angular.x = 0;
    set_vel.angular.y = 0;
    set_vel.angular.z = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        if (search_obj == objectID || search_obj == homeID)
        {
            if (distL > 1)
            {
                distL = 1;
            }
            if (distR > 1)
            {
                distR = 1;
            }
            if (distL > 0 && distR > 0)
            {
                set_vel.angular.z = (distL - distR) * 2;
                set_vel.linear.x = ((distL + distR) / 2) - ((sensorL_max + sensorR_max) / 4);
            }
            else if (distL > 0)
            {
                set_vel.angular.z = -0.25;
                set_vel.linear.x = -0.125;
            }
            else if (distR > 0)
            {
                set_vel.angular.z = 0.25;
                set_vel.linear.x = -0.125;
            }
            else
            {
                set_vel.linear.x = 0.25;
                set_vel.angular.z = 0;
            }
        }
        else
        {
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

### Running the nodes on ROSbots ###

To run your nodes you need two `launch` files, one for each robot. First
will be running on one robot:

``` launch
<launch>

    <include file="$(find astra_launch)/launch/astra.launch"/>

    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/camera/rgb/image_raw"/>
        <param name="gui" value="true"/>
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

Here you are runing `astra.launch` and `find_object_2d` nodes as in previous
tutorials. You are also adding nodes that you just created,
`mission_controller_node` without any parameters and
`search_controller_node` with `objectID` and `homeID` parameters that
identify objects which will be searched for by robot.

Second `launch` file will be running on another robot:

``` launch
<launch>

    <include file="$(find astra_launch)/launch/astra.launch">
        <arg name="camera" value="camera_2" />
    </include>

    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d_2">
        <remap from="image" to="/camera/rgb/image_raw_2"/>
        <param name="gui" value="true"/>
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

Here you are also running `astra.launch` and `find_object_2d` nodes, but all
names are remapped with suffix `_2` to distinguish them between both
robots. For `search_controller_node` you will also be remapping all names and
additionally you should change values of `objectID` and `homeID`
parameters to match different objects from image database.

Remember that for `CORE2` bridge node on the second robot you also need to
remap all topic names.

**Task 2** On the device that you have chosen for `master` run `roscore`.
Then on one of the robots run first `launch` file with `CORE2` bridge
node:

    $ /opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2

On another robot run second `launch` file with `CORE2` bridge node:

    $ /opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2 __name:=serial_node_2 
    cmd_vel:=cmd_vel_2 rangeL:=rangeL_2 rangeR:=rangeR_2 pose:=pose_2

Observe as one of your robots moves avoiding obstacles. When it finds an
object, second robot starts seearching. They should move sequentially
until all objects are recognized.

### Running the nodes in Gazebo ###

Gazego will be running on one machine, thus you will use only one launch file. In this case you will use namespaces to distinguish robots, each robot will publish and subscribe topics with its own prefix. Only the `mission_controller_node` will be working without any namespace to communicate with both robots. All parameters for nodes must be set with the same values as for ROSbots.

You can use below launch file:

```launch
<launch>

    <include file="$(find rosbot_gazebo)/launch/world.launch"/>

    <param name="robot_description" command="$(find xacro)/xacro.py '$(find rosbot_description)/urdf/rosbot.xacro'"/>

    <node name="rosbot_spawn_first" pkg="gazebo_ros" type="spawn_model" output="log" args="-urdf -param robot_description -model rosbot_s -y 0 -namespace first/search_controller">
    </node>

    <node name="rosbot_spawn_second" pkg="gazebo_ros" type="spawn_model" output="log" args="-urdf -param robot_description -model rosbot -y 2.5 -namespace second/search_controller">
    </node>

    <node pkg="tutorial_pkg" type="mission_controller_node" name="mission_controller" output="screen">
    </node>

    <node ns="first/search_controller" pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="camera/rgb/image_raw"/>
        <param name="gui" value="true"/>
        <param name="objects_path" value="$(find tutorial_pkg)/image_rec/"/>
    </node>

    <node ns="first" pkg="tutorial_pkg" type="search_controller_node" name="search_controller" output="screen">
        <param name="objectID" value="8"/>
        <param name="homeID" value="12"/>
    </node>

    <node ns="second/search_controller" pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="camera/rgb/image_raw"/>
        <param name="gui" value="true"/>
        <param name="objects_path" value="$(find tutorial_pkg)/image_rec/"/>
    </node>

    <node ns="second" pkg="tutorial_pkg" type="search_controller_node" name="search_controller">
        <param name="objectID" value="6"/>
        <param name="homeID" value="11"/>
    </node>

</launch>

```



## Summary ##

After completing this tutorial you should be able to configure your
CORE2 devices to work together and exchange data with each other. You
should also know how to program robots for performing tasks in
cooperation.

---------

*by Łukasz Mitka, AGH Krakow*

*Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com*
