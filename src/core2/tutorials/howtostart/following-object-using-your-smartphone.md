---
title: 'Following object using your smartphone'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 9
---

# Following object using your smartphone #

In this tutorial you will learn how to use smartphone sensors in your robotic design. Husarion created <a href="https://play.google.com/store/apps/details?id=com.husarion.node">hNode Android app</a> that provides smartphone sensors for the robotic system based on ROS. Typical smartphone contains a camera, distance sensor, microphone, accelerometer, gyro and more. Smartphone are currently so common, that in many cases it's cheaper to use them, than buying dedicated sensors for a robot.

As a simple example to show you how to use it, and let you kickstart with your own project, we will present a robot that autonomously follows any object. 

## Architecture of the system ##

The robotic system consists of:
1.  a simple mobile robot platform using DC motors with quadrature encoders. We built it based on <a href="https://husarion.com/core2/manuals/core2/">CORE2-ROS controller</a> and LEGO Mindstorms mobile platform. But you can build any other mechanics, or buy ready to use like <a href="https://www.sparkfun.com/products/10336">this</a> .
2. smartphone running <a href="https://play.google.com/store/apps/details?id=com.husarion.node">hNode app</a>
3. your laptop running Linux, ROS and Husarnet, natively or in a Virtual Machine. <a href="https://files.husarion.com/husarion-vm.vmdk.xz">Here is a ready to use Linux image</a>

Robot, smarthone and laptop are connected using a Husarnet - a VPN, P2P network by Husarion. They form a so called "Virtual Robot". So even if they work in different network, they always know address of each other, and communication between robotic components is very fast, without any external servers etc. You can threat a Virtual Robot as a single robot with internet disrtributed components.

A CORE2-ROS computer has enough computing power to perform robot operation. Laptop is used only to show you that you are no limited to computing power inside your robot.

## Connect virtual robot elements to the cloud ##

### a. smartphone ###

1. Log into <a href="https://cloud.husarion.com">Husarion cloud in the browser</a>, click "Add new", choose a name for your smartphone (eg. "myPhone1"), and click "Next". You will see a QR code.
2. Install <a href="https://play.google.com/store/apps/details?id=com.husarion.node">hNode Android app</a> on your smartphone, open it, click "SCAN QR CODE" button, and scan QR code from your web browser
3. On your Husarion cloud account you should see:

<img src="/assets/img/husarnet/cloud_unasigned_phone.PNG" alt="unasigned phone" border="1">


### b. virtual machine running Linux ###

1. Using <a href="https://www.virtualbox.org/">VirtualBox</a> (or other virtual machine hypervisor) open a <a href="https://files.husarion.com/husarion-vm.vmdk.xz">ready to use machine provided by Husarion</a> 
```
To make sure you have all packet up to date run the following commands in the terminal inside virtual machine:
>sudo apt-get update
>sudo apt-get dist-upgrade
```

2. Run that virutal machine, open a terminal inside, and type:
`sudo husarnet websetup`
3. open a link that will appear in the web browser:

<img src="/assets/img/husarnet/husarnet_websetup.PNG" alt="husarnet websetup" border="1">
    
4. you will see a web panel. Then choose a name for your virtual machine, eg. "ubuntu-vm". In section "Join virtual robot" select "None" and click "Add device to your account" button.
5. Now on your Husarion cloud account you should see two unasigned elements:

<img src="/assets/img/husarnet/cloud_unasigned_vm.PNG" alt="cloud unasigned" border="1">

```
Ready to use virual machine is prepared for your convenience. 

If you would like to connect any Linux machine here is the manual: https://todo
```

### c. robot based on CORE2-ROS controller ###

1. Install <a href="https://files.husarion.com/ros-image-stable.img.xz">the newest Linux image for CORE2-ROS</a>  on the micro SD card (<a href="https://etcher.io/">Etcher</a> is a great tool to do this). Then attach that micro SD card back to your CORE2-ROS single board computer slot.
2. Click hCfg button and turn CORE2-ROS on. Hold hCfg pressed until LR1 and LR2 will start blinking alternately
3. Log into <a href="https://cloud.husarion.com">Husarion cloud in the browser</a>, click "Add new", choose a name for your robot (eg. "core2-robot"), and click "Next". You will see a QR code.
4. Install hConfig - Husarion config app for  <a href="https://play.google.com/store/apps/details?id=com.husarion.configtool2">Android</a> or  <a href="https://itunes.apple.com/us/app/hconfig/id1283536270?mt=8">iOS</a> . 
5. Connect your smartphone to a Wi-Fi network hosted by your CORE2-ROS (password: "husarion") and run hConfig. Wizard will guide you through a configuration process. On the last step you will scan a QR code displayed in the web browser (see step 3.)
6. After the process you should see a "core2-robot" in your Husarion cloud account:

<img src="/assets/img/husarnet/cloud_robot.PNG" alt="cloud robot" border="1">


## Create a Virtual Robot ##

1. Click "Edit" button next to "ubuntu-vm" and select "Link to virtual robot". Select "Create new virtual robot" and choose a name, eg. "myVirtualRobot"

<img src="/assets/img/husarnet/virtual_robot.PNG" alt="virtual robot 1" border="1">

2. Click "Add" button and you will see:

<img src="/assets/img/husarnet/virtual_robot2.PNG" alt="virtual robot 2" border="1">

3. Now click "Add component" and add your smartphone and robot to a "myVirtualRobot". When work will be done you will see:

<img src="/assets/img/husarnet/virtual_robot3.PNG" alt="virtual robot 3" border="1">


## Using Husarnet with the Cloud ##

Now we will show you example of using Husarnet. We will create network with Virtual Machine (hostname ubuntu-vm), CORE2ROS (hostname-core2ros) and smartphone (hostname smartphone). 
After adding all three device to your Virtual Robot following to the above instructions, your network should look like this:

![image](/assets/img/husarnet/husarnet_5.png)

TIP: Now you can use:

```
ping6 hostname
```
to ping device, and:

```
ssh hostname
```
to communicate with device.


You must choose which device will be ROS_MASTER. You can do it by expanding the bar below the list of devices connected to the network and clicking set. In my network it will be laptop with Ubuntu image running on Virtual Machine. 

![image](/assets/img/husarnet/husarnet_7.png)

On start we have to program our CORE2-ROS with this code using Husarion cloud or VS Code:

```
#include "hFramework.h"
#include "hCloudClient.h"
#include <ros.h>
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

using namespace hFramework;

ros::NodeHandle nh;
geometry_msgs::PoseStamped pose;
ros::Publisher pose_pub("/pose", &pose);

uint16_t delay = 50; // milliseconds
float delay_s = (float)delay / (float)1000;
uint16_t enc_res = 1000; // encoder tics per wheel revolution

int32_t enc_L = 0;           // encoder tics
float wheel_L_ang_pos = 0; // radians
float wheel_L_ang_vel = 0; // radians per second

int32_t enc_R = 0;           // encoder tics
float wheel_R_ang_pos = 0; // radians
float wheel_R_ang_vel = 0; // radians per second

float robot_angular_pos = 0; // radians
float robot_angular_vel = 0; // radians per second

float robot_x_pos = 0; // meters
float robot_y_pos = 0; // meters
float robot_x_vel = 0; // meters per second
float robot_y_vel = 0; // meters per second

float robot_width = 0.2;    // meters
float robot_length = 0.105; //meters
float wheel_radius = 0.02;  //meters

void twistCallback(const geometry_msgs::Twist &twist)
{
    float lin = twist.linear.x;
    float ang = twist.angular.z;
    float motorL = lin - ang * 0.5;
    float motorR = lin + ang * 0.5;
    hMot1.setPower(motorR * (200));
    hMot2.setPower(motorL * (200));
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &twistCallback);

void hMain()
{
    platform.begin(&RPi);
    nh.getHardware()->initWithDevice(&platform.LocalSerial);
    nh.initNode();
    nh.subscribe(sub);
    hMot1.setEncoderPolarity(Polarity::Reversed);
    hMot2.setEncoderPolarity(Polarity::Reversed);
    hMot1.setSlewRate(0.01);
    hMot2.setSlewRate(0.01);
    LED1.on();

    pose.header.frame_id = "robot";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionFromYaw(0);

    nh.advertise(pose_pub);

    while (true)
    {
        enc_L = hMot2.getEncoderCnt();
        enc_R = hMot1.getEncoderCnt();

        wheel_L_ang_vel = ((2 * 3.14 * enc_L / enc_res) - wheel_L_ang_pos) / delay_s;
        wheel_R_ang_vel = ((2 * 3.14 * enc_R / enc_res) - wheel_R_ang_pos) / delay_s;

        wheel_L_ang_pos = 2 * 3.14 * enc_L / enc_res;
        wheel_R_ang_pos = 2 * 3.14 * enc_R / enc_res;

        robot_angular_vel = (((wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / robot_width) -
                             robot_angular_pos) / delay_s;
        robot_angular_pos = (wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / robot_width;

        robot_x_vel = (wheel_L_ang_vel * wheel_radius + robot_angular_vel * robot_width / 2) *
                      cos(robot_angular_pos);
        robot_y_vel = (wheel_L_ang_vel * wheel_radius + robot_angular_vel * robot_width / 2) *
                      sin(robot_angular_pos);

        robot_x_pos = robot_x_pos + robot_x_vel * delay_s;
        robot_y_pos = robot_y_pos + robot_y_vel * delay_s;

        pose.pose.position.x = robot_x_pos;
        pose.pose.position.y = robot_y_pos;
        pose.pose.orientation = tf::createQuaternionFromYaw(robot_angular_pos);
        pose_pub.publish(&pose);

        nh.spinOnce();
        sys.delay(delay);
    }
}

```

Now open terminal on your ROS_MASTER device and run:

```
roscore
```

![image](/assets/img/husarnet/console_1.png)

In second one connect yourself with CORE2-ROS by ssh. Use

```
ssh yourCORE2ROShostname
```

then answer `yes` and type password. You will see Husarion graphic, it's mean that you are connected to CORE2-ROS terminal.

![image](/assets/img/husarnet/console_2.png)

Next type:

```
/opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2
```
 
to start communication between CORE2 and linux SBC. You will see the list of publishers and subscribers. 

[obraz pokazuj¹cy wynik komendy /opt/...]

Now we will transport video stream to our local topic to make more conveniente to use it. Open new tab in your terminal and type:

`rosrun image_transport republish compressed in:=/yourphonehostname/camera1/image out:=/localimg`

You have to create your own ROS node. The full instruction of creating workspace and nodes and all explanations you can find <a href="https://husarion.com/core2/tutorials/ros-tutorials/2-creating-nodes/">here</a>. 

Open console on device you want to create workspace and type:

```
mkdir ~/ros_workspace
mkdir ~/ros_workspace/src
cd ~/ros_workspace/src
catkin_init_workspace
cd ~/ros_workspace
catkin_make
```
Waite until finish. You should see:

[obraz wyniku catkin_make]

```
####
#### Running command: "make -j4 -l4" in "/home/pi/ros_workspace/build"
####
```

in console after it. You should also edit file .bashrc to make possible using your own node in any directory. You can do it by:

`nano ~/.bashrc`

When you will see .bashrc file content, go to the end and add line:

`. /home/husarion/ros_workspace/devel/setup.sh`

Save the file and close editor.

Now we will create our own package. Note that your newly created package should depend on package roscpp. 

```
cd ~/ros_workspace/src
catkin_create_pkg tutorial_pkg roscpp
```

New package will also contain opencv libraries, so it's important to modify the CMakeList.txt file from tutorial_pkg directory properly:

At the beginning find line:
```
# add_compile_options(-std=c++11)
```
and uncomment it (remove # sign). Find also:
```
find_package(catkin REQUIRED COMPONENTS
 roscpp
)
```
and modify it like below:
```
find_package(catkin REQUIRED COMPONENTS
 roscpp tf
)

find_package(OpenCV REQUIRED)
```
Next find line:
```
# add_executable(${PROJECT_NAME}_node src/tutorial_pkg_node.cpp)
```
and add after it:
```
add_executable(object_follower src/object_follower.cpp)
```
Find also:
```
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
```
and add after it:
```
target_link_libraries(object_follower
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
```

Now you have to add some object to detect. The manual how to save objects from scene you can find <a href="https://husarion.com/core2/tutorials/ros-tutorials/4-visual-object-recognition/">here</a>.

Now create source file for your node:

```
cd ~/ros_workspace/src/tutorial_pkg/src/
touch object_follower.cpp
```

This node has previously saved object for recognition in this case `#define OBJECT 1`. Remember that number of your object have to be the same as number of object define in line number 7. Open file object_follower.cpp and paste this code:

```
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#define OBJECT 1
int id = 0;
ros::Publisher action_pub;
geometry_msgs::Twist set_vel;
int camera_center = 240; // down 0, up 480
float max_ang_vel = 0.5;
void objectCallback(const std_msgs::Float32MultiArrayPtr &object) {
   if (object->data.size() > 0) {
      id = object->data[0];
      float objectWidth = object->data[1];
      float objectHeight = object->data[2];
      float x_pos;
      float speed_coefficient = (float) camera_center / max_ang_vel;
      // Find corners OpenCV
      cv::Mat cvHomography(3, 3, CV_32F);
      std::vector<cv::Point2f> inPts, outPts;
      switch (id) {
         case OBJECT:
            cvHomography.at<float>(0, 0) = object->data[3];
            cvHomography.at<float>(1, 0) = object->data[4];
            cvHomography.at<float>(2, 0) = object->data[5];
            cvHomography.at<float>(0, 1) = object->data[6];
            cvHomography.at<float>(1, 1) = object->data[7];
            cvHomography.at<float>(2, 1) = object->data[8];
            cvHomography.at<float>(0, 2) = object->data[9];
            cvHomography.at<float>(1, 2) = object->data[10];
            cvHomography.at<float>(2, 2) = object->data[11];
            inPts.push_back(cv::Point2f(0, 0));
            inPts.push_back(cv::Point2f(objectWidth, 0));
            inPts.push_back(cv::Point2f(0, objectHeight));
            inPts.push_back(cv::Point2f(objectWidth, objectHeight));
            cv::perspectiveTransform(inPts, outPts, cvHomography);
            x_pos = (int) (outPts.at(0).y + outPts.at(1).y + outPts.at(2).y + 
            outPts.at(3).y) / 4;
            set_vel.angular.z = -(x_pos - camera_center) / speed_coefficient;
        set_vel.linear.x = 1;
            break;
         default: // other object
            set_vel.linear.x = 0;
            set_vel.angular.z = 0;
      }
      action_pub.publish(set_vel);
   } else {
      // No object detected
      set_vel.linear.x = 0;
      set_vel.angular.z = 0;
      action_pub.publish(set_vel);
   }
}
int main(int argc, char **argv) {
   std_msgs::String s;
   std::string str;
   str.clear();
   str.append("");
   std::to_string(3);
   s.data = str;
   ros::init(argc, argv, "object_follower");
   ros::NodeHandle n("~");
   ros::Subscriber sub = n.subscribe("/objects", 5, objectCallback);
   ros::Rate loop_rate(100);
   action_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
   set_vel.linear.x = 0;
   set_vel.linear.y = 0;
   set_vel.linear.z = 0;
   set_vel.angular.x = 0;
   set_vel.angular.y = 0;
   set_vel.angular.z = 0;
   while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
   }
}
```

Now change number "1" to your object number in line number 7:

```
#define OBJECT 1
```

Go to your workspace directory one more time and build your new node:

```
cd ~/ros_workspace
catkin_make
```

[obraz dla wyniku catkin_make z node object_follower]

Now we will create find.launch file. Type in console:

```
cd ~/Desktop
touch find.launch
```

Open file find.launch and paste:

```
<launch>
    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/localimg" />
        <param name="gui" value="true" />
        <param name="objects_path" value="/home/husarion/ros_workspace/object" />
    </node>
    
    <node pkg="tutorial_pkg" type="object_follower" name="object_follower">
    </node>

</launch>

```

Open new tab in terminal, reach directory of your launch file and type 

```
cd ~/Desktop
roslaunch find.launch
```

[obraz z wywo³anym plikem .launch]

You will see find_object_2d GUI with your saved object.

[obraz z GUI ze znalezionym obiektem] 

Now you can try put saved object in the field of view of the camera and observe the behaving of your robot.

After robot has found the objects, it will follow them, as you can see on video below:

[[[[[video]]]]]

## Camera support ##

If you want to use camera image from your smartphone, just install compressed image transport:

```apt-get install -y ros-kinetic-compressed-image-transport```

To view image from camera locally, use:

```rosrun image_view image_view image:=/yourphonehostname/camera0/image _image_transport:=compressed```

Make sure to accept permission dialog on the device. Use camera1 instead of camera0 to access second camera, if you have one.

If you have some node that can't process compressed image, launch the node that decompresses it locally (republisher):

``` rosrun image_transport republish compressed in:=/yourphonehostname/camera0/image out:=/localimg ```

After that, raw image will be available in `/localimg` channel.
