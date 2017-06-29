---
title: '4 Visual object recognition'
platform_title: 'CORE2'
core2: true
autotoc: true
layout: layout.hbs
order: 4
---

# Visual object recognition #

## Introduction ##

Objects can ce recognized by robot with use of vision system. It is
based on image characteristics like points, lines, edges colours and
their relative positions.

Process of object recognition consists of two steps. First of them is
teaching and should be executed before main robot operation, at this
step object is presented to vision system, image and extracted set of
features are saved as a pattern. Many objects can be presented to
system.

Second step is actual recognition, this is executed constantly during
robot operation. Every frame of camera is processed, image features are
extracted and compared to data set in memory. If enough features matches
pattern, then object is recognized.

In this tutorial we will use `find_object_2d` node from `find_object_2d`
package for both teaching and recognition.

As an image source we will use `usb_cam` node as in tutorial 1.

## Teaching objects ##

Object to recognize could be anything, but remember, that the more edges
and contrast colours it has, the easier it will be recognized. A piece
of paper with something draw on it would be enough for tutorial.

Now you should run `find_object_2d` and `usb_cam` node. Node
`find_object_2d` by default subscribe to `image` topic, you should remap
it to topic published by `usb_cam` node. We will also decrease the
camera framerate, as it is not necessary to check for objects so often
and it will use less CPU making interface more responsive.

You can use below `launch` file:

```
<launch>

    <node pkg="usb_cam" type="usb_cam_node" name="usb_cam">
        <param name="video_device" value="/dev/video0"/>
        <param name="image_width" value="640"/>
        <param name="image_height" value="480"/>
        <param name="pixel_format" value="yuyv"/>
        <param name="framerate" value="2"/>
    </node>

    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/usb_cam/image_raw"/>
        <param name="gui" value="true"/>
    </node>

</launch>
```

After launching the `find_object_2d` node with properly adjusted image
topic, new window should appear:

![image](/assets/img/ros/man_4_1.png)

On the left side of the window there are thumbnails of saved images
(should be empty at first run). Application main window contain camera
view. Yellow circles on the image are marking extracted image features.

To begin teaching process choose from the main toolbar **`Edit`**
→ **`Add object from scene...`.** New window will appear:

![image](/assets/img/ros/man_4_2.png)

Now move the object and camera in order to cover as many features of
object as possible, also try not to catch object surrounding. When it’s
done, click **`Take picture`**.

![image](/assets/img/ros/man_4_3.png)

In next view click **`Select region`** and select only that part of
taken picture, that covers desired object and click **`Next`**.

![image](/assets/img/ros/man_4_4.png)

You will get confirmation of features extracted from selected image
region. If presented image is in accordance with what you selected,
click **`End`**

![image](/assets/img/ros/man_4_5.png)

You should see new thumbnail in left panel. Notice the number outside of
parentheses on the left of image, this is object ID.

Now you can add some more objects to recognize, remember their IDs, you
will need them later:

![image](/assets/img/ros/man_4_6.png)

When you have enough objects in database choose from the main toolbar
**`File`** → **`Save objects...`** and choose folder to
store recognized objects. Close the window and stop all running nodes.

## Recognizing objects ##

Objects are recognized by the same node as for teaching, but it will
work in slightly different configuration. We will set two new parameters
for node. For parameter `gui` we will set value `false`, this will run
node without window for learning objects as we no longer need it.
Another parameter will be `objects_path`, this should be path to folder
that you just chosen to store recognized objects.

You can use below `launch` file:

```
<launch>

    <node pkg="usb_cam" type="usb_cam_node" name="usb_cam">
        <param name="video_device" value="/dev/video0"/>
        <param name="image_width" value="640"/>
        <param name="image_height" value="480"/>
        <param name="pixel_format" value="yuyv"/>
        <param name="framerate" value="2"/>
    </node>

    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/usb_cam/image_raw"/>
        <param name="gui" value="false"/>
        <param name="objects_path" value="/home/pi/ros_workspace/find_obj"/>
    </node>

</launch>
```

Node is publishing to `/objects` topic with message type
`std_msgs/Float32MultiArray`. Data in this message is covering: object
ID, size of recognized object and its orientation. The most interesting
for us is object ID, this is first element of array.

Whenever object is recognized, it will be published in that topic. Place
different objects in front of camera and observe as their data is
published in the topic.

To watch messages published in the topic, you can use `rostopic` tool:

    $ rostopic echo /objects

## Making decision with recognized object ##

To perform a robot action based on recognized object, we will make a new
node. It’s task will be to subscribe `/objects` topic and publish
message to `/cmd_vel` with speed and direction depending on object.

Create a new file, name it `action_controller.cpp` and place it in `src`
folder under `tutorial_pkg`. Then open it in text editor and paste:

```
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

#define SMILE 4
#define ARROW_LEFT 3
#define ARROW_UP 5
#define ARROW_DOWN 6

int id = 0;
ros::Publisher action_pub;
geometry_msgs::Twist set_vel;

void objectCallback(const std_msgs::Float32MultiArrayPtr &object) {
   if (object->data.size() > 0) {
      id = object->data[0];

      switch (id) {
         case ARROW_LEFT:
            set_vel.linear.x = 0;
            set_vel.angular.z = 4.5;
            break;
         case ARROW_UP:
            set_vel.linear.x = 2;
            set_vel.angular.z = 0;
            break;
         case ARROW_DOWN:
            set_vel.linear.x = -2;
            set_vel.angular.z = 0;
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

   ros::init(argc, argv, "action_controller");
   ros::NodeHandle n("~");
   ros::Rate loop_rate(50);
   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);
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
   }
}   
```

Below is explanation for code line by line.

Include required headers:

    #include <ros/ros.h>
    #include <std_msgs/Float32MultiArray.h>
    #include <geometry_msgs/Twist.h>

Define constants for recognized objects, adjust values to IDs of objects
recognized by your system:

    #define SMILE 8
    #define ARROW_LEFT 9
    #define ARROW_UP 10
    #define ARROW_DOWN 11

Integer for storing object identifier:

    int id = 0;

Publisher for velocity commands:

    ros::Publisher action_pub;

Velocity command message:

    geometry_msgs::Twist set_vel;

Callback function for handling incoming messages with recognized objects
data:

    void objectCallback(const std_msgs::Float32MultiArrayPtr &object) {

Check if size of `data` field is non zero, if it is, then object is
recognized. When `data` field size is zero, then no object was
recognized.

    if (object->data.size() > 0) {

Read id of recognized object:

    id = object->data[0];

Depending on recognized object, set appropriate speed values:

    switch (id) {
             case ARROW_LEFT:
                set_vel.linear.x = 0;
                set_vel.angular.z = 1;
                break;
             case ARROW_UP:
                set_vel.linear.x = 1;
                set_vel.angular.z = 0;
                break;
             case ARROW_DOWN:
                set_vel.linear.x = -1;
                set_vel.angular.z = 0;
                break;
             default: // other object
                set_vel.linear.x = 0;
                set_vel.angular.z = 0;
          }

Publish velocity command message:

    action_pub.publish(set_vel);

Stop all motors when no object was detected:

    } else {
          // No object detected
          set_vel.linear.x = 0;
          set_vel.angular.z = 0;
          action_pub.publish(set_vel);
       }

Main function, node initialization and set main loop interval:

    int main(int argc, char **argv) {
       ros::init(argc, argv, "action_controller");
       ros::NodeHandle n("~");
       ros::Rate loop_rate(50);

Subscribe to `/objects` topic:

    ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);

Prepare publisher for velocity commands:

    action_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

Set zeros for initial speed values:

    set_vel.linear.x = 0;
       set_vel.linear.y = 0;
       set_vel.linear.z = 0;
       set_vel.angular.x = 0;
       set_vel.angular.y = 0;
       set_vel.angular.z = 0;

Main loop, wait trigger messages:

    while (ros::ok()) {
          ros::spinOnce();
          loop_rate.sleep();
       }

Last thing to do is edit the `CMakeLists.txt` file. Find line:

    add_executable(tutorial_pkg_node src/tutorial_pkg_node.cpp)

and add after it:

    add_executable(action_controller_node src/action_controller.cpp)

Find also:

    target_link_libraries(tutorial_pkg_node
      ${catkin_LIBRARIES}
    )

and add after it:

    target_link_libraries(action_controller_node
      ${catkin_LIBRARIES}
    )

Now you can build your node and test it.

**Task 1** Run your node along with `find_object_2d` and `usb_cam`
nodes. Then use `rosnode`, `rostopic` and `rqt_graph` tools to examine
system. Place different objects in front of your robot by turns. Observe
how it drives and turns depending on objects.

You can use below `launch` file:

```
<launch>

    <node pkg="usb_cam" type="usb_cam_node" name="usb_cam">
        <param name="video_device" value="/dev/video0"/>
        <param name="image_width" value="640"/>
        <param name="image_height" value="480"/>
        <param name="pixel_format" value="yuyv"/>
        <param name="framerate" value="2"/>
    </node>

    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/usb_cam/image_raw"/>
        <param name="gui" value="false"/>
        <param name="objects_path" value="/home/pi/ros_workspace/find_obj"/>
    </node>

    <node pkg="tutorial_pkg" type="action_controller_node" name="action_controller" 
    	output="screen"/>

</launch>
```

## Getting position of recognized object ##

Beside the ID of recognized object, `find_object_2d` node is also
publishing a homography matrix of recognized object. in computer vision
homography is used to define position of object relative to camera. We
will use it to obtain horizontal position of object. Homography matrix
is published in the same topic as ID, but in the next cells of array,
they are formatted as
`[objectId1, objectWidth, objectHeight, h11, h12, h13, h21, h22, h23, h31, h32, h33, object2]`.

We will modify node to rotate robot to direction of recognized object.
Open `action_controller.cpp` file in text editor.

Begin with including of required header file:

    #include <opencv2/opencv.hpp>

Variable for storing camera centre, this should be half of your camera
horizontal resolution:

    int camera_center = 320; // left 0, right 640

Variable for defining maximum rotation speed:

    float max_ang_vel = 6.0;

Variable for object width and height:

    float objectWidth = object->data[1];
    float objectHeight = object->data[2];

Variable for storing calculated object centre:

    float x_pos;

Variable defining how much rotation speed should increase with every
pixel of object displacement:

    float speed_coefficient = (float) camera_center / max_ang_vel;

Object for homography matrix:

    cv::Mat cvHomography(3, 3, CV_32F);

Vectors for storing input and output planes:

    std::vector<cv::Point2f> inPts, outPts;

Add new case in `switch` statement:

    case SMILE:

Extract coefficients homography matrix:

    cvHomography.at<float>(0, 0) = object->data[3];
    cvHomography.at<float>(1, 0) = object->data[4];
    cvHomography.at<float>(2, 0) = object->data[5];
    cvHomography.at<float>(0, 1) = object->data[6];
    cvHomography.at<float>(1, 1) = object->data[7];
    cvHomography.at<float>(2, 1) = object->data[8];
    cvHomography.at<float>(0, 2) = object->data[9];
    cvHomography.at<float>(1, 2) = object->data[10];
    cvHomography.at<float>(2, 2) = object->data[11];

Define corners of input plane:

    inPts.push_back(cv::Point2f(0, 0));
    inPts.push_back(cv::Point2f(objectWidth, 0));
    inPts.push_back(cv::Point2f(0, objectHeight));
    inPts.push_back(cv::Point2f(objectWidth, objectHeight));

Calculate perspective transformation:

    cv::perspectiveTransform(inPts, outPts, cvHomography);

Calculate centre of object from its corners:

    x_pos = (int) (outPts.at(0).x + outPts.at(1).x + outPts.at(2).x + outPts.at(3).x) / 4;

Calculate angular speed value proportional to position of object and put
it into velocity message:

    set_vel.angular.z = -(x_pos - camera_center) / speed_coefficient; 

Your final file should look like this:

```
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>

#define SMILE 4
#define ARROW_LEFT 3
#define ARROW_UP 5
#define ARROW_DOWN 6
int id = 0;
ros::Publisher action_pub;
geometry_msgs::Twist set_vel;
int camera_center = 320; // left 0, right 640
float max_ang_vel = 6.0;

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
         case ARROW_LEFT:
            set_vel.linear.x = 0;
            set_vel.angular.z = 4.5;
            break;
         case ARROW_UP:
            set_vel.linear.x = 2;
            set_vel.angular.z = 0;
            break;
         case ARROW_DOWN:
            set_vel.linear.x = -2;
            set_vel.angular.z = 0;
            break;
         case SMILE:
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

            x_pos = (int) (outPts.at(0).x + outPts.at(1).x + outPts.at(2).x + 
	    	outPts.at(3).x) / 4;
            set_vel.angular.z = -(x_pos - camera_center) / speed_coefficient;
            
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
   ros::init(argc, argv, "action_controller");
   ros::NodeHandle n("~");
   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);
   ros::Rate loop_rate(50);
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
   }
}
```

Last thing to do is edit the `CMakeLists.txt` file. Find line:

    find_package(catkin REQUIRED COMPONENTS roscpp )

and add after it:

    find_package( OpenCV REQUIRED )

Find also:

    include_directories(
            ${catkin_INCLUDE_DIRS}
    )

and change it to:

    include_directories(
            ${catkin_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
    )

Find:

    target_link_libraries(action_controller_node
            ${catkin_LIBRARIES}
            )

and change it to:

    target_link_libraries(action_controller_node
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            )

Now you can build your node and test it.

**Task 2** Run your node along with `find_object_2d` and `usb_cam`
nodes. Place object with ID bonded to new case in switch statement in
front of your robot. Observe how it turns towards object.

## Following the object ##

In this section you will modify your robot to turn and also drive
towards object while keeping distance to it. For keeping the distance we
will use proximity sensors. Connect right sensor to
`hSens1` port of `CORE2` and left to `hSens2`.

Log in to Husarion Cloud and open project that you created in previous
manual, you will edit it a little.

Include required header files:

    #include <DistanceSensor.h>
    #include "sensor_msgs/Range.h"

Instantiate sensor objects:

    using namespace hModules;
    DistanceSensor sensR(hSens1);
    DistanceSensor sensL(hSens2);

Message objects for left and right sensor measurement:

    sensor_msgs::Range rangeL;
    sensor_msgs::Range rangeR;

Publishers for measurement messages:

    ros::Publisher rangeL_pub("/rangeL", &rangeL);
    ros::Publisher rangeR_pub("/rangeR", &rangeR);

In main function, put initial values for measurement messages:

    rangeL.header.frame_id="left";
    rangeL.radiation_type=sensor_msgs::Range::ULTRASOUND;
    rangeL.field_of_view = 0.5; // rad
    rangeL.min_range = 0.05; // meters
    rangeL.max_range = 2; // meters

    rangeR.header.frame_id="right";
    rangeR.radiation_type=sensor_msgs::Range::ULTRASOUND;
    rangeR.field_of_view = 0.5; // rad
    rangeR.min_range = 0.05; // meters
    rangeR.max_range = 2; // meters

Run publishers:

    nh.advertise(rangeL_pub);
    nh.advertise(rangeR_pub);

Read sensors:

    int distL = sensL.getDistance();
    int distR = sensR.getDistance();

Convert measurement values to meters and put them into messages:

    rangeL.range = (float)distL/100;
    rangeR.range = (float)distR/100;

Publish messages:

    rangeL_pub.publish(&rangeL);
    rangeR_pub.publish(&rangeR);

Whole file should look like this:

```
#include "hFramework.h"
#include "hCloudClient.h"
#include <ros.h>
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include <DistanceSensor.h>
#include "sensor_msgs/Range.h"

using namespace hModules;
DistanceSensor sensR(hSens1);
DistanceSensor sensL(hSens2);

using namespace hFramework;

ros::NodeHandle nh;
geometry_msgs::PoseStamped pose;
ros::Publisher pose_pub("/pose", &pose);

sensor_msgs::Range rangeL;
sensor_msgs::Range rangeR;

ros::Publisher rangeL_pub("/rangeL", &rangeL);
ros::Publisher rangeR_pub("/rangeR", &rangeR);

uint16_t delay = 10; // milliseconds
float delay_s = (float)delay/(float)1000;
uint16_t enc_res = 1000; // encoder tics per wheel revolution

int32_t enc_FL = 0; // encoder tics
int32_t enc_RL = 0; // encoder tics
int32_t enc_FR = 0; // encoder tics
int32_t enc_RR = 0; // encoder tics

int32_t enc_L = 0; // encoder tics
float wheel_L_ang_pos = 0; // radians
float wheel_L_ang_vel = 0; // radians per second

int32_t enc_R = 0; // encoder tics
float wheel_R_ang_pos = 0; // radians
float wheel_R_ang_vel = 0; // radians per second

float robot_angular_pos = 0; // radians
float robot_angular_vel = 0; // radians per second

float robot_x_pos = 0; // meters
float robot_y_pos = 0; // meters
float robot_x_vel = 0; // meters per second
float robot_y_vel = 0; // meters per second

float robot_width = 0.2; // meters
float robot_length = 0.15; //meters
float wheel_radius = 0.04; //meters

void twistCallback(const geometry_msgs::Twist &twist) {
    float lin = twist.linear.x;
    float ang = twist.angular.z;
    float motorL = lin - ang * 0.5;
    float motorR = lin + ang * 0.5;
	hMot1.setPower(motorR*100);
	hMot2.setPower(motorR*100);
    hMot3.setPower(motorL*100);
	hMot4.setPower(motorL*100);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &twistCallback);

void hMain() {
    platform.begin(&RPi);
    RPi.setBaudrate(500000);
    nh.getHardware()->initWithDevice(&platform.LocalSerial);
	nh.initNode();
    nh.subscribe(sub);
    hMot3.setMotorPolarity(Polarity::Reversed);
	hMot3.setEncoderPolarity(Polarity::Reversed);
    hMot4.setMotorPolarity(Polarity::Reversed);
	hMot4.setEncoderPolarity(Polarity::Reversed);
    
    pose.header.frame_id="robot";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionFromYaw(0);
    
    nh.advertise(pose_pub);
    
    rangeL.header.frame_id="left";
    rangeL.radiation_type=sensor_msgs::Range::ULTRASOUND;
    rangeL.field_of_view = 0.5; // rad
    rangeL.min_range = 0.05; // meters
    rangeL.max_range = 2; // meters

    rangeR.header.frame_id="right";
    rangeR.radiation_type=sensor_msgs::Range::ULTRASOUND;
    rangeR.field_of_view = 0.5; // rad
    rangeR.min_range = 0.05; // meters
    rangeR.max_range = 2; // meters
    
    nh.advertise(rangeL_pub);
    nh.advertise(rangeR_pub);
    
	while(true) {
	    enc_FR = hMot1.getEncoderCnt();
	    enc_RR = hMot2.getEncoderCnt();
	    enc_RL = hMot3.getEncoderCnt();
	    enc_FL = hMot4.getEncoderCnt();
	    
	    enc_L = (enc_FL+enc_RL)/2;
	    enc_R = (enc_FR+enc_RR)/2;
	    
	    wheel_L_ang_vel = ((2 * 3.14 * enc_L / enc_res) - wheel_L_ang_pos) / delay_s;
	    wheel_R_ang_vel = ((2 * 3.14 * enc_R / enc_res) - wheel_R_ang_pos) / delay_s;
	    
	    wheel_L_ang_pos = 2 * 3.14 * enc_L / enc_res;
	    wheel_R_ang_pos = 2 * 3.14 * enc_R / enc_res;
	    
	    robot_angular_vel = (((wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / robot_width)
	    	- robot_angular_pos)/delay_s;
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
      
	    int distL = sensL.getDistance();
	    int distR = sensR.getDistance();
	    
	    rangeL.range = (float)distL/100;
	    rangeR.range = (float)distR/100;
	    rangeL_pub.publish(&rangeL);
	    rangeR_pub.publish(&rangeR);
		
		nh.spinOnce();
		sys.delay(delay);
	}
}
```

Build project and upload it to device. Then open `action_controller.cpp`
file in text editor.

Begin with including of required header file:

    #include <sensor_msgs/Range.h>

Add variables for measured object distance, average distance and desired
distance to obstacle:

    float distL = 0;
    float distR = 0;
    float average_dist = 0;
    float desired_dist = 0.3;

Callback functions for incoming sensor messages, their task is only to
put values into appropriate variables:

    void distL_callback(const sensor_msgs::Range &range) {
       distL = range.range;
    }

    void distR_callback(const sensor_msgs::Range &range) {
       distR = range.range;
    }

Then, in `switch` statement, calculate average distance and set velocity
proportional to it only if both sensors found an obstacle, else set zero
value for linear velocity:

    if (distL > 0 && distR > 0) {
        average_dist = (distL + distR) / 2;
        set_vel.linear.x = (average_dist - desired_dist) * 6;
    } else {
       set_vel.linear.x = 0;
    } 

In main function, subscribe to sensor topics:

    ros::Subscriber distL_sub = n.subscribe("/rangeL", 1, distL_callback);
    ros::Subscriber distR_sub = n.subscribe("/rangeR", 1, distR_callback);

Final file should look like this:

```
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Range.h>

#define SMILE 4
#define ARROW_LEFT 3
#define ARROW_UP 5
#define ARROW_DOWN 6
int id = 0;
ros::Publisher action_pub;
geometry_msgs::Twist set_vel;
int camera_center = 320; // left 0, right 640
float max_ang_vel = 6.0;

float distL = 0;
float distR = 0;
float average_dist = 0;
float desired_dist = 0.3;

void distL_callback(const sensor_msgs::Range &range) {
   distL = range.range;
}

void distR_callback(const sensor_msgs::Range &range) {
   distR = range.range;
}

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
         case ARROW_LEFT:
            set_vel.linear.x = 0;
            set_vel.angular.z = 4.5;
            break;
         case ARROW_UP:
            set_vel.linear.x = 2;
            set_vel.angular.z = 0;
            break;
         case ARROW_DOWN:
            set_vel.linear.x = -2;
            set_vel.angular.z = 0;
            break;
         case SMILE:
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

            x_pos = (int) (outPts.at(0).x + outPts.at(1).x + outPts.at(2).x +
	    	outPts.at(3).x) / 4;
            set_vel.angular.z = -(x_pos - camera_center) / speed_coefficient;

            if (distL > 0 && distR > 0) {
               average_dist = (distL + distR) / 2;
               set_vel.linear.x = (average_dist - desired_dist) * 6;
            } else {
               set_vel.linear.x = 0;
            }
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
   ros::init(argc, argv, "action_controller");
   ros::NodeHandle n("~");
   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);

   ros::Subscriber distL_sub = n.subscribe("/rangeL", 1, distL_callback);
   ros::Subscriber distR_sub = n.subscribe("/rangeR", 1, distR_callback);
   ros::Rate loop_rate(50);
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
   }
}
```

Now you can build your node and test it.

**Task 3** Run your node along with `find_object_2d` and `usb_cam`
nodes. Place in front of your robot the same object as in Task 2.
Observe how it turns and drives towards object.

## Summary ##

After completing this tutorial you should be able to configure your
CORE2 device with vision system to recognize objects. You should also be
able to determine position of recognized object relative to camera and
make node that perform specific action related to recognized objects.
You also know how to handle proximity sensors with `sensor_msgs/Range`
message type.

*by Łukasz Mitka, AGH Krakow*
