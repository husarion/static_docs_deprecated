---
title: '4 Visual object recognition'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 4
---

# Visual object recognition #

## Introduction ##

Objects can ce recognized by a robot with use of a vision system. It is
based on image characteristics like points, lines, edges colours and
their relative positions.

Processing of object recognition consists of two steps. First is
teaching and should be executed before main robot operation. During this
step object is presented to the vision system, image and extracted set of
features are saved as a pattern. Many objects can be presented to the
system.

Second step is actual recognition which is executed constantly during
robot operation. Every frame of camera is processed, image features are
extracted and compared to data set in the memory. If enough features matches
the pattern, then the object is recognized.

In this tutorial we will use `find_object_2d` node from `find_object_2d`
package for both teaching and recognition.

As an image source we will use nodes from `astra.launch` as in tutorial 1.

## Teaching objects ##

Anything could be an object to recognize, but remember, that the more edges
and contrast colours it has, the easier it will be recognized. A piece
of paper with something drawn on it would be enough for this tutorial.

First you should run `find_object_2d` and `astra.launch`. Node
`find_object_2d` by default subscribe to `image` topic, you should remap
it to topic `/camera/rgb/image_raw`. We will also decrease the
camera framerate, as it is not necessary to check for objects so often
and it will use less CPU making interface more responsive.

You can use below `launch` file:

``` launch
<launch>

    <include file="$(find astra_launch)/launch/astra.launch"/>

    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/camera/rgb/image_raw"/>
        <param name="gui" value="true"/>
    </node>

</launch>
```

After launching the `find_object_2d` node with properly adjusted image
topic, new window should appear:

![image](/assets/img/ros/man_4_1.png)

On the left side of the window there are thumbnails of saved images
(should be empty at first run). Application main window contains camera
view. Yellow circles on the image are marking extracted image features.

To begin teaching process choose from the main toolbar **`Edit`**
→ **`Add object from scene...`.** New window will appear:

![image](/assets/img/ros/man_4_2.png)

Now move the object and camera in order to cover as many features of
the object as possible. While doing that try not to catch object surroundings. When it’s
done, click **`Take picture`**.

![image](/assets/img/ros/man_4_3.png)

In next view click **`Select region`** and select only that part of
taken picture, that covers desired object and click **`Next`**.

![image](/assets/img/ros/man_4_4.png)

You will get confirmation of features extracted from selected image
area. If presented image is in accordance with what you selected,
click **`End`**

![image](/assets/img/ros/man_4_5.png)

You should see new thumbnail in the left panel. Notice the number outside of
parentheses on the left of the image, this is the object ID.

Now you can add some more objects to be recognized. Remember their IDs, you
will need them later:

![image](/assets/img/ros/man_4_6.png)

When you have enough objects in the database choose from the main toolbar
**`File`** → **`Save objects...`** and choose a folder to
store recognized objects. Close the window and stop all running nodes.

## Recognizing objects ##

Objects will be recognized by the same node which was used for teaching but 
it works in slightly different configuration. We will set two new parameters
for the node. For parameter `gui` we will set value `false`, this will run
node without window for learning objects as we no longer need it.
Another parameter will be `objects_path`, this should be a path to a folder
that you have just chosen to store recognized objects.

You can use below `launch` file:

``` launch
<launch>

    <include file="$(find astra_launch)/launch/astra.launch"/>

    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/camera/rgb/image_raw"/>
        <param name="gui" value="false"/>
        <param name="objects_path" value="/home/husarion/ros_workspace/find_obj"/>
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
message to `/cmd_vel` with speed and direction depending on the object.

Create a new file, name it `action_controller.cpp` and place it in `src`
folder under `tutorial_pkg`. Then open it in text editor and paste below code:

``` cpp
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

Below is an explanation of the code line by line.

Including required headers:

``` cpp
    #include <ros/ros.h>
    #include <std_msgs/Float32MultiArray.h>
    #include <geometry_msgs/Twist.h>
``` 

Defining constants for recognized objects, adjusting values to IDs of objects
recognized by your system:

``` cpp
    #define SMILE 8
    #define ARROW_LEFT 9
    #define ARROW_UP 10
    #define ARROW_DOWN 11
``` 

Integer for storing object identifier:

``` cpp
    int id = 0;
``` 

Publisher for velocity commands:

``` cpp
    ros::Publisher action_pub;
``` 

Velocity command message:

``` cpp
    geometry_msgs::Twist set_vel;
``` 

Callback function for handling incoming messages with recognized objects
data:

``` cpp
    void objectCallback(const std_msgs::Float32MultiArrayPtr &object) {
``` 

Checking if size of `data` field is non zero, if it is, then object is
recognized. When `data` field size is zero, then no object was
recognized.

``` cpp
    if (object->data.size() > 0) {
``` 

Reading id of recognized object:

``` cpp
    id = object->data[0];
``` 

Depending on recognized object, setting appropriate speed values:

``` cpp
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
``` 

Publishing velocity command message:

``` cpp
    action_pub.publish(set_vel);
``` 

Stopping all motors when no object was detected:

``` cpp
    } else {
          // No object detected
          set_vel.linear.x = 0;
          set_vel.angular.z = 0;
          action_pub.publish(set_vel);
       }
``` 

Main function, node initialization and setting main loop interval:

``` cpp
    int main(int argc, char **argv) {
       ros::init(argc, argv, "action_controller");
       ros::NodeHandle n("~");
       ros::Rate loop_rate(50);
``` 

Subscribing to `/objects` topic:

``` cpp
    ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);
``` 

Preparing publisher for velocity commands:

``` cpp
    action_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
``` 

Setting zeros for initial speed values:

``` cpp
    set_vel.linear.x = 0;
       set_vel.linear.y = 0;
       set_vel.linear.z = 0;
       set_vel.angular.x = 0;
       set_vel.angular.y = 0;
       set_vel.angular.z = 0;
``` 

Main loop, waiting for trigger messages:

``` cpp
    while (ros::ok()) {
          ros::spinOnce();
          loop_rate.sleep();
       }
``` 

Last thing to do is editting the `CMakeLists.txt` file. Find line:

    add_executable(tutorial_pkg_node src/tutorial_pkg_node.cpp)

and add below code after it:

    add_executable(action_controller_node src/action_controller.cpp)

Find also:

    target_link_libraries(tutorial_pkg_node
      ${catkin_LIBRARIES}
    )

and add below code after it:

    target_link_libraries(action_controller_node
      ${catkin_LIBRARIES}
    )

Now you can build your node and test it.

**Task 1** Run your node along with `find_object_2d` and `astra.launch`.
Then use `rosnode`, `rostopic` and `rqt_graph` tools to examine the
system. Place different objects in front of your robot one by one. Observe
how it drives and turns depending on differnt objects.

You can use below `launch` file:

``` launch
<launch>

    <include file="$(find astra_launch)/launch/astra.launch"/>

    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/camera/rgb/image_raw"/>
        <param name="gui" value="false"/>
        <param name="objects_path" value="/home/husarion/ros_workspace/find_obj"/>
    </node>

    <node pkg="tutorial_pkg" type="action_controller_node" name="action_controller" 
    	output="screen"/>

</launch>
```

## Getting position of recognized object ##

Besides the ID of recognized object, `find_object_2d` node is also
publishing a homography matrix of recognized object. In computer vision
homography is used to define position of object relative to the camera. We
will use it to obtain horizontal position of the object. Homography matrix
is published in the same topic as the ID, but in the next cells of array,
they are formatted as

`[objectId1, objectWidth, objectHeight, h11, h12, h13, h21, h22, h23, h31, h32, h33, object2]`.

We will modify node to rotate robot to the direction of recognized object.

Open `action_controller.cpp` file in text editor.

Begin with including of required header file:

``` cpp
    #include <opencv2/opencv.hpp>
``` 

Variable for storing camera centre- this should be half of your camera
horizontal resolution:

``` cpp
    int camera_center = 320; // left 0, right 640
``` 

Variables for defining rotation speed:

``` cpp
    float max_ang_vel = 0.6;
    float min_ang_vel = 0.4;
    float ang_vel = 0;
``` 

Variable for object width and height:

``` cpp
    float objectWidth = object->data[1];
    float objectHeight = object->data[2];
``` 

Variable for storing calculated object centre:

``` cpp
    float x_pos;
``` 

Variable defining how much rotation speed should increase with every
pixel of object displacement:

``` cpp
    float speed_coefficient = (float) camera_center / max_ang_vel /4;
``` 

Object for homography matrix:

``` cpp
    cv::Mat cvHomography(3, 3, CV_32F);
``` 

Vectors for storing input and output planes:

``` cpp
    std::vector<cv::Point2f> inPts, outPts;
```

Adding new case in `switch` statement:

``` cpp
    case SMILE:
``` 

Extracting coefficients homography matrix:

``` cpp
    cvHomography.at<float>(0, 0) = object->data[3];
    cvHomography.at<float>(1, 0) = object->data[4];
    cvHomography.at<float>(2, 0) = object->data[5];
    cvHomography.at<float>(0, 1) = object->data[6];
    cvHomography.at<float>(1, 1) = object->data[7];
    cvHomography.at<float>(2, 1) = object->data[8];
    cvHomography.at<float>(0, 2) = object->data[9];
    cvHomography.at<float>(1, 2) = object->data[10];
    cvHomography.at<float>(2, 2) = object->data[11];
``` 

Defining corners of input plane:

``` cpp
    inPts.push_back(cv::Point2f(0, 0));
    inPts.push_back(cv::Point2f(objectWidth, 0));
    inPts.push_back(cv::Point2f(0, objectHeight));
    inPts.push_back(cv::Point2f(objectWidth, objectHeight));
``` 

Calculating perspective transformation:

``` cpp
    cv::perspectiveTransform(inPts, outPts, cvHomography);
``` 

Calculating centre of object from its corners:

``` cpp
    x_pos = (int) (outPts.at(0).x + outPts.at(1).x + outPts.at(2).x + outPts.at(3).x) / 4;
``` 

Calculating angular speed value proportional to position of object and putting
it into velocity message:

``` cpp
            ang_vel = -(x_pos - camera_center) / speed_coefficient;
            
            if (ang_vel >= -(min_ang_vel/2) && ang_vel <= (min_ang_vel/2)){
                set_vel.angular.z = 0;
	    }
	    else if (ang_vel >=max_ang_vel){
		set_vel.angular.z = max_ang_vel;
	    }
	    else if (ang_vel <=-max_ang_vel){
		set_vel.angular.z = -max_ang_vel;
	    }
	    else {
		set_vel.angular.z = ang_vel;
	    }
``` 

Your final file should look like this:

``` cpp
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
float max_ang_vel = 0.6;
float min_ang_vel = 0.4;
float ang_vel = 0;

void objectCallback(const std_msgs::Float32MultiArrayPtr &object) {
   if (object->data.size() > 0) {
      id = object->data[0];

      float objectWidth = object->data[1];
      float objectHeight = object->data[2];
      float x_pos;
      float speed_coefficient = (float) camera_center / max_ang_vel /4;

      // Find corners OpenCV
      cv::Mat cvHomography(3, 3, CV_32F);
      std::vector<cv::Point2f> inPts, outPts;
      switch (id) {
         case ARROW_LEFT:
            set_vel.linear.x = 0;
            set_vel.angular.z = 1.0;
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
            ang_vel = -(x_pos - camera_center) / speed_coefficient;
            
            if (ang_vel >= -(min_ang_vel/2) && ang_vel <= (min_ang_vel/2)){
                set_vel.angular.z = 0;
	    }
	    else if (ang_vel >=max_ang_vel){
		set_vel.angular.z = max_ang_vel;
	    }
	    else if (ang_vel <=-max_ang_vel){
		set_vel.angular.z = -max_ang_vel;
	    }
	    else {
		set_vel.angular.z = ang_vel;
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

Last thing to do is to edit the `CMakeLists.txt` file. Find line:

    find_package(catkin REQUIRED COMPONENTS roscpp )

and add below code after it:

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

**Task 2** Run your node along with `find_object_2d` and `astra.launch`. 
Place the object with ID bonded to new case in switch statement in
front of your robot. Observe how it turns towards the object.

## Following the object ##

In this section you will modify your robot to turn and also drive
towards the object while keeping distance to it. For keeping the distance we
will use proximity sensors. Connect multiplexer to
`hSens3` port of `CORE2` and sensors to proper chanel like below:

ch0 - RR sensor
ch1 - RL sensor
ch6 - FL sensor
ch7 - FR sensor

Log in to Husarion Cloud and open project that you created in previous
manual. You will need to edit it a little.

Include required header files:

``` cpp
    #include "sensor_msgs/Range.h"
``` 

Instantiate sensor objects:

``` cpp
    hSensor& sensMUX(hSens3);
``` 

Message objects for all sensor measurement:

``` cpp
sensor_msgs::Range rangeFL;
sensor_msgs::Range rangeFR;
sensor_msgs::Range rangeRL;
sensor_msgs::Range rangeRR;
``` 

Publishers for measurement messages:

``` cpp
ros::Publisher rangeFL_pub("/rangeFL", &rangeFL);
ros::Publisher rangeFR_pub("/rangeFR", &rangeFR);
ros::Publisher rangeRL_pub("/rangeRL", &rangeRL);
ros::Publisher rangeRR_pub("/rangeRR", &rangeRR);
``` 

Structure and mutex to use sensor data:

``` 
struct hMUX{
    bool p2;
    bool p3;
    bool p4;
    bool active;
    float* dis;
    hMUX(bool tp2, bool tp3, bool tp4, bool tactive, float* tdis):p2(tp2), p3(tp3), p4(tp4), active(tactive), dis(tdis){}
    hMUX(bool tp2, bool tp3, bool tp4, bool tactive):p2(tp2), p3(tp3), p4(tp4), active(tactive){}
};

hMUX tMUX[] = {
    hMUX(false, false, false, true, &dis4),//ch0
    hMUX(false, false, true, true, &dis3),//ch1
    hMUX(false, true, false, false),//ch2
    hMUX(false, true, true, false),//ch3
    hMUX(true, false, false, false),//ch4
    hMUX(true, false, true, false),//ch5
    hMUX(true, true, false, true, &dis1),//ch6
    hMUX(true, true, true, true, &dis2)//ch7
};
``` 

Function to read sensors data:

``` cpp
void IRSensorReadout(){
    sensMUX.pin1.enableADC();
    sensMUX.pin2.setOut();
    sensMUX.pin3.setOut();
    sensMUX.pin4.setOut();
    for(;;){
        for(size_t i = 0; i<8; i++){
            if(tMUX[i].active == true){
                sensMUX.pin2.write(tMUX[i].p2);
                sensMUX.pin3.write(tMUX[i].p3);
                sensMUX.pin4.write(tMUX[i].p4);
                sys.delay(MUXStepTime);
                float temp = 20*(1/(sensMUX.pin1.analogReadRaw()*0.001220703));
                if(temp > 30) temp = 30;
                if(temp < 4) temp = 4;
                *tMUX[i].dis = temp;
            }
        }
    }
}
``` 

In main function, put initial values for measurement messages:

``` cpp
    rangeFL.header.frame_id = "front_left";
    rangeFL.radiation_type = sensor_msgs::Range::ULTRASOUND;
    rangeFL.field_of_view = 0.3; // rad
    rangeFL.min_range = 0.04;    // meters
    rangeFL.max_range = 0.3;        // meters
    
    rangeFR.header.frame_id = "front_right";
    rangeFR.radiation_type = sensor_msgs::Range::ULTRASOUND;
    rangeFR.field_of_view = 0.3; // rad
    rangeFR.min_range = 0.04;    // meters
    rangeFR.max_range = 0.3;        // meters
    
    rangeRL.header.frame_id = "rear_left";
    rangeRL.radiation_type = sensor_msgs::Range::ULTRASOUND;
    rangeRL.field_of_view = 0.3; // rad
    rangeRL.min_range = 0.04;    // meters
    rangeRL.max_range = 0.3;        // meters
    
    rangeRR.header.frame_id = "rear_right";
    rangeRR.radiation_type = sensor_msgs::Range::ULTRASOUND;
    rangeRR.field_of_view = 0.3; // rad
    rangeRR.min_range = 0.04;    // meters
    rangeRR.max_range = 0.3;        // meters
``` 

Running publishers:

``` cpp
    nh.advertise(rangeFL_pub);
    nh.advertise(rangeFR_pub);
    nh.advertise(rangeRL_pub);
    nh.advertise(rangeRR_pub);
``` 

Assigning values to variables and put them into messages:

``` cpp
        rangeFL.range = dis1;
        rangeFR.range = dis2;
        rangeRL.range = dis3;
        rangeRR.range = dis4;
        
        rangeFL_pub.publish(&rangeFL);
        rangeFR_pub.publish(&rangeFR);
        rangeRL_pub.publish(&rangeRL);
        rangeRR_pub.publish(&rangeRR);
``` 

Publishing messages:

``` cpp
        rangeFL_pub.publish(&rangeFL);
        rangeFR_pub.publish(&rangeFR);
        rangeRL_pub.publish(&rangeRL);
        rangeRR_pub.publish(&rangeRR);
``` 

Whole file should look like this:

``` cpp
#include "hFramework.h"
#include "hCloudClient.h"
#include <ros.h>
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Range.h"
#include <stddef.h>
#include <stdio.h>
using namespace hFramework;
hSensor& sensMUX(hSens3);
float MUXStepTime = 50; //200ms for scan
float dis1 = 0; //Sharp LF
float dis2 = 0; //Sharp RF
float dis3 = 0; //Sharp LR
float dis4 = 0; //Sharp RR
bool batteryLow = false;
struct hMUX{
    bool p2;
    bool p3;
    bool p4;
    bool active;
    float* dis;
    hMUX(bool tp2, bool tp3, bool tp4, bool tactive, float* tdis):p2(tp2), p3(tp3), p4(tp4), active(tactive), dis(tdis){}
    hMUX(bool tp2, bool tp3, bool tp4, bool tactive):p2(tp2), p3(tp3), p4(tp4), active(tactive){}
};
hMUX tMUX[] = {
    hMUX(false, false, false, true, &dis4),//ch0
    hMUX(false, false, true, true, &dis3),//ch1
    hMUX(false, true, false, false),//ch2
    hMUX(false, true, true, false),//ch3
    hMUX(true, false, false, false),//ch4
    hMUX(true, false, true, false),//ch5
    hMUX(true, true, false, true, &dis1),//ch6
    hMUX(true, true, true, true, &dis2)//ch7
};
ros::NodeHandle nh;
geometry_msgs::PoseStamped pose;
ros::Publisher pose_pub("/pose", &pose);
sensor_msgs::Range rangeFL;
sensor_msgs::Range rangeFR;
sensor_msgs::Range rangeRL;
sensor_msgs::Range rangeRR;
ros::Publisher rangeFL_pub("/rangeFL", &rangeFL);
ros::Publisher rangeFR_pub("/rangeFR", &rangeFR);
ros::Publisher rangeRL_pub("/rangeRL", &rangeRL);
ros::Publisher rangeRR_pub("/rangeRR", &rangeRR);
uint16_t delay = 100; // milliseconds
float delay_s = (float)delay / (float)1000;
uint16_t enc_res = 1400; // encoder tics per wheel revolution
int32_t enc_FL = 0; // encoder tics
int32_t enc_RL = 0; // encoder tics
int32_t enc_FR = 0; // encoder tics
int32_t enc_RR = 0; // encoder tics
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
float robot_width = 0.3;    // meters
float robot_length = 0.105; //meters
float wheel_radius = 0.04;  //meters
void IRSensorReadout(){
    sensMUX.pin1.enableADC();
    sensMUX.pin2.setOut();
    sensMUX.pin3.setOut();
    sensMUX.pin4.setOut();
    for(;;){
        for(size_t i = 0; i<8; i++){
            if(tMUX[i].active == true){
                sensMUX.pin2.write(tMUX[i].p2);
                sensMUX.pin3.write(tMUX[i].p3);
                sensMUX.pin4.write(tMUX[i].p4);
                sys.delay(MUXStepTime);
                float temp = 20*(1/(sensMUX.pin1.analogReadRaw()*0.001220703));
                if(temp > 30) temp = 30;
                if(temp < 4) temp = 4;
                *tMUX[i].dis = temp;
            }
        }
    }
}
void twistCallback(const geometry_msgs::Twist &twist)
{
    float lin = twist.linear.x;
    float ang = twist.angular.z;
    float motorL = lin - ang * 0.5;
    float motorR = lin + ang * 0.5;
    hMot1.setPower(motorR * (-700) * !batteryLow);
    hMot2.setPower(motorR * (-700) * !batteryLow);
    hMot3.setPower(motorL * (-700) * !batteryLow);
    hMot4.setPower(motorL * (-700) * !batteryLow);
}
void batteryCheck()
{
    int i = 0;
    for (;;) {
        if (sys.getSupplyVoltage() > 11.1) {
            i--;
        } else {
            i++;
        }
        if (i > 50) {
            batteryLow = true;
            i = 50;
        }
        if (i < -50) {
            batteryLow = false;
            i = -50;
        }
        if (batteryLow == true) {
            LED1.toggle();
        } else {
            LED1.on();
        }
        sys.delay(250);
    }
}
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &twistCallback);
void hMain()
{
    platform.begin(&RPi);
    nh.getHardware()->initWithDevice(&platform.LocalSerial);
    nh.initNode();
    nh.subscribe(sub);
    hSens6.pin1.enableADC();  
    hSens2.pin1.enableADC();
    hMot3.setMotorPolarity(Polarity::Reversed);
    hMot3.setEncoderPolarity(Polarity::Reversed);
    hMot4.setMotorPolarity(Polarity::Reversed);
    hMot4.setEncoderPolarity(Polarity::Reversed);
    LED1.on();
    sys.taskCreate(batteryCheck);
    sys.taskCreate(IRSensorReadout);
    pose.header.frame_id = "robot";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionFromYaw(0);
    nh.advertise(pose_pub);
    rangeFL.header.frame_id = "front_left";
    rangeFL.radiation_type = sensor_msgs::Range::ULTRASOUND;
    rangeFL.field_of_view = 0.3; // rad
    rangeFL.min_range = 0.04;    // meters
    rangeFL.max_range = 0.3;        // meters
    
    rangeFR.header.frame_id = "front_right";
    rangeFR.radiation_type = sensor_msgs::Range::ULTRASOUND;
    rangeFR.field_of_view = 0.3; // rad
    rangeFR.min_range = 0.04;    // meters
    rangeFR.max_range = 0.3;        // meters
    
    rangeRL.header.frame_id = "rear_left";
    rangeRL.radiation_type = sensor_msgs::Range::ULTRASOUND;
    rangeRL.field_of_view = 0.3; // rad
    rangeRL.min_range = 0.04;    // meters
    rangeRL.max_range = 0.3;        // meters
    
    rangeRR.header.frame_id = "rear_right";
    rangeRR.radiation_type = sensor_msgs::Range::ULTRASOUND;
    rangeRR.field_of_view = 0.3; // rad
    rangeRR.min_range = 0.04;    // meters
    rangeRR.max_range = 0.3;        // meters
    nh.advertise(rangeFL_pub);
    nh.advertise(rangeFR_pub);
    nh.advertise(rangeRL_pub);
    nh.advertise(rangeRR_pub);
    while (true)
    {
        enc_FR = hMot1.getEncoderCnt();
        enc_RR = hMot2.getEncoderCnt();
        enc_RL = hMot3.getEncoderCnt();
        enc_FL = hMot4.getEncoderCnt();
        enc_L = (enc_FL + enc_RL) / 2;
        enc_R = (enc_FR + enc_RR) / 2;
        wheel_L_ang_vel = ((2 * 3.14 * enc_L / enc_res) - wheel_L_ang_pos) / delay_s;
        wheel_R_ang_vel = ((2 * 3.14 * enc_R / enc_res) - wheel_R_ang_pos) / delay_s;
        wheel_L_ang_pos = 2 * 3.14 * enc_L / enc_res;
        wheel_R_ang_pos = 2 * 3.14 * enc_R / enc_res;
        robot_angular_vel = (((wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / robot_width) -
                             robot_angular_pos) /
                            delay_s;
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
        rangeFL.range = dis1;
        rangeFR.range = dis2;
        rangeRL.range = dis3;
        rangeRR.range = dis4;
        
        rangeFL_pub.publish(&rangeFL);
        rangeFR_pub.publish(&rangeFR);
        rangeRL_pub.publish(&rangeRL);
        rangeRR_pub.publish(&rangeRR);
        nh.spinOnce();
        sys.delay(delay);
    }
}
```

Build the project and upload it to your device. Then open `action_controller.cpp`
file in text editor.

Begin with including required header file:

``` cpp
    #include <sensor_msgs/Range.h>
``` 

Add variables for measured object distance, average distance and desired
distance to obstacle:

``` cpp
    float distFL = 0;
    float distFR = 0;
    float average_dist = 0;
    float desired_dist = 20;
``` 

Callback functions for incoming sensor messages, their task is only to
put values into appropriate variables:

``` cpp
    void distFL_callback(const sensor_msgs::Range &range) {
       distFL = range.range;
    }

    void distFR_callback(const sensor_msgs::Range &range) {
       distFR = range.range;
    }
``` 

Then, in `switch` statement, calculate average distance and set velocity
proportional to it only if both sensors found an obstacle, else set zero
value for linear velocity:

``` cpp
    if (distFL > 0 && distFR > 0) {
        average_dist = (distFL + distFR) / 2;
        set_vel.linear.x = (average_dist - desired_dist) /40;
    }
    else {
        set_vel.linear.x = 0;
    }
``` 

In main function, subscribe to sensor topics:

``` cpp
    ros::Subscriber distFL_sub = n.subscribe("/rangeFL", 1, distFL_callback);
    ros::Subscriber distFR_sub = n.subscribe("/rangeFR", 1, distFR_callback);
``` 

Final file should look like this:

``` cpp
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
float max_ang_vel = 0.6;
float min_ang_vel = 0.4;
float ang_vel = 0;

float distFL = 0;
float distFR = 0;
float average_dist = 0;
float desired_dist = 20;

void distFL_callback(const sensor_msgs::Range &range) {
   distFL = range.range;
}

void distFR_callback(const sensor_msgs::Range &range) {
   distFR = range.range;
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
            ang_vel = -(x_pos - camera_center) / speed_coefficient;
            
            if (ang_vel >= -(min_ang_vel/2) && ang_vel <= (min_ang_vel/2)){
                set_vel.angular.z = 0;
                    if (distFL > 0 && distFR > 0) {
                        average_dist = (distFL + distFR) / 2;
                        set_vel.linear.x = (average_dist - desired_dist) /40;
                    }
                    else {
                        set_vel.linear.x = 0;
                    }
	    }
	    else if (ang_vel >=max_ang_vel){
		set_vel.angular.z = max_ang_vel;
	    }
	    else if (ang_vel <=-max_ang_vel){
		set_vel.angular.z = -max_ang_vel;
	    }
	    else {
		set_vel.angular.z = ang_vel;
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

   ros::Subscriber distL_sub = n.subscribe("/rangeFL", 1, distFL_callback);
   ros::Subscriber distR_sub = n.subscribe("/rangeFR", 1, distFR_callback);
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
<iframe name="I1" src="https://raw.githubusercontent.com/husarion/hFramework/master/examples/GPIO_adc.cpp">
</iframe>

test

Now you can build your node and test it.

**Task 3** Run your node along with `find_object_2d` and `astra.launhc`. 
Place the same object as in Task 2 in front of your robot.
Observe how it turns and drives towards the object.

## Summary ##

After completing this tutorial you should be able to configure your
CORE2 device with vision system to recognize objects. You should also be
able to determine the position of recognized object relative to camera and
create a node that perform specific action related to recognized objects.
You should also know how to handle proximity sensors with `sensor_msgs/Range`
message type.

---------

*by Łukasz Mitka, AGH Krakow*

*Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com*
