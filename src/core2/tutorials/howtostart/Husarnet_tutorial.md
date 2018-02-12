---
title: 'Husarnet tutorials'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: ???
---

# Husarnet tutorial #

!!!!!!!!!!!!!!!!!!!!!!Idea zastosowanie i ogólny opis!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

## Configuration of the network ##

Before you start using husarnet you have to configure network settings of your device. 

### Installation guide for any Linux device ###

1. Install Husarnet: 
```
wget https://files.husarion.com/repo/repo.key -O- | apt-key add - 
echo 'deb https://files.husarion.com/repo/ xenial main' | sudo tee /etc/apt/sources.list.d/husarnet.list
sudo apt-get install -y apt-transport-https
sudo apt-get update
sudo apt-get install -y husarnet
sudo systemctl start husarnet
```
2. Link Husarnet to your account. Run husarnet websetup and copy the URL to your browser.

### Configuration for any Linux device ###

1. Enable IPv6 mode (in .bashrc): ```export ROS_IPV6=on```
2. Make sure ROS hostname is same as the one specified in Husarnet Web UI (e.g. mydev): ```export ROS_HOSTNAME=mydev```
3. Set master URI to http://master:11311: ```export ROS_MASTER_URI=http://master:11311```
4. Restart terminal to make sure the changes get applied.

### Configuration of the phone application ###

1. Install the application from http://users.atomshare.net/~zlmch/hNode.apk
2. Start the application and enter the link shown (http://cloud.husarion.com/c/XXXX) in your browser.
![image](/assets/img/husarnet/hNode.png)
3. Chose the histname for your device and click "Add device for your account"
![image](/assets/img/husarnet/platform.png)
4. The app will connect to ROS master and publish data from phone sensors. Use `rostopic list` to check what's available.
![image](/assets/img/husarnet/console.png)

## Using Husarnet with the Cloud ##

Now we will show you example of using husarnet. We create network with Virtual Machine, CORE2ROS and smartphone. After correctly completing all the configuration steps, your network should look like this:

![image](/assets/img/husarnet/platform_2.png)

You must choose which device will be ROS_MASTER. In my network it will be laptop with ubuntu image runing on Virtual Machine. 

On start we have to program our CORE2-ROS with this code:

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

uint16_t delay = 100; // milliseconds
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

float robot_width = 0.3;    // meters
float robot_length = 0.105; //meters
float wheel_radius = 0.04;  //meters

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

Now open terminal in your Virtual Machine and in first tab run `roscore`.

In secound one connect yourself with CORE2-ROS by ssh. Type `ssh yourCORE2-ROShostname`, then answer `yes` and tape passsword. You will see husarion graphic, it's mean that you are conected to CORE2-ROS terminal. Tape `/opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2` to start communication between CORE2 and linux SBC. You will see the list of publisher and subscriber. 

![image](/assets/img/husarnet/console_2.png)

Now we transport video stream to our local topic to make more conveniente to use it. Tape `rosrun image_transport republish compressed in:=/phone/camera1/image out:=/localimg`

You have to create your own ROS node. The instruction of creating node you can find <a href="https://husarion.com/core2/tutorials/ros-tutorials/2-creating-nodes/">here</a>. Source file object_follower.cpp should look like this:

```
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>

#define POXIPOL 10

int id = 0;
ros::Publisher action_pub;
geometry_msgs::Twist set_vel;
int camera_center = 240; // down 0, up 480
float max_ang_vel = 0.7;

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
	if (object->data.size() > 0)
	{
		id = object->data[0];

		float objectWidth = object->data[1];
		float objectHeight = object->data[2];
		float x_pos;
		float speed_coefficient = (float)camera_center / max_ang_vel;

		// Find corners OpenCV
		cv::Mat cvHomography(3, 3, CV_32F);
		std::vector<cv::Point2f> inPts, outPts;
		switch (id)
		{

		case POXIPOL:
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

			x_pos = (int)(outPts.at(0).y + outPts.at(1).y + outPts.at(2).y +
						  outPts.at(3).y) /
					4;
			set_vel.angular.z = -(x_pos - camera_center) / speed_coefficient;
			set_vel.linear.x = 0.8;

			break;

		default: // other object
			set_vel.linear.x = 0;
			set_vel.angular.z = 0;
		}
		action_pub.publish(set_vel);
	}
	else
	{
		// No object detected
		set_vel.linear.x = 0;
		set_vel.angular.z = 0;
		action_pub.publish(set_vel);
	}
}

int main(int argc, char **argv)
{
	std_msgs::String s;
	std::string str;
	str.clear();
	str.append("");
	std::to_string(3);
	s.data = str;
	ros::init(argc, argv, "object_follower");
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
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}
```

This node has previesly saved object for recognition in this case `#define POXIPOL 10`. The manual how to save obcect from scene you can find <a href="https://husarion.com/core2/tutorials/ros-tutorials/4-visual-object-recognition/">here</a>.

Now we will create find.launch file.  

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
Open new tab in terminal, reach directory of your .launch file and tape `roslaunch find.launch`. You will see find_object_2d GUI with saved by you objects.

![image](/assets/img/husarnet/find_object_2d.png)

After robot find the objects he will follow them.

## Camera support ##

If you want to use camera image from your smartphone, just install compressed image transport:

```apt-get install -y ros-kinetic-compressed-image-transport```

To view image from camera locally, use:

```rosrun image_view image_view image:=/yourphonehostname/camera0/image _image_transport:=compressed```

Make sure to accept permission dialog on the device. Use camera1 instead of camera0 to access second camera, if you have one.

If you have some node that can't process compressed image, launch the node that decompresses it locally (republisher):

``` rosrun image_transport republish compressed in:=/yourphonehostname/camera0/image out:=/localimg ```

After that, raw image will be available in `/localimg` channel.
