---
title: '3 Simple kinematics for mobile robot'
platform: 'CORE2'
core2: true
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 3
---

# Simple kinematics for mobile robot #


## Forward kinematics task ##

Task of forward kinematics in mobile robotics is to determine robot
position and orientation based on wheel rotation measurement. In this
tutorial we will use four wheeled robot with separate drive for each
wheel. Robot scheme is presented below:

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/robot_scheme.png" width="50%" height="50%"/></center></div>

We will use following symbols and their meaning:

-   <div>R<sub>C</sub> - robot geometric centre</div>
-   <div>x<sub>C</sub> - robot geometric centre x position</div>
-   <div>y<sub>C</sub> - robot geometric centre y position</div>
-   <div>x<sub>C</sub> - robot geometric centre x speed</div>
-   <div>y<sub>C</sub> - robot geometric centre y speed</div>
-   <div>x<sub>r</sub> - robot x axis, points front of robot</div>
-   α - robot angular position
-   α - robot angular speed
-   <div>W<sub>FL</sub> - front left wheel</div>
-   <div>W<sub>FR</sub> - front right wheel</div>
-   <div>W<sub>RL</sub> - rear left wheel</div>
-   <div>W<sub>RR</sub> - rear right wheel</div>
-   <div>W<sub>L</sub> - middle left wheel</div>
-   <div>W<sub>R</sub> - middle right wheel</div>
-   <div>l<sub>1</sub> - distance between robot centre and front/rear wheels</div>
-   <div>l<sub>2</sub> - distance between robot left and right wheels</div>
-   v - linear speed
-   φ - wheel angular position
-   ω - wheel angular speed
-   r - wheel radius

To determine robot position we need robot kinematic model, it is much
easier to calculate position for two wheeled robot with rotation centre
between them. We can simplify our four wheeled robot to this model by
adding two virtual wheels (marked as W<sub>L</sub> and W<sub>R</sub> on the scheme).
Their speed and position will be average for front and left wheel:

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_formula_1.png" /></center></div>

We can determine robot orientation with:

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_formula_2.png" /></center></div>

Then robot x and y speed component:

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_formula_3.png" /></center></div>

To get position:

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_formula_4.png" /></center></div>

Using above equations you can perform forward kinematics task for mobile
robot.

We assume that wheels are connected to ports in following manner:

-   <div>front left wheel (W<sub>FL</sub>) - hMot4</div>
-   <div>front right wheel (W<sub>FR</sub>) - hMot1</div>
-   <div>rear left wheel (W<sub>RL</sub>) - hMot3</div>
-   <div>rear right wheel (W<sub>RR</sub>) - hMot2</div>

## Controlling the motor ##

Most common way to send movement commands to robot is with use
`geometry_msgs/Twist` message type. Then motor driver node should use
data stored in them to control the motor.

### Publishing the motion command for robot ###

You will use keyboard to control movement of your robot. For getting the
key events and converting them to `geometry_msgs/Twist` messages can be
used `teleop_twist_keyboard.py` node from package
`teleop_twist_keyboard`.

Alternatively you can use joystick to control your robot, then you will
need `universal_teleop` node from `universal_teleop` package and
`keyboard` node from `keyboard` package

### Converting motion command to motor drive signal ###

In this section you will create node for interfacing motors. Your node
will subscribe to topic with `geometry_msgs/Twist` messages, drive the
motors, read encoders and publish their state to appropriate topic. To
create this node you will use Husarion Cloud. Create new project and
paste following:


    #include "hFramework.h"
    #include "hCloudClient.h"
    #include <stdio.h>
    #include <iostream>
    #include <ros.h>
    #include "std_msgs/String.h"
    #include "geometry_msgs/Twist.h"
    
    using namespace hFramework;
    ros::NodeHandle nh;
    
    int voltage=1;
    
    void twistCallback(const geometry_msgs::Twist &twist) {
		float lin = twist.linear.x;
		float ang = twist.angular.z;
		float motorL = lin - ang * 0.5;
		float motorR = lin + ang * 0.5;
		hMot1.setPower(motorR*700*voltage);
		hMot2.setPower(motorR*700*voltage);
		hMot3.setPower(motorL*700*voltage);
		hMot4.setPower(motorL*700*voltage);
    }
    
	void batteryCheck(){
	    int i=0;
	    for(;;){
			if(sys.getSupplyVoltage()>11.1){
			    i=0;
			}
			else{
			    i++;
			}
			if(i>50){
			    voltage=0;
			}
			if(voltage==0){
			    LED1.toggle();
			}
			sys.delay(100);
	    }
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
        LED1.on();
        sys.taskCreate(batteryCheck);
        while(true) {
            nh.spinOnce();
            sys.delay(100);
        }
    }


Below is explanation for code line by line.

Include required headers:

    #include "hFramework.h"
    #include "hCloudClient.h"
    #include <stdio.h>
    #include <iostream>
    #include <ros.h>
    #include "std_msgs/String.h"
    #include "geometry_msgs/Twist.h"

Load namespace for Husarion functions:

    using namespace hFramework;

Handle for node:

    ros::NodeHandle nh;
    
Variable for turnig off the motors in case of low voltage:

    int voltage=1;

Function for handling incoming messages:

    void twistCallback(const geometry_msgs::Twist &twist) 

Function for checking battery voltage:

	void batteryCheck()

Read linear and angular target velocities, then calculate motor
velocities:

    float lin = twist.linear.x;
        float ang = twist.angular.z;
        float motorL = lin - ang * 0.5;
        float motorR = lin + ang * 0.5;
    }

Set target power for motors:

    hMot1.setPower(motorR*700*voltage);
    hMot2.setPower(motorR*700*voltage);
    hMot3.setPower(motorL*700*voltage);
    hMot4.setPower(motorL*700*voltage);

Define subscriber for velocity topic:

    ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &twistCallback);

Main function, tasks and node initialization:

    void hMain() {
        platform.begin(&RPi);
        RPi.setBaudrate(500000);
        nh.getHardware()->initWithDevice(&platform.LocalSerial);
        nh.initNode();
		sys.taskCreate(batteryCheck);

Subscribe to topic:

    nh.subscribe(sub);

Define reversed polarity for left front and rear motors, this may vary,
depending on your machine configuration:

    hMot3.setMotorPolarity(Polarity::Reversed);
    hMot3.setEncoderPolarity(Polarity::Reversed);
    hMot4.setMotorPolarity(Polarity::Reversed);
    hMot4.setEncoderPolarity(Polarity::Reversed);

Infinite loop, wait for incoming messages:

	while(true) {
		nh.spinOnce();
		sys.delay(100);
	}

Build your project and upload it to device.

### Running motor controller ###

In this section you will learn how to control your robot movement with
keyboard. You will need `teleop_twist_keyboard` node from
`teleop_twist_keyboard` package.

Log in to your CORE2 device through remote desktop and run terminal. In
first terminal window run `$ roscore`, in second run:

    $ /opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2

This program is responsible for bridging your CORE2 to ROS network. In
third terminal window run:

    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Now you can control your robot with keyboard with following functions
for buttons:

-   `’i’` - move forward

-   `’,’` - move backward

-   `’j’` - turn left

-   `’l’` - turn right

-   `’k’` - stop

-   `’q’` - increase speed

-   `’z’` - decrease speed

You should get similar view in `rqt_graph`:

![image](/assets/img/ros/man_3_1.png)

### Determining robot position ###

Now we will perform forward kinematics task, we will use encoders that
are attached to every motor and process their measurements with
equations shown in section 1.

Open Husarion WebIDE and open project that you created in section 2.2.

Add header file:

    #include "geometry_msgs/PoseStamped.h"

Define message type for robot position:

    geometry_msgs::PoseStamped pose;

Define publisher for robot pose:

    ros::Publisher pose_pub("/pose", &pose);

Variables for storing cycle time:

    uint16_t delay = 10; // milliseconds
    float delay_s = (float)delay/(float)1000; //seconds

Variable for storing encoder resolution, adjust this value to parameter
of your robot:

    uint16_t enc_res = 1400; // encoder tics per wheel revolution

Variables for storing encoder values:

    int32_t enc_FL = 0; // encoder tics
    int32_t enc_RL = 0; // encoder tics
    int32_t enc_FR = 0; // encoder tics
    int32_t enc_RR = 0; // encoder tics

Variables for storing left and right wheel position:

    int32_t enc_L = 0; // encoder tics
    float wheel_L_ang_pos = 0; // radians
    float wheel_L_ang_vel = 0; // radians per second

    int32_t enc_R = 0; // encoder tics
    float wheel_R_ang_pos = 0; // radians
    float wheel_R_ang_vel = 0; // radians per second

Variables for storing robot position:

    float robot_angular_pos = 0; // radians
    float robot_angular_vel = 0; // radians per second

    float robot_x_pos = 0; // meters
    float robot_y_pos = 0; // meters
    float robot_x_vel = 0; // meters per second
    float robot_y_vel = 0; // meters per second

Variables for storing robot parameters, adjust these values to your
robot:

    float robot_width = 0.3; // meters
    float robot_length = 0.105; //meters
    float wheel_radius = 0.04; //meters

In main function, set start values for robot position:

    pose.header.frame_id="robot";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionFromYaw(0);

Run publisher:

    nh.advertise(pose_pub);

In infinite while loop, read encoder values:

    enc_FR = hMot1.getEncoderCnt();
    enc_RR = hMot2.getEncoderCnt();
    enc_RL = hMot3.getEncoderCnt();
    enc_FL = hMot4.getEncoderCnt();

Calculate virtual left and right encoder values:

    enc_L = (enc_FL+enc_RL)/2;
    enc_R = (enc_FR+enc_RR)/2;

Calculate angular velocity for wheels:

    wheel_L_ang_vel = ((2 * 3.14 * enc_L / enc_res) - wheel_L_ang_pos) / delay_s;
    wheel_R_ang_vel = ((2 * 3.14 * enc_R / enc_res) - wheel_R_ang_pos) / delay_s;

Calculate angular position for wheels:

    wheel_L_ang_pos = 2 * 3.14 * enc_L / enc_res;
    wheel_R_ang_pos = 2 * 3.14 * enc_R / enc_res;

Calculate angular velocity for robot:

    robot_angular_vel = (((wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / robot_width) - 
    	robot_angular_pos)/delay_s;

Calculate angular position for robot:

    robot_angular_pos = (wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / robot_width;

Calculate linear velocities for robot:

    robot_x_vel = (wheel_L_ang_vel * wheel_radius + robot_angular_vel * robot_width / 2) * 
    	cos(robot_angular_pos);
    robot_y_vel = (wheel_L_ang_vel * wheel_radius + robot_angular_vel * robot_width / 2) * 
    	sin(robot_angular_pos);

Calculate linear positions for robot:

    robot_x_pos = robot_x_pos + robot_x_vel * delay_s;
    robot_y_pos = robot_y_pos + robot_y_vel * delay_s;

Put calculated values to message:

    pose.pose.position.x = robot_x_pos;
    pose.pose.position.y = robot_y_pos;
    pose.pose.orientation = tf::createQuaternionFromYaw(robot_angular_pos);

Publish message:

    pose_pub.publish(&pose);

Your final code should look like this:

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

int voltage=1;

uint16_t delay = 10; // milliseconds
float delay_s = (float)delay/(float)1000;
uint16_t enc_res = 1400; // encoder tics per wheel revolution

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

float robot_width = 0.3; // meters
float robot_length = 0.105; //meters
float wheel_radius = 0.04; //meters

void twistCallback(const geometry_msgs::Twist &twist) {
    float lin = twist.linear.x;
    float ang = twist.angular.z;
    float motorL = lin - ang * 0.5;
    float motorR = lin + ang * 0.5;
    hMot1.setPower(motorR*700*voltage);
    hMot2.setPower(motorR*700*voltage);
    hMot3.setPower(motorL*700*voltage);
    hMot4.setPower(motorL*700*voltage);
}

void batteryCheck(){
    int i=0;
    for(;;){
        if(sys.getSupplyVoltage()>11.1){
            i=0;
        }
        else{
            i++;
        }
        if(i>50){
            voltage=0;
        }
        if(voltage==0){
	    LED1.toggle();
        }
        sys.delay(100);
    }
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
    LED1.on();
    sys.taskCreate(batteryCheck);
    
    pose.header.frame_id="robot";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionFromYaw(0);
    
    nh.advertise(pose_pub);
    
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
	    
	    robot_angular_vel = (((wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / robot_width) - 
	    	robot_angular_pos)/delay_s;
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

Build your project and upload it to device.

### Running motor controller with forward kinematics task ###

In this section you will control your robot with keyboard and observe as
it publishes its own position.

Log in to your CORE2 device through remote desktop and run terminal and
start your robot as previously. In another terminal window run:

    $ rostopic echo /pose

Remember, that you need to have active window with
`teleop_twist_keyboard` to control robot movement.

You should get something like this on your screen:

![image](/assets/img/ros/man_3_2.png)

## Robot visualization with Rviz ##

Rviz is tool that allow visualization of robot position, travelled path,
planned trajectory, sensor state or obstacles surrounding robot.

To run it type in terminal:

    $ rviz

New window will appear:

![image](/assets/img/ros/man_3_3.png)

Application main view consists of:

1.  Toolbar

2.  Visualized items list

3.  Visualization window

4.  Object manipulation buttons
  
By default you will see only base frame, to add any other object push
**Add** from object manipulation buttons. In new window, there are two
tabs **By display type** and **By topic**. First one is for manual
selection from all possible objects, second one contains only currently
published topics.

After you choose object to display, click **OK** button and it will
appear in visualization window.

Now we will visualize position published by your robot, run `rviz`,
click **Add** and choose tab **By topic**.

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_4.png" /></center></div>

Find topic `/pose` and choose `Pose` and click **OK**.

Then in visualized items list find position `Fixed Frame` and change it
to `robot`.

After this you should see an arrow representing position and orientation
of your robot. Move your robot and observe as arrow changes its
position.

![image](/assets/img/ros/man_3_5.png)
 
Visualization of any other item is performed in the same way. In further
lessons, as you will produce more objects to visualize, you will add them
to the same view.

## Summary ##

After completing this tutorial you should be able to control motor
attached to your CORE2 device, set desired velocity for robot with
`geometry_msgs/Twist` message, determine position of your robot using
odometry, publish it as a `PoseStamped` message and visualize position
of your robot using `rviz`.

---------

*by Łukasz Mitka, AGH Krakow*

*Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com*
