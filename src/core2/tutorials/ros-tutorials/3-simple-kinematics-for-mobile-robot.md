---
title: '3 Simple kinematics for mobile robot'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 3
---

# Simple kinematics for mobile robot #


## Forward kinematics task ##

The purpose of forward kinematics in mobile robotics is to determine robot
position and orientation based on wheel rotation measurement. In this
tutorial we will use four wheeled robot with separate drive for each
wheel. Robot schematic is presented below:

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/robot_scheme.png" width="50%" height="50%"/></center></div>

We will use the following symbols:

-   <div>R<sub>C</sub> - robot geometric centre</div>
-   <div>x<sub>C</sub> - robot geometric centre x position</div>
-   <div>y<sub>C</sub> - robot geometric centre y position</div>
-   <div>x<sub>C</sub> - robot geometric centre x speed</div>
-   <div>y<sub>C</sub> - robot geometric centre y speed</div>
-   <div>x<sub>r</sub> - robot x axis, points front of robot</div>
-   α - robot angular position
-   α′- robot angular speed
-   <div>W<sub>FL</sub> - front left wheel</div>
-   <div>W<sub>FR</sub> - front right wheel</div>
-   <div>W<sub>RL</sub> - rear left wheel</div>
-   <div>W<sub>RR</sub> - rear right wheel</div>
-   <div>W<sub>L</sub> - middle left wheel</div>
-   <div>W<sub>R</sub> - middle right wheel</div>
-   <div>l<sub>1</sub> - distance between robot centre and front/rear wheels</div>
-   <div>l<sub>2</sub> - distance between robot left and right wheels</div>
-   v - linear speed
-   Φ - wheel angular position
-   ω - wheel angular speed
-   r - wheel radius

To determine robot position we need a robot kinematic model. It is much
easier to calculate position for two wheeled robot with rotation centre
between them. We can simplify our four wheeled robot to this model by
adding two virtual wheels (marked as W<sub>L</sub> and W<sub>R</sub> on the schematic).
Their speed and position will be average for front and left wheel:

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_formula_1_1.jpg" width="30%"/></center></div>
<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_formula_1_2.jpg" width="30%"/></center></div>
<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_formula_1_3.jpg" width="30%"/></center></div>
<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_formula_1_4.jpg" width="30%"/></center></div>

We can determine robot angular position and speed with:

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_formula_2_1.jpg" width="30%"/></center></div>
<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_formula_2_2.jpg" width="12%"/></center></div>


Then robot x and y speed component:

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_formula_3_1.jpg" width="40%"/></center></div>
<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_formula_3_2.jpg" width="40%"/></center></div>

To get position:

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_formula_4_1.jpg" width="17%"/></center></div>
<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/ros/man_3_formula_4_2.jpg" width="17%"/></center></div>

Using above equations you can perform forward kinematics task for mobile
robot.

We assume that wheels are connected to ports in following manner:

-   <div>front left wheel (W<sub>FL</sub>) - hMot4</div>
-   <div>front right wheel (W<sub>FR</sub>) - hMot1</div>
-   <div>rear left wheel (W<sub>RL</sub>) - hMot3</div>
-   <div>rear right wheel (W<sub>RR</sub>) - hMot2</div>

## Controlling the motor ##

Most common way to send movement commands to the robot is with use of
`geometry_msgs/Twist` message type. Then motor driver node should use
data stored in them to control the motor.

### Publishing the motion command for robot ###

You will use keyboard to control the movement of your robot. For getting the
key events and converting them to `geometry_msgs/Twist` messages you can
use `teleop_twist_keyboard.py` node from package
`teleop_twist_keyboard`.

Alternatively you can use joystick to control your robot, then you will
need `universal_teleop` node from `universal_teleop` package and
`keyboard` node from `keyboard` package

### Converting motion command to motor drive signal ###

In this section you will create a node for interfacing motors. Your node
will subscribe to topic with `geometry_msgs/Twist` messages, drive the
motors, read encoders and publish their state to appropriate topic. To
create this node you will use Husarion Cloud. Create new project and
paste following:

```cpp
#include "hFramework.h"
#include "hCloudClient.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Bool.h"
#include "ROSbot.h"

using namespace hFramework;

// Uncomment one of these lines, accordingly to range sensor type of your ROSbot
// If you have version with infared sensor:
// static const SensorType sensor_type = SENSOR_INFRARED;
// If you have version with laser sensor:
static const SensorType sensor_type = SENSOR_LASER;
// If you want to use your own sensor:
// static const SensorType sensor_type = NO_DISTANCE_SENSOR;

// Uncomment one of these lines, accordingly to IMU sensor type of your device
// If you have version with MPU9250:
static const ImuType imu_type = MPU9250;
// If you want to use your own sensor:
// static const ImuType imu_type = NO_IMU;

ros::NodeHandle nh;
sensor_msgs::BatteryState battery;
ros::Publisher *battery_pub;

int publish_counter = 0;

void twistCallback(const geometry_msgs::Twist &twist)
{
	rosbot.setSpeed(twist.linear.x, twist.angular.z);
}

void initCmdVelSubscriber()
{
	ros::Subscriber<geometry_msgs::Twist> *cmd_sub = new ros::Subscriber<geometry_msgs::Twist>("/cmd_vel", &twistCallback);
	nh.subscribe(*cmd_sub);
}

void resetCallback(const std_msgs::Bool &msg)
{
	if (msg.data == true)
	{
		rosbot.reset_odometry();
	}
}

void initResetOdomSubscriber()
{
	ros::Subscriber<std_msgs::Bool> *odom_reset_sub = new ros::Subscriber<std_msgs::Bool>("/reset_odom", &resetCallback);
	nh.subscribe(*odom_reset_sub);
}

void initBatteryPublisher()
{
	battery_pub = new ros::Publisher("/battery", &battery);
	nh.advertise(*battery_pub);
}

void hMain()
{
	rosbot.initROSbot(sensor_type, imu_type);
	platform.begin(&RPi);
	nh.getHardware()->initWithDevice(&platform.LocalSerial);
	nh.initNode();

	initBatteryPublisher();
	initCmdVelSubscriber();
	initResetOdomSubscriber();

	while (true)
	{
		nh.spinOnce();
		publish_counter++;
		if (publish_counter > 10)
		{
			// get battery voltage
			battery.voltage = rosbot.getBatteryLevel();
			// publish battery voltage
			battery_pub->publish(&battery);
			publish_counter = 0;
		}
		sys.delay(10);
	}
}
```

Below is explanation for code line by line.

Include required headers:

``` cpp
#include "hFramework.h"
#include "hCloudClient.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Bool.h"
#include "ROSbot.h"
``` 

Load namespace for Husarion functions:

``` cpp
    using namespace hFramework;
``` 

Define which type of distance sensor you are using in your robot:

```cpp
// Uncomment one of these lines, accordingly to range sensor type of your ROSbot
// If you have version with infared sensor:
// static const SensorType sensor_type = SENSOR_INFRARED;
// If you have version with laser sensor:
static const SensorType sensor_type = SENSOR_LASER;
// If you want to use your own sensor:
// static const SensorType sensor_type = NO_DISTANCE_SENSOR;
```

Define which type of IMU you are using in your robot:

```cpp
// Uncomment one of these lines, accordingly to IMU sensor type of your device
// If you have version with MPU9250:
static const ImuType imu_type = MPU9250;
// If you want to use your own sensor:
// static const ImuType imu_type = NO_IMU;
```

Create handle for node:

``` cpp
    ros::NodeHandle nh;
``` 

Define type of message and publisher for a battery:

``` cpp
    sensor_msgs::BatteryState battery;
	ros::Publisher *battery_pub;
``` 

Function for handling incoming messages:

``` cpp
	void twistCallback(const geometry_msgs::Twist &twist)
	{
		rosbot.setSpeed(twist.linear.x, twist.angular.z);
	}
``` 

Function for initialization of velocity command subscriber:

``` cpp
	void initCmdVelSubscriber()
	{
		ros::Subscriber<geometry_msgs::Twist> *cmd_sub = new ros::Subscriber<geometry_msgs::Twist>("/cmd_vel", &twistCallback);
		nh.subscribe(*cmd_sub);
	}
``` 

Function for initialization of battery state publisher:

``` cpp
	void initBatteryPublisher()
	{
		battery_pub = new ros::Publisher("/battery", &battery);
		nh.advertise(*battery_pub);
	}
``` 

Function for handling incoming requests of robot odometry reset:

``` cpp
	void resetCallback(const std_msgs::Bool &msg)
	{
		if (msg.data == true)
		{
			rosbot.reset_odometry();
		}
	}
``` 

Function for initialization of odometry reset requests subscriber:

```cpp
void initResetOdomSubscriber()
{
	ros::Subscriber<std_msgs::Bool> *odom_reset_sub = new ros::Subscriber<std_msgs::Bool>("/reset_odom", &resetCallback);
	nh.subscribe(*odom_reset_sub);
}
``` 

Main function, device and messages initialization:

``` cpp
    void hMain() {
	rosbot.initROSbot(sensor_type, imu_type);
	platform.begin(&RPi);
	nh.getHardware()->initWithDevice(&platform.LocalSerial);
	nh.initNode();

	initBatteryPublisher();
	initCmdVelSubscriber();
	initResetOdomSubscriber();
``` 
 
Infinite loop, waiting for incoming messages:

``` cpp
	while (true)
	{
		nh.spinOnce();
		publish_counter++;
		if (publish_counter > 10)
		{
			// get battery voltage
			battery.voltage = rosbot.getBatteryLevel();
			// publish battery voltage
			battery_pub->publish(&battery);
			publish_counter = 0;
		}
		sys.delay(10);
	}
``` 

Build your project and upload it to device.

### Running motor controller ###

In this section you will learn how to control your robot movement with
keyboard. You will need `teleop_twist_keyboard` node from
`teleop_twist_keyboard` package.

Log in to your CORE2 device through remote desktop and run terminal. In
first terminal window run `$ roscore`, in second run:

```
    $ /opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2
```

This program is responsible for bridging your CORE2 to ROS network. When you are working with simulator, then above bridge is not necessary. Gazebo will subscribe appropriate topics automatically.
In third terminal window run:

```
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

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

This section is required only for ROSbot. Gazebo has already implemented it's own plugin to publish robot position.
Now we will perform forward kinematics task- we will use encoders that
are attached to every motor and process their measurements with
equations shown in section **Forward kinematics task**.

Open Husarion WebIDE and open project that you created in section **Converting motion command to motor drive signal**.

Add header file:

``` cpp
    #include "geometry_msgs/PoseStamped.h"
	#include "tf/tf.h"
``` 

Define message type and publisher for robot position:

``` cpp
    geometry_msgs::PoseStamped pose;
	ros::Publisher *pose_pub;
```

Create a data structure:

```cpp
std::vector<float> rosbot_pose;
```

Function for publishing robot position:

```cpp
void initPosePublisher()
{
	pose.header.frame_id = "base_link";
	pose.pose.orientation = tf::createQuaternionFromYaw(0);
	pose_pub = new ros::Publisher("/pose", &pose);
	nh.advertise(*pose_pub);
}
```

In main function for initialization of PosePublisher:

	initPosePublisher();

Put values to message and publish them:

``` cpp
	// get ROSbot pose
	rosbot_pose = rosbot.getPose();
	pose.pose.position.x = rosbot_pose[0];
	pose.pose.position.y = rosbot_pose[1];
	pose.pose.orientation = tf::createQuaternionFromYaw(rosbot_pose[2]);
	// publish pose
	pose_pub->publish(&pose);
``` 

Your final code should look like this:

``` cpp
#include "hFramework.h"
#include "hCloudClient.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "ROSbot.h"

using namespace hFramework;

// Uncomment one of these lines, accordingly to range sensor type of your ROSbot
// If you have version with infared sensor:
// static const SensorType sensor_type = SENSOR_INFRARED;
// If you have version with laser sensor:
static const SensorType sensor_type = SENSOR_LASER;
// If you want to use your own sensor:
// static const SensorType sensor_type = NO_DISTANCE_SENSOR;

// Uncomment one of these lines, accordingly to IMU sensor type of your device
// If you have version with MPU9250:
static const ImuType imu_type = MPU9250;
// If you want to use your own sensor:
// static const ImuType imu_type = NO_IMU;

ros::NodeHandle nh;
sensor_msgs::BatteryState battery;
ros::Publisher *battery_pub;
geometry_msgs::PoseStamped pose;
ros::Publisher *pose_pub;

std::vector<float> rosbot_pose;

int publish_counter = 0;

void twistCallback(const geometry_msgs::Twist &twist)
{
	rosbot.setSpeed(twist.linear.x, twist.angular.z);
}

void initCmdVelSubscriber()
{
	ros::Subscriber<geometry_msgs::Twist> *cmd_sub = new ros::Subscriber<geometry_msgs::Twist>("/cmd_vel", &twistCallback);
	nh.subscribe(*cmd_sub);
}

void resetCallback(const std_msgs::Bool &msg)
{
	if (msg.data == true)
	{
		rosbot.reset_odometry();
	}
}

void initResetOdomSubscriber()
{
	ros::Subscriber<std_msgs::Bool> *odom_reset_sub = new ros::Subscriber<std_msgs::Bool>("/reset_odom", &resetCallback);
	nh.subscribe(*odom_reset_sub);
}

void initBatteryPublisher()
{
	battery_pub = new ros::Publisher("/battery", &battery);
	nh.advertise(*battery_pub);
}

void initPosePublisher()
{
	pose.header.frame_id = "base_link";
	pose.pose.orientation = tf::createQuaternionFromYaw(0);
	pose_pub = new ros::Publisher("/pose", &pose);
	nh.advertise(*pose_pub);
}

void hMain()
{
	rosbot.initROSbot(sensor_type);
	platform.begin(&RPi);
	nh.getHardware()->initWithDevice(&platform.LocalSerial);
	nh.initNode();

	initBatteryPublisher();
	initCmdVelSubscriber();
	initResetOdomSubscriber();
	initPosePublisher();

	while (true)
	{
		nh.spinOnce();
		publish_counter++;
		if (publish_counter > 10)
		{	
			// get ROSbot pose
			rosbot_pose = rosbot.getPose();
			pose.pose.position.x = rosbot_pose[0];
			pose.pose.position.y = rosbot_pose[1];
			pose.pose.orientation = tf::createQuaternionFromYaw(rosbot_pose[2]);
			// publish pose
			pose_pub->publish(&pose);	
			
			// get battery voltage
			battery.voltage = rosbot.getBatteryLevel();
			// publish battery voltage
			battery_pub->publish(&battery);
			publish_counter = 0;
		}
		sys.delay(10);
	}
}
```

Build your project and upload it to the device.

### Running motor controller with forward kinematics task ###

In this section you will control your robot with keyboard and observe as it publishes its own position.

If you are working with ROSbot:
Log in to your CORE2 device through remote desktop, run terminal and start your robot as previously. In another terminal window run:

```bash
    $ rostopic echo /pose
```

If you are working with Gazebo:
Start Gazebo as prevoiusly. In another terminal window run:

```bash
    $ rostopic echo /odom
```


Above difference comes from the fact, that Gazebo and ROSbot are publishing its position in different ways.

Remember, that you need to have active window with `teleop_twist_keyboard` to control robot movement.

You should get something like this on your screen:

![image](/assets/img/ros/man_3_2.png)

## Robot visualization with Rviz ##

Rviz is tool which allows visualization of robot position, travelled path,
planned trajectory, sensor state or obstacles surrounding robot.

To run it type in terminal:

```bash
    $ rviz
```

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

If you are working with ROSbot:
Find topic `/pose` and choose `Pose` and click **OK**.

If you are working with Gazebo:
Find topic `/odom` and choose `Odometry` and click **OK**.

Then in visualized items list find position `Fixed Frame` and change it
to `base_link`. At this stage, you will need to type it. Later it will be possible to choose frame names from dropdown list, this will be covered in tutorial 6.

After this is done, you should see an arrow representing position and orientation
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
