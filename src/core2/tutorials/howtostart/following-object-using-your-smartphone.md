---
title: 'Following object using your smartphone'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 9
---

# Following object using your smartphone #

In this tutorial you will learn how to use smartphone sensors in your robotic design. Husarion created <a href="https://play.google.com/store/apps/details?id=com.husarion.node">hNode Android app</a> which allows using smartphone sensors in the robotic system based on ROS. Typical smartphone contains a camera, distance sensor, microphone, accelerometer, gyro and more. Smartphones are currently so common, that in many cases it's much cheaper to use them instead of buying dedicated sensors for a robot.

To get you started with the hNode app we prepared a quick example showing how to use it. We made a robot that autonomously follows any object. 

## Architecture of the system ##

The robotic system consists of:
1.  a simple mobile robot platform using DC motors with quadrature encoders. We built it based on <a href="https://husarion.com/core2/manuals/core2/">CORE2-ROS controller</a> and LEGO Mindstorms mobile platform. Of course you can use any other mechanics, or buy a ready set such as <a href="https://www.sparkfun.com/products/10336">this one</a> .
2. smartphone running <a href="https://play.google.com/store/apps/details?id=com.husarion.node">hNode app</a>
3. your laptop running Linux, ROS and Husarnet, natively or in a Virtual Machine. <a href="https://files.husarion.com/husarion-vm.vmdk.xz">Here is a ready to use Linux image</a>

Robot, smarthone and laptop are connected using Husarnet - a VPN, P2P network by Husarion. They form a so called "Virtual Robot". So even if they work in different networks, they always know the addresses of each other, and the communication between them is very fast, without the need of using any external servers etc. You can threat the "Virtual Robot" as a single robot with internet disrtributed components.

Even tough CORE2-ROS computer has enough computing power to perform robot operation we are using Laptop to show you that you are not limited to the computing power inside the robot itself.

## Connect virtual robot elements to the cloud ##

### a. smartphone ###

1. Log into <a href="https://cloud.husarion.com">Husarion cloud in the web browser</a>, click "Add new", choose a name for your smartphone (eg. "myPhone1"), and click "Next". You will see a QR code appear.
2. Install <a href="https://play.google.com/store/apps/details?id=com.husarion.node">hNode Android app</a> on your smartphone, open it, click "SCAN QR CODE" button, and scan QR code from your web browser
3. On your Husarion cloud account you should now see your phone listed:

<img src="/assets/img/husarnet/cloud_unasigned_phone.PNG" alt="unasigned phone" style="border:1px solid LightGray">


### b. virtual machine running Linux ###

1. Using <a href="https://www.virtualbox.org/">VirtualBox</a> (or other virtual machine hypervisor) open a <a href="https://files.husarion.com/husarion-vm.vmdk.xz">ready to use machine provided by Husarion</a> 

To make sure all packets are up to date run the following commands in the terminal inside virtual machine:

```
>sudo apt-get update
>sudo apt-get dist-upgrade
```

2. Run the virutal machine, open a terminal inside of it, and type in the following command:
`sudo husarnet websetup`
3. Open a link which will appear in the web browser:

<img src="/assets/img/husarnet/husarnet_websetup.PNG" alt="husarnet websetup" style="border:1px solid LightGray">
    
4. You will see a web panel. Choose a name for your virtual machine, eg. "ubuntu-vm". In the section "Join virtual robot" select "None" and click "Add device to your account" button.
5. Now on your Husarion cloud account you should see two unasigned elements (your phone and your virtual machine:

<img src="/assets/img/husarnet/cloud_unasigned_vm.PNG" alt="cloud unasigned" style="border:1px solid LightGray">

```
Ready to use virual machine is prepared for your convenience. 

If you would like to connect any Linux machine here is the manual: https://todo
```

### c. robot based on CORE2-ROS controller ###

1. Install <a href="https://files.husarion.com/ros-image-stable.img.xz">the newest Linux image for CORE2-ROS</a>  on the micro SD card (<a href="https://etcher.io/">Etcher</a> is a great tool to do this). Then put that micro SD card back in your CORE2-ROS single board computer slot.
2. Click hCfg button and turn CORE2-ROS on. Hold hCfg pressed until LR1 and LR2 will start blinking alternately
3. Log into <a href="https://cloud.husarion.com">Husarion cloud in the web browser</a>, click "Add new", choose a name for your robot (eg. "core2-robot"), and click "Next". You will see a QR code appear.
4. Install hConfig - Husarion config app for  <a href="https://play.google.com/store/apps/details?id=com.husarion.configtool2">Android</a> or  <a href="https://itunes.apple.com/us/app/hconfig/id1283536270?mt=8">iOS</a> . 
5. Connect your smartphone to a Wi-Fi network hosted by your CORE2-ROS (password: "husarion") and run hConfig. Wizard will guide you through the configuration process. In the last step you will scan a QR code displayed in the web browser (see step 3.)
6. After the process is finished you should see a "core2-robot" under your Husarion cloud account:

<img src="/assets/img/husarnet/cloud_robot.PNG" alt="cloud robot" style="border:1px solid LightGray">


## Creating a Virtual Robot ##

1. Click "Edit" button next to "ubuntu-vm" and select "Link to virtual robot". Select "Create new virtual robot" and choose a name, eg. "myVirtualRobot"
<img src="/assets/img/husarnet/virtual_robot.PNG" alt="virtual robot 1" style="border:1px solid LightGray">
2. Click "Add" button and you will see the following box:
<img src="/assets/img/husarnet/virtual_robot2.PNG" alt="virtual robot 2" style="border:1px solid LightGray">
3. Now click "Add component" and add your smartphone and robot to "myVirtualRobot". After completing those steps you will see your components listed as below:
<img src="/assets/img/husarnet/virtual_robot3.PNG" alt="virtual robot 3" style="border:1px solid LightGray">
4. Now select which of the virtual robot components will run "ROS master" node ('roscore' command). In this project "ubuntu-vm" runs ROS master:
<img src="/assets/img/husarnet/select-ros-master.png" alt="select ROS master" style="border:1px solid LightGray">

## Communicating between Virtual Robot elements ##

Open a terminal inside CORE2-ROS (host: core2-robot) or the virtual machine (host: ubuntu-vm) and type in:

* `ping6 hostname` - ping other device inside a virtual robot 

* `ssh hostname` - SSH other device inside a virtual robot

## Programing CORE2-ROS ##

First we have to program the CORE2-ROS using the following code. Web IDE at cloud.husarion.com or Visual Studio Code with Husarion extension are recommended for doing that.

<script src="https://gist.github.com/DominikN/2d07b385c6f9ed20339dc924a4be5fe1.js"></script>

## Setting up ROS workspace inside the virtual machine ##

Now open the terminal on your ROS_MASTER device and run:

```
roscore
```

![image](/assets/img/husarnet/console_1.png)

In the second terminal connect with CORE2-ROS via ssh. Use

```
ssh yourCORE2ROShostname
```

then answer `yes` and type password. You will see Husarion graphic appear- it means that you are connected to CORE2-ROS terminal.

![image](/assets/img/husarnet/console_2.png)

Next type in:

```
/opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2
```
 
to start the communication between CORE2 and linux SBC. You will see the list of publishers and subscribers. 

You have to create your own ROS node. You can find the full instruction on creating workspace and nodes explained <a href="https://husarion.com/core2/tutorials/ros-tutorials/2-creating-nodes/">here</a>. 

Open console on the device you want to create workspace on and type in:

```
mkdir ~/ros_workspace
mkdir ~/ros_workspace/src
cd ~/ros_workspace/src
catkin_init_workspace
cd ~/ros_workspace
catkin_make
```
Wait until the process is finished. You should see the following:

[obraz wyniku catkin_make]

```
####
#### Running command: "make -j4 -l4" in "/home/pi/ros_workspace/build"
####
```

in the console after it. You should also edit file .bashrc to make it possible to use your own node in any directory. You can do it by typing in:

`nano ~/.bashrc`

When you will see .bashrc file content, go to the end and add the following line:

`. /home/husarion/ros_workspace/devel/setup.sh`

Save the file and close the editor.

Now you will create our own package. Note that your newly created package should depend on package roscpp. 

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
and add the following phrase after it:
```
add_executable(object_follower src/object_follower.cpp)
```
Find also:
```
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
```
and add the following phrase after it:
```
target_link_libraries(object_follower
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
```

## Setting up object recognition ##

### a. stream video to "/localimg" topic ###
Open new tab in the terminal inside your virtual machine (ubuntu-vm) and type (if your host name for smarthpone is "myphone1"):

`rosrun image_transport republish compressed in:=/myphone1/camera1/image out:=/localimg`

### b. learn the object which will be followed by the robot ###

1. Create a file called "find.launch" on your Desktop at ubuntu-vm machine and paste the following code inside:
```
<launch>
    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/localimg"/>
        <param name="gui" value="true"/>
    </node>
</launch>
```
2. Open a new tab in the terminal at ubuntu-vm and run "find.launch" file:
```cd ~/Desktop```
```roslaunch find.launch```
3. A new window with "Find-Object" program will appear. Click Edit -> Add object from scene... , place the object you want the robot to learn in front of your smartphones camera  and click "Take picture" button.
4. Now select th area representing the object and click "Next" button, and then "End" button
5. ID of your image is shown in the upper-left corner of the learned image (if you do it the first time, the ID should be "1")
6. Click File -> Save objects ... and save it in the "/home/husarion/ros_workspace/object" folder
7. Kill *find_object_2d* program using ```rosnode kill /find_object_2d```
8. If the above steps weren't clear enough, visit <a href="https://husarion.com/core2/tutorials/ros-tutorials/4-visual-object-recognition/">detailed tutorial on learning new objects</a>.


## Run a control code on ubuntu-vm ##

Now create source file for your node:

```
cd ~/ros_workspace/src/tutorial_pkg/src/
touch object_follower.cpp
```

This node contains previously saved object for recognition, in this case `#define OBJECT 1`. Remember that number of your object have to be the same as the number of the object defined in line number 7. Open file object_follower.cpp and paste this code:

<script src="https://gist.github.com/DominikN/484d54686a0fbd46b2fa138620dfca28.js"></script>

Now change number "1" to your object number in line number 7:

```
#define OBJECT 1
```

Go to your workspace directory one more time and build your new node:

```
cd ~/ros_workspace
catkin_make
```

Now you will create find.launch file. Type in the following in the console:

```
cd ~/Desktop
touch find.launch
```

Modify "find.launch" which was previously created on the desktop

```
<launch>
    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/localimg" />
        <param name="gui" value="true" />
        <param name="objects_path" value="/home/husarion/ros_workspace/object" />
    </node>
    
    <node pkg="tutorial_pkg" type="object_follower" name="object_follower"></node>
</launch>
```

Open new tab in the terminal, reach directory of your launch file and type in

```
cd ~/Desktop
roslaunch find.launch
```

You will see find_object_2d GUI with your saved object.

## Testing ##

Place saved object in the field of view of the smartphone camera and observe the behaviour of your robot.

After robot has recognized the object, it will follow it, as you can see on video below:


## Additional tips ##

If you want to use camera image from your smartphone, just install compressed image transport:

```apt-get install -y ros-kinetic-compressed-image-transport```

To view image from the camera locally, use:

```rosrun image_view image_view image:=/yourphonehostname/camera0/image _image_transport:=compressed```

Make sure to accept the permission dialog on the device. Use camera1 instead of camera0 to access second camera, if you have one.

If you have some node which can't process compressed image, launch the node that decompresses it locally (republisher):

``` rosrun image_transport republish compressed in:=/yourphonehostname/camera0/image out:=/localimg ```

After that, raw image will be available in `/localimg` channel.
