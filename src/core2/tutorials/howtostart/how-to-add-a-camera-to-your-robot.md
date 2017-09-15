---
title: 'How to add a camera to your robot'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 2
---

# How to add a camera to your robot #
## Setting up the hardware ##
### CORE2 + smartphone/tablet ###
Your robot with CORE2 can use a camera that is built into a smartphone or a tablet. The video from camera can be viewed in the robot's user interface. Currently this feature is available only for Android devices.

At first, your CORE2 needs to be connected to your account on [cloud.husarion.com](https://cloud.husarion.com/).

Install the [hVideo](https://play.google.com/store/apps/details?id=com.husarion.video2&hl=en) app on your smartphone or tablet. Enable Wi-Fi or mobile data transmission on the device.

Login to the Husarion Cloud, find your robot, click <img src="/assets/img/howToStart/plus.png"/> and "Add camera".

<img src="/assets/img/howToStart/cloud_add_camera.png"/>

Click "Generate new QR code" in the pop-up window:

<img src="/assets/img/howToStart/add_camera.png"/>

Launch the hVideo app on your smartphone, choose the "Pair with robot" option in the hVideo menu:

<img src="/assets/img/howToStart/pair-with-robot.png"/>

and scan the QR code displayed at Husarion Cloud with your smartphone.

Now the camera in your smarthpone is linked to your Cloud account and the stream should be visible in the Web UI of the robot. 

### CORE2-ROS + USB camera ###

CORE2-ROS contains a single board computer with Linux operating system and USB ports. It means that in most cases you only need to connect a USB camera and enable it.

At first, your CORE2-ROS needs to be connected to your account on [cloud.husarion.com](https://cloud.husarion.com/).

Login to the Husarion Cloud, find your robot, click <img src="/assets/img/howToStart/plus.png"/> and "Add camera".

<img src="/assets/img/howToStart/cloud_add_camera.png"/>

Click "Enable USB camera" in the pop-up window:

<img src="/assets/img/howToStart/add_camera.png"/>

If you don't have a non-standard camera, it shouldn't need any additional configuration and the video stream should now be available in the Web UI of your robot.

## Displaying the stream in the robot UI ##

