---
title: 'Telepresence robot kit - quick start'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 4
---
# Telepresence robot kit - quick start #

<iframe width="800" height="388" src="https://www.youtube.com/embed/JkIj5ssHpKw" frameborder="0" gesture="media" allowfullscreen></iframe>

Telepresence robot kit allows you to build your own telepresence robot in just about minutes. It's source code is open, and is available at our <a href="https://github.com/husarion/self-balancing-telepresence-robot">GitHub page</a> or at IDE in <a href="https://cloud.husarion.com/">Husarion cloud</a>. Feel free to modify that code, add your own sensors or actuators on top of the robot. To read more about algorithm working on the self balancing robot visit our <a href="https://medium.com/husarion-blog/fresh-look-at-self-balancing-robot-algorithm-d50d41711d58">article at Medium</a>.

If you don't have one, you can purchase the complete kit <a href="https://store.husarion.com/">here</a>.

## Unboxing ##

The kit contains:

* CORE2 robotic controller
* Metal robot frame
* 2 x Wheel
* 3 x Li-Ion batteries 18650
* Li-Ion charger 
* Screwdriver

## Assembly ##

To make your robot work properly, you need to complete few following steps written below. Start with mechanical assembly:

*	Put each wheel on the coupler and screw it on with M4 screw.
*	Put CORE2 on the designated place and fix it with four M3 screws.

## Cables ##

Connect the peripherals:

* Left motor to hMotD
* Right motor to hMotC
* Servo to servo1
* IMU to hSens2
* Insert three Li-ion 18650 batteries into the battery holder
* Plug in DC power cable.

***<font color="red">very important: DC motors with encoders and servo should be connected to proper ports like this:</font>***
<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src//assets/img/howToStart/cables_ralph.png" height="50%" width="50%"></center></div>

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src//assets/img/telepresence_robot_kit/ralph_alu_back_hd.jpg"></center></div>
<div style="text-align: left"><i>Properly assembled CORE2 telepresence robot</i></div>

## Connecting to the cloud ##

* Press and hold the hCfg button on CORE2.
* Turn on a power switch.
* When blue and red LEDs start blinking, release the hCfg button.
* Connect your mobile device to Husarion Wi-Fi and open hConfig app (<a href="https://itunes.apple.com/us/app/hconfig/id1283536270?mt=8">hConfig in AppStore</a> or <a href="https://play.google.com/store/apps/details?id=com.husarion.configtool2">hConfig in Google Play</a>) to connect CORE2 to the Wi-Fi network and your user account at <a href="https://cloud.husarion.com">cloud.husarion.com</a> (<a href="https://husarion.com/core2/tutorials/howtostart/run-your-first-program/#run-your-first-program-connecting-to-the-cloud">how to do this</a>).
* Install (<a href="https://play.google.com/store/apps/details?id=com.husarion.video2">hVideo app</a>) on your tablet/smartphone that you are going to integrate into robot.
* Open hVideo app, and click "Add camera" next to your robot name at <a href="https://cloud.husarion.com">cloud.husarion.com</a>.

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src//assets/img/howToStart/add_camera_ralph.png"
/></center></div>

* Scan QR Code - your device is now paired with your robot.
* Insert a tablet/smartphone to your robot and screw on mounting elements.

## Programming ##

* Place your robot with CORE2 facing up and turn it on.
* Open WebIDE at <a href="https://cloud.husarion.com/">Husarion cloud</a> and create a new project using a template ***'08. self-balancing telepresence MPU9250 DMP (CORE2)'*** or '07. self-balancing telepresence - deprecated (CORE2)' and upload it to your CORE2.

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src//assets/img/howToStart/create_new_project_ralph.png"
/></center></div>
* When robot is programmed, open its user interface from <a href="https://cloud.husarion.com/">Husarion cloud main page</a> (left-click it's name).

## Calibration procedure ##

After uploading a new firmware to the robot, you have to calibrate its IMU to work properly. You can do it in a couple easy steps:
* In opened Web UI of your robot press "start" button in Web UI to hide a leg. Hold lightly a robot on the top to prevent falling (when it's not calibrated it may happen)
* Press "f" button on your keyboard to turn off motors (keep holding your robot - without that it will fall)
* Pinch upper part of your robot and by pushing it back and forward set it to balanced vertical orientation (a point in which it is neither falling forward or backward) and press "c" (calibrate) button in opened Web UI to save that stable angle.
* Press "f" once more to turn on motors.
* Now robot is calibrated and ready to use. That procedure need only be done once after programming, but you can modify 'angle0' initial value in RobotController.cpp file to hard code proper calibrated value for your hardware.

***<font color="red">Make sure a web browser tab with robot web UI opened is active during these steps</font>***

## Controlling ##

To steer your robot use WSAD keys or arrows. Use 'P' key to switch leg position and 'U', 'I', 'O' keys to change max speed.
