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

This kit allows you to build your own telepresence robot in just about minutes. 

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
* Install (<a href="https://play.google.com/store/apps/details?id=com.husarion.video2">hVideo app</a>) on your tablet/smartphone.
* Open hVideo app, and click "Add camera" next to your robot name at <a href="https://cloud.husarion.com">cloud.husarion.com</a>.

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src//assets/img/howToStart/add_camera_ralph.png"
/></center></div>

* Scan QR Code - your device is now paired with your robot.
* Insert device to your robot and screw on mounting elements.

## Programming ##

* Place your robot with CORE2 facing up and turn it on.
* Open WebIDE (cloud.husarion.com) and create a new project using template 'self-balancing telepresence' and upload it to your CORE2.

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src//assets/img/howToStart/create_new_project_ralph.png"
/></center></div>
* When robot is programmed, open its user interface and click 'Turn off motors'.
* Pinch upper part of your robot and by pushing it back and forward set it to balanced vertical orientation.
* When you find proper position (when you are using the less force to hold it stable) click on 'Calibrate' button in Web UI.
* Now after clicking 'turn on motors' your robot will start balancing correctly.



To steer your robot use WSAD keys or arrows. Use G key to switch leg position and 1, 2, 3 keys to change gears.
