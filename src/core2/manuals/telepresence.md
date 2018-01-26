---
title: 'Telepresence robot kit manual'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
order: 4
page: 'Manuals'
onepager: true
---

<div class="gallery h300">

![Front](/assets/img/telepresence_robot_kit/ralph_alu_izo_hd.jpg "Front")
![Right](/assets/img/telepresence_robot_kit/ralph_alu_right_hd.jpg "Right")

</div>

# Overview #

<div align="center">
<iframe width="800" height="388" src="https://www.youtube.com/embed/JkIj5ssHpKw" frameborder="0" gesture="media" allowfullscreen></iframe>
</div>

Telepresence robot kit is a development kit based on Husarion CORE2 controller. It integrates:

- 2-wheels solid frame containing DC motors with encoders and an alluminium bumpers
- MPU 9250 inertial sensor (accelerometer + gyro)
- rear leg with mini servo
- 3 x Li-Ion batteries 18650
- Li-Ion charger 
- Screwdriver

If you don't have one, you can purchase the complete kit <a href="https://store.husarion.com/">here</a>.


#Hardware guide#

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

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src//assets/img/howToStart/ralph_alu_back_hd.jpg"></center></div>
<div style="text-align: left"><i>Properly assembled CORE2 telepresence robot</i></div>

# Software #

## Programming and interface ##

* Place your robot with CORE2 facing up and turn it on.
* Open WebIDE (cloud.husarion.com) and create a new project using template 'self-balancing telepresence' and upload it to your CORE2.

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src//assets/img/howToStart/create_new_project_ralph.png"
/></center></div>
* When robot is programmed, open its user interface and click 'Turn off motors'.
<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/telepresence_robot_kit/ralph_interface.jpg"
/></center></div>
* You can do it also by browser on your smartphone.
<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/telepresence_robot_kit/ralph_interface_mobile.jpg"
/></center></div>
* Pinch upper part of your robot and by pushing it back and forward set it to balanced vertical orientation.
* When you find proper position (when you are using the less force to hold it stable) click on 'Calibrate' button in Web UI.
* Now after clicking 'turn on motors' your robot will start balancing correctly.


To steer your robot use WSAD keys or arrows. Use G key to switch leg position and 1, 2, 3 keys to change gears.

# Docs and links #
All helpful documents and links in one place:

* [ROSbot Safety Instructions](https://files.husarion.com/docs2/ROSbot_safety_instructions_1.0.pdf "ROSbot Safety Instructions") - important!
* [ROSbot project on hackaday.io](https://hackaday.io/project/21885-rosbot-autonomous-robot-platform "ROSbot project on hackaday.io")
* [ROSbot project on instructables.com](http://www.instructables.com/id/ROSbot-Autonomous-Robot-With-LiDAR/ "ROSbot project on instructables.com")
* [ROSbot assembly instruction](https://cdn.hackaday.io/files/21885936327840/ROSbot_assembly_instruction.pdf "ROSbot assembly instruction")
* [ROSbot on ROS webpage](http://robots.ros.org/rosbot/)
* [ROSbot on ROS Wiki](http://wiki.ros.org/Robots/ROSbot/)
