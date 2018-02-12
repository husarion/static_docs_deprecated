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
- IMU inertial sensor (accelerometer + gyro)
- 3 x Li-Ion batteries 18650
- Li-Ion charger 
- Screwdriver

If you don't have one, you can purchase the complete kit <a href="https://store.husarion.com/">here</a>.


# Hardware guide #

## Specification ##

<table>
    <tr>
       <th>Attribute</th>
       <th>Description</th>
    </tr>
    <tr>
        <td>Dimensions with camera and LiDAR</td>
        <td>100 x 232 x 260mm / 3.94 x 9.13 x 10.24in [L x W x H]</td>
    </tr>
    <tr>
        <td>Weight</td>
        <td>0.95kg / 33.5oz (without tablet)</td>
    </tr>
    <tr>
        <td>Wheel diameter / Clearance</td>
        <td>85mm / 9mm</td>
    </tr>
    <tr>
        <td>Chassis material</td>
        <td>Powder-coated aluminum plate, 2mm thick</td>
    </tr>
    <tr>
        <td>Maximum speed</td>
        <td>0.5 m/s</td>
    </tr>
    <tr>
        <td>Battery life</td>
        <td>3h - 6h</td>
    </tr>
    <tr>
        <td>Maximum tablet dimensions</td>
        <td>202 x 136 x 9mm / 7.95 x 5.35 x 0.35in (iPad mini 7,9")</td>
    </tr>
</table>

## Components description ##

<table>
    <tr>
       <th>Component</th>
       <th>Quantity</th>
       <th>Description</th>
    </tr>
    <tr>
        <td>CORE2</td>
        <td>1</td>
        <td>CORE2 is real-time controller, based on STM32F407 microcontroller.<a href="https://husarion.com/core2/">More details here</a>.</td>
    </tr>
    <tr>
        <td>DC motor</td>
        <td>2</td>
        <td>Xinhe Motor XH-25D
		Motor used: RF-370, 6VDC nominal, 5000rpm
		No load speed at the output shaft: 165 rpm
		Stall torque: 2.9 kg*cm
		Stall current: 2.2A
		Gear ratio: ~34 (exact ratio is 30613/900)
		Encoder: magnetic, 48ppr, 12 poles</td>
    </tr>
    <tr>
        <td>IMU sensor</td>
        <td>1</td>
        <td>Powerful 9-Axis Accel/Gyro/Magnetometer sensor with MPU-9250, <a href="https://husarion.com/core2/manuals/core2/#hardware-mpu9250-inertial-mesurement-unit"> more details</a>.</td>
    </tr>
    <tr>
        <td>Batteries</td>
        <td>3</td>
        <td>Li-Ion 18650 protected, rechargeable batteries, 2600mAh capacity, 3.7V nominal voltage <br>
		Note: Device may be shipped interchangeably with similar batteries.</td>
    </tr>
    <tr>
        <td>Servo</td>
        <td>1</td>
        <td>Tower Pro MG90S micro servo to move the foot. <a href="http://www.towerpro.com.tw/product/mg90s-3/">More details here</a>.</td>
    </tr>	
</table>

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
<div style="text-align: center"><i>Properly assembled CORE2 telepresence robot</i></div>

## Charging ##

Telepresence robot gets power from 3 Li-Ion cells. As with all reachargeable batteries they should be treated in an appropriate way. The basic rules are:
* Use the charger included in the kit. If you don't have one, use a charger for Li-Ion or Li-Poly batteries with at most 1A charging current and balancing feature.
* Don't discharge the batteries below 3.5V per cell (10.5V for all cells). Further discharge can trigger the overdischarge protection, built into each cell. In such case some of chargers would not allow to charge batteries because of too low voltage - the protection simply cut-offs on of terminals. The charger can interpret this as damaged battery and will not charge it. The workaround for that is to connect the battery to the other current-limited voltage source for a few seconds - it will turn off the protection and the battery would charge again with normal charger as before (if it is not damaged).
* Use robot UI or LED controls to monitor the battery state. The best option is when the firmware disables the motors in case of low battery state. Such feature will help you to avoid discharging batteries too much.
* If you don't use robot for a long time, leave the batteries partially charged (about 11.4V for all cells). It's a recommended charge level for a long-term storage.

# Software #

## Programming and interface ##

* Place your robot with CORE2 facing up and turn it on.
* Open WebIDE (cloud.husarion.com) and create a new project using template 'self-balancing telepresence' and upload it to your CORE2.

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src//assets/img/howToStart/create_new_project_ralph.png"
/></center></div>
* When robot is programmed, open its user interface and click 'Turn off motors'.
<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/telepresence_robot_kit/interface.png"
/></center></div>
* Pinch upper part of your robot and by pushing it back and forward set it to balanced vertical orientation.
* When you find proper position (when you are using the less force to hold it stable) click on 'Calibrate' button in Web UI.
* Now after clicking 'turn on motors' your robot will start balancing correctly.


To steer your robot use WSAD keys or arrows. Use G key to switch leg position and 1, 2, 3 keys to change gears.

# Docs and links #
All helpful documents and links in one place:

* [Telepresence robot kit Safety Instructions](https://files.husarion.com/docs2/ROSbot_safety_instructions_1.0.pdf "ROSbot Safety Instructions") - important!
* [Telepresence robot kit Tutorial](https://husarion.com/core2/tutorials/howtostart/telepresence-robot-kit---quick-start/ "ROSbot Safety Instructions")
