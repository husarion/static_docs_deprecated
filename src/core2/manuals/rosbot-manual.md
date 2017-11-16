---
title: 'ROSbot manual'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
order: 2
page: 'Manuals'
onepager: true
---

<div class="gallery h300">

![Front](/assets/img/ROSbot_manual/colour_front.jpg "Front")
![Back](/assets/img/ROSbot_manual/colour_back.jpg "Back")
![Perspective](/assets/img/ROSbot_manual/colour_perspective.jpg "Perspective")

</div>

# Overview #

<iframe width="854" height="480" src="https://vimeo.com/225576807" frameborder="0" allowfullscreen> </iframe>

ROSbot is an autonomous robot platform powered by Husarion. It's hard to find an affordable robot platform for rapid autonomous robot development. To find a platform that can be a base for custom service robots, inspection robots and robots working in swarms. This is why we are creating ROSbot, as a project that everybody can use to avoid building own autonomous robot platforms from scratch.

![Side scheme](/assets/img/ROSbot_manual/scheme_side.png "Side scheme")
![image](https://github.com/husarion/static_docs/blob/master/src/assets/img/ROSbot_manual/scheme_connection.png)

<table>
    <tr>
       <th>Component</th>
       <th>Quantity</th>
       <th>Description</th>
    </tr>
    <tr>
        <td>CORE2</td>
        <td>1</td>
        <td>Development board for Internet-connected automation & robotics devices, <a href="https://husarion.com/core2/">details 
here</a>.</td>
    </tr>
    <tr>
        <td>Asus Tinker Board</td>
        <td>1</td>
        <td>Powerful single board computer powered by Asus to use ROS, <a href="https://www.asus.com/us/Single-Board-Computer/Tinker-Board/">more details</a>.</td>
    </tr>
    <tr>
        <td>LIDAR</td>
        <td>1</td>
        <td>RpLidar A2, 360 degree and up to 8m range, <a href="https://www.slamtec.com/en/Lidar"> more details</a>.</td>
    </tr>
    <tr>
        <td>Ultrasonic distance sensor</td>
        <td>4</td>
        <td>SHARP GP2Y0A41SK0F with 4 to 30 cm range, <a href="https://husarion.com/core2/manuals/hardware/#hardware-sharp-distance-sensor"> more details</a>.</td>
    </tr>
    <tr>
        <td>DC motor</td>
        <td>4</td>
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
        <td>Powerful 9-Axis Accel/Gyro/Magnetometer sensor with MPU-9250, <a href="https://husarion.com/core2/manuals/hardware/#hardware-mpu9250-inertial-mesurement-unit"> more details</a>.</td>
    </tr>
    <tr>
        <td>Digital camera</td>
        <td>1</td>
        <td>1.0 megapixel RGB camera with CMOS OV9712 H.264</td>
    </tr>
    <tr>
        <td>Batteries</td>
        <td>3</td>
        <td>Li-on 18650 protected, rechargeable batteries, 2600mAh capacity, 3.7V nominal voltage 
		Notes: Use only protected batteries! Using unprotected batteries may result in serious injuries or fire.  Device may be shipped interchangeably with similar batteries.</td>
    </tr>
</table>

***

# Rear panel description #

![image](https://github.com/husarion/static_docs/blob/master/src/assets/img/ROSbot_manual/scheme_back.png)

<table>
    <tr>
       <th>Component</th>
       <th>Quantity</th>
       <th>Description</th>
    </tr>
    <tr>
        <td>HDMI</td>
        <td>1</td>
        <td>From SBC</td>
    </tr>
    <tr>
        <td>USB 2.0</td>
        <td>2</td>
        <td>From SBC</td>
    </tr>
    <tr>
        <td>Micro USB</td>
        <td>1</td>
        <td>Serial USB from CORE2</td>
    </tr>
    <tr>
        <td>Charging connector</td>
        <td>1</td>
        <td>Standard 4-pin connector for charging 3 Li-Ion batteries</td>
    </tr>
    <tr>
        <td>hCfg button</td>
        <td>1</td>
        <td>Button used for connecting ROSbot to [husarion cloud](https://cloud.husarion.com/)</td>
    </tr>
    <tr>
        <td>Power switch</td>
        <td>1</td>
        <td>Switch to turn on and off the robot.</td>
    </tr>
    <tr>
        <td>LED</td>
        <td>3</td>
        <td>LR1(yellow), LR2(blue), L1(red), more details <a href="https://husarion.com/core2/manuals/hardware/#hardware-leds">here</a>.</td>
    </tr>
    <tr>
        <td>Outputs for servo</td>
        <td>6</td>
        <td>Servo output with PWM, more details <a href="https://husarion.com/core2/manuals/hardware/#hardware-hservo">here</a>.</td>
    </tr>
	    <tr>
        <td>hExt</td>
        <td>1</td>
        <td>12xGPIO, 7x ADC, SPI, I2C, UART, more details <a href="https://husarion.com/core2/manuals/hardware/#hardware-hext">here</a>.</td>
    </tr>
    <tr>
        <td>hSens</td>
        <td>1</td>
        <td>4 xGPIO, ADC, UART, more details <a href="https://husarion.com/core2/manuals/hardware/#hardware-hsensor">here</a>.</td>
    </tr>
</table>

# Software #

ROSbot uses OS based on Ubuntu 16.04 which contains all components needed to start working with ROS immediately. You can find the image and flash manual [here](https://husarion.com/core2/manuals/hardware/#hardware-os-image-for-raspberrypi-tinkerboard). We also created the set of [ROS tutorials dedicated for this platform](https://husarion.com/core2/tutorials/ros-tutorials/1-ros-introduction/ "ROS tutorials dedicated for this platform") to make it easier to familiarize yourself with these frameworks. 


# Docs and links #
All helpful documents and links in one place:

* [CORE2 Safety Instructions](http://files.husarion.com/doc_files/CORE2_Safety_Instructions.pdf "CORE2 Safety Instructions") - important!
* [ROSbot project on hackaday.io](https://hackaday.io/project/21885-rosbot-autonomous-robot-platform "ROSbot project on hackaday.io")
* [ROSbot project on instructables.com](http://www.instructables.com/id/ROSbot-Autonomous-Robot-With-LiDAR/ "ROSbot project on instructables.com")
* [ROSbot assembly instruction](https://cdn.hackaday.io/files/21885936327840/ROSbot_assembly_instruction.pdf "ROSbot assembly instruction")
