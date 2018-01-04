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

<div align="center">
<iframe width="800" height="388" src="https://www.youtube.com/embed/aqJC23XcnyI" frameborder="0" gesture="media" allowfullscreen></iframe>
</div>

ROSbot is an autonomous robot platform based on Husarion CORE2-ROS robot controller. It integrates:
- 4-wheels mobile platform containing DC motors with encoders and an alluminium frame
- Digital camera
- RPLIDAR A2 laser scanner
- MPU 9250 inertial sensor (accelerometer + gyro)
- rear panel providing interfaces for additional modules

ROSbot is an affordable robot platform for rapid development of autonomous robots. It can be a base for custom service robots, inspection robots and robots working in swarms. 

You can use your ROSbot offline however we recommend connecting to Husarion Cloud as it gives you access to a lot of additional functionalities like remote management and firmware updates.

If you do not own ROSbot yet, you can purchase it <a href="https://store.husarion.com/">here</a>.

# Hardware guide #

## Specification ##

<table>
    <tr>
       <th>Attribute</th>
       <th>Description</th>
    </tr>
    <tr>
        <td>Dimensions with camera and LiDAR</td>
        <td>285 x 235 x 147mm / 11.2 x 9.25 x 5.79in [L x W x H]</td>
    </tr>
    <tr>
        <td>Dimensions without camera and LiDAR</td>
        <td>240 x 235 x 106mm / 9.45 x 9.25 x 4.17in [L x W x H]</td>
    </tr>
    <tr>
        <td>Weight</td>
        <td>2,84kg / 100oz (with camera and LiDAR), 2,45kg / 86oz (without camera and LiDAR)</td>
    </tr>
    <tr>
        <td>Wheel diameter / Clearance / Wheelbase</td>
        <td>85mm / 22mm / 105mm</td>
    </tr>
    <tr>
        <td>Chassis material</td>
        <td>Powder-coated aluminum plate, 1.5mm thick</td>
    </tr>
    <tr>
        <td>Maximum speed</td>
        <td>0.7 m/s</td>
    </tr>
    <tr>
        <td>Maximum load capacity</td>
        <td>10kg / 352oz</td>
    </tr>
    <tr>
        <td>Battery life</td>
        <td>1.5h - 3h</td>
    </tr>
</table>

## Components ##

![Side scheme](/assets/img/ROSbot_manual/scheme_side.png "Side scheme")

![Scheme back](/assets/img/ROSbot_manual/scheme_back.png "Scheme back")

### Internal connections diagram ###

![Scheme connection](/assets/img/ROSbot_manual/scheme_connection.png "Scheme connection")

### Components description ###

<table>
    <tr>
       <th>Component</th>
       <th>Quantity</th>
       <th>Description</th>
    </tr>
    <tr>
        <td>CORE2-ROS</td>
        <td>1</td>
        <td>Advanced version of CORE2 with an ASUS Tinker board computer. CORE2 real-time controller is based on STM32F407 microcontroller. The SBC runs on Ubuntu-based OS, customized to use ROS. <a href="https://husarion.com/core2/">More details here</a>.</td>
    </tr>
    <tr>
        <td>LIDAR</td>
        <td>1</td>
        <td>RpLidar A2, 360 degree and up to 8m range, <a href="https://www.slamtec.com/en/Lidar"> more details</a>.</td>
    </tr>
    <tr>
        <td>Infrared distance sensor</td>
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
        <td>Li-Ion 18650 protected, rechargeable batteries, 2600mAh capacity, 3.7V nominal voltage <br>
		Note: Device may be shipped interchangeably with similar batteries.</td>
    </tr>
    <tr>
        <td>Antenna</td>
        <td>1</td>
        <td>Connected directly to the ASUS Tinker Board Wi-Fi module. Uses an RP-SMA(m) <-> I-PEX MHF4 cable to connect the antenna with SBC.</td>
    </tr>	
</table>

## Rear panel description ##

![Rear panel](/assets/img/ROSbot_manual/rear_panel.png "Rear panel")

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
        <td>Button used for connecting ROSbot to [Husarion Cloud](https://cloud.husarion.com/). Please use non-metallic object to press this button.</td>
    </tr>
    <tr>
        <td>Power switch</td>
        <td>1</td>
        <td>Switch to turn on and off the robot. It completely disconnectc all components of ROSbot from its power source.</td>
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

## Power supply ##

ROSbot is supplied from an internal, rechargeable Li-Ion battery pack (3x protected 18650 cells). ROSbot shall be charged using a dedicated Li-Ion or Li-Poly charger with 4-pin JST XH connector. 
If only the right firmware is preloaded to the internal controller (CORE2), the LED1 is programmed to indicate the power status:
- the LED1 is on when the robot is turned on
- the LED1 is blinking when battery is low – please charge immediately!

Please make sure that the user firmware always contains the function that monitors the supply voltage level. Deep discharging of batteries may decrease their lifecycle. Discharging to the voltage lower than 3.0V/cell can also trigger the overdischarge protection. If the voltage is too low please charge batteries as soon as possible.

If you are going to use ROSbot stationary for a long time, please leave the charger connected all the time. It will increase the batteries lifetime. In case you need to replace batteries, use only 18650 Li-Ion batteries, with the capacity in a range of 1800...3500mAh and with a protection circuit! Using unprotected batteries may result in serious injuries or fire.

## Software ##

Software for ROSbot can be divided into 2 parts:
 * A firmware that works on the real-time controller (CORE2) and can be developed and uploaded from [Husarion Cloud](https://cloud.husarion.com/) with WebIDE. It can also be developed offline using [Visual Studio Code IDE](https://husarion.com/core2/tutorials/howtostart/offline-development-tools/).
 * OS based on Ubuntu 16.04, which runs on the SBC (ASUS Tinker Board) and contains all components needed to start working with ROS immediately. The microSD card with OS is included with each ROSbot. The OS has been modified to make the file system insensitive to sudden power cuts.
 
 In some cases you will need to flash the OS image to the microSD card once again:
 - in case of accidential damage of the system,
 - to update the OS (it can be udpated remotely, but flashing the microSD card can be easier sometimes),
 - to clear all user changes and restore factory settings.
 To do that, you have to disassembly the top cover, unscrew the 4 screws on the CORE2 corners and carefully carry up CORE2 with SBC. Then you can change the microSD card and flash the OS. You can find the image and flash manual [here](https://husarion.com/core2/manuals/hardware/#hardware-os-image-for-raspberrypi-tinkerboard). If you want to replace the included card, remember that you need to use at least 16 GB capacity and 10 speed class micro SD card. 

# First steps #

## Connection to Husarion Cloud ##

* Things you need: the ROSbot, any Android device with Wi-Fi connectivity and with hConfig app installed (available on Google Play and appStore), any PC computer to work with ROSbot, the Wi-Fi network.
* Login or register on cloud.husarion.com.
* Register your ROSbot on your cloud account by clicking “Add new device”.
* Launch the hConfig application and follow the instructions.
Note: The app will ask you to hold hCfg button on CORE2 and to watch LR1, LR2 LEDs – they are all available on the rear panel.
* Now you should see your ROSbot online and you can start with ROSbot tutorial.

## ROS tutorials ##

ROS (Robot Operating System) provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. It's very powerful and 
functional tool dedicated to design robots. We created the set of [ROS tutorials dedicated for this platform](https://husarion.com/core2/tutorials/ros-tutorials/1-ros-introduction/ "ROS tutorials dedicated for this platform") to make it easier to familiarize yourself with these frameworks. 

# Docs and links #
All helpful documents and links in one place:

* [ROSbot Safety Instructions](https://files.husarion.com/docs2/ROSbot_safety_instructions_1.0.pdf "ROSbot Safety Instructions") - important!
* [ROSbot project on hackaday.io](https://hackaday.io/project/21885-rosbot-autonomous-robot-platform "ROSbot project on hackaday.io")
* [ROSbot project on instructables.com](http://www.instructables.com/id/ROSbot-Autonomous-Robot-With-LiDAR/ "ROSbot project on instructables.com")
* [ROSbot assembly instruction](https://cdn.hackaday.io/files/21885936327840/ROSbot_assembly_instruction.pdf "ROSbot assembly instruction")
* [ROSbot on ROS webpage](http://robots.ros.org/rosbot/)
* [ROSbot on ROS Wiki](http://wiki.ros.org/Robots/ROSbot/)
