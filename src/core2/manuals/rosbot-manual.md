---
title: 'ROSbot manual'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
order: 3
page: 'Manuals'
onepager: true
---
# Overview #

ROSbot is an autonomous robot platform based on Husarion CORE2-ROS robot controller available in two version: 2.0 and 2.0 PRO. 

<div align="center">
<iframe width="784" height="441" src="https://www.youtube.com/embed/QHJFNMX4Us8" frameborder="0" gesture="media" allowfullscreen></iframe>
</div>

ROSbot is an affordable robot platform for rapid development of autonomous robots. It can be a base for custom service robots, inspection robots and robots working in swarms. Both version integrates:

- 4-wheels mobile platform containing DC motors with encoders and an aluminum frame
- Orbbec Astra RGBD camera
- MPU 9250 inertial sensor (accelerometer + gyro)
- rear panel providing interfaces for additional modules

In ROSbot 2.0:
- CORE2-ROS controller with <a href="https://www.asus.com/pl/Single-Board-Computer/Tinker-Board/">Asus Tinker Board</a>
- RPLIDAR A2 laser scanner

<div class="gallery h300">

![Front](/assets/img/ROSbot_manual/colour_front.jpg "Front")
![Back](/assets/img/ROSbot_manual/colour_back.jpg "Back")
![Perspective](/assets/img/ROSbot_manual/colour_perspective.jpg "Perspective")

</div>

In ROSbot 2.0 PRO:
- CORE2-ROS controller with <a href="http://www.up-board.org/up/">UpBoard UP</a> 
- RPLIDAR A3 laser scanner

<div class="gallery h300">

![Front](/assets/img/ROSbot_manual/pro_colour_front.jpg "Front")
![Back](/assets/img/ROSbot_manual/pro_colour_back.jpg "Back")
![Perspective](/assets/img/ROSbot_manual/pro_colour_perspective.jpg "Perspective")

</div>

You can use your ROSbot offline however we recommend connecting to Husarion Cloud as it gives you access to a lot of additional functionalities like remote management and firmware updates.

If you do not own ROSbot yet, you can purchase it <a href="https://store.husarion.com/">here</a>.

You can also test the performance of ROSbot using our simulation model in Gazebo environment. It is available here, at our <a href="https://github.com/husarion/rosbot_description">GitHub page</a>.

![ROSbot gazebo](/assets/img/ROSbot_manual/rosbot_gazebo.png "ROSbot gazebo")

You can find free <b>ROS tutorials</b> dedicated for ROSbot under this <a href="https://husarion.com/tutorials/ros-tutorials/1-ros-introduction/">link</a>. They will guide you through different aspects of programming autonomous vehicles in ROS

# Hardware guide #

## Specification ##

<table>
    <tr>
       <th>Attribute</th>
       <th>Description</th>
    </tr>
    <tr>
        <td>Dimensions with camera and LiDAR</td>
        <td>200 x 235 x 220mm / 7.87 x 9.25 x 8.66in [L x W x H]</td>
    </tr>
    <tr>
        <td>Dimensions without LiDAR</td>
        <td>200 x 235 x 146mm / 7.87 x 9.25 x 5.74in [L x W x H]</td>
    </tr>
    <tr>
        <td>Dimensions without camera and LiDAR</td>
        <td>200 x 235 x 106mm / 7.87 x 9.25 x 4.17in [L x W x H]</td>
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
        <td>Maximum translational velocity</td>
        <td>1.25 m/s</td>
    </tr>
    <tr>
        <td>Maximum rotational velocity</td>
        <td>420 deg/s (7.33 rad/s)</td>
    </tr>
    <tr>
        <td>Maximum load capacity</td>
        <td>10kg / 352oz</td>
    </tr>
    <tr>
        <td>Battery life</td>
        <td>1.5h - 5h</td>
    </tr>
</table>

## Components ##

![Side scheme](/assets/img/ROSbot_manual/scheme_side.png "Side scheme")

![Back](/assets/img/ROSbot_manual/colour_back.jpg "Scheme back")

### Components description ###

<table>
    <tr>
       <th>Component</th>
       <th>Quantity</th>
       <th>Description</th>
    </tr>
    <tr>
    <tr>
        <td>Infrared distance sensor</td>
        <td>4</td>
        <td>VL53L0X Time-of-Flight distance sensor with up to 200 cm range, <a href="https://www.pololu.com/file/0J1187/VL53L0X.pdf"> more details</a>.</td>
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
        <td>Powerful 9-Axis Accel/Gyro/Magnetometer sensor with MPU-9250, <a href="https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf"> more details</a>.</td>
    </tr>
    <tr>
        <td>RGBD camera</td>
        <td>1</td>
        <td>Orbbec Astra with RGB image size 1280x960 and depth image size 640x480.</td> 
    </tr>
    <tr>
        <td>Batteries</td>
        <td>3</td>
        <td>Li-Ion 18650 protected, rechargeable batteries, 3500mAh capacity, 3.7V nominal voltage <br>
		Note: Device may be shipped interchangeably with similar batteries.</td>
    </tr>
    <tr>
        <td>Antenna</td>
        <td>1</td>
        <td>Connected directly to the ASUS Tinker Board Wi-Fi module. Uses an RP-SMA(m) <-> I-PEX MHF4 cable to connect the antenna with SBC.</td>
    </tr>	
</table>

In ROSbot 2.0:

<table>
    <tr>
       <th>Component</th>
       <th>Quantity</th>
       <th>Description</th>
    </tr>
    <tr>
        <td>CORE2-ROS</td>
        <td>1</td>
        <td>Advanced version of CORE2 with an ASUS Tinker board computer. CORE2 real-time controller is based on STM32F407 microcontroller. The SBC runs on Ubuntu-based OS, customized to use ROS, <a href="https://husarion.com/core2/">more details</a>.</td>
    </tr>
    <tr>
        <td>LIDAR</td>
        <td>1</td>
        <td>RpLidar A2, 360 degree and up to 8m range, <a href="https://www.slamtec.com/en/Lidar/A2"> more details</a>.</td>
    </tr>
</table>

In ROSbot 2.0 PRO:

<table>
    <tr>
       <th>Component</th>
       <th>Quantity</th>
       <th>Description</th>
    </tr>
    <tr>
        <td>CORE2-ROS</td>
        <td>1</td>
        <td>Advanced version of CORE2 with an Up Board computer. CORE2 real-time controller is based on STM32F407 microcontroller. The SBC runs on Ubuntu-based OS, customized to use ROS, <a href="https://husarion.com/core2/">more details</a>.</td>
    </tr>
    <tr>
        <td>LIDAR</td>
        <td>1</td>
        <td>RpLidar A3, 360 degree and up to 25m range, <a href="https://www.slamtec.com/en/Lidar/A3"> more details</a>.</td>
    </tr>
</table>

## Rear panel description ##

![Rear panel description](/assets/img/ROSbot_manual/ROSbot2_rear_panel_v1.1.png "Rear panel description")

<table>
    <tr>
       <th>Component</th>
       <th>Quantity</th>
       <th>Description</th>
    </tr>
    <tr>
        <td>Antenna connector</td>
        <td>1</td>
        <td>Wi-Fi antenna RP-SMA socket. Required for Wi-Fi connectivity.</td>
    </tr>
    <tr>
        <td>USB</td>
        <td>2</td>
        <td>USB 2.0 host ports from SBC.</td>
    </tr>
    <tr>
        <td>HDMI</td>
        <td>1</td>
        <td>HDMI output from SBC.</td>
    </tr>
    <tr>
        <td>Power switch</td>
        <td>1</td>
        <td>Turns ROSbot completely ON or OFF.</td>
    </tr>
    <tr>
        <td>LEDs</td>
        <td>6</td>
        <td>LR1(yellow), LR2(blue), L1(red), L2(green), L3(green), PWR(red), more details <a href="https://husarion.com/core2/manuals/core2/#hardware-leds">here</a>.</td>
    </tr>
    <tr>
        <td>reset button</td>
        <td>1</td>
        <td>Button used for reset CORE2.</td>
    </tr>
    <tr>
        <td>hCfg button</td>
        <td>1</td>
        <td>Button used for connecting ROSbot to [Husarion Cloud](https://cloud.husarion.com/).</td>
    </tr>
    <tr>
        <td>hBtn</td>
        <td>2</td>
        <td>hBtn1, hBtn2 - programmable buttons.</td>
    </tr>	
    <tr>
        <td>Power switch</td>
        <td>1</td>
        <td>Switch to turn on and off the robot. It completely disconnectc all components of ROSbot from its power source.</td>
    </tr>
    <tr>
        <td>Outputs for servo</td>
        <td>6</td>
        <td>Servo output with PWM, more details <a href="https://husarion.com/core2/manuals/core2/#hardware-hservo">here</a>.</td>
    </tr>
    <tr>
        <td>USB serial</td>
        <td>1</td>
        <td>USB serial port used for debugging the firmware on CORE2-ROS controller.</td>
    </tr>
    <tr>
        <td>Charging connector</td>
        <td>1</td>
        <td>6-pin connector for charging internal Li-Ion batteries.</td>
    </tr>
    <tr>
        <td>Time-of-Flight distance sensor</td>
        <td>2</td>
        <td>VL53L0X Time-of-Flight distance sensor with up to 200 cm range, more details <a href="https://www.pololu.com/file/0J1187/VL53L0X.pdf">here</a>.</td>.
    <tr>
        <td>hExt</td>
        <td>1</td>
        <td>12xGPIO, 7x ADC, SPI, I2C, UART, more details <a href="https://husarion.com/core2/manuals/core2/#hardware-hext">here</a>.     </td>
    </tr>
    <tr>
        <td>hSens</td>
        <td>1</td>
        <td>4 xGPIO, ADC, UART, more details <a href="https://husarion.com/core2/manuals/core2/#hardware-hsensor">here</a>.</td>
    </tr>
</table>

## Power supply ##

ROSbot is powered from an internal, rechargeable Li-Ion battery pack that contains 3 Li-Ion cells, connected in series. This type of connection is called “3S”. The schematic below explains how the cells are wired together and with the charging connector (on ROSbot side).

<div class="image center h300">
![Battery connections](/assets/img/ROSbot_manual/batt_connection.png "Battery connections")
</div>

The BAT+ and BAT- are the power connections and the “bal Bxx” wires are used to monitor the voltage on each cell. It is strongly recommended to keep equal voltages on each cell during the charging process. The charger included with ROSbot can charge batteries in the described way and, thanks to that, the long life of the battery set is possible.

The nominal voltage of each cell is 3.7V but the useful range is 3.2V to 4.2V.

**Important - discharge indicator**
If only the right firmware is preloaded to the internal controller (CORE2), the LED1 is programmed to indicate the power status:
- the LED1 is on when the robot is turned on
- the LED1 is blinking when battery is low – please charge immediately!

Please make sure that the user firmware always contains the function that monitors the supply voltage level. Deep discharging of batteries may decrease their lifecycle. Discharging to the voltage lower than 3.0V/cell can also trigger the over discharge protection. If the voltage is too low, turn ROSbot off and charge batteries as soon as possible.

## Charging ROSbot ##

<div class="image center">
![Charging kit](/assets/img/ROSbot_manual/charger+cables+PSU.jpg "Charging kit")
</div>

The ROSbot kit contains the Redox Beta charger. It is an universal charger, suitable for charging NiCd, NiMH, Li-Po, Li-Fe, Li-Ion and Pb (AGM, VRLA) batteries. ROSbot shall be charged using an included charger and cable.

Charger kit includes:
- Redox Beta charger
- AC/DC power adapter 100...240V to 12V 5A with 5.5/2.5mm plug on the 12V side
- a cable to connect charger with ROSbot charging port

**Quick charging guide:**
1. Connect the power adapter to the charger and the output cable between charger and ROSbot (2 connectors on charger side, 1 black connector to ROSbot charging port).
2. Use red and blue buttons to select “LiPo BATT” mode and press [Start].
3. Use arrows to select “LiPo CHARGE” mode.
4. Press [Start] - the current value should start blinking. Use arrows to set the current to 1.5A. 
5. Press [Start] again - the voltage value should start blinking. Select “11.1V(3S)” using arrows. The picture below shows the desired result.
6. Press and hold [Start] for 2 seconds. The charger should now ask for confirmation. Press [Start] again. The charging process should begin now.
7. When the charging will be finished (after about 3 hours), the charger will generate a loud “beep” sound and will finish charging at the same time.

<div class="image center h100">
![Charge config](/assets/img/ROSbot_manual/charge-config.png "Charge config")
</div>

If you need more information about charging, please read the [Charging manual for ROSbot](https://files.husarion.com/docs2/Charging%20manual%20for%20ROSbot.pdf) in PDF format.

**Notes** 
- You can change charging current to maximum 3A. Please note that a regular charging with the maximum current can shorten the battery life.
- If you are going to use ROSbot stationary for a long time, you can use ROSbot with charger connected (and charging) all the time. It will increase the batteries lifetime. Align the charging current to keep the voltage at about 11.1V-12V.
- In case you need to replace batteries, use only 18650 Li-Ion batteries, with the capacity in a range of 1800...3500mAh and with a protection circuit! Using unprotected batteries may result in serious injuries or fire.
- Unplug charging connectors carefully. You shall not unplug the charger connectors holding the wires. The balancer connection on ROSbot side has a latching tab (see photo below) that must be pressed before unplugging. On the charger side there is no latching tab but you should also unplug this connector holding the white plug.

<div class="image center h200">
![Latched connector](/assets/img/ROSbot_manual/charger-connector.jpg "Latched connector")
</div>

## Software ##

Software for ROSbot can be divided into 2 parts:
 * A firmware that works on the real-time controller (CORE2) and can be developed and uploaded from [Husarion Cloud](https://cloud.husarion.com/) with WebIDE. It can also be developed offline using [Visual Studio Code IDE](https://husarion.com/core2/tutorials/howtostart/offline-development-tools/).
 * OS based on Ubuntu 16.04, which runs on the SBC (ASUS Tinker Board) and contains all components needed to start working with ROS immediately. The microSD card with OS is included with each ROSbot. The OS has been modified to make the file system insensitive to sudden power cuts.
 
 In some cases you will need to flash the OS image to the microSD card once again:
 - in case of accidential damage of the system,
 - to update the OS (it can be udpated remotely, but flashing the microSD card can be easier sometimes),
 - to clear all user changes and restore factory settings.
 To do that, you have to disassembly the top cover, unscrew the 4 screws on the CORE2 corners and carefully carry up CORE2 with SBC. Then you can change the microSD card and flash the OS. You can find the image and flash manual [here](https://husarion.com/core2/manuals/core2/#hardware-os-image-for-raspberrypi-tinkerboard). If you want to replace the included card, remember that you need to use at least 16 GB capacity and 10 speed class micro SD card. 

# First steps #

## Connection to Husarion Cloud ##

* Things you need: the ROSbot, any Android device with Wi-Fi connectivity and with hConfig app installed (available on <a href="https://play.google.com/store/apps/details?id=com.husarion.configtool2">Google Play</a> and <a href="https://itunes.apple.com/us/app/hconfig/id1283536270?app=itunes&platform=iphone&preserveScrollPosition=true">App Store</a>), any PC computer to work with ROSbot, the Wi-Fi network.
* Login or register on cloud.husarion.com.
* Register your ROSbot on your cloud account by clicking “Add new device”.
* Launch the hConfig application and follow the instructions.
Note: The app will ask you to hold hCfg button on CORE2 and to watch LR1, LR2 LEDs – they are all available on the rear panel.
* Now you should see your ROSbot online and you can start with ROSbot tutorial.

## ROS tutorials ##

ROS (Robot Operating System) provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. It's very powerful and 
functional tool dedicated to design robots. We created the set of [ROS tutorials dedicated for this platform](https://husarion.com/core2/tutorials/ros-tutorials/1-ros-introduction/ "ROS tutorials dedicated for this platform") to make it easier to familiarize yourself with these frameworks. 

## Configuring ROSbot to work with 5GHz WiFi. ##

By default ROSbot supports WiFi in 2.4GHz band, this is sufficent for most cases.
If you encounter problems with data transfers e.g. due to processing large amounts of data or noise from other networks you can try to use connection in 5GHz band.

To do this, you will need a USB 5GHz WiFi card (any device based on rtl8812au should be fine, tested models are TP-Link Archer T4U and D-Link DWA-172)

If you have recent image version you can skip update and kernel instal, otherwise do:

```  
sudo apt-get update
```

This will update packages list.

Then do:
```
sudo apt-get install tinkerboard-kernel
```

This will install the newest available kernel version, that supports 5GHz WiFi.


Load kernel module:

```
modprobe rtl8812au
```

Type `ifconfig` to list network interfaces.
You should see now new interface named `wlan1`

Now, You can list all available newtworks:

```
sudo iwlist wlan1 scanning | grep ESSID
```

You can connect to your WiFi with:

```
nmcli d wifi connect <ESSID> password <pass> iface wlan1
```

Remember to replace `ESSID` and `pass` with name and passowrd of chosen network.

ROSbot will try to connect to this network each time it boots.

# Docs and links #
All helpful documents and links in one place:

* [ROSbot Safety Instructions](https://files.husarion.com/docs2/ROSbot_safety_instructions_1.0.pdf "ROSbot Safety Instructions") - important!
* [Charging manual for ROSbot](https://files.husarion.com/docs2/Charging%20manual%20for%20ROSbot.pdf)
* [ROS tutorials for ROSbot](https://husarion.com/tutorials/ros-tutorials/1-ros-introduction/)
* [ROSbot on ROS webpage](https://robots.ros.org/rosbot-2.0/)
* [ROSbot on ROS Wiki](http://wiki.ros.org/Robots/ROSbot-2.0)
* [URDF model of ROSbot - for Gazebo integrated with ROS](https://github.com/husarion/rosbot_description)
* [ROSbot project on hackaday.io](https://hackaday.io/project/21885-rosbot-autonomous-robot-platform "ROSbot project on hackaday.io")
* [ROSbot project on instructables.com](http://www.instructables.com/id/ROSbot-Autonomous-Robot-With-LiDAR/ "ROSbot project on instructables.com")
