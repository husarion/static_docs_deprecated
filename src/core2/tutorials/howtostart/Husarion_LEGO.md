---
title: 'Getting started with Husarion + LEGO Mindstorms'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 6
---

# Getting started with Husarion + LEGO Mindstorms #

In this tutorial we will help you build autonomic, mechatronic, 3-wheel tracking robot. It needs to be equipped with two motors, two ultrasonic distance meter sensors, a power supply and CORE2. You can control your robot via cloud.husarion.com using W S A D keys, change speed with Q/Z keys and turn on automatic mode with E key or hBtn1.
See video: YouTube


Wstawiæ tabele!!!


## Building and Connections ##

Sensors will detect object in front of the robot therefore you need to place both sensors on the front of the robot. Right one should be directed  a little bit to the right, and the left one symmetrically to the left.

Connect right sensor to hSens1 port and left one to hSens2 port.
Left motor should be connected to A motor port and the right one to D motor port.

Zdjêcia (1)(2)(3)

## Code ##

After assembling your robot, you can run the following program. You can do it either offline with Visual Studio Code (how to) or online with Husarion Cloud (how to).

'
kod programu
'

## Useful links ##

https://github.com/husarion/hSensors - open source hFramework libraries for LEGO Mindstorms sensors

https://wiki.husarion.com/robocore:hardware:description - RoboCORE hardware documentation

https://husarion.com/core2/manuals/hardware/ - CORE2 hardware documentation

https://husarion.com/core2/tutorials/howtostart/offline-development-tools/ - conifguring offline dev tools

https://cloud.husarion.com/ - Husarion cloud to manage all your CORE2 or RoboCORE based robots. Web IDE (Integrated Development Environment) is on-site, so you can develop a code directly from a web browser. Write control algorithms and web user interface for your robot and control it from anywhere.

https://play.google.com/store/apps/details?id=com.husarion.configtool2 - hConfig app to connect CORE2 to the internet and cloud.husarion.com

https://play.google.com/store/apps/details?id=io.robocore.rcapp.prod - RoboCORE app to connect RoboCORE to the internet and cloud husarion.com. And to stream video through Web RTC!

https://play.google.com/store/apps/details?id=com.husarion.video2 - hVideo app for CORE2. Pair your phone with your CORE2 based robot and integrate video streaming with its user interface
