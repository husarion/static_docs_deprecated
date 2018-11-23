---
title: 'ROSbot - quick start'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 6
---
# ROSbot - quick start #

ROSbot is an autonomous, open source robot platform running on CORE2-ROS controller. It can be used as a learning platform for Robot Operating System as well as a base for a variety of robotic applications like inspection robots, custom service robots etc.

If you don't have one, you can purchase it <a href="https://store.husarion.com/">here</a>.

## Unboxing ##

What's in the box:

* carrying case
* ROSbot 2.0 (with optional 3D camera and LiDAR already assembled)
* Wi-Fi 2.4GHz antenna
* 3x 18650 Li-Ion reachargeable batteries
* universal charger with power adapter
* charging cable
* microSD card with the software for ROSbot
* USB to Ethernet adapter

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/howToStart/ROSbot_unboxing.jpg"/></center></div>

## Assembly ##

Your ROSbot is assembled, but to get it ready to work, you need to provide a power supply and attach the antenna. 

To mount batteries:

* turn ROSbot upside down
* unscrew battery cover mounted with two screws
* remove the battery cover
* place batteries accordingly to the symbols, keeping the black strip under the batteries
* place batery cover and mount it with screws

To charge the batteries, follow this <a href="https://files.husarion.com/docs2/Charging%20manual%20for%20ROSbot.pdf">guide</a>.

To attach the antenna, screw it to the antenna connector on the ROSbot rear panel.

## Connecting to the cloud ##

* Press and hold the hCfg button on ROSbot rear panel.
* Turn on a power switch.
* When blue and yellow LEDs start blinking, release the hCfg button.
* Connect your mobile device to Husarion Wi-Fi and open hConfig app (<a href="https://itunes.apple.com/us/app/hconfig/id1283536270?mt=8">hConfig in AppStore</a> or <a href="https://play.google.com/store/apps/details?id=com.husarion.configtool2">hConfig in Google Play</a>) to connect ROSbot to the Wi-Fi network and your user account at <a href="https://cloud.husarion.com">cloud.husarion.com</a> (<a href="https://husarion.com/core2/tutorials/howtostart/run-your-first-program/#run-your-first-program-connecting-to-the-cloud">how to do this</a>).

## Programming ##

First you will program the Core2 part:

* Turn on your ROSbot.
* Click "edit" next to your device name and sellect "IDE".
* Create a new project using Core2 as your board and 'ROSbot default firmwre' as a template.
* Build and upload program to the deivce.
* Go back to cloud panel

Next you will proceed to ROS part of software:

* Click "edit" next to your device name and sellect "More".
* Choose "SSH terminal"
* In terminal issue following commands:

Updates the package lists for upgrades for packages:

`sudo apt update`

Install the required packages:

`sudo apt install python-tornado python-pip ros-kinetic-rosbridge-suite ros-kinetic-web-video-server nginx`

Create new work space and change directory:

`mkdir ~/ros_workspace`

`mkdir ~/ros_workspace/src`

`cd ~/ros_workspace/src`

Clone repository containing rosbot webui:

`git clone https://github.com/husarion/rosbot_webui.git`

Clone `husarion_ros` repository:

`git clone https://github.com/lukaszmitka/husarion_ros.git`

Change directory and build code using catkin_make: 

`cd ~/ros_workspace`

`catkin_make`

Add environmental variables from the following file:

`source devel/setup.sh`


```
Note that you have to do it every time you want to use ROSbot webui. You can also set it up permanently. Open .bashrc file in text editor:

nano ~/.bashrc

Go to the end of file and add line:

. /home/husarion/ros_workspace/devel/setup.sh 
```

* Staying in terminal issue command: 

`sudo nano /etc/nginx/sites-enabled/default`

This will open text editor with configuration file, find line:  

`root /var/www/html;`

and change it to:  

`root /home/husarion/ros_workspace/src/rosbot_webui/edit;`


* To exit text editor press: "Ctrl + x", "y", "Enter"
* Again in terminal issue command:  

`sudo systemctl restart nginx`


## Usage

Programming procedure needs to be done only once, on further uses, you can start from this point:

* Turn on your ROSbot.
* Click "edit" next to your device name and sellect "More".
* Choose "SSH terminal"
* Note the address next to "Local IP", you will need it in a while.
* In terminal issue following command:

`roslaunch rosbot_webui demo.launch`

* Connect your laptop or mobile device to the same network as ROSbot.
* Launch web browser and type the local IP of your ROSbot (the one you noted before)
* You should see interface as below, use it to test and control your ROSbot.

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/howToStart/ROSbot_UI.png"
/></center></div>