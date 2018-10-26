---
title: 'Video streaming from a camera'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 2
---

# Video streaming from a camera #

Husarion Cloud supports streaming a video from a camera attached to your robot to the Web User Interface.

## Setting up the hardware ##

<dev><em>
<b>Hint:</b> Remember that you can use USB camera <b>OR</b> smartphone camera but not both at the same time.
</em></dev>

### CORE2 + smartphone/tablet ###
Your robot with CORE2 can use a camera that is built into a smartphone or a tablet. The video from camera can be viewed in the robot's user interface. Currently this feature is available only for Android devices.

At first, your CORE2 needs to be connected to your account on [cloud.husarion.com](https://cloud.husarion.com/).

Install the [hVideo](https://play.google.com/store/apps/details?id=com.husarion.video2&hl=en) app on your smartphone or tablet. Enable Wi-Fi or mobile data transmission on the device.

Login to the Husarion Cloud, find your robot, click ![image](/assets/img/howToStart/plus.png) and "Add camera".

<div style="text-align: center">![image](/assets/img/howToStart/cloud_add_camera.png)</div>

Click "Generate new QR code" in the pop-up window:

<div style="text-align: center">![image](/assets/img/howToStart/add_camera.png)</div>

Launch the hVideo app on your smartphone, choose the "Pair with robot" option in the hVideo menu:

<div style="text-align: center">![image](/assets/img/howToStart/pair-with-robot.png)</div>

and scan the QR code displayed at Husarion Cloud with your smartphone.

Now the camera in your smarthpone is linked to your Cloud account and the stream should be visible in the Web UI of the robot. 

### CORE2-ROS + USB camera ###

CORE2-ROS contains a single board computer with Linux operating system and USB ports. It means that in most cases you only need to connect a USB camera and enable it.

At first, your CORE2-ROS needs to be connected to your account on [cloud.husarion.com](https://cloud.husarion.com/).

Login to the Husarion Cloud, find your robot, click ![image](/assets/img/howToStart/plus.png) and "Add camera".

<div style="text-align: center">![image](/assets/img/howToStart/cloud_add_camera.png)</div>

Click "Enable USB camera" in the pop-up window:

<div style="text-align: center">![image](/assets/img/howToStart/add_camera.png)</div>

Now reboot the robot. If you don't have a non-standard camera, it shouldn't need any additional configuration and the video stream should now be available in the Web UI of your robot.

## Displaying the stream in the robot UI ##

Once you have configured the video stream, you can display it in the Web UI of your robot now.

Create a new project and use the "UI example" template to see how it can be done:

<div style="text-align: center">![image](/assets/img/howToStart/create-project-webui.png)</div>

The stream is enabled in cfgHandler() function:
```
void cfgHandler()
{
    auto l = platform.ui.label("l1");
    l.setText("label1");
    auto b = platform.ui.button("btn1");
    b.setText("button1");
    b.setLocation(100, 100);
    platform.ui.video.enable();  // enable the video stream and display in the default web UI
}
```
The video stream will be displayed on default robot user interface.

In some cases you will need to align the image position or other properties, together with other features like buttons, sliders, images etc. It means that you need to create a custom robot UI in HTML. Choose another template - "4-wheels spy robot" - and explore the more advanced example of user interface. The additional line is added to the cfgHandler():
```
void cfgHandler()
{
	auto l1 = platform.ui.label("l1");
	auto g1 = platform.ui.button("g1");
	auto g2 = platform.ui.button("g2");
	auto g3 = platform.ui.button("g3");
	platform.ui.loadHtml({Resource::WEBIDE, "/web_ui/ui.html"});    // overwrite the default web UI with the custom one
	platform.ui.video.enable();                                     // enable the video stream and display in the web UI
}
```
Two files: "ui.html" and "ui.css" define the video container, its location and size, together with other page elements. Feel free to modify the template and create your own, great UI for your robot :) See the screenshot from the "4-wheels spy robot" user interface:

<div style="text-align: center">![image](/assets/img/howToStart/interface2.jpg)</div>
