---
title: 'Husarnet tutorials'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: ???
---

# Husarnet tutorial #

!!!!!!!!!!!!!!!!!!!!!!Idea zastosowanie i ogólny opis!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

## Configuration of the network ##

Before you start using husarnet you have to configure network settings of your device. 

### Installation guide for any Linux device ###

1. Install Husarnet: 
```wget https://files.husarion.com/repo/repo.key -O- | apt-key add - 
echo 'deb https://files.husarion.com/repo/ xenial main' | sudo tee /etc/apt/sources.list.d/husarnet.list
sudo apt-get install -y apt-transport-https
sudo apt-get update
sudo apt-get install -y husarnet
sudo systemctl start husarnet```
2. Link Husarnet to your account. Run husarnet websetup and copy the URL to your browser.

### Configuration for any Linux device ###

1. Enable IPv6 mode (in .bashrc): ```export ROS_IPV6=on```
2. Make sure ROS hostname is same as the one specified in Husarnet Web UI (e.g. mydev): ```export ROS_HOSTNAME=mydev```
3. Set master URI to http://master:11311: ```export ROS_MASTER_URI=http://master:11311```
4. Restart terminal to make sure the changes get applied.

### Configuration of the phone application ###

1. Install the application from http://users.atomshare.net/~zlmch/hNode.apk
2. Start the application and enter the link shown (http://cloud.husarion.com/c/XXXX) in your browser.
![image](/assets/img/husarnet/hNode.png)
3. Chose the histname for your device and click "Add device for your account"
![image](/assets/img/husarnet/platform.png)
4. The app will connect to ROS master and publish data from phone sensors. Use `rostopic list` to check what's available.
![image](/assets/img/husarnet/console.png)

## Using Husarnet with the Cloud ##

Opis z przyk³adami mo¿liwoœci.


## Camera support ##

If you want to use camera image from your smartphone, just install compressed image transport:

```apt-get install -y ros-kinetic-compressed-image-transport```

To view image from camera locally, use:

```rosrun image_view image_view image:=/yourphonehostname/camera0/image _image_transport:=compressed```

Make sure to accept permission dialog on the device. Use camera1 instead of camera0 to access second camera, if you have one.

If you have some node that can't process compressed image, launch the node that decompresses it locally (republisher):

```rosrun image_transport republish compressed in:=/yourphonehostname/camera0/image out:=/localimg```

After that, raw image will be available in `/localimg` channel.




















































Before you perform the next steps, install the hConfig mobile application on your smartphone or tablet:
* [Google Play](https://play.google.com/store/apps/details?id=com.husarion.configtool2&hl=en)
* [AppStore](https://itunes.apple.com/us/app/hconfig/id1283536270?mt=8)

If you have CORE2-ROS, you can also connect to the cloud via command line - see below.

You also need to register your own account on [Husarion Cloud](https://cloud.husarion.com).

Open the hConfig app on your smartphone and follow the wizard that will show you how to connect CORE2 to your Wi-Fi network and your Husarion cloud account. After you select the Wi-Fi network for your CORE2 in the hConfig app, you can proceed to the next steps.

hConfig app will ask you to add a new device. Open https://cloud.husarion.com in your browser and sign in.

![image](/assets/img/howToStart/1_signin.png)

Click "Add new device".

![image](/assets/img/howToStart/2_addNewDevice.png)

Enter a name for your CORE2 powered device.

![image](/assets/img/howToStart/3_enterName.png)

Scan QR code using the hConfig app.

![image](/assets/img/howToStart/4_scanQr.png)

Well done! You just added your first device to the cloud!

![image](/assets/img/howToStart/5_devAdded.png)

### Connecting via the command line ###

If you have CORE2-ROS, you can also connect to the cloud via the command line. This is recommended only if you are able to open terminal or connect via SSH to your device.

 * Open https://cloud.husarion.com in your browser, sign in and click "Add new device". 
 * Enter a name for you device.
 * Copy the text code, located below the QR code.
 * Run `husarion-register` on your CORE2-ROS device. When asked, paste the previously copied code.

## Writing your first program ##

Click "+" next to your device name and sellect "IDE".

![image](/assets/img/howToStart/6_openWebIDE.png)

Click "Create" button to open new project wizard.

![image](/assets/img/howToStart/7_createNewProj.png)

Select CORE2 board, chose HowToStart project template and enter name, eg. myFirstProject, and click "Create project" button.

![image](/assets/img/howToStart/8_projSettings.png)

This is a web Integrated Development Environment in which you can write a firmware for your device, and upload the firmware through the Internet.

![image](/assets/img/howToStart/9_webIDEmain.png)

Click "&lt;none&gt;" next to "selected device" and select "myFirstDev" device.

![image](/assets/img/howToStart/10_webIDEselectDev.png)

Click a button with a "cloud with arrow" to upload new firmware to your device. Well done! now you can check how your first program works.

![image](/assets/img/howToStart/11_webIDEprogram.png)

In the previous step you have uploaded the firmware into your CORE2. Let's check how it works!<br/>

Go to https://cloud.husarion.com and click the myFirstDev's avatar. It's web user interface will start loading.

![image](/assets/img/howToStart/12_openDevUI.png)

After a while your device UI will appear. Now spend a few seconds playing with a dev's interface.

![image](/assets/img/howToStart/13_devUI.png)

## Share your device with friends ##
Husarion Cloud allows you to share your devices conntected to Husarion cloud with other people with just a few clicks.

Click "+" next to your device name at https://cloud.husarion.com and select "Share".

![image](/assets/img/howToStart/14_shareSelect.png)

Select "Share via Link" and click "Generate link". Now you can send this link to anybody you want to access your device.

![image](/assets/img/howToStart/15_shareDetails.png)

When you open generated link, you’ll see your device’s web UI.

![image](/assets/img/howToStart/16_shareUI.png)

Now you can share the link with anybody!