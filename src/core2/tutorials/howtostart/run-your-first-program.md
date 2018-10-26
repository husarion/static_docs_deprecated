---
title: 'Run your first program'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 1
---

# Run your first program #
## Preparing hardware ##

Connect your CORE2 to a DC power supply. The power connector is a standard DC 5.5/2.1 (center-positive), and provides 6 to 16V output. You can use:

* DC adapter
* Li-poly/Li-ion packages - 2S or 3S (e.g. 18650 batteries)
* AA alkaline batteries (4-10 pieces)


<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/howToStart/core2_power_supply.png"
/></center></div>

Set the power switch to "ON" position and now your device is ready to use!

## Connecting to the Cloud ##

There are a few methods to connect your controller to [Husarion Cloud](https://cloud.husarion.com). First one is universal for all of our devices and the other two apply only to CORE2-ROS controllers. For all of them you will need to register your own account on [Husarion Cloud](https://cloud.husarion.com). We will describe all three in great detail. 

### Connecting using mobile device and browser ###

Before you perform the next steps, install the hConfig mobile application on your smartphone or tablet:
* [Google Play](https://play.google.com/store/apps/details?id=com.husarion.configtool2&hl=en)
* [AppStore](https://itunes.apple.com/us/app/hconfig/id1283536270?mt=8)

1\. Open the hConfig app on your smartphone and follow the wizard that will show you how to connect CORE2 to your Wi-Fi network and your Husarion cloud account. The phone is required only once for configuration and connecting CORE2 with cloud, later you will not be using it. After you select the Wi-Fi network for your CORE2 in the hConfig app, you can proceed to the next steps.

The WiFi bradcasted by CORE2 can be used only for configuration, it does not allow to connect to the internet. Your phone may show warning regarding no internet connection while connected to this WiFi. You can ignore these warnings, as this will not interrupt the process.

<b>For Android users: turn off mobile internet on your smartphone while using hConfig app </b>

2\. hConfig app will ask you to add a new device. Open https://cloud.husarion.com in your browser and sign in.

![image](/assets/img/howToStart/1_signin.png)

3\. Click "Add new device".

![image](/assets/img/howToStart/2_addNewDevice.png)

4\. Enter a name for your CORE2 powered device.

![image](/assets/img/howToStart/3_enterName.png)

5\. Scan QR code using the hConfig app.

![image](/assets/img/howToStart/4_scanQr.png)

6\. Well done! You just added your first device to the cloud!

![image](/assets/img/howToStart/5_devAdded.png)

### Connecting using only browser ###

1\. Open [Husarion Cloud](https://cloud.husarion.com) in your browser and sign in.

![image](/assets/img/howToStart/configuration_1.png)

2\. Click "Add new device".

![image](/assets/img/howToStart/configuration_2.png)

3\. Enter a name for your CORE2 powered device.

![image](/assets/img/howToStart/configuration_3.png)

4\. Next copy the text code, located below the QR code. It should look similar to this: "prod|7gx9KNhfZhnnmowmDxxxxx"

![image](/assets/img/howToStart/configuration_4.png)

5\. Turn on your CORE2-ROS. Wait 20 - 30s until Linux system on SBC boot and hold hCfg button for more than 2 sec. Controller will be in configuration mode after that. Now connect to Wi-Fi:

name: HusarionConfigXXXX
password: husarion

6\. Then open new tab in your browser and type adress: http://192.168.50.1:8600 

![image](/assets/img/howToStart/configuration_5.png)

7\. Chose option "Connect to Wi-Fi network" (unless you are already connected to Wi-Fi) and pick up one of wireless from list. Type the password and click "Continue".

![image](/assets/img/howToStart/configuration_6.png)

8\. Now chose "Connect device to cloud.husarion.com account".

![image](/assets/img/howToStart/configuration_7.png)

9\. Paste previously copied code and click "Save".

![image](/assets/img/howToStart/configuration_8.png)

10\. You will receive message "You might now close this page."

![image](/assets/img/howToStart/configuration_9.png)

11\. Now your device should be visible as "Online" in your cloud.

![image](/assets/img/howToStart/configuration_10.png)

### Connecting via the command line ###

If you have CORE2-ROS, you can also connect to the cloud via the command line. This is recommended only if you are able to open terminal or connect via SSH to your device.

 * Open https://cloud.husarion.com in your browser, sign in and click "Add new device". 
 * Enter a name for you device.
 * Copy the text code, located below the QR code.
 * Run `sudo husarion-register` on your CORE2-ROS device. When asked, paste the previously copied code.
 * Reboot your device to complete the configuration.

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
