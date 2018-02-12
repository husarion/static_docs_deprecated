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
```
wget https://files.husarion.com/repo/repo.key -O- | apt-key add - 
echo 'deb https://files.husarion.com/repo/ xenial main' | sudo tee /etc/apt/sources.list.d/husarnet.list
sudo apt-get install -y apt-transport-https
sudo apt-get update
sudo apt-get install -y husarnet
sudo systemctl start husarnet
```
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

``` rosrun image_transport republish compressed in:=/yourphonehostname/camera0/image out:=/localimg ```

After that, raw image will be available in `/localimg` channel.
