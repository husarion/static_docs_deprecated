---
title: 'Run ROS tutorials using AWS RoboMaker'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 4
---
# Run ROS tutorials using AWS RoboMaker

## Introduction

**AWS RoboMaker** is the newest service by *Amazon* for robot developers community. It’s ROS based and provides web tools for robot development, simulation and deployment - https://aws.amazon.com/robomaker/ .

In this tutorial we will show you how to setup environment at AWS RoboMaker to learn ROS with our ROS tutorials:
1. [ROS introduction](https://husarion.com/tutorials/ros-tutorials/1-ros-introduction/)
2. [Creating nodes](https://husarion.com/tutorials/ros-tutorials/2-creating-nodes/)
3. [Simple kinematics for mobile robot](https://husarion.com/tutorials/ros-tutorials/3-simple-kinematics-for-mobile-robot/)
4. [Visual object recognition](https://husarion.com/tutorials/ros-tutorials/4-visual-object-recognition/)
5. [Running ROS on multiple machines](https://husarion.com/tutorials/ros-tutorials/5-running-ros-on-multiple-machines/ )
6. [SLAM navigation](https://husarion.com/tutorials/ros-tutorials/6-slam-navigation/ )
7. [Path planning](https://husarion.com/tutorials/ros-tutorials/7-path-planning/)
8. [Unknown environment exploration](https://husarion.com/tutorials/ros-tutorials/8-unknown-environment-exploration/ )
9. [Object search](https://husarion.com/tutorials/ros-tutorials/9-object-search/ )


## Setting up Husarion ROS tutorial on AWS

To begin, you need to have an active AWS account.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img1.png" width="900px" />
</p>

1. Go to “AWS management console” at https://aws.amazon.com/ .
2. In search field start typing “IAM”.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img2.png" width="900px"/>
</p>

3. Open IAM module.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img3.png" width="900px"/>
</p>

4. Go to “Roles”.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img4.png" width="900px"/>
</p>

5. Click “Create role”.
6. In dialog choose “EC2”.
7. Click “Next: Permissions”.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img5.png" width="900px"/>
</p>

8. In “Attach permissions policies” dialog, add (You can use filter to search for them):
    * `CloudWatchFullAccess`
    * `AWSRoboMakerFullAccess`
    * `AmazonS3FullAccess`

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img6.png" width="900px"/>
</p>

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img7.png" width="900px"/>
</p>

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img8.png" width="900px"/>
</p>

9. Click “Next: Tags”, we do not need any tags, thus we proceed to Reviev.
10. Click “Next: Review”. In field “Role name” type “robomaker_role” and make sure that in “Policies” section you have three entries.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img9.png" width="900px"/>
</p>

11. Click “Create role” button, you will be redirected to “Roles” view.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img10.png" width="900px"/>
</p>

12. Open role setting by clicking its name and choose tab “Trust relationships”.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img11.png" width="900px"/>
</p>

13. Click button “Edit trust relationships” and edit policy document, find entry “ec2.amazonaws.com” and change it to “robomaker.amazonaws.com”.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img12.png" width="900px"/>
</p>

14. Click “Update trust policy” button. Note the “Role ARN” entry, this will be required later.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img14.png" width="900px"/>
</p>

15.  Open “AWS Console”, in filter type “S3”.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img15.png" width="900px"/>
</p>

16. Open “S3” module.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img16.png" width="900px"/>
</p>

17.  Click “Create bucket” button, in field “Bucket name” type DNS style name like `yourusername-bucket-robomaker` (it must be unique accross all names in Amazon S3, do not use `_` and `.` in the name), from “Region dropdown” menu choose entry appropriate to your localization.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img17.png" width="900px"/>
</p>

18. Proceed through creator, do not modify default values.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img18.png" width="900px"/>
</p>

19. When you create bucket, open it, by clicking its name, it should be empty now. Note the bucket name, you will need it later.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img19.png" width="900px"/>
</p>

20. Open “AWS Console”, in filter type “RoboMaker” and open “AWSRoboMaker” module.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img20.png" width="900px"/>
</p>

21. Open “Development environments” tab.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img21.png" width="900px"/>
</p>

22. Click “Create environment”, in field “Name” type “robomaker_env” and as “instance type” choose “c3.2xlarge”.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img22.png" width="900px"/>
</p>

23. Click "Create".

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img23.png" width="900px"/>
</p>

24. Download `ROSbotTutorial.tar.gz` and unpack it.  

25. Open the environment, from menu “File” choose “Upload local files…” -> “Select folder” and upload the folder “ROSbotTutorial”. Wait until upload is done.

26. In editor open file “RoboMakerSettings.json” from folder “ROSbotTutorial”. Find all instances of `robomaker-bucket` and replace it with the name of your own bucket you've created in point `17`.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img23a.png" width="900px"/>
</p>

27. In the same file find element `iamRole` and change its value to "Role ARN" entry you've saved in pont `14`.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img23b.png" />
</p>

28.  Choose menu “RoboMakerRun” -> “Add or Edit Configurations”. 

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img24.png" width="900px"/>
</p>

29. Click button “Switch config”.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img25.png" width="900px"/>
</p>

30. Choose “RoboMakerSettings.json” from folder “ROSbotTutorial” and click “OK” button and then "Save" button.

31. Choose menu “RoboMakerRun” -> “Workflow” -> “ROSbotTutorial - Build and Bundle All”.
<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img26.png" width="900px"/>
</p>

32. Package build process will start, when it is done, choose menu “RoboMakerRun” -> “Launch Simulation” -> “ROSbotTutorial”. Simulation job will be sent to RoboMaker. Wait until "Your simulation job was created." message appears in console.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img27.png" width="900px"/>
</p>

33. Go to RoboMaker an open “Simulation jobs” menu.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img28.png" width="900px"/>
</p>

34. Open simulation by clicking its name.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img29.png" width="900px"/>
</p>

35. When it starts, you can open Gazebo view to watch as simulation proceeds.

<p align = "center">
<img alt="" src="./../../../assets/img/aws-tutorials/aws_tutorial_img30.png" width="900px"/>
</p>

**Congratulations!**
 
You’ve just run Gazebo version of [Tutorial 9](https://husarion.com/tutorials/ros-tutorials/9-object-search/) using AWS RoboMaker service. More tutorials can be launch by editing “RoboMakerSettings.json” and launch files found in `./ROSbotTutorial/simulation_ws/install/tutorial_pkg/share/tutorial_pkg/launch` . We will provide a instructions on how to do it soon.

## Summary

After completing this tutorial you should be familiar with AWS RoboMaker Service. You should be able to create your own environment and run various robotics simulations (in this case Husarion ROS Tutorial 9).

Possibility of outsourcing computation to powerful cloud servers hardware opens doors for even more advanced robotics simulations for users that are not equipped with efficient work stations. It also gives you an opportunity to share the same workspace among different devices and run simulations on them regardless of their computation power. We hope this tutorial will encourage you to experiment with more advanced use cases of your projects.

---------

*by Łukasz Mitka, AGH Krakow*

*Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com*