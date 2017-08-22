---
title: 'hFramework library development'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 3
---

# hFramework library development #

If you like to develop [hFramework](https://github.com/husarion/hFramework) together with our Community, or to adapt it to your purposes, this is a way how to do it.

## hFramework sourcecode compilation ##

 1. Download current version of hFramework on your computer (Github link: https://github.com/husarion/hFramework). 
 2. Unpack downloaded file.
 3. Open Visual Studio Code.
 4. Close the opened project if you have one already loaded.
 5. Install Husarion plugin for Visual Studio Code. 
 	
	* Press [Ctrl]+[Shift]+[P]. Then type “flash project to CORE2” and press Enter to accept. Terminal window on the bottom should open.

	or

	* Find terminal tab on bottom and click on it. Then type 

		`set PATH=%PATH%;C:%HOMEPATH%\.vscode\HusarionTools\bin;`

6. Use `cd` command to change directory to folder you've previously unzipped. Example:

		cd C:\Users\husarion\Downloads\hFramework-master\hFramework-master</code>

7. Type

		mkdir build\stm32_core2_1.0.0\
		cd build\stm32_core2_1.0.0\
		cmake ../.. -DBOARD_TYPE=core2 -DPORT=stm32 -DHFRAMEWORK_PATH=. -GNinja

8. Type

		ninja
		
9. Compiler should run and create static library for hFramework.
If you see on terminal the following line - compilation has succeed.

![image](/assets/img/howToStart/lib_p9.png)

## Using the library compiled by yourself ##
     
1. Create a new folder for your project.
2. In VSCode File -> Open Folder

![image](/assets/img/howToStart/com_p2.png)

In the opened window find the directory you just created. Project tree should be empty.
3. Press [Ctrl]+[Shift]+[P]. Small console will open on the top.
4. Type “Create Husarion project” and press Enter to accept.
5. In the project tree you should find files like on this screenshot:

![image](/assets/img/howToStart/com_p5.png)

6. Open CMakeLists.txt and remove line: <code>enable_module(hCloudClient)</code>
7. Open main.cpp and remove lines: "#include <code>hCloudClient.h</code> and <code>platform.begin(&RPi);</code>.
(These lines contain functions used to connect platform to cloud. You can add them later, after first compilation).
8. Press [Ctrl]+[Shift]+[P], type “Change Husarion project variable” and press Enter.
9. Type “HFRAMEWORK_PATH” and press Enter.
10. Type path to hFramework directory. Example:

		C:/Users/husarion/Downloads/hFramework-master/hFramework-master

11. Press [Ctrl]+[Shift]+[B] to compile your project. If everything goes well, no message should pop up and in project tree you should find tree new output files:

		myproject.bin
		myproject.elf
		myproject.hex
	
That means you have successfully builded your first project, together with hFramework sources.
