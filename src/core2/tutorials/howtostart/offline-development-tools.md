---
title: 'Offline development tools'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 2
---

# Offline development tools #

If you like our Cloud ;) but you need or prefer to work with offline tools, you are in the right chapter.

## Offline development tools for Windows ##

### Current functionality ###

Code editor: Visual Studio Code

Build tools - Husarion extension for VSCode that include:
* GNU ARM Embedded Toolchain
* support for CMake
* flasher program for downloading the code to CORE2

### Installation guide ###

1. Download and install VS Code from [https://code.visualstudio.com/](https://code.visualstudio.com/)
2. Launch VS Code, press Ctrl-Shift-X, find “husarion" extension and click “Install". VS Code will ask you if you want to install also the dependencies - agree with it. Reload VSCode.
3. ***<font color="green">Info: At this point you should have the following extensions installed: C/C++, CMake and Husarion. Do not install "CMake Tools" extension. If you already have it installed, disable it for workspaces with Husarion projects.</font>***
4. Download Zadig (http://zadig.akeo.ie/), connect CORE2 via micro USB cable and launch Zadig. Click [Options] -> [List All Devices], choose “FT230X Basic UART" from the drop-down list, select "WinUSB (v6.1.7600.16385)" driver and click “Replace Driver".
5. ***<font color="green">Info: This step is needed to flash the program to the microcontroller on CORE2. You need the administrator rights to do change the driver for USB device. If you encounter any problems with installation of the new driver, try to remove all old drivers related to USB Serial port or FTDI chip.</font>***

### Using the VSCode + extension ###

To create new project, select empty folder, press Ctrl-Shift-P, type “create husarion project" and press enter.

***<font color="green">Info: The "Husarion" extension may start downloading additional data now; please be patient.</font>***

To compile project press Ctrl-Shift-B (this happens automatically when you flash project).
To flash project to CORE2 via micro USB, press Ctrl-Shift-P and type “flash core2" and press enter.

***<font color="green">Info: Autocomplete function is partially supported.</font>***

### Useful extensions ###
* Builtin Git support - Ctrl-Shift-G You need to install Git first. We recommend using Chocolatey to do it.
* Git History (git log)

### Changing hardware platform ###

1. Click Ctrl-Shift-P, then chose “Change Husarion project variable.”
2. Select BOARD_TYPE to change board type or BOARD_VERSION to change board version.

### Debugging ###

1. Plug in ST-LINK device.
2. Using Windows automatic driver detection software install ST-LINK driver for your device.
3. Simply press F5 button in VSCode window to run flash application and start debugging.


## Offline development tools for Linux ##

### Current functionality ###

Build tools:
* GNU ARM Embedded Toolchain
* support for CMake
* flasher tool for downloading the code to CORE2 via USB cable
* st-flash tool for downloading the code to CORE2 via ST-LINK V/2 debugger

### Installation guide ###

***<font color="green">Info: In order to compile projects for CORE2, you need a compiler that can compile code for its ARM processor. We recommend installing the cross-compiler and other dependencies from your distribution packages.</font>***

1. Install C++ cross-compiler:
..* On Ubuntu/Debian: `sudo apt-get install gcc-arm-none-eabi cmake libusb-1.0-0 g++ ninja-build`
..* On Arch install gcc-arm-none-eabi-bin, cmake, ninja-build and libusb..
..* If your distribution doesn’t have compiler package for arm-none-eabi architecture, install the binary package from https://launchpad.net/gcc-arm-embedded
2. Download and install VS Code from https://code.visualstudio.com/
3. Launch VS Code, press Ctrl-Shift-X, find “husarion" extension and click “Install". VS Code will ask you if you want to install also the dependencies - agree with it. Reload VSCode.

### Using the VSCode + extension ###

To create new project, select empty folder, press Ctrl-Shift-P, type “create husarion project" and press enter.

***<font color="green">Info: The “husarion" extension can start downloading additional data now; please be patient.</font>***

To compile project press Ctrl-Shift-B (this happens automatically when you flash project).
To flash project to CORE2 via micro USB, press Ctrl-Shift-P and type “flash core2" and press enter.

***<font color="green">Info: Debugging is not yet supported. Autocomplete function is partially supported.</font>***

### Flashing with ST-LINK V/2 ###

***<font color="green">Info: For more advanced users. You don’t need this tool if flashing CORE2 via USB is enough. In addition, currently it has to be used from command-line, because it is not integrated with VS Code yet.</font>***

To install the st-flash software, follow the instructions from here: https://github.com/texane/stlink/blob/master/README.md

For Debian/Ubuntu you have to compile this tool from source:

* $ `wget https://github.com/texane/stlink/archive/master.zip && unzip master.zip`
* $ `cd stlink-master`
* $ `make release`
* $ `mkdir release && cd release`
* $ `cmake -DCMAKE_BUILD_TYPE=Release ..`

***<font color="green">Info: The following commands install the tool system-wide:</font>***

* `$ sudo make install`
* `$ sudo ldconfig`

To flash the output file to CORE2, use the st-flash command:

* `$ sudo st-flash write myproject.bin 0x08010000`

***<font color="green">Info: The binary file "myproject.bin" is located in the folder selected during previous point.</font>***

## Offline development tools for Mac ##

### Current functionality ###

Build tools:
* GNU ARM Embedded Toolchain
* support for CMake
<!-- * flasher tool for downloading the code to CORE2 via USB cable -->
* st-flash tool for downloading the code to CORE2 via ST-LINK V/2 debugger

### Installation guide ###

***<font color="green">Info: In order to compile projects for CORE2, you need a compiler that can compile code for its ARM processor. We recommend installing the cross-compiler and other dependencies from your distribution packages.</font>***

1. Install C++ cross-compiler - type in the terminal:
* $ brew tap PX4/homebrew-px4
* $ brew update
* $ brew install gcc-arm-none-eabi-48
2. Install CMake:
* $ brew install cmake
3. Install Ninja:
* $ brew install ninja
4. Download and install VS Code from https://code.visualstudio.com/
5. Launch VS Code, press Ctrl-Shift-X, find "husarion" extension and click "Install". VS Code will ask you if you want to install also the dependencies - agree with it. Reload VSCode.

### Using the VSCode + extension ###

To create new project, select empty folder, press Ctrl-Shift-P, type "create husarion project" and press enter.

***<font color="green">Info: The "husarion" extension can start downloading additional data now; please be patient.</font>***

To compile project press Ctrl-Shift-B (this happens automatically when you flash project).
<!-- To flash project to CORE2 via micro USB, press Ctrl-Shift-P and type "flash core2" and press enter. -->

***<font color="green">Info: Flashing from the VSCode window and debugging are not yet supported. Autocomplete function is partially supported.</font>***

### Flashing with ST-LINK V/2 ###

<!-- ***<font color="green">Info: For more advanced users. You don’t need this tool if flashing CORE2 via USB is enough. In addition, currently it has to be used from command-line, because it is not integrated with VS Code yet.</font>*** -->

***<font color="green">The always up-to-date manual how to install the tool can be found here: https://github.com/texane/stlink/blob/master/README.md </font>***

For Mac write in a terminal:
 
* $ brew install stlink

To flash the output file to CORE2, use the st-flash command:

* $ sudo st-flash write myproject.bin 0x08010000

***<font color="green">Info: The binary file "myproject.bin" is located in the folder selected during previous point.</font>***
