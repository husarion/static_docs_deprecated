---
title: 'Offline development tools'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 2
---

# Offline development tools #

If you prefer or need offline development tools, Husarion created an extension for Visual Studio Code, that will configure all you need to get started. This extension works both for Windows, Linux and MacOS. Installation process is described in the following sections and in the video below.

<div style="text-align: center">
<iframe width="854" height="480" src="https://www.youtube.com/embed/mdHPdcL7gaA" frameborder="0" allowfullscreen>
</iframe>
</div>

## Offline development tools for Windows ##

### Current functionality ###

Code editor: Visual Studio Code

Build tools - Husarion extension for VSCode that include:
* GNU ARM Embedded Toolchain
* support for CMake
* flasher program for downloading the code to CORE2 via USB
* code autocompletion
* native debugger support

### Installation guide ###

1. Download and install VS Code from [https://code.visualstudio.com/](https://code.visualstudio.com/)
2. Launch VS Code, press Ctrl-Shift-X, find “husarion" extension and click “Install". VS Code will ask you if you want to install also the dependencies - agree with it. Reload VSCode. ***<font color="grey">Info: At this point you should have the following extensions installed: C/C++, CMake and Husarion. Do not install "CMake Tools" extension. If you already have it installed, disable it for workspaces with Husarion projects.</font>***
3. Download Zadig (http://zadig.akeo.ie/), connect CORE2 via micro USB cable and launch Zadig. Click [Options] -> [List All Devices], choose “FT230X Basic UART" from the drop-down list, select "WinUSB (v6.1.7600.16385)" driver and click “Replace Driver". ***<font color="grey">Info: This step is needed to flash the program to the microcontroller on CORE2. You need the administrator rights to do change the driver for USB device. If you encounter any problems with installation of the new driver, try to remove all old drivers related to USB Serial port or FTDI chip.</font>***

### Using the VSCode + extension ###

To create new project, select empty folder, press Ctrl-Shift-P, type “create husarion project" and press enter.

***<font color="green">Info: The "Husarion" extension may start downloading additional data now; please be patient.</font>***

To compile project press Ctrl-Shift-B. To flash the project to CORE2 via micro USB, press Ctrl-Shift-P and type “flash core2" and press enter.

### Useful extensions ###
* Builtin Git support - Ctrl-Shift-G You need to install Git first. We recommend using Chocolatey to do it.
* Git History (git log)

### Changing hardware platform ###

1. Click Ctrl-Shift-P, then chose “Change Husarion project variable.”
2. Select BOARD_TYPE to change board type or BOARD_VERSION to change board version.

### Debugging ###

1. Plug in ST-LINK device.
2. Using Windows automatic driver detection software install ST-LINK driver for your device.
3. Simply press F5 button in VSCode window to flash application and start debugging.


## Offline development tools for Linux ##

### Current functionality ###

Build tools:
* GNU ARM Embedded Toolchain
* support for CMake
* flasher tool for downloading the code to CORE2 via USB cable
* st-flash tool for downloading the code to CORE2 via ST-LINK V/2 debugger
* code autocompletion

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

To compile project press Ctrl-Shift-B. To flash the project to CORE2 via micro USB, press Ctrl-Shift-P and type “flash core2" and press enter.

### Changing hardware platform ###

1. Click Ctrl-Shift-P, then chose “Change Husarion project variable.”
2. Select BOARD_TYPE to change board type or BOARD_VERSION to change board version.

### Installing ST-LINK ###

***<font color="green">Info: For advanced users. You don’t need this tool if flashing CORE2 via USB is enough.</font>***

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

### Debugging ###

1. Plug in ST-LINK device.
2. Make sure you have ST-LINK installed.
3. Simply press F5 button in VSCode window to flash application and start debugging.

## Offline development tools for Mac ##

### Current functionality ###

Build tools:
* GNU ARM Embedded Toolchain
* support for CMake
* flasher tool for downloading the code to CORE2 via USB cable
* st-flash tool for downloading the code to CORE2 via ST-LINK V/2 debugger
* code autocompletion

### Installation guide ###

***<font color="green">Info: In order to compile projects for CORE2, you need a compiler that can compile code for its ARM processor. We recommend installing the cross-compiler and other dependencies from your distribution packages.</font>***

1. If you don't have brew installed, follow the instructions at https://brew.sh
2. Install C++ cross-compiler - execute in the terminal:
* `$ brew tap PX4/homebrew-px4`
* `$ brew update`
* `$ brew install gcc-arm-none-eabi-48`
3. Install CMake, Ninja and STLink:
* `$ brew install cmake ninja stlink`
4. Download and install VS Code from https://code.visualstudio.com/
5. Launch VS Code, press Ctrl-Shift-X, find "husarion" extension and click "Install". VS Code will ask you if you want to install also the dependencies - agree. Reload VSCode.

### Using the VSCode + extension ###

To create new project, select empty folder, press Ctrl-Shift-P, type "create husarion project" and press enter.

***<font color="green">Info: The "husarion" extension can start downloading additional data now; please be patient.</font>***

To compile project press Ctrl-Shift-B. To flash the project to CORE2 via micro USB, press Ctrl-Shift-P and type "flash core2" and press enter. 

### Debugging ###

1. Plug in the ST-LINK device.
2. Simply press F5 button in VSCode window to flash application and start debugging.
