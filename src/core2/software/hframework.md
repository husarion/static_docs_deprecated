---
title: 'hFramework'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
order: 2
page: 'Software'
onepager: true
---

# About #

hFramework is a library for creating software for mechatronic devices (e.g. robots). It's completely open sorce. You can find it in the Husarion's repository on <a href="https://github.com/husarion/hFramework">GitHub</a>.

It has the following ports:

    STM32 port for hardware created by Husarion - CORE2, CORE2mini and RoboCORE boards
    Linux port for Raspberry Pi and Tinkerboard (experimental)
    ESP32 port (experimental)

# API reference #

hFramework API documentation is available at https://husarion.com/core2/api_reference/ . API it is a set of clearly defined methods of communication between various software components our API makes it easier to develop a program by providing all the building blocks. Mainly use by advanced users. 

# Using hFramework #

The easiest way to experience hFramework is to use Husarion WebIDE or install Husarion plugin to Visual Studio Code. Each of these methods gives us different options. With WebIDE we can start programing our device right after unpacking and the controller does not have to be even close to us, if you just starting follow this <a href="https://husarion.com/tutorials/howtostart/run-your-first-program/">tutorial</a>. Programming via VSC allows us to rebuild the default implemented library but we have to connect board with computer via USB and and depending on the system we use, install several packages. full instruction of using offline development tools you can find <a href="https://husarion.com/tutorials/other-tutorials/offline-development-tools/">here</a>.

# Building hFramework #

If you need to change code of hFramework you have to follow the instructions from <a href="https://husarion.com/tutorials/other-tutorials/hframework-library-development/">manual</a>. 


