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

hFramework API documentation is available at https://husarion.com/core2/api_reference/classes.html . API it is a set of clearly defined methods of communication between various software components our API makes it easier to develop a program by providing all the building blocks. Mainly use by advanced users. 

# Using hFramework #

The easiest way to experience hFramework is to use Husarion WebIDE or install Husarion plugin to Visual Studio Code. Each of these methods gives us different options. With WebIDE we can start programing our device right after unpacking and the controller does not have to be even close to us, if you just starting follow this <a href="https://husarion.com/tutorials/howtostart/run-your-first-program/">tutorial</a>. Programming via VSC allows us to rebuild the default implemented library but we have to connect board with computer via USB and and depending on the system we use, install several packages. full instruction of using offline development tools you can find <a href="https://husarion.com/tutorials/other-tutorials/offline-development-tools/">here</a>.

# Building hFramework #

If you need to change code of hFramework you have to follow the instructions from <a href="https://husarion.com/tutorials/other-tutorials/hframework-library-development/">manual</a>. 

# Examples #

In this chapter you can find few examples of code for CORE2. You can check other examples on GitHub:

<a href="https://github.com/husarion/hFramework/tree/master/examples">hFramework examples</a>. 
<a href="https://github.com/husarion/hSensors/tree/master/examples">hSensors examples</a>. 
<a href="https://github.com/husarion/modules/tree/master/examples">Modules examples</a>. 

## Basics ##

<script src="https://gist.github.com/Hubert424/e33f11805eeaf66474bd517af6713654.js"></script>

## GPIO ##

### adc ###

<script src="https://gist.github.com/Hubert424/7d91a1cb9ca6b5ec051ae6b30a91ad85.js"></script>

### gpio ###

<script src="https://gist.github.com/Hubert424/16698beee46bba524a57ddd2298ca03a.js"></script>

### gpio inout ###

<script src="https://gist.github.com/Hubert424/a25dbb5de8513af9fb43d06de930be04.js"></script>

## LED ##

### LED ###

<script src="https://gist.github.com/Hubert424/6f8461c9c3b83806295208a6945911a9.js"></script>

## Interfaces ##

### buttons simple ###

<script src="https://gist.github.com/Hubert424/efd53bf39493cead6193fcb29ffcd49d.js"></script>

## Motors ##

### motor angle ###

<script src="https://gist.github.com/Hubert424/e9bd6bbc3ebed8aee69e2ff246969a86.js"></script>

## SPI ##

### spi ###

<script src="https://gist.github.com/Hubert424/cafe66affe9cbf890de04bc7f8023780.js"></script>

## Serial I/O ##

### serial basic ###

<script src="https://gist.github.com/Hubert424/f8a6f4fb5c956eb5777c88b5c656f056.js"></script>

## System ##

### sys_mutex ###

<script src="https://gist.github.com/Hubert424/140428a98ffaf6eff2a1ba23516e1320.js"></script>

### sys_queue ###

<script src="https://gist.github.com/Hubert424/d3c00cf86ec91c7ea5b1b84215c79fac.js"></script>

### sys_task ###

<script src="https://gist.github.com/Hubert424/0750f233bc614e11d002a08fe5891d4e.js"></script>

## Sensors ##

### lego touch ###

<script src="https://gist.github.com/Hubert424/aa4e4116e8ca2714677c046f5f346c6d.js"></script>
