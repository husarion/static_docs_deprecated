---
title: 'External modules'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
order: 4
page: 'Manuals'
onepager: true
---


***

# External hardware #
This section will show you how to connect and use different types of external hardware modules with CORE2. CORE2 has UART, I2C, SPI, CAN, ADC and external interrupt channels to connect and efficiently use a lot of market available external modules. The following examples will help you connect not only the modules made by Husarion but also the third-party drivers, sensors etc.

***

## Servo controller for CORE2 ##

Allows to connect 12 additional servos to CORE2.

Communicates with CORE2 via hSensor interface. Up to four servo controllers can work on one hSensor port thanks to 2-bit addressing. The step down DC/DC converter with software-controlled output voltage is available on board to simplify power connections - you can supply the servo controller and CORE2 from the same power source.

Servo controller can deliver up to 3A average current to the servos.

The picture below describes the pinout of the Servo driver.

<div class="thumb center h350">
![](/assets/img/core2-hardware/servo_driver.png)
</div>

### First run ###

Connect the Vin voltage (+6V...+16V) and servos that you need. Connect the UART interface to hSens3 or hSens4 on CORE2 using the flat IDC cable. 
Build your program for CORE2 following the example available on https://husarion.com/core2/examples/
Library for ServoDriver is available on GitHub:
https://github.com/husarion/modules

### Specification ###

 * Integrated DC/DC converter
 * Output voltage (+V servo) selectable by software: 5V / 6V / 7.4V / 8.6V
 * 12 PWM outputs, 3.3V logic level, compatible with most analog and digital RC servos
 * 3A nominal output current
 * 5A peak output current
 * Input voltage range (Vin): 6...16V
 * Logic supply voltage: +5V typical, +4...+10V is acceptable.
 * Overcurrent and short-circuit protection

### Address selection ###

You can connect up to four servo controllers to one UART interface (hSens3 or hSens4 port of CORE2) in parallel. The IDC flat cable allows to crimp additional connectors anywhere along the cable. 
Each command sent by CORE2 to servo controller contains an address from 0 to 3. Each servo controller will execute only the commands that match the local address, configured with jumper. The default address is 0 (without jumper).

### LED behavior ###

In the current firmware, the LED is turned on if both power supplies (+5V and +Vin) are connected. LED is blinking when servo controller receives commands.

***

## Sharp distance sensor ##

Bellow schematic and source code shows you how to connect [Sharp GP2Y0A41SK0F](http://www.sharp-world.com/products/device/lineup/data/pdf/datasheet/gp2y0a41sk_e.pdf "Sharp GP2Y0A41SK0F") infrared proximity sensor to CORE2. Sample source code shows how to output readings from sensor directly to the user interface of your device at [](https://cloud.husarion.com).

<div class="thumb center h200">

![](/assets/img/core2-hardware/sharp.svg)

</div>

```
uint16_t v_int;

platform.begin(&RPi);
platform.ui.setProjectId("@@@PROJECT_ID@@@");

hSens1.pin1.enableADC();

while (true) {
	v_int = hSens1.pin1.analogReadRaw();
	platform.ui.label("l1").setText("Sensor raw output = %d\r\n", v_int);
	sys.delay(100);
}
```

***

## MPU9250 inertial mesurement unit ##

MPU9250 is a nine-axis (gyro, accelerometer, compass) inertial measuerement unit useful in a large variety of applications, e.g. drones. Below image and sample code will help you start using this awesome sensor!

<div class="thumb center h200">

![](/assets/img/core2-hardware/mpu9250.svg)

</div>