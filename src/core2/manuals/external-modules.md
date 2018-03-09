---
title: 'External modules'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
order: 5
page: 'Manuals'
onepager: true
---

**External modules**

This section will show you how to connect and use different types of external hardware modules with CORE2. CORE2 has UART, I2C, SPI, CAN, ADC and external interrupt channels to connect and efficiently use a lot of market available external modules. The following examples will help you connect not only the modules made by Husarion but also the third-party drivers, sensors etc.

***

# Sharp distance sensor #

Bellow schematic and source code shows you how to connect [Sharp GP2Y0A41SK0F](http://www.sharp-world.com/products/device/lineup/data/pdf/datasheet/gp2y0a41sk_e.pdf "Sharp GP2Y0A41SK0F") infrared proximity sensor to CORE2. Sample source code shows how to output readings from sensor directly to the user interface of your device at [](https://cloud.husarion.com).

<div class="thumb center h200">

![](/assets/img/external-modules/sharp.svg)

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

# MPU-9250 inertial mesurement unit #

MPU-9250 is a nine-axis (gyro, accelerometer, compass) inertial measuerement unit useful in a large variety of applications, e.g. drones. Below image will help you start using this awesome sensor!

<div class="thumb center h200">

![](/assets/img/external-modules/mpu9250.svg)

</div>

The example firmware using MPU-9250 is available as a template from Husarion Cloud.