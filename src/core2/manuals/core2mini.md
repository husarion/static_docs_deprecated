---
title: 'CORE2mini manual'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
order: 2
page: 'Manuals'
onepager: true
---

<div class="gallery h300">

![Appearance](/assets/img/core2mini/coremini.jpg "Appearance")
![Pinout cheatsheet](/assets/img/core2mini/scheme.jpg "Pinout cheatsheet")

</div>

# Electrical specification #

<table>
    <tr>
       <th>Interface</th>
       <th>Description</th>
       <th>Parameters</th>
    </tr>
    <tr>
        <td>Power input</td>
        <td>6.8-16V</td>
        <td>70...2500mA current consumption, depends on external modules<br>standard 5.5/2.1 mm DC plug (centre-positive)</td>
    </tr>
    <tr>
        <td>I/O ports</td>
        <td>20</td>
        <td>3.3V/5V tolerant GPIOs<br>series resistance is 330 ohms</td>
    </tr>
    <tr>
        <td>ADC</td>
        <td>3 channels</td>
        <td>12-bit resolution</td>
    </tr>
    <tr>
        <td>PWM</td>
        <td>6 channels:<br/>
        - 4x 3.3V<br/>
        - 2x H-bridge output
        </td>
        <td>Frequency range for H-bridge: 1Hz...21khz (in 16 steps)<br/>
        	Period range for 3.3V outputs: 1...65535 us
        </td>
    </tr>
    <tr>
        <td>UART</td>
        <td>2 channels</td>
        <td>baudrate: 4800, 9600, 14400, 19200, 38400, 57600, 115200, 128000, 256000, 1000000, 2000000, 4000000</td>
    </tr>
    <tr>
        <td>I2C</td>
        <td>1 channel</td>
        <td>up to 400kHz</td>
    </tr>
    <tr>
        <td>External Interrupts</td>
        <td>up to 3 channels</td>
        <td>triggered by an edge or voltage level</td>
    </tr>
</table>

***

# Ports description #

## hSensor ##

CORE2mini is equipped with three hSensor ports (Shrouded Box Header: 2×3-Pin, 0.1" (2.54 mm) Male).

The hSensor is intended to be used with many different sensors, such as spatial orientation sensors (like MPU9250), light sensors, sound sensors, limit switches and many others.

**Each hSensor port contains three basic elements:**

1. An **Analog to Digital Converter (ADC)** channel. The full range is 0 – 3.3V. The alternative function is interrupt input.

2. An **auxiliary 5V supply output** dedicated to supplying the sensor circuit. The maximum current is not limited for each port, but due to the 5 V line total current limit, we recommend keeping the current below 50mA. If you are sure that the total 5V current will not exceed 1.5A, you can go higher, up to 500mA. See "Power supply" section for more details.

3. **4 digital inputs/outputs.** Additionally, some hSensor ports have a hardware UART interface assigned to these IO’s, and some have a hardware I2C interface. If you want UART or I2C, please check the software documentation which physical ports to use.

<div class="image center h300">

![](/assets/img/core2-hardware/hsensor.svg)

</div>

<table class="text_table">
<tbody>
    <tr>
        <th>hSensor pin</th>
        <th>Software name</th>
        <th>Default function</th>
        <th colspan="2">Alternate function</th>
    </tr>
    <tr>
        <td align="center">1</td>
        <td>hSensX.pin1</td>
        <td>GPIO</td>
        <td>external interrupt input</td>
        <td>ADC converter</td>
    </tr>
    <tr>
        <td align="center">2</td>
        <td>hSensX.pin2</td>
        <td>GPIO</td>
        <td></td>
        <td></td>
    </tr>
    <tr>
        <td align="center">3</td>
        <td>hSensX.pin3</td>
        <td>GPIO</td>
        <td>I2C_SCL (in hSens 1) <br>
            UART_TX (in hSens 2) <br>
	    no alt. function (in hSens 3)</td>
        <td></td>
    </tr>
    <tr>
        <td align="center">4</td>
        <td>hSensX.pin4</td>
        <td>GPIO</td>
        <td>I2C_SDA (in hSens 1)<br>
        UART_RX (in hSens 2) <br>
	no alt. function (in hSens 3)</td>
        <td></td>
    </tr>
    <tr>
        <td align="center">5</td>
        <td>-</td>
        <td>+5V power supply output</td>
        <td></td>
        <td></td>
    </tr>
    <tr>
        <td align="center">6</td>
        <td>-</td>
        <td>GND (0V)</td>
        <td></td>
        <td></td>
    </tr>
</tbody>
</table>

[comment]: <> (***Advice: use <mark>ctrl + SPACE</mark> after typing "software_name." to see methods in the web IDE.***)

Using ADC
```
hSens1.pin1.enableADC();
while (true) {
	// read analog value (voltage 0.0 - 3.3V)
	float v_analog = hSens1.pin1.analogReadVoltage();

	// read raw value (voltage 0x0000 - 0x0fff)
	uint16_t v_int = hSens1.pin1.analogReadRaw();

	printf("%f | %d\r\n", v_analog, v_int);
	sys.delay(50);
```

### Communication interfaces ###

<table class="text_table">
<tbody>
    <tr>
        <th align="left">hExt communication interface</th>
        <th align="left">Software name</th>
        <th align="left">Parameters</th>
    </tr>
    <tr>
        <td>USART</td>
        <td>hExt.serial</td>
        <td>baudrate: 4800, 9600, 14400, 19200, 38400, 57600, 115200, 128000, 256000, 1000000, 2000000, 4000000</td>
    </tr>
    <tr>
        <td>SPI</td>
        <td>hExt.spi</td>
        <td>up to 1 Mbps</td>
    </tr>
    <tr>
        <td>I2C</td>
        <td>hExt.i2c</td>
        <td>up to 400kHz</td>
    </tr>
</tbody>
</table>


## hServo ##

You can connect up to 4 servo motors directly to CORE2mini. Power supply is onboard thanks to integrated DC/DC converter with selectable voltage output (remeber that there is one power supply voltage for all servos).

<div class="image center h300">

![](/assets/img/core2-hardware/hservo.svg)

</div>

<table class="text_table">
<tbody>
    <tr>
        <th>hServo pin</th>
        <th>Description</th>
        <th>Parameters</th>
    </tr>
    <tr>
        <td align="center">1</td>
        <td align="left">PWM output</td>
        <td>3.3V standard, pulse width: 1 - 65535 us, pulse period: 1 - 65535 us (can not be higher than pulse width)</td>
    </tr>
    <tr>
        <td align="center">2</td>
        <td align="left">Servo power supply output</td>
        <td>selectable voltage level: 5V / 6V / 7.4V / 8.6V (tolerance +/- 0.2V) <br>
        Maximum current comsuption for all servos: 2A (continous) , 3.5A (peak)</td>
	</tr>
    <tr>
        <td align="center">3</td>
        <td align="left">GND (0V)</td>
        <td>-</td>
    </tr>
</tbody>
</table>

```
hServoModule.enablePower();
hServoModule.s1.setPeriod(20000); //PWM period 20ms
while (1) {
	hServoModule.s1.setWidth(1000); //set pulse width: 1000us
	sys.delay(1000);
	hServoModule.s1.setWidth(2000);
	sys.delay(1000);
}
```

## hMotor ##
CORE2mini is equipped with two hMotor ports.

The hMotor is intended to be used with a DC motor with encoder, but you don’t have to use the encoder interface if you have a standard DC motor. The hMotor interface is fully-compatible with LEGO® MINDSTORMS® sets (remeber that you have to use special adapter to use these sets).

**Each hMot port contains three basic elements:**

An **H - bridge** for driving the DC motor (with or without encoder) or a stepper motor. It can also be used for driving other devices, but don’t forget about a PWM signal on the output and its limitations. The maximum average output current for each hMot is 1A, and the maximum peak current is 2A. The H-bridge is supplied from the CORE2mini power supply voltage Vin (6 - 16V) and you can expect the same at the H-bridge output.

An **auxiliary 5V supply** output dedicated to supplying the encoder circuit. The maximum current is not limited for each port, but due to the 5 V line total current limit, it is recommended keeping the current below 50mA. If you are sure that the total 5 V current will not exceed 1.5A, you can go higher, up to 1A. See "Power supply" section for more details.

A **quadrature encoder interface** is used to control position or speed of the electric motor shaft, so you know whether your control algorithm works as you expected. Husarion CORE2mini uses hardware quadrature encoder interface provided by STM32F4 microcontroller (functionality built into some timer peripheral interfaces of STM32F4). For this reason you don't waste processing power of CPU to detect each slope on encoder output signal. Everything is done by special hardware interface, so you don't have to worry about missing any change of your motor shaft.
Wikipedia provides an accessible explanation how encoders work: [incremental rotary encoder](https://en.wikipedia.org/wiki/Rotary_encoder).

The encoder interface is compatible with the majority of popular optical and magnetic encoders integrated with motors or sold separately.

<div class="image center h300">

![](/assets/img/core2-hardware/hmot.svg)

</div>

<table class="text_table">
<tbody>
    <tr>
        <th align="center">hMot pin</th>
        <th align="left">Default function</th>
        <th align="left">Description</th>
    </tr>
    <tr>
        <td align="center">1</td>
        <td>Output A</td>
        <td>Output voltage: 0 - Vin (6-16V) <br>
	        Output current: 1A (continuous), 2A (peak) with built-in overcurrent protection</td>
	</tr>
    <tr>
        <td align="center">2</td>
        <td>Output B</td>
        <td>Output voltage: 0 - Vin (6-16V) <br>
        Output current: 1A (continuous), 2A (peak) with built-in overcurrent protection
        </td>
    </tr>
    <tr>
        <td align="center">3</td>
        <td>GND</td>
        <td>Ground terminal</td>
    </tr>
    <tr>
        <td align="center">4</td>
        <td>+5V output</td>
        <td>Voltage supply for encoder circuit. Keep the maximum current below 50mA for each hMot port.</td>
    </tr>
    <tr>
        <td align="center">5</td>
        <td>Encoder input A</td>
        <td>5V standard digital input for encoder interface (channel A)</td>
    </tr>
    <tr>
        <td align="center">6</td>
        <td>Encoder input B</td>
        <td>5V standard digital input for encoder interface (channel B)</td>
    </tr>
</tbody>
</table>

### Supported motor types ###

**DC motor with encoder**

<div class="thumb w180 right">

![](/assets/img/core2-hardware/motors_encoders.jpg)

</div>

This motor type is suitable for more professional applications. It can be identified by 6 wires coming out of the encoder board. DC motor with quadrature encoder interface allows you to create own sophisticated control algorithm optimized for your application in contrast to RC servos that can't give any feedback to your algorithms.

Remember not to power your motors using higher voltage than recommended in their specification.


**DC motor without encoder**

<div class="thumb w180 right">

![](/assets/img/core2-hardware/dc_motor.jpg)

</div>

Of course, in many cases you don't need the encoder - e.g. if you need to drive wheels without sensing their position. In that case you can use a simple DC motor with gearbox. It can be identified by 2 wires coming out of the motor.

Despite the lack of the encoder, you still can recognize the extreme positions of your mechanism using the limit switches.

**LEGO® motor**

<div class="thumb w180 right">

![](/assets/img/core2-hardware/lego_motors.jpg)

</div>

CORE2mini is fully compatible with servomotors from LEGO® Mindstorms® when used with connector adapter. There are 3 types of LEGO® servomotors: motor from NXT/NXT2.0 kit and two types from EV3 kit. In fact, they are all motors with quadrature encoder. \\
Remember that LEGO® motors have 9V nominal voltage and when you supply CORE2mini with higher voltage, you should limit the PWM duty cycle.


**Stepper motor**

<div class="thumb w180 right">

![](/assets/img/core2-hardware/hstep.png)

</div>

Connecting a bipolar stepper motor is also possible. In this case, you need two hMotor ports to drive one stepper motor. If your motor windings have 4 or 6 terminals, it can work in the bipolar configuration (the 6-terminal motors can work in both unipolar and bipolar configuration). In the picture you can see how to connect the bipolar motor with two H-bridge outputs.

## DBG ##

<div class="image center h300">

![](/assets/img/core2-hardware/dbg.svg)

</div>

If you are an advanced code developer you will probably appreciate it! The DBG connector
allows you not only to upload the code to the CORE2mini, but it is also a debugging interface for the STM32F4 microcontroller.

To use the DBG interface, you need an additional hardware programmer/debugger: ST-LINK/V2. You can find the original one here: [ST-LINK/V2](http://www.st.com/web/catalog/tools/FM146/CL1984/SC724/SS1677/PF251168)

You also need to configure the offline development environment. You will find the instructions here: [Husarion SDK](https://wiki.husarion.com/howto:installation)

## hSerial ##
The hSerial port is an USB device port with a standard micro B USB connector, but it's called "hSerial" because this port is connected to the serial port of the microcontroller. It is not the native USB port - it uses the FTDI® chip to connect the internal serial port to your computer or other USB host.

The hSerial port can be used to:
* read logs from the CORE2mini device on your computer,
* upload new software to the CORE2mini microcontroller (if the wireless connection is not available),
* other communication with any USB host device (FTDI driver is needed).

CORE2mini cannot be powered via the USB hSerial port!

## hSD ##
Just a connector for a standard microSD card. It uses one of the SPI interfaces available in the microcontroller. The rest is software.

## LEDs ##

<div class="thumb w270 right">

![User&apos;s leds](/assets/img/core2-hardware/leds.svg "User&apos;s leds")

</div>

There are 3 green LEDs to be controlled by user on CORE2mini: LED1, LED2 and LED3. They are described **L1**, **L2**, **L3** on the PCB.

The **PWR** LED is indicating that CORE2mini board is powered and switched on.

The **LR1**, **LR2** LEDs are used by modules connected to RPI connector.

```
LED1.off(); // initially off
LED2.on(); // LED2 will stay on
while (true) {
	LED1.toggle(); // toggle LED1
	sys.delay(500); // wait for 500 ms
}
```


***

# Power supply #
Before powering the CORE2mini you should know something about its power supply input.

The **CORE2mini** input voltage (Vin) must be in the range 6 - 16V. The recommended input voltage range is 7 - 15V. The power connector is a standard DC 5.5/2.1 (centre-positive) type. The minimum power supply output current to run CORE2 itself is about 150mA@12V and 200mA@9V.

The CORE2mini power supply input has overvoltage (>16V), reverse-polarity and overcurrent (~3A) protections. The long-term overvoltage  or reverse-polarity state shall be avoided!

## Block diagram ##

<div class="thumb center">

![](/assets/img/core2-hardware/powersupply.svg)

</div>

<table class="text_table">
<tbody>
    <tr>
        <th>Voltage line name</th>
        <th align="center">I max/th>
        <th>Available on port:</th>
        <th>Info</th>
    </tr>
    <tr>
        <td>Vin</td>
        <td align="center">-</td>
        <td>-</td>
        <td>main power input</td>
    </tr>
    <tr>
        <td>+5V</td>
        <td align="center">1.5A</td>
        <td>hMot, hRPI, USB host</td>
        <td></td>
    </tr>
    <tr>
        <td>+5V(sw)</td>
        <td align="center">1A</td>
        <td>hSensor</td>
        <td>drawn from +5V, switched by software</td>
    </tr>
    <tr>
        <td>+3.3V</td>
        <td align="center">0.5A</td>
        <td>-</td>
        <td>only for internal circuits</td>
    </tr>
    <tr>
        <td>Vservo</td>
        <td align="center">3A</td>
        <td>hServoModule</td>
        <td>programmable voltage: 5/6/7.4/8.2V</td>
    </tr>
</tbody>
</table>

## Controlling servo power supply ##

```
hServoModule.enablePower(); //turn servo DC/DC power converter on
hServoModule.setPowerMedium(); //set 6V on DC/DC power converter output
```

## How to power CORE2mini? ##
You can supply the CORE2mini with:

* 5 - 10 AA/AAA cells;
* 6 - 11 NiCd/NiMH cells;
* 2 or 3 Li-Ion/Li-Poly cells (e.g. 18650 batteries);
* an AC-to-DC wall adapter;
* a 12 V lead-acid battery.

**CORE2mini cannot be supplied from the USB port of your laptop.** Why? This is a controller designed for automation & robotics applications and has motor drivers on its board. Motors cannot be supplied from USB due to the current and voltage requirements. To avoid the risk of damaging the USB port we decided to supply CORE2mini separately. CORE2mini is designed to be programmed wirelessly and the USB connection is not the basic way to program or supply the controller (however, programming is possible through hSerial).

How much current does it need? It strongly depends on the robot configuration. A CORE2mini without any devices connected needs up to 80mA. When you connect certain motors, current peaks can reach several amperes. The average current should not exceed 3.5A, otherwise the overcurrent protection will be triggered and unexpected resets will occur. Remember this when you are designing your device.

CORE2mini has two internal voltage regulators. The input voltage (behind protection circuit) Vin(p) is converted to 5V by a switching regulator, and then to 3.3V by a linear voltage regulator. Be aware of the current limits – the total current must not exceed 1.5A through the 5V line. We will also remind you about power limitations in the description of individual interfaces.

The supply voltage +5V(sw) for hSens connectors can be switched on and off. It is enabled by default but can be switched off in the software.</br>

**Power supply alternatives**

If you are not willing to use AA or similar alkaline batteries, the first alternative is to use NiCd or NiMH rechargeable batteries - but they have much lower nominal voltage. The better alternative are Li-Ion or Li-Poly batteries. Fortunately, these are available in the same shape as AA batteries and they are called “14500”. The name comes from their dimensions: 14x50mm.

Some examples:
[14500 reachargeable battery on AliExpress](http://www.aliexpress.com/wholesale?catId=0&initiative_id=SB_20160428221617&SearchText=14500+rechargeable)

Of course, you will also need a charger.

Remebmer that Li-Ion and Li-Poly batteries have higher nominal voltage and you have to use 3 cells instead of 6 cells. To do that, you can:
* use only one 3*AA battery holder with 3 Li-Ion/Poly batteries,
* use 6*AA battery holder with 3 “dummy” batteries and 3 Li-Ion/Poly batteries.
The “dummy” (placeholder) batteries examples:
[AA placeholder on AliExpress](http://www.aliexpress.com/wholesale?catId=0&initiative_id=AS_20160428223009&SearchText=AA+placeholder)
They cannot be charged - they are only the “link” to omit 3 unnecessary places in the battery holder.

***

# Internet access #

To use CORE2mini hardware from the cloud, you need to provide the Internet connection for CORE2mini.

This can be done thanks to cheap Wi-Fi module, such as ESP32, as well as a Linux computer (e.g. RaspberryPi). All depends on your application. In most cases ESP32 is sufficient, but in some cases more computing power and andvanced onboard libraries (e.g. ROS - Robotic Operating System) are necessary. This section will help you to choose the configuration you need.

By now you know 2 basic ways to connect CORE2mini to the Internet, ESP32 adapter or a Raspberry Pi computer. In the future other options will be available.

## Connecting CORE2mini to the cloud ##

Use hConfig app (to be found on AppStore or Google Play) where wizard will guide you through all the steps required to connect your CORE2mini to the Husarion cloud.

### Status LEDs ###

There are 2 status LEDs - LR1 and LR2 - controlled directly from the ESP32/RPi device. These LEDs can't be controlled from STM32 microcontroller. The following table shows their behavior under different conditions:

<table class="text_table">
<tbody>
    <tr>
        <th>Mode</th>
        <th>LR1 (yellow)</th>
        <th>LR2 (blue)</th>
	<th>Period</th>
	<th>Behavior</th>
    </tr>
    <tr>
        <td>Config mode</td>
        <td colspan="2">blinking alternately</td>
	<td>600 ms</td>
	<td>![](/assets/img/core2-hardware/lr12_gif/LR12_config.gif)</td>
    </tr>
    <tr>
        <td>Connecting</td>
        <td>OFF</td>
        <td>blinking</td>
	<td>300 ms</td>
	<td>![](/assets/img/core2-hardware/lr12_gif/LR12_connecting.gif)</td>
    </tr>
    <tr>
        <td>Connected</td>
        <td>OFF</td>
        <td>ON</td>
	<td>-</td>
	<td>![](/assets/img/core2-hardware/lr12_gif/LR12_connected.gif)</td>
    </tr>
    <tr>
        <td>Not configured</td>
        <td>blinking</td>
        <td>OFF</td>
	<td>100/1000 ms</td>
	<td>![](/assets/img/core2-hardware/lr12_gif/LR12_not_conf.gif)</td>
    </tr>
    <tr>
        <td>Invalid auth.</td>
        <td>blinking</td>
        <td>OFF</td>
	<td>100 ms</td>
	<td>![](/assets/img/core2-hardware/lr12_gif/LR12_invalid_auth.gif)</td>
    </tr>
    <tr>
        <td>No Internet</td>
        <td>blinking</td>
        <td>ON</td>
	<td>100 ms</td>
	<td>![](/assets/img/core2-hardware/lr12_gif/LR12_no_internet.gif)</td>
    </tr>
</tbody>
</table>

## hRPI connector ##

<div class="thumb right w180">

![](/assets/img/core2-hardware/rpi_connector.png "hRPI connector")

</div>

Although the connector's name comes from Raspberry Pi, it is designed to be used with both ESP
and Raspberry. CORE2mini comes without any connector soldered because the connector
type depends on module for Internet connection you are going to use in your project.

If your ESP32 or Raspberry Pi is not installed to CORE2mini yet, see the instruction here:
[Assembling the ESP32 adapter](howtostart#preparing-hardware). This page also serves as the
guide for connecting CORE2mini with our cloud.

<table class="text_table">
<tbody>
    <tr>
        <th>hRPI pin</th>
        <th>Signal name</th>
        <th>Description</th>
    </tr>
    <tr>
        <td align="center">1</td>
        <td>---</td>
        <td>Not connected</td>
    </tr>
    <tr>
        <td align="center">2</td>
        <td>+5V</td>
        <td>Supply voltage (max. 1A)</td>
    </tr>
    <tr>
        <td align="center">3</td>
        <td>LR2</td>
        <td>LED LR2 (cathode)</td>
    </tr>
    <tr>
        <td align="center">4</td>
        <td>+5V</td>
        <td>Supply voltage (max. 1A)</td>
    </tr>
    <tr>
        <td align="center">5</td>
        <td>GPIO</td>
        <td>GPIO (STM34F4)</td>
    </tr>
    <tr>
        <td align="center">6</td>
        <td>GND</td>
        <td>Ground (0V)</td>
    </tr>
    <tr>
        <td align="center">7</td>
        <td>GPIO</td>
        <td>GPIO (STM34F4)</td>
    </tr>
    <tr>
        <td align="center">8</td>
        <td>UART RX</td>
        <td>UART RX (STM32F4)</td>
    </tr>
    <tr>
        <td align="center">9</td>
        <td>GND</td>
        <td>Ground (0V)</td>
    </tr>
    <tr>
        <td align="center">10</td>
        <td>UART TX</td>
        <td>UART TX (STM32F4)</td></tr>
    <tr>
        <td align="center">11</td>
        <td>LR1 / BOOT0</td>
        <td>LED LR1 (anode) / BOOT0 pin (STM32F4)</td>
    </tr>
    <tr>
        <td align="center">12</td>
        <td>RST</td>
        <td>RST active-high (STM32F4)</td>
    </tr>
    <tr>
        <td align="center">13</td>
        <td>hCFG</td>
        <td>hCFG button</td>
    </tr>
    <tr>
        <td align="center">14</td>
        <td>GND</td>
        <td>Ground (0V)</td>
    </tr>
</tbody>
</table>

***

# Updating firmware #
In this section you will find instructions on how to update CORE2mini bootloader when a newer version is available. You can also find information on how to install the newest image for external modules, that provide Internet access for CORE2mini.

## Updating CORE2mini bootloader ##

You need to have Visual Studio Code installed with Husarion extension. Please follow this guide if you haven't done this before: [VSCode installation](https://husarion.com/core2/tutorials/howtostart/offline-development-tools/#offline-development-tools-installation-guide) 
1. Locate core2-flasher utility (YOUR_HOME_PATH/.vscode/extensions/husarion.husarion-VERSION/sdk/tools/YOUR_ARCH/core2-flasher).
2. Download the bootloader [HEX file](https://files.husarion.com/bootloader/bootloader_1_0_0_core2.hex) to the folder with core2-flasher
3. Connect CORE2mini to PC via USB.
4. Install drivers (Windows only)
  * Download Zadig [Usb driver installator - Zadig](http://zadig.akeo.ie/)
  * Run Zadig
  * Select FT231X USB UART in Zadig window
  * Choose WinUSB driver and click **Replace Driver**
5. Open command line prompt.
6. Flash bootloader with commands:

  on Linux in the terminal:
  ```
  ./core2-flasher --unprotect
  ./core2-flasher bootloader_1_0_0_core2.hex
  ./core2-flasher --protect
  ```
  on Windows in the terminal:
  ```
  core2-flasher.exe --unprotect
  core2-flasher.exe bootloader_1_0_0_core2.hex
  core2-flasher.exe --protect
  ```

## Updating ESP32 firmware ##

Make sure that your CORE2mini is connected with your cloud account. [This is a guide](https://husarion.com/core2/tutorials/howtostart/run-your-first-program/#run-your-first-program-connecting-to-the-cloud) that explains how to do it.

1. Turn CORE2mini on and login to the cloud account. The device should be visible as "online".
2. Click "+" and "More".
3. Click OTA upgrade. Process should start immediately and inform you about progress. Do not turn off power supply!
4. When finished, you shall see the message: "OTA progress: success: upgrade completed". 

That's all, your ESP32 firmware is up-to-date.

***

# Docs for download #
All downloadable documents in one place:

* [CORE2mini Safety Instructions](http://files.husarion.com/doc_files/CORE2mini_Safety_Instructions.pdf "CORE2mini Safety Instructions") - important!
* [CORE2mini board mechanical drawing](http://files.husarion.com/doc_files/CORE2mini_board.pdf "CORE2mini board mechanical drawing")
