# RB3204-RBCX

<p align="center">
<a href="https://hits.seeyoufarm.com"><img src="https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2FRoboticsBrno%2FRB3204-RBCX&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=views&edge_flat=true"/></a>
<img src="https://img.shields.io/github/license/RoboticsBrno/RB3204-RBCX?style=flat-square">
<img src="https://img.shields.io/github/issues/RoboticsBrno/RB3204-RBCX?style=flat-square">
<img src="https://img.shields.io/github/stars/RoboticsBrno/RB3204-RBCX?style=flat-square">
</p>

RBCX is a universal controller for hobby robots. It is a successor of [RB3201-RBControl](https://github.com/RoboticsBrno/RB3201-RBControl).
It is used in our robot [Robotka](https://robotka.robotickytabor.cz) on [Robo camp 2020](https://robotickytabor.cz).

<div align="center">
  	<td><img src="./docs/media/rbcx-front.png" width="49%"></td>
	<td><img src="./docs/media/rbcx-back.png" width="49%"></td>
</div>

![RBCX](./docs/media/rbcx-schematic.png)

## Features
- 4 DC motors with encoders
- 4 servos with feedback (via ADC)
- 4 LEDs
- buttons with the gamepad interface
- Smart Servo connector
- piezo buzzer
- RTC
- ultra-sonic distance sensor connector
- I2C connectors
- OLED display connector
- accelerometer and gyroscope
- micro USB and USB-C connectors
- battery connector (without charging circuit)

![RBCX](./docs/media/robotka.jpg)

## Hardware
Board is based on `STM32F103VC` microcontroller and `ESP32-DevKitC`.

STM32 is used for motor control and other low-level tasks.
ESP32 is used for high-level tasks like communication with mobile app and sending commands to STM32 via UART.


### [STM32F103VC](https://www.st.com/en/microcontrollers-microprocessors/stm32f103vc.html)

-  72 MHz, 32bit ARM Cortex-M3
-  48 KiB RAM, 256 KiB Flash
-  5x UART, USB
-  Debug support (with ST-Link)
-  Programmable in C++, with FreeRTOS and stm32cube framework, in Platform.io
-  Our [bootloader](https://github.com/RoboticsBrno/sboot_stm32) based on [sboot_stm32](https://github.com/dmitrystu/sboot_stm32).


### [ESP32-DevKitC](https://www.espressif.com/en/products/devkits/esp32-devkitc)
