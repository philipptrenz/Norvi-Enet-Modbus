# NORVI ENET AE-06-T Modbus TCP Server

This project implements a Modbus TCP server for the NORVI ENET AE-06-T, which is a *USB Programmable Industrial ESP32 with OLED Display and din-rail mount* \[[1](https://norvi.lk/product/industrial-esp32-ethernet/)\].

Features:
* 4 outputs as Modbus holding registers (register 0 to 3, read/write)
* 8 inputs as Modbus input registers (register 0 to 7, read-only)
* 3 device buttons readable via Modbus input registers (register 8 to 10, read-only)
* Ethernet connectivity via integrated W5500 chipset
* Basic information shown on the OLED display (logs, IP, link status)

The idea behind this project is to use the NORVI ENET as a "dumb" translator of the IOs into the network and implement the control logic in easier to maintain controllers, like [IOBroker](https://github.com/ioBroker/ioBroker) or [Node-RED](https://github.com/node-red/node-red).


## Why not build upon an existing IoT platform?

In fact, with stability and maintenance in mind, the original intention was to build on one of the existing IoT projects for the ESP32, such as [Tasmota](https://github.com/arendst/Tasmota), [ESPEasy](https://github.com/letscontrolit/ESPEasy), and [ESPHome](https://github.com/esphome/esphome). However, it quickly became clear that none of the projects were compatible with the W5500 Ethernet chip used in the NORVI ENET.
I also did some research to add Ethernet functionality using the W5500 chip at ESPHome \[[2](https://github.com/esphome/feature-requests/issues/1235#issuecomment-1169079495)\], but without support from the ESPHome community, the implementation was technically and temporally out of scope for me.
So, in the end, implementing the functionality myself without using an IoT platform was the best way to go.


## Important note for compiling

A change introduced to `Server.h` in the [Arduino Core for ESP32](https://github.com/espressif/arduino-esp32) breaks compatiblity to certain libraries, including the [eModbus](https://github.com/eModbus/eModbus) used here. The current, ugly, workaround is to change line 28 of `Server.h` from `virtual void begin(uint16_t port=0) =0;` to `virtual void begin() =0;` \[[3](https://github.com/arduino-libraries/Ethernet/issues/88#issuecomment-455498941)\]. 
On my system, `Server.h` is located at `~/.platformio/packages/framework-arduinoespressif32/cores/esp32/Server.h`.
 

## Configurations

Currently, only the wired Ethernet connection is used. Adapting `src/main.cpp` to use `WiFi.h` should be straight forward, though.

MAC address and IP configuration can be set in `src/config.h`. Using DHCP should also be feasible, but has not been tested.

The display shows a boot image when starting the ESP32. In `src/img/` is described how this can be replaced.


## Credits

This project was initially based on https://github.com/PuceBaboon/ESP32_W5500_NTP_CLIENT.


