# NORVI ENET AE-06-T Modbus TCP Server

This project implements a Modbus TCP server for the NORVI ENET AE-06-T, which is a *USB Programmable Industrial ESP32 with OLED Display and din-rail mount* \[[1](https://norvi.lk/product/industrial-esp32-ethernet/)\].

Features:
* 4 outputs as Modbus holding registers (Register 0 to 3, read/write)
* 8 inputs as Modbus input registers (Register 0 to 7, read-only)
* 3 device buttons readable via Modbus input registers (Register 8 to 10, read-only)
* Ethernet connectivity via integrated W5500 chipset
* Basic information shown on the OLED display (logs, IP, link status)

The idea behind this project is to use the NORVI ENET as a "dumb" translator of the IOs into the network and implement the control logic in easier to maintain controllers, like [IOBroker](https://github.com/ioBroker/ioBroker) or [Node-RED](https://github.com/node-red/node-red).

## Configurations

Currently, only the wired ethernet connection is used. Adapting `src/main.cpp` to use `WiFi.h` should be straight forward, though.

MAC address and IP configuration can be set in `src/config.h`. Using DHCP should also be feasible, but has not been tested.

## Credits

This project was based on https://github.com/PuceBaboon/ESP32_W5500_NTP_CLIENT.


