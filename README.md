# Bluetooth MESH adaptor for Home Automatio server

ESP32 board used for connection [home_automation_server](https://github.com/vpq-is-me/home_auto_nodejs.git) (running on Raspberry Pi) to Bluetooth MESH. Raspberry Pi and ESP-wroom-32 connected to each other via TTL UART. For this aim Raspberry's GPIO are used. Power supply for ESP-wroom-32 also drain from RPi via USB. 

> [!TIP]
> In future it will be good to use Raspberry's Bluetooth itself.
> And it won't be required any external devices at all. 

> [!NOTE]
> For UART reliable connection it is used Serial Line IP (SLIP)