# MCU Code

## Pins (GPIOs) Setting
The pin settings are listed in `Proj_Setup.h`.

## WiFi Configuration
Change the following two lines in `esp32s3_control_new.ino` to let the MCU connect to different WiFi.
```cpp
const char * ssid = "your_actual_wifi_name"; 
const char * pass = "your_actual_wifi_password"; // if there is no wifi password, make it NULL
```
For retriving the MAC address of the MCU, use `WiFi.macAddress()`.

