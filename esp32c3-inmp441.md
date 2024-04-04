## Get Started for INMP441 to ESP32C3 (tested with ESP32-C3 super mini)
- Set the device_type to 9 in the settings.ini
- Configure pins on /devices/ESP32C3-Inmp441.hpp
- Connect the ESP32C3 super mini to your computer via USB, like a regular esp32
- First flash, set "upload" as method under the OTA section in the settings.ini. After that you can set it to "ota" as well 
- Build and flash the project with PlatformIO, for example run `pio run --target upload` from the commandline
