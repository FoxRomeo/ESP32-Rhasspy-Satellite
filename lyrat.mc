## Get started for LyraT
- Set the device_type to 10 in the settings.ini
- Configure pins on /devices/LyraT.hpp
- Connect the LyraT to your computer via USB, like a regular esp32
- First flash, set "upload" as method under the OTA section in the settings.ini. After that you can set it to "ota" as well 
- Build and flash the project with PlatformIO, for example run `pio run --target upload` from the commandline

# Known Issues
there are multiple versions of the new ESP32-A1S in the wild
with different pinouts for I2S and I2C and different audio controllers
see https://github.com/Ai-Thinker-Open/ESP32-A1S-AudioKit/issues/26
