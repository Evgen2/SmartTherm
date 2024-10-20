# SmartTherm

Version 0.7.5

Open source for [SmartTherm](https://www.umkikit.ru/index.php?route=product/product&path=67&product_id=103) ESP8266/ESP32 OpenTherm controller

Use:
* code of [OpenTherm Library by ihormelnyk](https://github.com/ihormelnyk/opentherm_library)
* [AutoConnect Library by Hieromon](https://github.com/Hieromon/AutoConnect)
* [DS18B20 Library by robtillaart](https://github.com/RobTillaart/DS18B20_RT)

Build with [PlatformIO](https://platformio.org/)

Features:
* [Captive portal](https://en.wikipedia.org/wiki/Captive_portal) before WiFi connection
* Web interface after WiFi connectiom
* [OpenTherm](https://en.wikipedia.org/wiki/OpenTherm) interface for Gas/Electric boiler contol (HVAC)
* no external cloud control used
* TCP/UDP API interface
* up to 2 DS18B20 temperature sensors

0.7.5
* add  WinterMode (ID0:HB5) and  Use_OTC (ID0:HB3) support
* speedup OT startup ~2 sec
* add binary CH and HW sensors to MQTT
* MQTT connect after detecting boiler Capabilities if OT work
* MQTT connect to server without reset at MQTT config changes

0.7.4 changes
* PID + weather-compensated automation (standalone + HA)

0.7.3 changes
* fixed autoreconnect to WiFi

0.7.1 changes
* Add MQTT and MQTT discovery for home assistant

v 0.6 changes
* TCP/UDP interface, Windows/Linux application [SmartServer](https://github.com/Evgen2/SmartServer) for TCP/UDP API
* config saved and read after reboot
* Hot water and CH2 enabled
* Increased free RAM 


## License
Copyright (c) 2022-2023 Evgen2. Licensed under the [MIT license](/LICENSE?raw=true).