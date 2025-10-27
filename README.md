# SmartTherm

Version 0.8.4.5

Open source for [SmartTherm](https://www.umkikit.ru/index.php?route=product/product&path=67&product_id=103) ESP8266/ESP32 OpenTherm controller

Use:
* code of [OpenTherm Library by ihormelnyk](https://github.com/ihormelnyk/opentherm_library)
* [AutoConnect Library by Hieromon](https://github.com/Hieromon/AutoConnect)
* [DS18B20 Library by robtillaart](https://github.com/RobTillaart/DS18B20_RT)

Build with [PlatformIO](https://platformio.org/)

Features:
* [Captive portal](https://en.wikipedia.org/wiki/Captive_portal) before WiFi connection
* Web interface after WiFi connection
* [OpenTherm](https://en.wikipedia.org/wiki/OpenTherm) interface for Gas/Electric boiler contol (HVAC)
* [Personal cloud control](https://github.com/Evgen2/SmartServer) used
* [Android application for local/remote control](https://github.com/Evgen2/SmartThermClient) (betatest)
* TCP/UDP API interface
* up to 2 DS18B20 temperature sensors

0.8.4.5
* delayed write config to flash after MQTT change mode or target temperature

0.8.4.4
* Add watchdog, changes in libraries arduino-home-assistant and pubsubclient

0.8.4.3
* bugfix

0.8.4.2
* bugfix in [AutoConnect](https://github.com/Evgen2/AutoConnect)

0.8.4.1
* bugfix

0.8.4
* Add remote OT log
* Close AP after conection to WiFi router after timeout

0.8.3
* Add support for slave OpenTherm interface
* Change MQTT server string up to 80 characters
* MQTT settings read/write to separate config file
* Add link to controller web page from HA MQTT device card
* Add effective modulation for the previous hour MQTT sensor

0.8.2
* Add build variant with onboard relay
* Add support for OT:MaxRelModLevelSetting
* Add support for OT:RemoteRequest (BLOR)

0.8.1
* At PID startup and room setpoint change recalculate the integral part of PID
  to speed up reaching the target setpoint. I.e start and restart PID with non zero integral

0.8.0
* TCP API changes for Andriod application & remote server support
* PID changes
* add use ID29 (Tstorage) as Indirect Water Heaters temperature for Buderus
* Immergas fix

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
Copyright (c) 2022-2024 Evgen2. Licensed under the [MIT license](/LICENSE?raw=true).