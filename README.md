# SmartTherm
Version 0.5

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
* UDP interface
* up to 2 DS18B20 temperature sensors


Todo:
* MQTT interface for [Home Assistant](https://www.home-assistant.io/)

## License
Copyright (c) 2022-2023 Evgen2. Licensed under the [MIT license](/LICENSE?raw=true).