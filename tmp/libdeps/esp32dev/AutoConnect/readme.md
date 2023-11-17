# temp patch to AutoConnect library

esp32: frozen loop() and kicked out of the WiFi connection at Configure new AP #615

https://github.com/Hieromon/AutoConnect/issues/615


change WiFi.scanNetworks(x, y) to WiFi.scanNetworks(x, y, true);


