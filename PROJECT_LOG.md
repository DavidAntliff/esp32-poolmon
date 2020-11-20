# Project Log

## 2020-11-21

### Raspberry Pi Upgrade

Installed RPi4 4GB.

Observed error:

```
mosquitto_1   | 1605904158: Loading config file /mqtt/config/conf.d/websockets.conf
mosquitto_1   | 1605904158: Error: Websockets support not available.
mosquitto_1   | 1605904158: Error found at /mqtt/config/conf.d/websockets.conf:2.
mosquitto_1   | 1605904158: Error found at /mqtt/config/mosquitto.conf:18.
```

Had to disable mosquitto websockets by commenting out the following in srv/mosquitto/config/conf.d/websockets.conf

Root cause not yet identified.

### v0.98

Built and installed v0.98, which consists of submodule updates to master, and built with ESP-IDF 3.3.

### v0.99

Address issue #3 [Inhibit Circulation Pump operation when purge pump is operating](https://github.com/DavidAntliff/esp32-poolmon/issues/3).

Built with ESP-IDF v3.3 and installed via OTA.

### v1.0

Address issue #4 [Prevent potential freezing in winter by running circulation pump when array temperature is below threshold](https://github.com/DavidAntliff/esp32-poolmon/issues/4).

