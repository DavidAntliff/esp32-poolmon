# Pool Monitoring & Control - ESP32 Application

Requires at least ESP IDF v3.0rc1.

Raspberry Pi side of the project: [DavidAntliff/poolmon](https://github.com/DavidAntliff/poolmon)

[system block diagram, software architecture diagram and ESP32 schematic](https://easyeda.com/ef784f36/Test_Project-2fcb46b5097649a4b018bc6c12a2d6d0)

Veroboard [design](https://www.draw.io/?lightbox=1&highlight=0000ff&edit=_blank&layers=1&nav=1&title=Veroboard.xml#Uhttps%3A%2F%2Fraw.githubusercontent.com%2FDavidAntliff%2Fpoolmon%2Fmaster%2Fsupport%2Fboard%2FVeroboard.xml)

## Notes

### One Wire Bus

The DS18B20 needs extra current for EEPROM writes and Temperature measurements.

https://www.maximintegrated.com/en/app-notes/index.mvp/id/4255

Vpup is the pullup voltage.

Vpupmin is the minimum pull up voltage and is usually 2.8V (not sure how this can be calculated from datasheet).

For a given Vpup and Rpup, the difference between Vpup and Vpupmin determines the current available for special functions:

    Iavail = (Vpup - Vpip_min) / Rpup

For the DS18B20 at 3.3V:

    Vpup = 3.3V
    Rpup = 4.7kΩ
    Vpupmin = 2.8V
    => Iavail = 0.106mA
    
This is probably too low for a single device.

TODO: calculate a better Rpup resistor value.



## MQTT

Subscribe to all messages:

```
$ mosquitto_sub -h rpi -v -t \#
```

Send a message (e.g. reset ESP32):

```
$ mosquitto_pub -h rpi -t poolmon/esp32/reset -m 1
```

Update OTA (use the .bin file):

Locally:
```
$ python -m http.server --directory build/
```

```
$ mosquitto_pub -h 192.168.1.67 -t poolmon/ota/url -m "http://192.168.1.72:8000/esp32-poolmon.bin"
$ mosquitto_pub -h rpi -t poolmon/esp32/reset -m 1
```

NOTE: Things to check before doing OTA!

* The WiFi credentials in the new .bin match the current ones,
* The partition table is set to Factory + 2 x OTA (not the default!)
* The FLASH size is set to 4 MB

