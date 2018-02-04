# Pool Monitoring & Control - ESP32 Application

Requires ESP IDF v3.0rc1.


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

