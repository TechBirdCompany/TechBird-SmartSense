# Introduction
TechBird-SmartSense is an IoT device with a 4-channel 12-Bit ADC, to measure up to 4 temperatures. In addition it also provides an internal temperature sensore for ambiant temperatures.
<img src="Pictures/SmartSense_ProductPic.png" alt="Bild" height="300; width: auto;">

# Features
- ESP32-C6-MINI-1-N4: As SoC
- EEMB LP103454: 2000 mAh Lipo Battery
  - 7 days of uptime with 10 seconds time gab, and ringbuffer size of 60
- ADS1015IDGSR: 2 Differntiall or 4 Channel 12-Bit ADC
- FM24CL64B-GTR: 64 kB FRAM
- RV-3028-C7: Internal RTC
- TMP1075D: Internal Temperature Sensor
- USB Type-C for communication and charging
<img src="Pictures/SmartSense_002.JPG" alt="Bild" height="300; width: auto;">

# Software
- Use at your own risk!
- Basic UI via USB (115200, 8, n, 1)
- Sending Data via MQTT
- Again... use at your own risk!
- Bugs are free to find, and free to fix.

# Known Bugs
## Hardware
- ~{WKUP} need to be routed to IO2
<img src="Pictures/flywire_001.jpg" alt="Bild" height="300; width: auto;">

## Software
- When ringbuffer loops, one set of wrong data will be send

# Known Nuisances
- When in deep sleep, attaching USB connection won't reset the device and the lid need to be removed
- Main connector is not made to be removed frequently, which makes charging pretty annoying
