# ESP32 DustTracker
*Particle matter double sensors and air phisical parameters tracker with GPS coordinates, with small display and SQLite data base buffer on SD card and WiFi data upload to Google Spreadsheets and OpenSenseMap services. And its done Sloeber IDE, proper IDE to work with Arduino-based projects ;-)*

Software highlights:
---
* ESP32 with FreeRTOS threads

Basic opendata-stuttgart project was done on ESP8266 chip, with no OS usage. Main thing to move to ESP32 was 3 hardware UART available, more IO pins for SD card support and enough RAM for SQLite and trouble free engineering.
* SQLite on SD card

To make it portable some data buffer required. For easy data store and acess and modification SQLite library was used. Works like a charm. 
* SoftwareSerial for debug output

All 3 available hardware UARTs used for sensors communication. Some time Bluetooth was used as a fourth debug USART, but some moment it appears that it wery hungry for RAM in Arduino libraryes, so switched back to SoftwareSerial for debug port. It brokes characters time to time, but its better than nothing.
ESP32 SoftwareSerial was little bit reconfigured to be able to use build-in USB-UART adapter pins (https://github.com/plerup/espsoftwareserial/pull/134#issue-364533170)

* HTTPS Redirect library

Google spreadsheets used that protocol to upload data. Most complicate library as for me)
* TinyGPS++ for NEO6M sensor

All GPS processing was provided by opendata-stuttgart basic project
* SDS011 and PMS3003 double particle matter sensors

General information page https://aqicn.org/sensor/. That type of laser sensors not really measures different grades of PM. Up to now in my measurements there are very straight corelation between PM2.5 and PM10.0 measurements, so it measures amount of dust in air, but not measures its particles size.
* BME280 sensor

Provides Humidity, Temperature and precise pressure for logging
* Sloeber IDE

Perfect tool I was looking for. I was trying to modify opendata-stuttgart project in Arduino IDE, but it is completly useless. Wery glad to find Sloeber, all libraryes available and all testy things of real IDE. No problems with Arduino tutorials, all functions are available.

Schematics:
------------------------
https://easyeda.com/executer/ESP32_PMS_GPS_Portable


Referenced libraryes and projects
---

Base project:
https://github.com/opendata-stuttgart/sensors-software/tree/master/airrohr-firmware

Its codebase was most used but, changed a lot, because that project is a portable, mobile station.
Original stationary station from opendata-stuttgart works well out of my window for a year, very robust!

https://github.com/noerw/mobile-sensebox

AQI calculation:
https://www3.epa.gov/airnow/ani/pm25_aqi_reporting_nowcast_overview.pdf?fbclid=IwAR3x9rvidAmwUB2_rBXvZB6xviRX2Vrxxi6mbT1fyVTiGkt4w3ONSEcnVzw

Low cost sensors studying articles:
https://www.researchgate.net/publication/333891464_PM25_low-cost_sensors_and_calibration_data_for_SDS011_and_PMS7003
https://aqicn.org/sensor/pms5003-7003/ru/ (shows particle sizes not really measured by PMS sensors)


Special thanks to @savednipro organization for information support.

OpenSenseMap (OSM) service
---------------------------------
One of the two used data upload services. It is a public one, so, for privacy sensor record mode implemented to choose to record position or to not. OSM data uploads only for records with GPS coordinates available.
Service registration tutorial:
https://sensebox.github.io/books-v2/home/en/erste-schritte/registrierung-auf-der-openSenseMap.html
1) For new box choose "senseBox:edu" as a template and drop number of sensors you need as a templates 
2) Edit phenomens and sensors descriptions to match with your real data
3) Its ready, it should be a Arduino project sent to your e-mail, I use it as a template and modify for batch data upload.
Very helpful OpenStreetMap API reference for batch data upload. One-by-one measurement overloads connection from server side:
https://docs.opensensemap.org/#api-Measurements-postNewMeasurements

Google Sheets data upload
---------------------------
It is a second, private, data upload destination. Many thanks for https://github.com/electronicsguy/ESP8266/tree/master/HTTPSRedirect project, with example project and detailed description it was easy to upload data to private SpreadSheet without any 3rd party services. Peace of cake!


Installation references
==

Official [manual](http://eclipse.baeyens.it/stable.php?OS=Windows) I used. Easy Bundle setup for clear system.

In Project settings need to extra define following key, to disable built-in Serial object
![Serial disabled](https://github.com/executer-uno/ESP32_DustTracker/blob/master/SW_Serial%20to%20USB.png)

Sloeber usually detects all libs by itself, but here is a my working list:
![libs](https://github.com/executer-uno/ESP32_DustTracker/blob/master/Libraryes.png). Double check that FreeRTOS is unselected. It is already in ESP32 module and library.

Just in case, mu JSON references, just standard ESP references added
![JSON](https://github.com/executer-uno/ESP32_DustTracker/blob/master/JSON%20references.png)

My board's configuration
![boardCFG](https://github.com/executer-uno/ESP32_DustTracker/blob/master/BoardConfig.png)
