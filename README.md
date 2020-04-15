# ESP32 DustTracker
*Particle matter double sensors and air phisical parameters tracker with GPS coordinates, with small display and SQLite data base buffer on SD card and WiFi data upload to Google Spreadsheets and OpenSenseMap services*
Software highlights:
---
* ESP32 with FreeRTOS threads

Basic opendata-stuttgart project was done on ESP8266 chip, with no OS usage. Main thing to move to ESP32 was 3 hardware UART available, more IO pins for SD card support and enough RAM for SQLite and trouble free engineering.
* SQLite on DS card

To make it portable some data buffer required. For easy data store and acess and modification SQLite library was used. Works like a charm. 
* SoftwareSerial for debug output

All 3 available hardware UARTs used for sensors communication. Some time Bluetooth was used as a fourth debug USART, but some moment it appears that it wery hungry for RAM in Arduino libraryes, so switched back to SoftwareSerial for debug port. It brokes characters time to time, but its better than nothing. 
* HTTPS Redirect library

Google spreadsheets used that protocol to upload data
* TinyGPS++ for NEO6M sensor

All GPS processing was provided by opendata-stuttgart basic project
* SDS011 and PMS3003 double particle matter sensors

General information page https://aqicn.org/sensor/. That type of laser sensors not really measures different grades of PM. Up to now in my measurements there are very straight corelation between PM2.5 and PM10.0 measurements, so it measures amount of dust in air, but not measures its particles size.
* BME280 sensor

Provides Humidity, Temperature and precise pressure for logging

Schematics:
------------------------
https://easyeda.com/executer/ESP32_PMS_GPS_Portable

Base project:
https://github.com/opendata-stuttgart/sensors-software/tree/master/airrohr-firmware

Its codebase was most used but, changed a lot, becouse mine it is a portable, mobile station.
Original stationary station from opendata-stuttgart works well out my window for a year, very robust!

Referenced libraryes and projects
https://github.com/noerw/mobile-sensebox


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
