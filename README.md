# ESP32_DustTracker
ESP32 with FreeRTOS, SQLITE, SoftwareSerial, HTTPS Redirect and TinyGPS++ (SDS011 and PMS3003 double sensors)

Schematics:
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
