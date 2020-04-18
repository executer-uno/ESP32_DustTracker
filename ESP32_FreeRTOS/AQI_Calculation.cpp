/*
 * AQI_Calculation.cpp
 *
 *  Created on: Apr 17, 2020
 *      Author: https://diyprojects.io/calculate-air-quality-index-iaq-iqa-dsm501-arduino-esp8266
 */


#include <Arduino.h>

#define COUNTRY                       2         // 0. France, 1. Europe, 2. USA/China
#define EXCELLENT                     "Excellent"
#define GOOD                          "Good"
#define ACCEPTABLE                    "Moderate"
#define MODERATE                      "Unhealthy for Sensitive"
#define HEAVY                         "Unhealthy"
#define SEVERE                        "Very Unhealthy"
#define HAZARDOUS                     "Hazardous"

/*
 * Calcul l'indice de qualité de l'air français ATMO
 * Calculate French ATMO AQI indicator
 */
int getATMO( int sensor, float density ){
  if ( sensor == 0 ) { //PM2,5
    if ( density <= 11 ) {
      return 1;
    } else if ( density > 11 && density <= 24 ) {
      return 2;
    } else if ( density > 24 && density <= 36 ) {
      return 3;
    } else if ( density > 36 && density <= 41 ) {
      return 4;
    } else if ( density > 41 && density <= 47 ) {
      return 5;
    } else if ( density > 47 && density <= 53 ) {
      return 6;
    } else if ( density > 53 && density <= 58 ) {
      return 7;
    } else if ( density > 58 && density <= 64 ) {
      return 8;
    } else if ( density > 64 && density <= 69 ) {
      return 9;
    } else {
      return 10;
    }
  } else {
    if ( density <= 6 ) {
      return 1;
    } else if ( density > 6 && density <= 13 ) {
      return 2;
    } else if ( density > 13 && density <= 20 ) {
      return 3;
    } else if ( density > 20 && density <= 27 ) {
      return 4;
    } else if ( density > 27 && density <= 34 ) {
      return 5;
    } else if ( density > 34 && density <= 41 ) {
      return 6;
    } else if ( density > 41 && density <= 49 ) {
      return 7;
    } else if ( density > 49 && density <= 64 ) {
      return 8;
    } else if ( density > 64 && density <= 79 ) {
      return 9;
    } else {
      return 10;
    }
  }
}

/*
 * CAQI Européen - European CAQI level
 * source : http://www.airqualitynow.eu/about_indices_definition.php
 */

int getACQI( int sensor, float density ){
  if ( sensor == 0 ) {  //PM2,5
    if ( density == 0 ) {
      return 0;
    } else if ( density <= 15 ) {
      return 25 ;
    } else if ( density > 15 && density <= 30 ) {
      return 50;
    } else if ( density > 30 && density <= 55 ) {
      return 75;
    } else if ( density > 55 && density <= 110 ) {
      return 100;
    } else {
      return 150;
    }
  } else {              //PM10
    if ( density == 0 ) {
      return 0;
    } else if ( density <= 25 ) {
      return 25 ;
    } else if ( density > 25 && density <= 50 ) {
      return 50;
    } else if ( density > 50 && density <= 90 ) {
      return 75;
    } else if ( density > 90 && density <= 180 ) {
      return 100;
    } else {
      return 150;
    }
  }
}

/*
 * AQI formula: https://en.wikipedia.org/wiki/Air_Quality_Index#United_States
 * Arduino code https://gist.github.com/nfjinjing/8d63012c18feea3ed04e
 * On line AQI calculator https://www.airnow.gov/index.cfm?action=resources.conc_aqi_calc
 */
static float calcAQI(float I_high, float I_low, float C_high, float C_low, float C) {
  return (I_high - I_low) * (C - C_low) / (C_high - C_low) + I_low;
}

int getAQI(bool isPM10, float density) { //PM10 = True for PM10.0 AQI
  int d10 = (int)(density * 10);
  if (!isPM10) {
    if (d10 <= 0) {
      return 0;
    }
    else if(d10 <= 120) {
      return calcAQI(50, 0, 120, 0, d10);
    }
    else if (d10 <= 354) {
      return calcAQI(100, 51, 354, 121, d10);
    }
    else if (d10 <= 554) {
      return calcAQI(150, 101, 554, 355, d10);
    }
    else if (d10 <= 1504) {
      return calcAQI(200, 151, 1504, 555, d10);
    }
    else if (d10 <= 2504) {
      return calcAQI(300, 201, 2504, 1505, d10);
    }
    else if (d10 <= 3504) {
      return calcAQI(400, 301, 3504, 2505, d10);
    }
    else if (d10 <= 5004) {
      return calcAQI(500, 401, 5004, 3505, d10);
    }
    else if (d10 <= 10000) {
      return calcAQI(1000, 501, 10000, 5005, d10);
    }
    else {
      return 1001;
    }
  } else {
    if (d10 <= 0) {
      return 0;
    }
    else if(d10 <= 540) {
      return calcAQI(50, 0, 540, 0, d10);
    }
    else if (d10 <= 1540) {
      return calcAQI(100, 51, 1540, 541, d10);
    }
    else if (d10 <= 2540) {
      return calcAQI(150, 101, 2540, 1541, d10);
    }
    else if (d10 <= 3550) {
      return calcAQI(200, 151, 3550, 2541, d10);
    }
    else if (d10 <= 4250) {
      return calcAQI(300, 201, 4250, 3551, d10);
    }
    else if (d10 <= 5050) {
      return calcAQI(400, 301, 5050, 4251, d10);
    }
    else if (d10 <= 6050) {
      return calcAQI(500, 401, 6050, 5051, d10);
    }
    else {
      return 1001;
    }
  }
}

String updateAQIDisplay(int AQI){
  /*
   * 1 EXCELLENT
   * 2 GOOD
   * 3 ACCEPTABLE
   * 4 MODERATE
   * 5 HEAVY
   * 6 SEVERE
   * 7 HAZARDOUS
   */
  if ( COUNTRY == 0 ) {
    // Système ATMO français - French ATMO AQI system
    switch ( AQI) {
      case 10:
        return SEVERE;
        break;
      case 9:
        return HEAVY;
        break;
      case 8:
        return HEAVY;
        break;
      case 7:
        return MODERATE;
        break;
      case 6:
        return MODERATE;
        break;
      case 5:
        return ACCEPTABLE;
        break;
      case 4:
        return GOOD;
        break;
      case 3:
        return GOOD;
        break;
      case 2:
        return EXCELLENT;
        break;
      case 1:
        return EXCELLENT;
        break;
      }
  } else if ( COUNTRY == 1 ) {
    // European CAQI
    switch ( AQI) {
      case 25:
        return GOOD;
        break;
      case 50:
        return ACCEPTABLE;
        break;
      case 75:
        return MODERATE;
        break;
      case 100:
        return HEAVY;
        break;
      default:
        return SEVERE;
      }
  } else if ( COUNTRY == 2 ) {
    // USA / CN
    if ( AQI <= 50 ) {
        return GOOD;
    } else if ( AQI > 50 && AQI <= 100 ) {
        return ACCEPTABLE;
    } else if ( AQI > 100 && AQI <= 150 ) {
        return MODERATE;
    } else if ( AQI > 150 && AQI <= 200 ) {
        return HEAVY;
    } else if ( AQI > 200 && AQI <= 300 ) {
        return SEVERE;
    } else {
       return HAZARDOUS;
    }
  }
}
