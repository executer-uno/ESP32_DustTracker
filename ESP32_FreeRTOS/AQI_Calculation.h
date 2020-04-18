/*
 * AQI_Calculation.h
 *
 *  Created on: Apr 17, 2020
 *      Author: https://diyprojects.io/calculate-air-quality-index-iaq-iqa-dsm501-arduino-esp8266
 */

#ifndef AQI_CALCULATION_H_
#define AQI_CALCULATION_H_


// Resources:
// https://forum.airnowtech.org/t/the-aqi-equation/169
// https://diyprojects.io/calculate-air-quality-index-iaq-iqa-dsm501-arduino-esp8266

int getATMO( int sensor, float density );
int getACQI( int sensor, float density );
int getAQI(bool isPM10, float density);
String updateAQIDisplay(int AQI);


#endif /* AQI_CALCULATION_H_ */
