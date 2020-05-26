/*
 * Sensors.h
 *
 *  Created on: Mar 24, 2020
 *      Author: E_CAD
 */

#ifndef SENSORS_H_
#define SENSORS_H_


#include "Definitions.h"


	enum class PmSensorCmd {
		Start,
		Stop,
		ContinuousMode,
		VersionDate
	};

	void sensorSDS(Stream *UART_PMS, PMmeas *measPM025, PMmeas *measPM100);
	void sensorPMS(Stream *UART_PMS, PMmeas *measPM010, PMmeas *measPM025, PMmeas *measPM100);

	void SDS_cmd(Stream *UART_SDS, PmSensorCmd cmd);
	void PMS_cmd(Stream *UART_PMS, PmSensorCmd cmd);


#endif /* SENSORS_H_ */
