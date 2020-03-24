/*
 * Sensors.h
 *
 *  Created on: Mar 24, 2020
 *      Author: E_CAD
 */

#ifndef SENSORS_H_
#define SENSORS_H_

	void sensorSDS();
	void sensorPMS();


	enum class PmSensorCmd {
		Start,
		Stop,
		ContinuousMode,
		VersionDate
	};

#endif /* SENSORS_H_ */
