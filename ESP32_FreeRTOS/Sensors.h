/*
 * Sensors.h
 *
 *  Created on: Mar 24, 2020
 *      Author: E_CAD
 */

#ifndef SENSORS_H_
#define SENSORS_H_

	enum class PmSensorCmd {
		Start,
		Stop,
		ContinuousMode,
		VersionDate
	};

	void sensorSDS();
	void sensorPMS();

	void SDS_cmd(PmSensorCmd cmd);
	void PMS_cmd(PmSensorCmd cmd);


#endif /* SENSORS_H_ */
