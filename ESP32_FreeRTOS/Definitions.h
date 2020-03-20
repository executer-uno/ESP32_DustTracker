/*
 * Definitions.h
 *
 *  Created on: Mar 17, 2020
 *      Author: E_CAD
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

	// Build in LED
	#define LED_BUILTIN 2

	// Definition der Debuglevel
	#define DEBUG_ERROR 	1
	#define DEBUG_WARNING 	2
	#define DEBUG_MIN_INFO 	3
	#define DEBUG_MED_INFO 	4
	#define DEBUG_MAX_INFO 	5
	// Wieviele Informationen sollenber die serielle Schnittstelle ausgegeben werden?
	#define DEBUG DEBUG_MED_INFO

	// BasicAuth config
	#define WWW_USERNAME "admin"
	#define WWW_PASSWORD "feinstaub"
	#define WWW_BASICAUTH_ENABLED 0

	// Sensor Wifi config (config mode)
	#define FS_SSID "sensor_cfg"
	#define FS_PWD  ""



	#define PM_SERIAL_RX    35	// SDS
	#define PM_SERIAL_TX    32	// SDS

	#define PM2_SERIAL_RX   25  // PMS
	#define PM2_SERIAL_TX   26	// PMS

	#define GPS_SERIAL_RX   16	// GPS
	#define GPS_SERIAL_TX   17	// GPS

	#define DEB_RX    		3	// Debug UART pins
	#define DEB_TX    		1

	#define BUT_1 			34	// no pullup!
	#define BUT_2 			33
	#define BUT_3 			27

	enum class SensorSt {
		raw,
		wait,
		ok,
		hold,
		timeout
	};

	enum class PmSensorCmd {
		Start,
		Stop,
		ContinuousMode,
		VersionDate
	};

	struct measure {
		int 	max;
		int 	min;
		int32_t sum;
	};

	struct PMmeas {
		SensorSt		status;		// Sensor initialized and started
		int32_t 		count; 			// number of measurements in accumulator
		struct measure 	pm010;
		struct measure 	pm025;
		struct measure 	pm100;
	};


#endif /* DEFINITIONS_H_ */
