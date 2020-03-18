/*
 * Extensions.h
 *
 *  Created on: Mar 17, 2020
 *      Author: E_CAD
 */

#ifndef LIBRARIES_EXTENSIONS_H_
#define LIBRARIES_EXTENSIONS_H_

	#include "Definitions.h"
	#include "SoftwareSerial.h"

	extern SoftwareSerial Serial;
	extern HardwareSerial serialSDS(0);
	extern HardwareSerial serialPMS(1);
	extern HardwareSerial serialGPS(2);

	namespace cfg {
		extern int	debug;
	}

	/*****************************************************************
	 * Debug output																									*
	 *****************************************************************/
	void debug_out(const String& text, const bool linebreak);

	String sensorSDS();
	String sensorPMS();
	static bool SDS_cmd(PmSensorCmd cmd);
	static bool PMS_cmd(PmSensorCmd cmd);

#endif /* LIBRARIES_EXTENSIONS_H_ */
