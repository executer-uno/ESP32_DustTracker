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

	/*****************************************************************
	 * Debug output																									*
	 *****************************************************************/
	void debug_out(const String& text, const bool linebreak);

	String sensorSDS();
	String sensorPMS();
	static bool SDS_cmd(PmSensorCmd cmd);
	static bool PMS_cmd(PmSensorCmd cmd);

	String Float2String(const double value);
	String Float2String(const double value, uint8_t digits);



#endif /* LIBRARIES_EXTENSIONS_H_ */
