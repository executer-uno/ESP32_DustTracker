/*
 * Extensions.cpp
 *
 *  Created on: Mar 17, 2020
 *      Author: E_CAD
 */

#include "Extensions.h"
#include "Definitions.h"

template<typename T, std::size_t N> constexpr std::size_t array_num_elements(const T(&)[N]) {
	return N;
}

/*****************************************************************
 * Debug output																									*
 *****************************************************************/
void debug_out(const String& text, const int level, const bool linebreak) {
	if (level <= cfg::debug) {
		if (linebreak) {
			Serial.println(text);
		} else {
			Serial.print(text);
		}
	}
}


/*****************************************************************
 * read SDS011 sensor values																		 *
 *****************************************************************/
void sensorSDS() {
	String s = "";
	char buffer;
	int value;
	int len = 0;
	int pm10_serial = 0;
	int pm25_serial = 0;
	int checksum_is = 0;
	int checksum_ok = 0;

	//debug_out(String(FPSTR(DBG_TXT_START_READING)) + FPSTR(SENSORS_SDS011), DEBUG_MED_INFO, 1);

	if ((SDSmeas.status == raw) && (millis() > 10000ULL)) {
		SDS_cmd(PmSensorCmd::Start);
		SDSmeas.status = wait;
	}
	if (SDSmeas.status != raw){
		while (serialSDS.available() > 0) {
			buffer = serialSDS.read();
	//		debug_out(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .", DEBUG_MAX_INFO, 1);
	//			"aa" = 170, "ab" = 171, "c0" = 192
			value = int(buffer);
			switch (len) {
			case (0):
				if (value != 170) {
					len = -1;
				};
				break;
			case (1):
				if (value != 192) {
					len = -1;
				};
				break;
			case (2):
				pm25_serial = value;
				checksum_is = value;
				break;
			case (3):
				pm25_serial += (value << 8);
				break;
			case (4):
				pm10_serial = value;
				break;
			case (5):
				pm10_serial += (value << 8);
				break;
			case (8):
//				debug_out(FPSTR(DBG_TXT_CHECKSUM_IS), DEBUG_MED_INFO, 0);
//				debug_out(String(checksum_is % 256), DEBUG_MED_INFO, 0);
//				debug_out(FPSTR(DBG_TXT_CHECKSUM_SHOULD), DEBUG_MED_INFO, 0);
//				debug_out(String(value), DEBUG_MED_INFO, 1);
				if (value == (checksum_is % 256)) {
					checksum_ok = 1;
				} else {
					len = -1;
				};
				break;
			case (9):
				if (value != 171) {
					len = -1;
				};
				break;
			}
			if (len > 2) { checksum_is += value; }
			len++;

			// Telegram received
			if (len == 10 && checksum_ok == 1 ) {
				if ((! isnan(pm10_serial)) && (! isnan(pm25_serial))) {

					SDSmeas.pm010.sum += pm10_serial;
					SDSmeas.pm025.sum += pm25_serial;

					SDSmeas.pm010.max = (SDSmeas.pm010.max<pm10_serial ? pm10_serial : SDSmeas.pm010.max);
					SDSmeas.pm025.max = (SDSmeas.pm025.max<pm25_serial ? pm25_serial : SDSmeas.pm025.max);

					SDSmeas.pm010.min = (SDSmeas.pm010.min>pm10_serial ? pm10_serial : SDSmeas.pm010.min);
					SDSmeas.pm025.min = (SDSmeas.pm025.min>pm25_serial ? pm25_serial : SDSmeas.pm025.min);

					SDSmeas.count++;


//					debug_out(F("PM1.0 : [")								, DEBUG_MED_INFO, 0);
//					debug_out(Float2String(double(SDSmeas.pm010.min) / 10)	, DEBUG_MED_INFO, 0);
//					debug_out(F(" : "), DEBUG_MED_INFO, 0);
//					debug_out(Float2String(double(SDSmeas.pm010.sum) / (10* SDSmeas.count))	, DEBUG_MED_INFO, 0);
//					debug_out(F(" : ")	, DEBUG_MED_INFO, 0);
//					debug_out(Float2String(double(SDSmeas.pm010.max) / 10)	, DEBUG_MED_INFO, 0);
//					debug_out(F(" ]") , DEBUG_MED_INFO, 1);
//
//					debug_out(F("PM2.5 : [")								, DEBUG_MED_INFO, 0);
//					debug_out(Float2String(double(SDSmeas.pm025.min) / 10)	, DEBUG_MED_INFO, 0);
//					debug_out(F(" : "), DEBUG_MED_INFO, 0);
//					debug_out(Float2String(double(SDSmeas.pm025.sum) / (10* SDSmeas.count))	, DEBUG_MED_INFO, 0);
//					debug_out(F(" : ")	, DEBUG_MED_INFO, 0);
//					debug_out(Float2String(double(SDSmeas.pm025.max) / 10)	, DEBUG_MED_INFO, 0);
//					debug_out(F(" ]") , DEBUG_MED_INFO, 1);


				}
				len = 0;
				checksum_ok = 0;
				pm10_serial = 0.0;
				pm25_serial = 0.0;
				checksum_is = 0;
			}
			yield();
			//debug_out(F("yield() called from 2803"), DEBUG_MIN_INFO, 0);
			//debug_out("", DEBUG_MIN_INFO, 1);

		}
	}
}



/*****************************************************************
 * send SDS011 command (start, stop, continuous mode, version		*
 *****************************************************************/
void SDS_cmd(PmSensorCmd cmd) {//static
	static constexpr uint8_t start_cmd[] PROGMEM = {
		0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB
	};
	static constexpr uint8_t stop_cmd[] PROGMEM = {
		0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB
	};
	static constexpr uint8_t continuous_mode_cmd[] PROGMEM = {
		0xAA, 0xB4, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x07, 0xAB
	};
	static constexpr uint8_t version_cmd[] PROGMEM = {
		0xAA, 0xB4, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB
	};
	constexpr uint8_t cmd_len = array_num_elements(start_cmd);

	uint8_t buf[cmd_len];
	switch (cmd) {
	case PmSensorCmd::Start:
		memcpy_P(buf, start_cmd, cmd_len);
		break;
	case PmSensorCmd::Stop:
		memcpy_P(buf, stop_cmd, cmd_len);
		break;
	case PmSensorCmd::ContinuousMode:
		memcpy_P(buf, continuous_mode_cmd, cmd_len);
		break;
	case PmSensorCmd::VersionDate:
		memcpy_P(buf, version_cmd, cmd_len);
		break;
	}
	serialSDS.write(buf, cmd_len);
}

/*****************************************************************
 * send Plantower PMS sensor command start, stop, cont. mode		 *
 *****************************************************************/
void PMS_cmd(PmSensorCmd cmd) {//static
	static constexpr uint8_t start_cmd[] PROGMEM = {
		0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74
	};
	static constexpr uint8_t stop_cmd[] PROGMEM = {
		0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73
	};
	static constexpr uint8_t continuous_mode_cmd[] PROGMEM = {
		0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71
	};
	constexpr uint8_t cmd_len = array_num_elements(start_cmd);

	uint8_t buf[cmd_len];
	switch (cmd) {
	case PmSensorCmd::Start:
		memcpy_P(buf, start_cmd, cmd_len);
		break;
	case PmSensorCmd::Stop:
		memcpy_P(buf, stop_cmd, cmd_len);
		break;
	case PmSensorCmd::ContinuousMode:
		memcpy_P(buf, continuous_mode_cmd, cmd_len);
		break;
	case PmSensorCmd::VersionDate:
		assert(false && "not supported by this sensor");
		break;
	}
	serialPMS.write(buf, cmd_len);
}


/*****************************************************************
 * convert float to string with a																*
 * precision of two (or a given number of) decimal places				*
 *****************************************************************/
String Float2String(const double value) {
	return Float2String(value, 2);
}

String Float2String(const double value, uint8_t digits) {
	// Convert a float to String with two decimals.
	char temp[15];

	dtostrf(value, 13, digits, temp);
	String s = temp;
	s.trim();
	return s;
}
