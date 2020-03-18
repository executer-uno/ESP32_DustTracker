/*
 * Extensions.cpp
 *
 *  Created on: Mar 17, 2020
 *      Author: E_CAD
 */

#include "Extensions.h"

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

	debug_out(String(FPSTR(DBG_TXT_START_READING)) + FPSTR(SENSORS_SDS011), DEBUG_MED_INFO, 1);


		while (serialSDS.available() > 0) {
			buffer = serialSDS.read();
			debug_out(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .", DEBUG_MAX_INFO, 1);
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
				debug_out(FPSTR(DBG_TXT_CHECKSUM_IS), DEBUG_MED_INFO, 0);
				debug_out(String(checksum_is % 256), DEBUG_MED_INFO, 0);
				debug_out(FPSTR(DBG_TXT_CHECKSUM_SHOULD), DEBUG_MED_INFO, 0);
				debug_out(String(value), DEBUG_MED_INFO, 1);
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
			if (len == 10 && checksum_ok == 1 && (msSince(starttime) > (cfg::sending_intervall_ms - READINGTIME_SDS_MS))) {
				if ((! isnan(pm10_serial)) && (! isnan(pm25_serial))) {
					sds_pm10_sum += pm10_serial;
					sds_pm25_sum += pm25_serial;
					if (sds_pm10_min > pm10_serial) {
						sds_pm10_min = pm10_serial;
					}
					if (sds_pm10_max < pm10_serial) {
						sds_pm10_max = pm10_serial;
					}
					if (sds_pm25_min > pm25_serial) {
						sds_pm25_min = pm25_serial;
					}
					if (sds_pm25_max < pm25_serial) {
						sds_pm25_max = pm25_serial;
					}
					debug_out(F("PM10 (sec.) : "), DEBUG_MED_INFO, 0);
					debug_out(Float2String(double(pm10_serial) / 10), DEBUG_MED_INFO, 1);
					debug_out(F("PM2.5 (sec.): "), DEBUG_MED_INFO, 0);
					debug_out(Float2String(double(pm25_serial) / 10), DEBUG_MED_INFO, 1);
					sds_val_count++;
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
 * read Plantronic PM sensor sensor values											 *
 *****************************************************************/
String sensorPMS() {
	String s = "";
	char buffer;
	int value;
	int len = 0;
	int pm1_serial = 0;
	int pm10_serial = 0;
	int pm25_serial = 0;
	int checksum_is = 0;
	int checksum_should = 0;
	int checksum_ok = 0;
	int frame_len = 24;				// min. frame length

	debug_out(String(FPSTR(DBG_TXT_START_READING)) + FPSTR(SENSORS_PMSx003), DEBUG_MED_INFO, 1);
	if (msSince(starttime) < (cfg::sending_intervall_ms - (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS))) {
		if (is_PMS_running) {
			is_PMS_running = PMS_cmd(PmSensorCmd::Stop);
		}
	} else {
		if (! is_PMS_running) {
			is_PMS_running = PMS_cmd(PmSensorCmd::Start);
		}

		while (serialPMS.available() > 0) {
			buffer = serialPMS.read();
			debug_out(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .", DEBUG_MAX_INFO, 1);
//			"aa" = 170, "ab" = 171, "c0" = 192
			value = int(buffer);
			switch (len) {
			case (0):
				if (value != 66) {
					len = -1;
				};
				break;
			case (1):
				if (value != 77) {
					len = -1;
				};
				break;
			case (2):
				checksum_is = value;
				break;
			case (3):
				frame_len = value + 4;
				break;
			case (10):
				pm1_serial += ( value << 8);
				break;
			case (11):
				pm1_serial += value;
				break;
			case (12):
				pm25_serial = ( value << 8);
				break;
			case (13):
				pm25_serial += value;
				break;
			case (14):
				pm10_serial = ( value << 8);
				break;
			case (15):
				pm10_serial += value;
				break;
			case (22):
				if (frame_len == 24) {
					checksum_should = ( value << 8 );
				};
				break;
			case (23):
				if (frame_len == 24) {
					checksum_should += value;
				};
				break;
			case (30):
				checksum_should = ( value << 8 );
				break;
			case (31):
				checksum_should += value;
				break;
			}
			if ((len > 2) && (len < (frame_len - 2))) { checksum_is += value; }
			len++;
			if (len == frame_len) {
				debug_out(FPSTR(DBG_TXT_CHECKSUM_IS), DEBUG_MED_INFO, 0);
				debug_out(String(checksum_is + 143), DEBUG_MED_INFO, 0);
				debug_out(FPSTR(DBG_TXT_CHECKSUM_SHOULD), DEBUG_MED_INFO, 0);
				debug_out(String(checksum_should), DEBUG_MED_INFO, 1);
				if (checksum_should == (checksum_is + 143)) {
					checksum_ok = 1;
				} else {
					len = 0;
				};
				if (checksum_ok == 1 && (msSince(starttime) > (cfg::sending_intervall_ms - READINGTIME_SDS_MS))) {
					if ((! isnan(pm1_serial)) && (! isnan(pm10_serial)) && (! isnan(pm25_serial))) {
						pms_pm1_sum += pm1_serial;
						pms_pm10_sum += pm10_serial;
						pms_pm25_sum += pm25_serial;
						if (pms_pm1_min > pm10_serial) {
							pms_pm1_min = pm1_serial;
						}
						if (pms_pm1_max < pm10_serial) {
							pms_pm1_max = pm1_serial;
						}
						if (pms_pm10_min > pm10_serial) {
							pms_pm10_min = pm10_serial;
						}
						if (pms_pm10_max < pm10_serial) {
							pms_pm10_max = pm10_serial;
						}
						if (pms_pm25_min > pm25_serial) {
							pms_pm25_min = pm25_serial;
						}
						if (pms_pm25_max < pm25_serial) {
							pms_pm25_max = pm25_serial;
						}
						debug_out(F("PM1 (sec.): "), DEBUG_MED_INFO, 0);
						debug_out(Float2String(double(pm1_serial)), DEBUG_MED_INFO, 1);
						debug_out(F("PM2.5 (sec.): "), DEBUG_MED_INFO, 0);
						debug_out(Float2String(double(pm25_serial)), DEBUG_MED_INFO, 1);
						debug_out(F("PM10 (sec.) : "), DEBUG_MED_INFO, 0);
						debug_out(Float2String(double(pm10_serial)), DEBUG_MED_INFO, 1);
						pms_val_count++;
					}
					len = 0;
					checksum_ok = 0;
					pm1_serial = 0.0;
					pm10_serial = 0.0;
					pm25_serial = 0.0;
					checksum_is = 0;
				}
			}
			yield();
			//debug_out(F("yield() called from 2977"), DEBUG_MIN_INFO, 0);
			//debug_out("", DEBUG_MIN_INFO, 1);

		}

	}
	if (send_now) {
		last_value_PMS_P0 = -1;
		last_value_PMS_P1 = -1;
		last_value_PMS_P2 = -1;
		if (pms_val_count > 2) {
			pms_pm1_sum = pms_pm1_sum - pms_pm1_min - pms_pm1_max;
			pms_pm10_sum = pms_pm10_sum - pms_pm10_min - pms_pm10_max;
			pms_pm25_sum = pms_pm25_sum - pms_pm25_min - pms_pm25_max;
			pms_val_count = pms_val_count - 2;
		}
		if (pms_val_count > 0) {
			last_value_PMS_P0 = double(pms_pm1_sum) / (pms_val_count * 1.0);
			last_value_PMS_P1 = double(pms_pm10_sum) / (pms_val_count * 1.0);
			last_value_PMS_P2 = double(pms_pm25_sum) / (pms_val_count * 1.0);
			debug_out("PM1:	 " + Float2String(last_value_PMS_P0), DEBUG_MIN_INFO, 1);
			debug_out("PM2.5: " + Float2String(last_value_PMS_P2), DEBUG_MIN_INFO, 1);
			debug_out("PM10:	" + Float2String(last_value_PMS_P1), DEBUG_MIN_INFO, 1);
			debug_out("-------", DEBUG_MIN_INFO, 1);
			s += Value2Json("PMS_P0", Float2String(last_value_PMS_P0));
			s += Value2Json("PMS_P1", Float2String(last_value_PMS_P1));
			s += Value2Json("PMS_P2", Float2String(last_value_PMS_P2));

			// Shift log buffer
			for(int i=sizeof(values_PMS); i>0; i--){
				values_PMS[i] = values_PMS[i-1];
			}
			values_PMS[0] = last_value_PMS_P1;

		}
		pms_pm1_sum = 0;
		pms_pm10_sum = 0;
		pms_pm25_sum = 0;
		pms_val_count = 0;
		pms_pm1_max = 0;
		pms_pm1_min = 20000;
		pms_pm10_max = 0;
		pms_pm10_min = 20000;
		pms_pm25_max = 0;
		pms_pm25_min = 20000;
		if (cfg::sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)) {
			is_PMS_running = PMS_cmd(PmSensorCmd::Stop);
		}
	}

	debug_out(String(FPSTR(DBG_TXT_END_READING)) + FPSTR(SENSORS_PMSx003), DEBUG_MED_INFO, 1);

	return s;
}


/*****************************************************************
 * send SDS011 command (start, stop, continuous mode, version		*
 *****************************************************************/
static bool SDS_cmd(PmSensorCmd cmd) {//static
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
	return cmd != PmSensorCmd::Stop;
}

/*****************************************************************
 * send Plantower PMS sensor command start, stop, cont. mode		 *
 *****************************************************************/
static bool PMS_cmd(PmSensorCmd cmd) {//static
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
	return cmd != PmSensorCmd::Stop;
}
