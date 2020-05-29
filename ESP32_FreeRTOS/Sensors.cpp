/*
 * Sensors.cpp
 *
 *  Created on: Mar 24, 2020
 *      Author: E_CAD
 */

#include <Arduino.h>
#include "Sensors.h"
#include "Definitions.h"
#include "html-content.h"


template<typename T, std::size_t N> constexpr std::size_t array_num_elements(const T(&)[N]) {
	return N;
}


/*****************************************************************
 * send SDS011 command (start, stop, continuous mode, version		*
 *****************************************************************/
void SDS_cmd(Stream *UART_SDS, PmSensorCmd cmd) {
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
	UART_SDS->write(buf, cmd_len);
}


/*****************************************************************
 * send Plantower PMS sensor command start, stop, cont. mode		 *
 *****************************************************************/
void PMS_cmd(Stream *UART_PMS, PmSensorCmd cmd) {
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
	UART_PMS->write(buf, cmd_len);
}


/*****************************************************************
 * read SDS011 sensor values																		 *
 *****************************************************************/
void sensorSDS(Stream *UART_PMS, PMmeas *measPM025, PMmeas *measPM100) {
	char buffer;
	int value;
	int len = 0;
	int pm100_serial = 0;	// 10x PM 10.0
	int pm025_serial = 0;	// 10x PM 2.5
	int checksum_is = 0;
	int checksum_ok = 0;

	// https://nettigo.pl/attachments/415
	// Sensor protocol

	debug_out(String(FPSTR(DBG_TXT_START_READING)) + FPSTR(SENSORS_SDS011), DEBUG_MED_INFO, 1);

	if ((measPM025->status == SensorSt::raw) && (millis() > STUP_TIME)) {
		SDS_cmd(UART_PMS, PmSensorCmd::Start);

		while (UART_PMS->available() > 0) // Initial buffer flush
		{
			buffer = UART_PMS->read();
		}
		measPM025->status = SensorSt::wait;
		measPM100->status = SensorSt::wait;

	}

	if (measPM025->status != SensorSt::raw){
		while (UART_PMS->available() > 0) {
			buffer = UART_PMS->read();
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
				pm025_serial = value;
				checksum_is = value;
				break;
			case (3):
				pm025_serial += (value << 8);
				break;
			case (4):
				pm100_serial = value;
				break;
			case (5):
				pm100_serial += (value << 8);
				break;
			case (8):

				debug_out(FPSTR(DBG_TXT_CHECKSUM_IS), 		DEBUG_MAX_INFO, 0);
				debug_out(String(checksum_is % 256), 		DEBUG_MAX_INFO, 0);
				debug_out(FPSTR(DBG_TXT_CHECKSUM_SHOULD), 	DEBUG_MAX_INFO, 0);
				debug_out(String(value), 					DEBUG_MAX_INFO, 1);

				if (value == (checksum_is % 256)) {
					checksum_ok = 1;
				} else {
					len = -1;
					measPM025->CRCError();
					measPM100->CRCError();
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
				if ((! isnan(pm100_serial)) && (! isnan(pm025_serial))) {

					measPM025->status = SensorSt::ok;
					measPM100->status = SensorSt::ok;

					measPM025->NewMeas(((float)pm025_serial)/10.0);
					measPM100->NewMeas(((float)pm100_serial)/10.0);

					/*
					debug_out(F("SDS:"), 												DEBUG_MIN_INFO, 0);
					debug_out(SDSmeasPM100.DebugAvg() + "," + measPM025.DebugAvg(),	DEBUG_MIN_INFO, 1);
					 */

					debug_out(F("SDS PM 2.5:"), 			DEBUG_MED_INFO, 0);
					debug_out(measPM025->DebugRange(),		DEBUG_MED_INFO, 1);
					debug_out(F("SDS PM10.0:"), 			DEBUG_MED_INFO, 0);
					debug_out(measPM100->DebugRange(),		DEBUG_MED_INFO, 1);

					debug_out(F("SDS CRC:"), 				DEBUG_MED_INFO, 1);
					debug_out(measPM100->DebugCRC(),		DEBUG_MED_INFO, 1);
				}
				len = 0;
				checksum_ok = 0;
				pm100_serial = 0.0;
				pm025_serial = 0.0;
				checksum_is = 0;
			}
		}
	}
}

/*****************************************************************
 * read Plantronic PM sensor sensor values											 *
 *****************************************************************/
void sensorPMS(Stream *UART_PMS, PMmeas *measPM010, PMmeas *measPM025, PMmeas *measPM100) {
	char buffer;
	int value;
	int len = 0;

	int pm010_serial = 0;	// PM1.0 (ug/m3)
	int pm025_serial = 0;	// PM2.5
	int pm100_serial = 0;	// PM10.0

	int TSI_pm010_serial = 0;	// PM1.0
	int TSI_pm025_serial = 0;	// PM2.5
	int TSI_pm100_serial = 0;	// PM10.0

	int checksum_is 	= 0;
	int checksum_should = 0;
	int checksum_ok 	= 0;
	int frame_len   	= 24;	// minimum frame length

	// https://github.com/avaldebe/AQmon/blob/master/Documents/PMS3003_LOGOELE.pdf
	// http://download.kamami.pl/p563980-PMS3003%20series%20data%20manual_English_V2.5.pdf
	// Sensor protocol

	if((measPM025->status == SensorSt::raw) && (millis() > STUP_TIME)) {
		PMS_cmd(UART_PMS, PmSensorCmd::Start);

		while (UART_PMS->available() > 0) // Initial buffer flush
		{
			buffer = UART_PMS->read();
		}
		measPM010->status = SensorSt::wait;
		measPM025->status = SensorSt::wait;
		measPM100->status = SensorSt::wait;
	}
	if(measPM025->status != SensorSt::raw){

		debug_out(String(FPSTR(DBG_TXT_START_READING)) + FPSTR(SENSORS_PMSx003), DEBUG_MED_INFO, 1);

		while (UART_PMS->available() > 0) {
			buffer = UART_PMS->read();
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

			// TSI Values
			case (4):
				TSI_pm010_serial += ( value << 8);
				break;
			case (5):
				TSI_pm010_serial += value;
				break;
			case (6):
				TSI_pm025_serial = ( value << 8);
				break;
			case (7):
				TSI_pm025_serial += value;
				break;
			case (8):
				TSI_pm100_serial = ( value << 8);
				break;
			case (9):
				TSI_pm100_serial += value;
				break;

			// Standard atmosphere values
			case (10):
				pm010_serial += ( value << 8);
				break;
			case (11):
				pm010_serial += value;
				break;
			case (12):
				pm025_serial = ( value << 8);
				break;
			case (13):
				pm025_serial += value;
				break;
			case (14):
				pm100_serial = ( value << 8);
				break;
			case (15):
				pm100_serial += value;
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

				debug_out(FPSTR(DBG_TXT_CHECKSUM_IS), 		DEBUG_MAX_INFO, 0);
				debug_out(String(checksum_is + 143), 		DEBUG_MAX_INFO, 0);
				debug_out(FPSTR(DBG_TXT_CHECKSUM_SHOULD), 	DEBUG_MAX_INFO, 0);
				debug_out(String(checksum_should), 			DEBUG_MAX_INFO, 1);

				if (checksum_should == (checksum_is + 143)) {
					checksum_ok = 1;
				} else {
					len = 0;
					measPM010->CRCError();
					measPM025->CRCError();
					measPM100->CRCError();
				};

				// Telegram received
				if (checksum_ok == 1) {
					if ((! isnan(pm100_serial)) && (! isnan(pm010_serial)) && (! isnan(pm025_serial))) {

						measPM010->status = SensorSt::ok;
						measPM025->status = SensorSt::ok;
						measPM100->status = SensorSt::ok;

						measPM010->NewMeas((float)pm010_serial);
						measPM025->NewMeas((float)pm025_serial);
						measPM100->NewMeas((float)pm100_serial);

						debug_out(F("PMS PM 1.0:"), 			DEBUG_MED_INFO, 0);
						debug_out(measPM010->DebugRange(),		DEBUG_MED_INFO, 1);
						debug_out(F("PMS PM 2.5:"), 			DEBUG_MED_INFO, 0);
						debug_out(measPM025->DebugRange(),		DEBUG_MED_INFO, 1);
						debug_out(F("PMS PM10.0:"), 			DEBUG_MED_INFO, 0);
						debug_out(measPM100->DebugRange(),		DEBUG_MED_INFO, 1);

						debug_out(F("PMS CRC:"), 				DEBUG_MED_INFO, 1);
						debug_out(measPM100->DebugCRC(),		DEBUG_MED_INFO, 1);


					}
					len = 0;
					checksum_ok = 0;
					pm100_serial = 0.0;
					pm010_serial = 0.0;
					pm025_serial = 0.0;

					TSI_pm100_serial = 0.0;
					TSI_pm010_serial = 0.0;
					TSI_pm025_serial = 0.0;

					checksum_is = 0;
				}
			}
		}
	}
}



