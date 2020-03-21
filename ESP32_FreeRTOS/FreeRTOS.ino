#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include <Arduino.h>
#include "SoftwareSerial.h"
#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino
#include "Credentials.h"
#include "html-content.h"
//#define WLANSSID   "MyWifiName"  //in Credentials.h
//#define WLANPWD    "MyWiFiPass"  //in Credentials.h

#include "Definitions.h"


// ***************************** Variables *********************************

BluetoothSerial Serial; 		//Object for Bluetooth
SoftwareSerial 	SWSerial;
HardwareSerial 	serialSDS(0);
HardwareSerial 	serialPMS(1);
HardwareSerial 	serialGPS(2);

namespace cfg {
	char wlanssid[35] 	= WLANSSID;
	char wlanpwd[65] 	= WLANPWD;

	char www_username[65] = WWW_USERNAME;
	char www_password[65] = WWW_PASSWORD;

	char fs_ssid[33]	= FS_SSID;
	char fs_pwd[65] 	= FS_PWD;
	int	debug 			= DEBUG;
}

int	debugPrev;

bool BUT_A_PRESS=false;
bool BUT_B_PRESS=false;
bool BUT_C_PRESS=false;

struct {
	SensorSt		status;		// Sensor initialized and started
	uint32_t 		count; 		// number of measurements in accumulator
	uint32_t 		CRCerr;		// number of CRC errors
	struct measure 	pm025;
	struct measure 	pm100;
} SDSmeas;
struct {
	SensorSt		status;		// Sensor initialized and started
	uint32_t 		count; 		// number of measurements in accumulator
	uint32_t 		CRCerr;		// number of CRC errors
	struct measure 	pm010;
	struct measure 	pm025;
	struct measure 	pm100;
} PMSmeas;

// define two tasks for Blink & AnalogRead
void TaskBlink( void *pvParameters );
void TaskReadPMSensors( void *pvParameters );
void TaskDiagLevel( void *pvParameters );
void TaskKeyboard( void *pvParameters );


// the setup function runs once when you press reset or power the board
void setup() {

	serialSDS.begin(9600, SERIAL_8N1, PM_SERIAL_RX,  PM_SERIAL_TX);			 		// for HW UART SDS
	serialPMS.begin(9600, SERIAL_8N1, PM2_SERIAL_RX, PM2_SERIAL_TX);			 	// for HW UART PMS
	serialGPS.begin(9600, SERIAL_8N1, GPS_SERIAL_RX, GPS_SERIAL_TX);			 	// for HW UART GPS


	// initialize serial communication at 115200 bits per second:
	SWSerial.begin(9600, SWSERIAL_8N1, DEB_RX, DEB_TX, false, 95, 11);
	SWSerial.println(F("SW serial started"));

	Serial.begin("ESP32_PMS_Station"); //Name of your Bluetooth Signal
	debug_out(F("SW serial started"), DEBUG_MED_INFO, 1);

	SWSerial.println("Bluetooth Device is Ready to Pair");


	// Configure buttons
	pinMode(BUT_1, INPUT_PULLUP);
	pinMode(BUT_2, INPUT_PULLUP);
	pinMode(BUT_3, INPUT_PULLUP);

 	// initialize digital LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);


	SDSmeas.status 		= SensorSt::raw;
	PMSmeas.status 		= SensorSt::raw;
	SDSmeas.pm100.min	= INT_MAX;
	SDSmeas.pm025.min	= INT_MAX;
	PMSmeas.pm010.min	= INT_MAX;
	PMSmeas.pm025.min	= INT_MAX;
	PMSmeas.pm100.min	= INT_MAX;


  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskBlink
    ,  "TaskBlink"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskReadPMSensors
    ,  "ReadSDSPMS"
    ,  9000  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
	TaskKeyboard
    ,  "Keyboard"
    ,  1024  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
	TaskDiagLevel
    ,  "DiagLevel"
    ,  1024  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
	// No job
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
    
  If you want to know what pin the on-board LED is connected to on your ESP32 model, check
  the Technical Specs of your board.
*/


  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay(100);  // one tick delay (1ms) in between reads for stability
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

    if(cfg::debug == debugPrev){
    	vTaskDelay(900);  // one tick delay (1ms) in between reads for stability
    }
    else{
    	debugPrev = cfg::debug;
    	vTaskDelay(100);  // one tick delay (1ms) in between reads for stability
    }

  }
}

void TaskDiagLevel(void *pvParameters)  // This is a task.
{
  (void) pvParameters;


  for (;;) // A Task shall never return or exit.
  {
		if (Serial.available()) //Check if we receive anything from Bluetooth
		{
			int incoming;
			incoming = Serial.read(); //Read what we receive

			if (incoming >= 49 && incoming <= 53 )
			{
				incoming -= 48;

				SWSerial.print("Deb lvl:");
				SWSerial.println(incoming );

				cfg::debug = incoming;
			}

		}
    vTaskDelay(100);  // one tick delay (1ms) in between reads for stability
  }
}

void TaskKeyboard(void *pvParameters)  // This is a task.
{
  (void) pvParameters;


  for (;;) // A Task shall never return or exit.
  {
	bool BUT_A = !digitalRead(BUT_3);
	bool BUT_B = !digitalRead(BUT_2);
	bool BUT_C = false; //!digitalRead(BUT_1);// unitll no pullup

	if (BUT_A && !BUT_A_PRESS)
	{
		debug_out(F("Button A"), DEBUG_MIN_INFO, 1);

		// Reset counters

		SDSmeas.pm100.min	= INT_MAX;
		SDSmeas.pm025.min	= INT_MAX;

		PMSmeas.pm010.min	= INT_MAX;
		PMSmeas.pm025.min	= INT_MAX;
		PMSmeas.pm100.min	= INT_MAX;

		SDSmeas.pm100.max	= 0;
		SDSmeas.pm025.max	= 0;

		PMSmeas.pm010.max	= 0;
		PMSmeas.pm025.max	= 0;
		PMSmeas.pm100.max	= 0;

		SDSmeas.pm100.sum	= 0;
		SDSmeas.pm025.sum	= 0;

		PMSmeas.pm010.sum	= 0;
		PMSmeas.pm025.sum	= 0;
		PMSmeas.pm100.sum	= 0;

		PMSmeas.CRCerr		= 0;
		SDSmeas.CRCerr		= 0;

		SDSmeas.count		= 0;
		PMSmeas.count		= 0;

	}

	if (BUT_B && !BUT_B_PRESS)
	{
		debug_out(F("Button B"), DEBUG_MIN_INFO, 1);
	}

	if (BUT_C && !BUT_C_PRESS)
	{
		debug_out(F("Button C"), DEBUG_MIN_INFO, 1);
	}



	BUT_A_PRESS = BUT_A;
	BUT_B_PRESS = BUT_B;
	BUT_C_PRESS = BUT_C;

    vTaskDelay(5);  // one tick delay (1ms) in between reads for stability
  }
}


void TaskReadPMSensors(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

/*
*/

  for (;;)
  {
	sensorPMS();
    vTaskDelay(400);  // one tick delay (1ms) in between reads for stability
    sensorSDS();
    vTaskDelay(400);  // one tick delay (1ms) in between reads for stability
  }
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

template<typename T, std::size_t N> constexpr std::size_t array_num_elements(const T(&)[N]) {
	return N;
}

/*****************************************************************
 * send SDS011 command (start, stop, continuous mode, version		*
 *****************************************************************/
static void SDS_cmd(PmSensorCmd cmd) {//static
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
static void PMS_cmd(PmSensorCmd cmd) {//static
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
 * read SDS011 sensor values																		 *
 *****************************************************************/
void sensorSDS() {
	String strDebug = "";
	char buffer;
	int value;
	int len = 0;
	int pm10_serial = 0;
	int pm25_serial = 0;
	int checksum_is = 0;
	int checksum_ok = 0;

	debug_out(String(FPSTR(DBG_TXT_START_READING)) + FPSTR(SENSORS_SDS011), DEBUG_MED_INFO, 1);

	if ((SDSmeas.status == SensorSt::raw) && (millis() > 10000ULL)) {
		SDS_cmd(PmSensorCmd::Start);

		while (serialSDS.available() > 0) // Initial buffer flush
		{
			buffer = serialSDS.read();
		}
		SDSmeas.status = SensorSt::wait;
	}
	if (SDSmeas.status != SensorSt::raw){
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

				debug_out(FPSTR(DBG_TXT_CHECKSUM_IS), 		DEBUG_MAX_INFO, 0);
				debug_out(String(checksum_is % 256), 		DEBUG_MAX_INFO, 0);
				debug_out(FPSTR(DBG_TXT_CHECKSUM_SHOULD), 	DEBUG_MAX_INFO, 0);
				debug_out(String(value), 					DEBUG_MAX_INFO, 1);

				if (value == (checksum_is % 256)) {
					checksum_ok = 1;
				} else {
					len = -1;
					SDSmeas.CRCerr++;
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

					SDSmeas.pm100.sum += pm10_serial;
					SDSmeas.pm025.sum += pm25_serial;


					SDSmeas.pm100.max = (SDSmeas.pm100.max<pm10_serial ? pm10_serial : SDSmeas.pm100.max);
					SDSmeas.pm025.max = (SDSmeas.pm025.max<pm25_serial ? pm25_serial : SDSmeas.pm025.max);

					SDSmeas.pm100.min = (SDSmeas.pm100.min>pm10_serial ? pm10_serial : SDSmeas.pm100.min);
					SDSmeas.pm025.min = (SDSmeas.pm025.min>pm25_serial ? pm25_serial : SDSmeas.pm025.min);

					SDSmeas.count++;

					// Debug:
					if(cfg::debug <= DEBUG_MIN_INFO ){
						strDebug  = "PM10=" + Float2String(double(SDSmeas.pm100.sum) / (10* SDSmeas.count)) + ",";
						strDebug += String("         ").substring(1, 13 - strDebug.length());
						strDebug +=  "PM2=" + Float2String(double(SDSmeas.pm025.sum) / (10* SDSmeas.count)) + "";
						debug_out(strDebug, DEBUG_MIN_INFO, 1);
					}

					strDebug  = "PM10.0 : [" + Float2String(double(SDSmeas.pm100.min) / 10) + " : ";
					strDebug += Float2String(double(SDSmeas.pm100.sum) / (10* SDSmeas.count)) + " : ";
					strDebug += Float2String(double(SDSmeas.pm100.max) / 10) + " ]";
					debug_out(strDebug, DEBUG_MED_INFO, 1);

					strDebug  = "PM2.5 : [" + Float2String(double(SDSmeas.pm025.min) / 10) + " : ";
					strDebug += Float2String(double(SDSmeas.pm025.sum) / (10* SDSmeas.count)) + " : ";
					strDebug += Float2String(double(SDSmeas.pm025.max) / 10) + " ]";
					debug_out(strDebug, DEBUG_MED_INFO, 1);

					debug_out(F("CRC errors rate: "), 										DEBUG_MED_INFO , 0);
					debug_out(Float2String(SDSmeas.CRCerr/(SDSmeas.CRCerr+SDSmeas.count)),	DEBUG_MED_INFO , 1);
					debug_out(F("Count: "), 												DEBUG_MED_INFO , 0);
					debug_out(Float2String(SDSmeas.count),									DEBUG_MED_INFO , 1);

				}
				len = 0;
				checksum_ok = 0;
				pm10_serial = 0.0;
				pm25_serial = 0.0;
				checksum_is = 0;
			}
		}
	}
}

/*****************************************************************
 * read Plantronic PM sensor sensor values											 *
 *****************************************************************/
void sensorPMS() {
	String strDebug = "";
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

	if((PMSmeas.status == SensorSt::raw) && (millis() > 10000ULL)) {
		PMS_cmd(PmSensorCmd::Start);

		while (serialPMS.available() > 0) // Initial buffer flush
		{
			buffer = serialPMS.read();
		}
		PMSmeas.status = SensorSt::wait;
	}
	if(PMSmeas.status != SensorSt::raw){

		debug_out(String(FPSTR(DBG_TXT_START_READING)) + FPSTR(SENSORS_PMSx003), DEBUG_MED_INFO, 1);

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

				debug_out(FPSTR(DBG_TXT_CHECKSUM_IS), 		DEBUG_MED_INFO, 0);
				debug_out(String(checksum_is + 143), 		DEBUG_MED_INFO, 0);
				debug_out(FPSTR(DBG_TXT_CHECKSUM_SHOULD), 	DEBUG_MED_INFO, 0);
				debug_out(String(checksum_should), 			DEBUG_MED_INFO, 1);

				if (checksum_should == (checksum_is + 143)) {
					checksum_ok = 1;
				} else {
					len = 0;
					PMSmeas.CRCerr++;
				};

				// Telegram received
				if (checksum_ok == 1) {
					if ((! isnan(pm1_serial)) && (! isnan(pm10_serial)) && (! isnan(pm25_serial))) {

						pm10_serial	*= 10;		// to make maths same with SDS
						pm25_serial	*= 10;
						pm1_serial	*= 10;

						PMSmeas.pm010.sum += pm10_serial;
						PMSmeas.pm025.sum += pm25_serial;
						PMSmeas.pm100.sum += pm1_serial;

						PMSmeas.pm010.max = (PMSmeas.pm010.max<pm10_serial ? pm10_serial : PMSmeas.pm010.max);
						PMSmeas.pm025.max = (PMSmeas.pm025.max<pm25_serial ? pm25_serial : PMSmeas.pm025.max);
						PMSmeas.pm100.max = (PMSmeas.pm100.max<pm25_serial ? pm1_serial  : PMSmeas.pm100.max);

						PMSmeas.pm010.min = (PMSmeas.pm010.min>pm10_serial ? pm10_serial : PMSmeas.pm010.min);
						PMSmeas.pm025.min = (PMSmeas.pm025.min>pm25_serial ? pm25_serial : PMSmeas.pm025.min);
						PMSmeas.pm100.min = (PMSmeas.pm100.min>pm25_serial ? pm1_serial  : PMSmeas.pm100.min);

						PMSmeas.count++;

						// Debug:
						if(cfg::debug <= DEBUG_MIN_INFO ){
							strDebug  = "PM10=" + Float2String(double(PMSmeas.pm100.sum) / (10* PMSmeas.count)) + ",";
							strDebug += String("         ").substring(1, 13 - strDebug.length());
							strDebug += "PM2=" + Float2String(double(PMSmeas.pm025.sum) / (10* PMSmeas.count)) + ",";
							strDebug += String("         ").substring(1, 25 - strDebug.length());
							strDebug += "PM1=" + Float2String(double(PMSmeas.pm010.sum) / (10* PMSmeas.count)) + "";
							debug_out(strDebug, DEBUG_MIN_INFO, 1);
						}

						strDebug  = "PM10.0 : [" + Float2String(double(PMSmeas.pm100.min) / 10) + " : ";
						strDebug += Float2String(double(PMSmeas.pm100.sum) / (10* PMSmeas.count)) + " : ";
						strDebug += Float2String(double(PMSmeas.pm100.max) / 10) + " ]";
						debug_out(strDebug, DEBUG_MED_INFO, 1);

						strDebug  = "PM2.5 : [" + Float2String(double(PMSmeas.pm025.min) / 10) + " : ";
						strDebug += Float2String(double(PMSmeas.pm025.sum) / (10* PMSmeas.count)) + " : ";
						strDebug += Float2String(double(PMSmeas.pm025.max) / 10) + " ]";
						debug_out(strDebug, DEBUG_MED_INFO, 1);

						strDebug  = "PM1.0 : [" + Float2String(double(PMSmeas.pm010.min) / 10) + " : ";
						strDebug += Float2String(double(PMSmeas.pm100.sum) / (10* PMSmeas.count)) + " : ";
						strDebug += Float2String(double(PMSmeas.pm100.max) / 10) + " ]";
						debug_out(strDebug, DEBUG_MED_INFO, 1);

						debug_out(F("CRC errors rate: "), 										DEBUG_MED_INFO , 0);
						debug_out(Float2String(PMSmeas.CRCerr/(PMSmeas.CRCerr+PMSmeas.count)),	DEBUG_MED_INFO , 1);
						debug_out(F("Count: "), 												DEBUG_MED_INFO , 0);
						debug_out(Float2String(PMSmeas.count),									DEBUG_MED_INFO , 1);



					}
					len = 0;
					checksum_ok = 0;
					pm1_serial = 0.0;
					pm10_serial = 0.0;
					pm25_serial = 0.0;
					checksum_is = 0;
				}
			}

		}

	}
}


/*****************************************************************
 * convert float to string with a      							 *
 * precision of two (or a given number of) decimal places		 *
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
