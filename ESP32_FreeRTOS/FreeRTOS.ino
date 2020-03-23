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


#ifdef CFG_LCD
	#include "oledfont.h"				// avoids including the default Arial font, needs to be included before SSD1306.h
	#include <SSD1306.h>
	#include <LiquidCrystal_I2C.h>
	#include "SSD1306Wire.h"
#endif

// ***************************** Variables *********************************

BluetoothSerial Serial; 		//Object for Bluetooth
SoftwareSerial 	SWSerial;
HardwareSerial 	serialSDS(0);
HardwareSerial 	serialPMS(1);
HardwareSerial 	serialGPS(2);

#ifdef CFG_LCD
	/*****************************************************************
	 * Display definitions																					 *
	 *****************************************************************/
	SSD1306 display(0x3c, I2C_PIN_SDA, I2C_PIN_SCL); // OLED_ADDRESS
#endif

namespace cfg {
	char wlanssid[35] 	= WLANSSID;
	char wlanpwd[65] 	= WLANPWD;

	char www_username[65] = WWW_USERNAME;
	char www_password[65] = WWW_PASSWORD;

	char fs_ssid[33]	= FS_SSID;
	char fs_pwd[65] 	= FS_PWD;
	int	debug 			= DEBUG;

	bool sds_read 		= SDS_READ;
	bool pms_read 		= PMS_READ;
	bool bme280_read 	= BME280_READ;
	bool gps_read 		= GPS_READ;
}

int	debugPrev;
unsigned long next_display_count = 0;

bool BUT_A_PRESS=false;
bool BUT_B_PRESS=false;
bool BUT_C_PRESS=false;

PMmeas SDSmeas;
PMmeas PMSmeas;

// define tasks
void TaskBlink( void *pvParameters );
void TaskReadPMSensors( void *pvParameters );
void TaskDiagLevel( void *pvParameters );
void TaskKeyboard( void *pvParameters );
void TaskArchiveMeas( void *pvParameters );
void TaskDisplay( void *pvParameters );

UBaseType_t uxHighWaterMark_TaskBlink;
UBaseType_t uxHighWaterMark_TaskReadPMSensors;
UBaseType_t uxHighWaterMark_TaskDiagLevel;
UBaseType_t uxHighWaterMark_TaskKeyboard;
UBaseType_t uxHighWaterMark_TaskArchiveMeas;
UBaseType_t uxHighWaterMark_TaskDisplay;


template<typename T, std::size_t N> constexpr std::size_t array_num_elements(const T(&)[N]) {
	return N;
}

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

	SDSmeas.Init();
	PMSmeas.Init();


	#ifdef CFG_LCD
		/*****************************************************************
		 * Init OLED display																						 *
		 *****************************************************************/
		display.init();
	#endif

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
    ,  2048  // Stack size
    ,  NULL
    ,  2  // Priority
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

  xTaskCreatePinnedToCore(
	TaskKeyboard
    ,  "Keyboard"
    ,  1024  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
	TaskArchiveMeas
    ,  "CyclicArcive"
    ,  1024 // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
	TaskDisplay
    ,  "Display"
    ,  2048  // Stack size
    ,  NULL
    ,  1  // Priority
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
    vTaskDelay(50);  // one tick delay (1ms) in between reads for stability
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

    if(cfg::debug == debugPrev && SDSmeas.status == SensorSt::ok && PMSmeas.status == SensorSt::ok){
    	vTaskDelay(950);  // one tick delay (1ms) in between reads for stability
    }
    else{
    	vTaskDelay(100);  // one tick delay (1ms) in between reads for stability
    }
    debugPrev = cfg::debug;

    uxHighWaterMark_TaskBlink = uxTaskGetStackHighWaterMark( NULL );
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
    uxHighWaterMark_TaskDiagLevel = uxTaskGetStackHighWaterMark( NULL );
  }
}


void TaskArchiveMeas(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {

	  if(SDSmeas.status == SensorSt::ok && PMSmeas.status == SensorSt::ok){
		  vTaskDelay(10000);  // one tick delay (1ms) in between reads for stability

		  debug_out(F("ARCH"), DEBUG_MIN_INFO, 1);

		  SDSmeas.ArchPush();
		  PMSmeas.ArchPush();
	  }

      uxHighWaterMark_TaskArchiveMeas = uxTaskGetStackHighWaterMark( NULL );
  }
}

void TaskDisplay(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
	  if(SDSmeas.status == SensorSt::ok && PMSmeas.status == SensorSt::ok){

	  }
	  #ifdef CFG_LCD
		  /*****************************************************************
		  * display values																								*
		  *****************************************************************/
		  display_values();
	  #endif
	  vTaskDelay(1000);  // one tick delay (1ms) in between reads for stability

      uxHighWaterMark_TaskDisplay = uxTaskGetStackHighWaterMark( NULL );
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
		next_display_count--;
	}

	if (BUT_B && !BUT_B_PRESS)
	{
		debug_out(F("Button B"), DEBUG_MIN_INFO, 1);
		next_display_count++;
	}

	if (BUT_C && !BUT_C_PRESS)
	{
		debug_out(F("Button C"), DEBUG_MIN_INFO, 1);
	}



	BUT_A_PRESS = BUT_A;
	BUT_B_PRESS = BUT_B;
	BUT_C_PRESS = BUT_C;

    vTaskDelay(5);  // one tick delay (1ms) in between reads for stability

    uxHighWaterMark_TaskKeyboard = uxTaskGetStackHighWaterMark( NULL );
  }
}


void TaskReadPMSensors(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
	sensorPMS();
    vTaskDelay(400);  // one tick delay (1ms) in between reads for stability
    sensorSDS();
    vTaskDelay(400);  // one tick delay (1ms) in between reads for stability

    uxHighWaterMark_TaskReadPMSensors = uxTaskGetStackHighWaterMark( NULL );
  }
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
					SDSmeas.CRCError();
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

					SDSmeas.status = SensorSt::ok;

					debug_out(F("Values: PM2.5, P10.0:"), 		DEBUG_MAX_INFO, 0);
					debug_out(Float2String(pm025_serial, 1, 6) ,DEBUG_MAX_INFO, 0);
					debug_out(Float2String(pm100_serial, 1, 6) ,DEBUG_MAX_INFO, 1);

					SDSmeas.NewMeas((float)0.0, ((float)pm025_serial)/10.0, ((float)pm100_serial)/10.0);

					SDSmeas.PrintDebug();

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
void sensorPMS() {
	char buffer;
	int value;
	int len = 0;
	int pm010_serial = 0;	// PM1.0 (ug/m3)
	int pm025_serial = 0;	// PM2.5
	int pm100_serial = 0;	// PM10.0

	int TSI_pm010_serial = 0;	// PM1.0
	int TSI_pm025_serial = 0;	// PM2.5
	int TSI_pm100_serial = 0;	// PM10.0

	int checksum_is = 0;
	int checksum_should = 0;
	int checksum_ok = 0;
	int frame_len   = 24;	// minimum frame length

	// https://github.com/avaldebe/AQmon/blob/master/Documents/PMS3003_LOGOELE.pdf
	// http://download.kamami.pl/p563980-PMS3003%20series%20data%20manual_English_V2.5.pdf
	// Sensor protocol

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

				debug_out(FPSTR(DBG_TXT_CHECKSUM_IS), 		DEBUG_MED_INFO, 0);
				debug_out(String(checksum_is + 143), 		DEBUG_MED_INFO, 0);
				debug_out(FPSTR(DBG_TXT_CHECKSUM_SHOULD), 	DEBUG_MED_INFO, 0);
				debug_out(String(checksum_should), 			DEBUG_MED_INFO, 1);

				if (checksum_should == (checksum_is + 143)) {
					checksum_ok = 1;
				} else {
					len = 0;
					PMSmeas.CRCError();
				};

				// Telegram received
				if (checksum_ok == 1) {
					if ((! isnan(pm100_serial)) && (! isnan(pm010_serial)) && (! isnan(pm025_serial))) {

						PMSmeas.status = SensorSt::ok;

						debug_out(F("Values: PM1.0, PM2.5, P10.0:"), 		DEBUG_MAX_INFO, 0);
						debug_out(Float2String(pm010_serial, 1, 6) ,DEBUG_MAX_INFO, 0);
						debug_out(Float2String(pm025_serial, 1, 6) ,DEBUG_MAX_INFO, 0);
						debug_out(Float2String(pm100_serial, 1, 6) ,DEBUG_MAX_INFO, 1);

						PMSmeas.NewMeas((float)pm010_serial, (float)pm025_serial, (float)pm100_serial);

						PMSmeas.PrintDebug();

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



#ifdef CFG_LCD
/*****************************************************************
 * display values																								*
 *****************************************************************/

static String displayGenerateFooter(unsigned int screen_count) {
	String display_footer;
	for (unsigned int i = 0; i < screen_count; ++i) {
		display_footer += (i != (next_display_count % screen_count)) ? " . " : " o ";
	}
	return display_footer;
}


void display_values() {

	String display_header = "";
	String display_lines[3] = { "", "", ""};

	int screens[9];
	int screen_count = 0;


	if (cfg::pms_read || cfg::sds_read ) {
		screens[screen_count++] = 1;
	}

	if (cfg::bme280_read) {
		screens[screen_count++] = 3;
	}
	if (cfg::gps_read) {
		screens[screen_count++] = 4;
	}

	screens[screen_count++] = 5;	// Wifi info
	screens[screen_count++] = 6;	// chipID, firmware and count of measurements
	screens[screen_count++] = 11;	// Trend, GPS, Values

	switch (screens[next_display_count % screen_count]) {

	case (1):
		display_header =  "  " + String(FPSTR(SENSORS_PMSx003)) + "  " + String(FPSTR(SENSORS_SDS011));
		display_lines[0] = "PM  0.1:  "  + check_display_value(PMSmeas.ArchPm010.avg[0], -1.0, 1, 6) + ";" + " ---- " 									+ " µg/m³";
		display_lines[1] = "PM  2.5:  "  + check_display_value(PMSmeas.ArchPm025.avg[0], -1.0, 1, 6) + ";" + check_display_value(SDSmeas.ArchPm025.avg[0], -1.0, 1, 6) + " µg/m³";
		display_lines[2] = "PM 10.0:  "  + check_display_value(PMSmeas.ArchPm100.avg[0], -1.0, 1, 6) + ";" + check_display_value(SDSmeas.ArchPm100.avg[0], -1.0, 1, 6) + " µg/m³";
		break;

	case (3):
		display_header = FPSTR(SENSORS_BME280);
		display_lines[0] = "Temp.: " + check_display_value(0.0 , -128.0				, 1, 6) + " °C";
		display_lines[1] = "Hum.:  " + check_display_value(0.0 , -1.0				, 1, 6) + " %";
		display_lines[2] = "Pres.: " + check_display_value(0.0  / 100, (-1 / 100.0)	, 1, 6) + " hPa";
		break;
	case (4):
		display_header = F("GPS NEO6M");
		display_lines[0] = "Lat: " + check_display_value(0.0 , -200.0, 6, 10);
		display_lines[1] = "Lon: " + check_display_value(0.0 , -200.0, 6, 10);
		display_lines[2] = "Alt: " + check_display_value(0.0 , -1000.0, 2, 10);
		break;
	case (5):
		display_header = F("Info");

		display_lines[0] = check_display_value(uxHighWaterMark_TaskBlink			, 0, 0, 6) + check_display_value(uxHighWaterMark_TaskReadPMSensors	, 0, 0, 6);
		display_lines[1] = check_display_value(uxHighWaterMark_TaskDiagLevel		, 0, 0, 6) + check_display_value(uxHighWaterMark_TaskKeyboard		, 0, 0, 6);
		display_lines[2] = check_display_value(uxHighWaterMark_TaskArchiveMeas		, 0, 0, 6) + check_display_value(uxHighWaterMark_TaskDisplay		, 0, 0, 6);

//		display_lines[0] = "";//"IP:      " + WiFi.localIP().toString();
//		display_lines[1] = "";//"SSID:    " + WiFi.SSID();
//		display_lines[2] = "";//"Signal:  " + String(calcWiFiSignalQuality(WiFi.RSSI())) + "%";
		//if (WiFi.status() != WL_CONNECTED) {
		//	display_lines[2] = "Signal:  NO CONNECTION!";
		//}

		break;
	case (6):
		display_header = F("Device Info");
		display_lines[0] = "";//"ID: " + esp_chipid;
		display_lines[1] = "";//"FW: " + String(SOFTWARE_VERSION);
		display_lines[2] = "";//"Meas: " + String(count_sends) + " Since last: " + String((long)((msSince(starttime) + 500) / 1000)) + " s.";
		break;
	case (11):
		display_header = "Measurements";


		break;
	}

	display.clear();
	display.displayOn();
	display.setTextAlignment(TEXT_ALIGN_CENTER);

	if(screens[next_display_count % screen_count] < 10){
		display.drawString(64, 0, display_header);
		display.setTextAlignment(TEXT_ALIGN_LEFT);
		display.drawString(0, 12, display_lines[0]);
		display.drawString(0, 24, display_lines[1]);
		display.drawString(0, 36, display_lines[2]);

		display.setTextAlignment(TEXT_ALIGN_CENTER);
		display.drawString(64, 52, displayGenerateFooter(screen_count));

//		// Show time on display
//		struct tm timeinfo;
//		if(got_ntp){
//			if (getLocalTime(&timeinfo)) {
//				char timeStringBuff[10];
//				strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S", &timeinfo);
//				display.drawString(108, 52, String(timeStringBuff));
//			}
//			else{
//				display.drawString(108, 52, "-- -- --");
//			}
//		}
//		else{
//			display.drawString(108, 52, "-- -- --");
//		}
	}
	else{
		for(int i=1; i<display.getWidth(); i++){
			int16_t Y=0;

			Y = int(PMSmeas.ArchPm025.avg[i]/2);

			Y = 64-18-Y;
			Y = (Y<1 ? 1 : Y);
			display.setPixel(i, Y);
			display.setPixel(i, 64-16);
		}
		display.drawString(64, 52, displayGenerateFooter(screen_count));
	}


	display.display();


}



#endif

