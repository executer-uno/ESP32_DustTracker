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

// *********************** Conditional includes ****************************

#ifdef CFG_LCD
	// https://www.winstar.com.tw/products/oled-module/graphic-oled-display/color-oled-display.html

	#include "oledfont.h"				// avoids including the default Arial font, needs to be included before SSD1306.h
	#include <SSD1306.h>
	#include <LiquidCrystal_I2C.h>
	#include "SSD1306Wire.h"
#endif
#ifdef CFG_BME280
	#include <Adafruit_BME280.h>
#endif

#ifdef CFG_GPS
	#include <TinyGPS++.h>
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
#ifdef CFG_BME280
	/*****************************************************************
	 * BME280 declaration																						*
	 *****************************************************************/
	Adafruit_BME280 bme280;
#endif
#ifdef CFG_GPS
	/*****************************************************************
	 * GPS declaration																							 *
	 *****************************************************************/
	TinyGPSPlus gps;

	double last_value_GPS_lat = -200.0;
	double last_value_GPS_lon = -200.0;
	double last_value_GPS_alt = -200.0;
	String last_value_GPS_date = "";
	String last_value_GPS_time = "";

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
long next_display_count = 0;

bool BUT_A_PRESS=false;
bool BUT_B_PRESS=false;
bool BUT_C_PRESS=false;

PMmeas SDSmeasPM025;
PMmeas SDSmeasPM100;
PMmeas PMSmeasPM010;
PMmeas PMSmeasPM025;
PMmeas PMSmeasPM100;

PMmeas BMEmeasP;
PMmeas BMEmeasT;
PMmeas BMEmeasH;

// define tasks
void TaskBlink( void *pvParameters );
void TaskReadSensors( void *pvParameters );
void TaskDiagLevel( void *pvParameters );
void TaskKeyboard( void *pvParameters );
void TaskArchiveMeas( void *pvParameters );
void TaskDisplay( void *pvParameters );

UBaseType_t uxHighWaterMark_TaskBlink;
UBaseType_t uxHighWaterMark_TaskReadSensors;
UBaseType_t uxHighWaterMark_TaskDiagLevel;
UBaseType_t uxHighWaterMark_TaskKeyboard;
UBaseType_t uxHighWaterMark_TaskArchiveMeas;
UBaseType_t uxHighWaterMark_TaskDisplay;

SemaphoreHandle_t I2C_mutex;	// Mutex to access to I2C interface

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

	I2C_mutex = xSemaphoreCreateMutex();
	#ifdef CFG_LCD
		/*****************************************************************
		 * Init OLED display																						 *
		 *****************************************************************/
		display.init();
	#endif
	#ifdef CFG_BME280
		if (cfg::bme280_read) {
			debug_out(F("Read BME280..."), DEBUG_MIN_INFO, 1);
			if (!initBME280(0x76) && !initBME280(0x77)) {
				debug_out(F("Check BME280 wiring"), DEBUG_MIN_INFO, 1);
			}
			else{
				BMEmeasH.status = SensorSt::ok;
				BMEmeasP.status = SensorSt::ok;
				BMEmeasT.status = SensorSt::ok;
			}
		}
	#endif
	#ifdef CFG_GPS
		initGPS();
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
    TaskReadSensors
    ,  "ReadSDSPMS"
    ,  2048 +1024  // Stack size
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
    vTaskDelay(50);  // one tick delay (1ms) in between reads for stability
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

    if(cfg::debug == debugPrev && SDSmeasPM025.status == SensorSt::ok && PMSmeasPM025.status == SensorSt::ok){
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

  uint counter=0;

  for (;;) // A Task shall never return or exit.
  {
	  vTaskDelay(10000);  // one tick delay (1ms) in between reads for stability

	  if(SDSmeasPM025.status == SensorSt::ok && PMSmeasPM025.status == SensorSt::ok){

		  debug_out(F("ARCH PM"), DEBUG_MED_INFO, 1);

		  SDSmeasPM025.ArchPush();
		  SDSmeasPM100.ArchPush();
		  PMSmeasPM010.ArchPush();
		  PMSmeasPM025.ArchPush();
		  PMSmeasPM100.ArchPush();
	  }

	  if(counter%3){
		  debug_out(F("ARCH BME"), DEBUG_MED_INFO, 1);

		  BMEmeasH.ArchPush();
		  BMEmeasT.ArchPush();
		  BMEmeasP.ArchPush();
	  }

	  counter++;
      uxHighWaterMark_TaskArchiveMeas = uxTaskGetStackHighWaterMark( NULL );
  }
}

void TaskDisplay(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
	  if(SDSmeasPM025.status == SensorSt::ok && PMSmeasPM025.status == SensorSt::ok){

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


void TaskReadSensors(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
	sensorPMS();
    vTaskDelay(200);  // one tick delay (1ms) in between reads for stability
    sensorSDS();
    vTaskDelay(200);  // one tick delay (1ms) in between reads for stability
    sensorBME280();
    vTaskDelay(200);  // one tick delay (1ms) in between reads for stability
    sensorGPS();
    vTaskDelay(200);  // one tick delay (1ms) in between reads for stability

    uxHighWaterMark_TaskReadSensors = uxTaskGetStackHighWaterMark( NULL );
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

	if ((SDSmeasPM025.status == SensorSt::raw) && (millis() > STUP_TIME)) {
		SDS_cmd(PmSensorCmd::Start);

		while (serialSDS.available() > 0) // Initial buffer flush
		{
			buffer = serialSDS.read();
		}
		SDSmeasPM025.status = SensorSt::wait;
		SDSmeasPM100.status = SensorSt::wait;

	}

	if (SDSmeasPM025.status != SensorSt::raw){
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
					SDSmeasPM025.CRCError();
					SDSmeasPM100.CRCError();
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

					SDSmeasPM025.status = SensorSt::ok;
					SDSmeasPM100.status = SensorSt::ok;

					SDSmeasPM025.NewMeas(((float)pm025_serial)/10.0);
					SDSmeasPM100.NewMeas(((float)pm100_serial)/10.0);

					debug_out(F("SDS:"), 												DEBUG_MIN_INFO, 0);
					debug_out(SDSmeasPM100.DebugAvg() + "," + SDSmeasPM025.DebugAvg(),	DEBUG_MIN_INFO, 1);

					debug_out(F("SDS PM 2.5:"), 			DEBUG_MED_INFO, 0);
					debug_out(SDSmeasPM025.DebugRange(),	DEBUG_MED_INFO, 1);
					debug_out(F("SDS PM10.0:"), 			DEBUG_MED_INFO, 0);
					debug_out(SDSmeasPM100.DebugRange(),	DEBUG_MED_INFO, 1);

					debug_out(F("SDS CRC:"), 				DEBUG_MAX_INFO, 1);
					debug_out(SDSmeasPM100.DebugCRC(),		DEBUG_MED_INFO, 1);
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

	if((PMSmeasPM025.status == SensorSt::raw) && (millis() > STUP_TIME)) {
		PMS_cmd(PmSensorCmd::Start);

		while (serialPMS.available() > 0) // Initial buffer flush
		{
			buffer = serialPMS.read();
		}
		PMSmeasPM010.status = SensorSt::wait;
		PMSmeasPM025.status = SensorSt::wait;
		PMSmeasPM100.status = SensorSt::wait;
	}
	if(PMSmeasPM025.status != SensorSt::raw){

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
					PMSmeasPM010.CRCError();
					PMSmeasPM025.CRCError();
					PMSmeasPM100.CRCError();
				};

				// Telegram received
				if (checksum_ok == 1) {
					if ((! isnan(pm100_serial)) && (! isnan(pm010_serial)) && (! isnan(pm025_serial))) {

						PMSmeasPM010.status = SensorSt::ok;
						PMSmeasPM025.status = SensorSt::ok;
						PMSmeasPM100.status = SensorSt::ok;

						PMSmeasPM010.NewMeas((float)pm010_serial);
						PMSmeasPM025.NewMeas((float)pm025_serial);
						PMSmeasPM100.NewMeas((float)pm100_serial);

						debug_out(F("PMS:"), 																				DEBUG_MIN_INFO, 0);
						debug_out(PMSmeasPM100.DebugAvg() + "," + PMSmeasPM025.DebugAvg() + "," + PMSmeasPM010.DebugAvg(),	DEBUG_MIN_INFO, 1);

						debug_out(F("PMS PM 1.0:"), 			DEBUG_MED_INFO, 0);
						debug_out(PMSmeasPM010.DebugRange(),	DEBUG_MED_INFO, 1);
						debug_out(F("PMS PM 2.5:"), 			DEBUG_MED_INFO, 0);
						debug_out(PMSmeasPM025.DebugRange(),	DEBUG_MED_INFO, 1);
						debug_out(F("PMS PM10.0:"), 			DEBUG_MED_INFO, 0);
						debug_out(PMSmeasPM100.DebugRange(),	DEBUG_MED_INFO, 1);

						debug_out(F("PMS CRC:"), 				DEBUG_MAX_INFO, 1);
						debug_out(PMSmeasPM100.DebugCRC(),		DEBUG_MED_INFO, 1);


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

	if(next_display_count<0){		// Fix bug with previous display of 0 screen
		next_display_count = screen_count;
	}

	switch (screens[next_display_count % screen_count]) {

	case (1):
		display_header =  "  " + String(FPSTR(SENSORS_PMSx003)) + "  " + String(FPSTR(SENSORS_SDS011));
		display_lines[0] = "PM  0.1:  "  + check_display_value(PMSmeasPM010.ArchMeas.avg[0], -1.0, 1, 6) + " µg/m³";
		display_lines[1] = "PM  2.5:  "  + check_display_value(PMSmeasPM025.ArchMeas.avg[0], -1.0, 1, 6) + ";" + check_display_value(SDSmeasPM025.ArchMeas.avg[0], -1.0, 1, 6) + " µg/m³";
		display_lines[2] = "PM 10.0:  "  + check_display_value(PMSmeasPM100.ArchMeas.avg[0], -1.0, 1, 6) + ";" + check_display_value(SDSmeasPM100.ArchMeas.avg[0], -1.0, 1, 6) + " µg/m³";
		break;

	case (3):
		display_header = FPSTR(SENSORS_BME280);
		display_lines[0] = "Temp.: " + check_display_value(BMEmeasT.ArchMeas.avg[0] , -1.0				, 1, 6) + " °C";
		display_lines[1] = "Hum.:  " + check_display_value(BMEmeasH.ArchMeas.avg[0] , -1.0				, 1, 6) + " %";
		display_lines[2] = "Pres.: " + check_display_value(BMEmeasP.ArchMeas.avg[0]  / 100, (-1 / 100.0), 1, 6) + " hPa";
		break;
	case (4):
		display_header = F("GPS NEO6M");
		display_lines[0] = "Lat: " + check_display_value(last_value_GPS_lat , -200.0, 6, 10);
		display_lines[1] = "Lon: " + check_display_value(last_value_GPS_lon , -200.0, 6, 10);
		display_lines[2] = "Alt: " + check_display_value(last_value_GPS_alt , -200.0, 2, 10);
		break;
	case (5):
		display_header = F("Stack free");

		display_lines[0] = "Blink" + check_display_value(uxHighWaterMark_TaskBlink			, 0, 0, 6) + "  Sens" + check_display_value(uxHighWaterMark_TaskReadSensors	, 0, 0, 6);
		display_lines[1] = "Diag " + check_display_value(uxHighWaterMark_TaskDiagLevel		, 0, 0, 6) + "  Keyb" + check_display_value(uxHighWaterMark_TaskKeyboard		, 0, 0, 6);
		display_lines[2] = "Arch " + check_display_value(uxHighWaterMark_TaskArchiveMeas	, 0, 0, 6) + "  Disp" + check_display_value(uxHighWaterMark_TaskDisplay		, 0, 0, 6);

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

	xSemaphoreTake(I2C_mutex, portMAX_DELAY);
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

			Y = int(PMSmeasPM025.ArchMeas.avg[i]/2);

			Y = 64-18-Y;
			Y = (Y<1 ? 1 : Y);
			display.setPixel(i, Y);
			display.setPixel(i, 64-16);
		}
		display.drawString(64, 52, displayGenerateFooter(screen_count));
	}

	display.display();
	xSemaphoreGive(I2C_mutex);
}

#endif


#ifdef CFG_BME280
/*****************************************************************
 * read BME280 sensor values																		 *
 *****************************************************************/
void sensorBME280() {

	debug_out(String(FPSTR(DBG_TXT_START_READING)) + FPSTR(SENSORS_BME280), DEBUG_MED_INFO, 1);

	xSemaphoreTake(I2C_mutex, portMAX_DELAY);

	bme280.takeForcedMeasurement();
	const auto t = bme280.readTemperature();
	const auto h = bme280.readHumidity();
	const auto p = bme280.readPressure();

	xSemaphoreGive(I2C_mutex);

	if (isnan(t) || isnan(h) || isnan(p)) {

		BMEmeasH.CRCError();
		BMEmeasT.CRCError();
		BMEmeasP.CRCError();

		debug_out(String(FPSTR(SENSORS_BME280)) + FPSTR(DBG_TXT_COULDNT_BE_READ), DEBUG_ERROR, 1);
	} else {
		debug_out(FPSTR(DBG_TXT_TEMPERATURE)	, DEBUG_MIN_INFO, 0);
		debug_out(Float2String(t) + " C"		, DEBUG_MIN_INFO, 1);
		debug_out(FPSTR(DBG_TXT_HUMIDITY)		, DEBUG_MIN_INFO, 0);
		debug_out(Float2String(h) + " %"		, DEBUG_MIN_INFO, 1);
		debug_out(FPSTR(DBG_TXT_PRESSURE)		, DEBUG_MIN_INFO, 0);
		debug_out(Float2String(p / 100) + " hPa", DEBUG_MIN_INFO, 1);

		BMEmeasH.NewMeas(h);
		BMEmeasT.NewMeas(t);
		BMEmeasP.NewMeas(p);
	}
}

/*****************************************************************
 * Init BME280																									 *
 *****************************************************************/
bool initBME280(char addr) {
	debug_out(F("Trying BME280 sensor on "), DEBUG_MIN_INFO, 0);
	debug_out(String(addr, HEX), DEBUG_MIN_INFO, 0);

	if (bme280.begin(addr)) {
		debug_out(F(" ... found"), DEBUG_MIN_INFO, 1);
		bme280.setSampling(
			Adafruit_BME280::MODE_FORCED,
			Adafruit_BME280::SAMPLING_X1,
			Adafruit_BME280::SAMPLING_X1,
			Adafruit_BME280::SAMPLING_X1,
			Adafruit_BME280::FILTER_OFF);
		return true;
	} else {
		debug_out(F(" ... not found"), DEBUG_MIN_INFO, 1);
		return false;
	}
}
#endif


#ifdef CFG_GPS

/*****************************************************************
 * disable unneeded NMEA sentences, TinyGPS++ needs GGA, RMC		 *
 *****************************************************************/
void disable_unneeded_nmea() {
	serialGPS.println(F("$PUBX,40,GLL,0,0,0,0*5C"));			 // Geographic position, latitude / longitude
//	serialGPS.println(F("$PUBX,40,GGA,0,0,0,0*5A"));			 // Global Positioning System Fix Data
	serialGPS.println(F("$PUBX,40,GSA,0,0,0,0*4E"));			 // GPS DOP and active satellites
//	serialGPS.println(F("$PUBX,40,RMC,0,0,0,0*47"));			 // Recommended minimum specific GPS/Transit data
	serialGPS.println(F("$PUBX,40,GSV,0,0,0,0*59"));			 // GNSS satellites in view
	serialGPS.println(F("$PUBX,40,VTG,0,0,0,0*5E"));			 // Track made good and ground speed
}


/*****************************************************************
 * read GPS sensor values																				*
 *****************************************************************/
void sensorGPS() {
	String s = "";
	String gps_lat = "";
	String gps_lon = "";

	debug_out(String(FPSTR(DBG_TXT_START_READING)) + "GPS", DEBUG_MED_INFO, 1);

	while (serialGPS.available() > 0) {
		if (gps.encode(serialGPS.read())) {
			if (gps.location.isValid()) {
				last_value_GPS_lat = gps.location.lat();
				last_value_GPS_lon = gps.location.lng();
				gps_lat = Float2String(last_value_GPS_lat, 6);
				gps_lon = Float2String(last_value_GPS_lon, 6);
			} else {
				last_value_GPS_lat = -200;
				last_value_GPS_lon = -200;
				debug_out(F("Lat/Lng INVALID"), DEBUG_MAX_INFO, 1);
			}
			if (gps.altitude.isValid()) {
				last_value_GPS_alt = gps.altitude.meters();
				String gps_alt = Float2String(last_value_GPS_alt, 2);
			} else {
				last_value_GPS_alt = -200;
				debug_out(F("Altitude INVALID"), DEBUG_MAX_INFO, 1);
			}
			if (gps.date.isValid()) {
				String gps_date = "";
				if (gps.date.month() < 10) {
					gps_date += "0";
				}
				gps_date += String(gps.date.month());
				gps_date += "/";
				if (gps.date.day() < 10) {
					gps_date += "0";
				}
				gps_date += String(gps.date.day());
				gps_date += "/";
				gps_date += String(gps.date.year());
				last_value_GPS_date = gps_date;
			} else {
				debug_out(F("Date INVALID"), DEBUG_MAX_INFO, 1);
			}

			// gps.time.hour() resets Updated flag!
			if(gps.time.isUpdated()){
				// Set time from GPS
//			    time_t t_of_day;
//			    struct tm t;
//			    timeval epoch;
//			    const timeval *tv = &epoch;
//			    timezone utc = {0,TimeZone};
//			    const timezone *tz = &utc;
//
//			    t.tm_year = gps.date.year()  - 1900;
//			    t.tm_mon  = gps.date.month() - 1;   // Month, 0 - jan
//			    t.tm_mday = gps.date.day();         // Day of the month
//			    t.tm_hour = gps.time.hour() + GMT_OFF;
//			    t.tm_min  = gps.time.minute();
//			    t.tm_sec  = gps.time.second();
//			    t_of_day  = mktime(&t);

//			    epoch = {t_of_day, 0};

//			    settimeofday(tv, tz);
//				setenv("TZ", TZ_INFO, 1);
//				tzset(); 							// Assign the local timezone from setenv

//				debug_out(F("GPS Time setted"), DEBUG_MAX_INFO, 1);
//				got_ntp = true;
//				timeUpdate = millis() + 300000;

//				time_str = printLocalTime();
//				Serial.println("----------> Local Time = " + time_str);

			}

			if (gps.time.isValid()) {
				String gps_time = "";
				if (gps.time.hour() < 10) {
					gps_time += "0";
				}
				gps_time += String(gps.time.hour());
				gps_time += ":";
				if (gps.time.minute() < 10) {
					gps_time += "0";
				}
				gps_time += String(gps.time.minute());
				gps_time += ":";
				if (gps.time.second() < 10) {
					gps_time += "0";
				}
				gps_time += String(gps.time.second());
				gps_time += ".";
				if (gps.time.centisecond() < 10) {
					gps_time += "0";
				}
				gps_time += String(gps.time.centisecond());
				last_value_GPS_time = gps_time;
			} else {
				debug_out(F("Time: INVALID"), DEBUG_MAX_INFO, 1);
			}

		}
	}

	if ( gps.charsProcessed() < 10) {
		debug_out(F("GPS : check wiring"), DEBUG_ERROR, 1);
	}

	debug_out(String(FPSTR(DBG_TXT_END_READING)) + "GPS", DEBUG_MED_INFO, 1);

}


bool initGPS() {

	if (cfg::gps_read) {
		int retryCount = 0;

		disable_unneeded_nmea();
		debug_out(F("GPS configuration done."), DEBUG_MIN_INFO, 1);

		serialGPS.flush();
		while(serialGPS.available()) serialGPS.read();
		debug_out(F("Wait GPS communication "), DEBUG_MIN_INFO, 0);
		while (true) {	// Wait for GPS loop

			if (serialGPS.available() > 0) {
				 debug_out(F("\nGPS alive."), DEBUG_MIN_INFO, 1);
				 return false;
			}
			if (++retryCount > 20) {
				 debug_out(F("\nGPS timeout."), DEBUG_MIN_INFO, 1);
				 return true;
			}

		}
	}
	return false;
}

#endif
