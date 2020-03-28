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
#include "Sensors.h"

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


#include <FS.h>										 // must be first
#ifdef CFG_SQL
	//#include <stdio.h>
	//#include <stdlib.h>
	//#include <SPI.h>

	#include <sqlite3.h>
	#include "SD.h"
/*	Connections:
		 * SD Card | ESP32
		 *	DAT2			 -
		 *	DAT3			 SS (D5)
		 *	CMD				MOSI (D23)
		 *	VSS				GND
		 *	VDD				3.3V
		 *	CLK				SCK (D18)
		 *	DAT0			 MISO (D19)
		 *	DAT1			 -			*/
	//#include "SPIFFS.h"
	/* You only need to format SPIFFS the first time you run a
		 test or else use the SPIFFS plugin to create a partition
		 https://github.com/me-no-dev/arduino-esp32fs-plugin */
	#define FORMAT_SPIFFS_IF_FAILED true
	#define DB_PATH "/sd/DB_portable.db"

	sqlite3 	*db;
	int 		rc;
	sqlite3_stmt *res;
	int 		rec_count = 0;
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

bool BUT_DB_CLEAR_FLAG=false;


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

static TaskHandle_t xTaskDisplay_handle = NULL;
static TaskHandle_t xTaskArchiveMeas_handle = NULL;

UBaseType_t uxHighWaterMark_TaskBlink;
UBaseType_t uxHighWaterMark_TaskReadSensors;
UBaseType_t uxHighWaterMark_TaskDiagLevel;
UBaseType_t uxHighWaterMark_TaskKeyboard;
UBaseType_t uxHighWaterMark_TaskArchiveMeas;
UBaseType_t uxHighWaterMark_TaskDisplay;

SemaphoreHandle_t I2C_mutex;	// Mutex to access to I2C interface


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
		//display.mirrorScreen();
		display.flipScreenVertically();

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

	#ifdef CFG_SQL
		SD.begin();
		if(!SD.begin()){
			debug_out(F("Card Mount Failed."),							DEBUG_ERROR, 1);
		}
		uint8_t cardType = SD.cardType();

		if(cardType == CARD_NONE){
			debug_out(F("No SD card attached."),						DEBUG_ERROR, 1);
		}
		else
		{
			debug_out(F("SD Card Type: "),						DEBUG_MED_INFO, 0);

			if(cardType == CARD_MMC){
					debug_out(F("MMC"),							DEBUG_MED_INFO, 1);
			} else if(cardType == CARD_SD){
					debug_out(F("SDSC"),						DEBUG_MED_INFO, 1);
			} else if(cardType == CARD_SDHC){
					debug_out(F("SDHC"),						DEBUG_MED_INFO, 1);
			} else {
					debug_out(F("UNKNOWN"),						DEBUG_MED_INFO, 1);
			}
			uint64_t cardSize = SD.cardSize() / (1024 * 1024);

			debug_out("SD Card Size: "+ String(int(cardSize)) 						+ "MB",DEBUG_MED_INFO, 1);
			debug_out("Total space:  "+ String(int(SD.totalBytes() / (1024 * 1024)))+ "MB",DEBUG_MED_INFO, 1);
			debug_out("Used space:   "+ String(int(SD.usedBytes() / (1024 * 1024)))	+ "MB",DEBUG_MED_INFO, 1);

			File root = SD.open("/");
			if (!root) {
				debug_out(F("- failed to open directory"),		DEBUG_ERROR, 1);
			}
			else
			{
				if (!root.isDirectory()) {
					debug_out(F("- not a directory"),			DEBUG_ERROR, 1);
				}
				else
				{

					sqlite3_initialize();

					if (!db_open(DB_PATH, &db))
						{

						rc = db_exec(db, "PRAGMA page_size = 512;");
						if (rc != SQLITE_OK) {
							 Serial.println("PRAGMA page_size set failure");
						}

						rc = db_exec(db, "PRAGMA default_cache_size = 200; PRAGMA cache_size = 200;");
						if (rc != SQLITE_OK) {
							 Serial.println("PRAGMA default_cache_size set failure");
						}

						rc = db_exec(db, "CREATE TABLE IF NOT EXISTS measBME (datetime, sendGS BOOL, sendAD BOOL, temp REAL, press REAL, humid REAL);");
						if (rc != SQLITE_OK) {
							 sqlite3_close(db);
							 Serial.println("Table measurements 'measBME' creation failure");
							 return;
						}

						rc = db_exec(db, "CREATE TABLE IF NOT EXISTS measPMS (datetime, sendGS BOOL, sendAD BOOL, PM010 REAL, PM025 REAL, PM100 REAL);");
						if (rc != SQLITE_OK) {
							 sqlite3_close(db);
							 Serial.println("Table measurements 'measPMS' creation failure");
							 return;
						}

						rc = db_exec(db, "CREATE TABLE IF NOT EXISTS measSDS (datetime, sendGS BOOL, sendAD BOOL, PM025 REAL, PM100 REAL);");
						if (rc != SQLITE_OK) {
							 sqlite3_close(db);
							 Serial.println("Table measurements 'measSDS' creation failure");
							 return;
						}

						rc = db_exec(db, "CREATE TABLE IF NOT EXISTS measGPS (datetime, sendGS BOOL, sendAD BOOL, lat REAL, lon REAL);");
						if (rc != SQLITE_OK) {
							 sqlite3_close(db);
							 Serial.println("Table measurements 'measGPS' creation failure");
							 return;
						}

						// get actual number of records to variable
						const char *sql = "Select count(*) from measurements";
						if (sqlite3_prepare_v2(db, sql, -1, &res, NULL) != SQLITE_OK) {
								String resp = "Failed to fetch data: ";
								resp += sqlite3_errmsg(db);
								Serial.println(resp);
						}
						else {
							if (sqlite3_step(res) != SQLITE_ROW) {
									String resp = "Step failure: ";
									resp += sqlite3_errmsg(db);
									Serial.println(resp);
							}
							else {
									rec_count = sqlite3_column_int(res, 0);
							}
						}
						sqlite3_finalize(res);
						sqlite3_close(db);
					}
				}
			}
		}


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
    ,  1024*8 // Stack size
    ,  NULL
    ,  1  // Priority
    ,  &xTaskArchiveMeas_handle
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
	TaskDisplay
    ,  "Display"
    ,  2048  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  &xTaskDisplay_handle
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
    vTaskDelay(10);  // one tick delay (1ms) in between reads for stability
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

    if(cfg::debug == debugPrev && SDSmeasPM025.status == SensorSt::ok && PMSmeasPM025.status == SensorSt::ok){
    	vTaskDelay(990);  // one tick delay (1ms) in between reads for stability
    }
    else{
    	vTaskDelay(100);  // one tick delay (1ms) in between reads for stability
    }
    debugPrev = cfg::debug;

    uxHighWaterMark_TaskBlink = uxTaskGetStackHighWaterMark( NULL );
  }
  vTaskDelete( NULL );
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
  vTaskDelete( NULL );
}


void TaskArchiveMeas(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
      uxHighWaterMark_TaskArchiveMeas = uxTaskGetStackHighWaterMark( NULL );

	  vTaskDelay(60000);  // one tick delay (1ms) in between reads for stability

	  if(BUT_DB_CLEAR_FLAG){
		  ClearDB();
		  BUT_DB_CLEAR_FLAG = false;
	  }

	  if(SDSmeasPM025.status == SensorSt::ok && PMSmeasPM025.status == SensorSt::ok){

		  debug_out(F("ARCH"), DEBUG_MED_INFO, 1);

		  SDSmeasPM025.ArchPush();
		  SDSmeasPM100.ArchPush();
		  PMSmeasPM010.ArchPush();
		  PMSmeasPM025.ArchPush();
		  PMSmeasPM100.ArchPush();

		  BMEmeasH.ArchPush();
		  BMEmeasT.ArchPush();
		  BMEmeasP.ArchPush();

		  Store2DB();
	  }

  }
  vTaskDelete( NULL );
}

void TaskDisplay(void *pvParameters)  // This is a task.
{
	(void) pvParameters;
	uint32_t ulNotificationValue;

	for (;;) // A Task shall never return or exit.
	{
		#ifdef CFG_LCD
			/*****************************************************************
			* display values																								*
			*****************************************************************/
			display_values();
		#endif

		/* Wait to be notified that the transmission is complete.  Note the first
		parameter is pdTRUE, which has the effect of clearing the task's notification
		value back to 0, making the notification value act like a binary (rather than
		a counting) semaphore.  */

		ulNotificationValue = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS( 2000 ) );

		if( ulNotificationValue == 1 )
		{
			/* The transmission ended as expected. */
			debug_out(F("DISPLAY: refresh by Notification"), 	DEBUG_WARNING, 1);
		}
		else
		{
			/* The call to ulTaskNotifyTake() timed out. */
			debug_out(F("DISPLAY: refresh by timeout"), 		DEBUG_MAX_INFO, 1);
		}



		uxHighWaterMark_TaskDisplay = uxTaskGetStackHighWaterMark( NULL );
	}
	vTaskDelete( NULL );
}


void TaskKeyboard(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50;

  // Initialize the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount ();


  for (;;) // A Task shall never return or exit.
  {
	bool BUT_A = !digitalRead(BUT_1);	// no internal pullup
	bool BUT_B = !digitalRead(BUT_2);
	bool BUT_C = !digitalRead(BUT_3);

	if (BUT_A && !BUT_A_PRESS)
	{
		debug_out(F("Button Left"), 	DEBUG_WARNING, 1);
		next_display_count--;
		xTaskNotifyGive(xTaskDisplay_handle);
	}

	if (BUT_B && !BUT_B_PRESS)
	{
		debug_out(F("Button Mid"), 		DEBUG_WARNING, 1);
	}

	if (BUT_C && !BUT_C_PRESS)
	{
		debug_out(F("Button Center"), 	DEBUG_WARNING, 1);
		next_display_count++;
		xTaskNotifyGive(xTaskDisplay_handle);
	}

	BUT_A_PRESS = BUT_A;
	BUT_B_PRESS = BUT_B;
	BUT_C_PRESS = BUT_C;

    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    uxHighWaterMark_TaskKeyboard = uxTaskGetStackHighWaterMark( NULL );
  }
  vTaskDelete( NULL );
}


void TaskReadSensors(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
	sensorPMS();
    vTaskDelay(150);  // one tick delay (1ms) in between reads for stability
    sensorSDS();
    vTaskDelay(150);  // one tick delay (1ms) in between reads for stability
    sensorBME280();
    vTaskDelay(150);  // one tick delay (1ms) in between reads for stability
    sensorGPS();
    vTaskDelay(150);  // one tick delay (1ms) in between reads for stability

    uxHighWaterMark_TaskReadSensors = uxTaskGetStackHighWaterMark( NULL );
  }
  vTaskDelete( NULL );
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
		display_lines[0] = "DB records: " + String(rec_count);//"ID: " + esp_chipid;
		display_lines[1] = "";//"FW: " + String(SOFTWARE_VERSION);
		display_lines[2] = "";//"Meas: " + String(count_sends) + " Since last: " + String((long)((msSince(starttime) + 500) / 1000)) + " s.";



		if(BUT_B_PRESS){
			BUT_DB_CLEAR_FLAG = true;
		}


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
		display.drawString(0, 13, display_lines[0]);
		display.drawString(0, 25, display_lines[1]);
		display.drawString(0, 37, display_lines[2]);

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



#ifdef CFG_SQL
	const char* data = "Callback function called";
	static int callback(void *data, int argc, char **argv, char **azColName) {
		 int i;
		 debug_out(String((const char*)data), 					DEBUG_MED_INFO, 0);
		 debug_out(F(": "), 									DEBUG_MED_INFO, 0);
		 for (i = 0; i<argc; i++){
				 debug_out(String(azColName[i]), 				DEBUG_MED_INFO, 0);
				 debug_out(F(" = "), 							DEBUG_MED_INFO, 0);
				 debug_out(String(argv[i] ? argv[i] : "NULL"), 	DEBUG_MED_INFO, 1);
		 }
		 debug_out(F(""), 										DEBUG_MED_INFO, 1);
		 return 0;
	}

	int db_open(const char *filename, sqlite3 **db) {
		 int rc = sqlite3_open(filename, db);
		 if (rc) {
				 debug_out(F("Can't open database: "), 			DEBUG_MED_INFO, 0);
				 debug_out(String(sqlite3_errmsg(*db)), 		DEBUG_MED_INFO, 1);
				 return rc;
		 } else {
				 debug_out(F("Opened database successfully"), 	DEBUG_MED_INFO, 1);
		 }
		 return rc;
	}

	char *zErrMsg = 0;
	int db_exec(sqlite3 *db, const char *sql) {
		 Serial.println(sql);
		 long start = micros();
		 int rc = sqlite3_exec(db, sql, callback, (void*)data, &zErrMsg);
		 if (rc != SQLITE_OK) {
				 debug_out(F("SQL error: "), 					DEBUG_MED_INFO, 0);
				 debug_out(String(zErrMsg), 					DEBUG_MED_INFO, 1);
				 sqlite3_free(zErrMsg);
		 } else {
				 debug_out(F("Operation done successfully"), 	DEBUG_MED_INFO, 1);
		 }
		 debug_out(F("Time taken:"), 							DEBUG_MED_INFO, 0);
		 debug_out(String(micros()-start), 						DEBUG_MED_INFO, 1);

		 return rc;
	}


void Store2DB(){
	if (!db_open(DB_PATH, &db))
	{
		/*
		"CREATE TABLE IF NOT EXISTS measGPS (datetime, sendGS BOOL, sendAD BOOL, lat REAL, lon REAL);"
		"CREATE TABLE IF NOT EXISTS measSDS (datetime, sendGS BOOL, sendAD BOOL, PM025 REAL, PM100 REAL);"
		"CREATE TABLE IF NOT EXISTS measPMS (datetime, sendGS BOOL, sendAD BOOL, PM010 REAL, PM025 REAL, PM100 REAL);"
		"CREATE TABLE IF NOT EXISTS measBME (datetime, sendGS BOOL, sendAD BOOL, temp REAL, press REAL, humid REAL);"
		*/

		String query = "";
		String DateTime = "2020.02.19 18:65:26.325";

		query  = "INSERT INTO measBME (datetime, temp, press, humid) VALUES ('" + DateTime + "',";
		query += Float2String(BMEmeasT.ArchMeas.avg[0]) + ",";
		query += Float2String(BMEmeasP.ArchMeas.avg[0]) + ",";
		query += Float2String(BMEmeasH.ArchMeas.avg[0]) + ")";

		rc = db_exec(db, query.c_str());
		if (rc != SQLITE_OK) {
			debug_out(F("Table 'measBME' not updated"),			DEBUG_ERROR, 1);
		}

		query  = "INSERT INTO measSDS (datetime, PM025, PM100) VALUES ('" + DateTime + "',";
		query += Float2String(SDSmeasPM025.ArchMeas.avg[0]) + ",";
		query += Float2String(SDSmeasPM100.ArchMeas.avg[0]) + ")";

		rc = db_exec(db, query.c_str());
		if (rc != SQLITE_OK) {
			debug_out(F("Table 'measSDS' not updated"),			DEBUG_ERROR, 1);
		}

		query  = "INSERT INTO measPMS (datetime, PM010, PM025, PM100) VALUES ('" + DateTime + "',";
		query += Float2String(PMSmeasPM010.ArchMeas.avg[0]) + ",";
		query += Float2String(PMSmeasPM025.ArchMeas.avg[0]) + ",";
		query += Float2String(PMSmeasPM100.ArchMeas.avg[0]) + ")";

		rc = db_exec(db, query.c_str());
		if (rc != SQLITE_OK) {
			debug_out(F("Table 'measPMS' not updated"),			DEBUG_ERROR, 1);
		}

		query  = "INSERT INTO measGPS (datetime, lat, lon) VALUES ('" + DateTime + "',";
		query += Float2String(last_value_GPS_lat) + ",";
		query += Float2String(last_value_GPS_lon) + ")";

		rc = db_exec(db, query.c_str());
		if (rc != SQLITE_OK) {
			debug_out(F("Table 'measGPS' not updated"),			DEBUG_ERROR, 1);
		}

		// get actual number of records to variable
		const char *sql = "Select count(*) from measBME";
		if (sqlite3_prepare_v2(db, sql, -1, &res, NULL) != SQLITE_OK) {
				String resp = "Failed to fetch data: ";
				resp += sqlite3_errmsg(db);
				debug_out(resp,								DEBUG_ERROR, 1);
		}
		else {
			if (sqlite3_step(res) != SQLITE_ROW) {
					String resp = "Step failure: ";
					resp += sqlite3_errmsg(db);
					debug_out(resp,							DEBUG_ERROR, 1);
			}
			else {
					rec_count = sqlite3_column_int(res, 0);
			}
		}

		debug_out("Count measBME records = " + String(rec_count),		DEBUG_MIN_INFO, 1);

		sqlite3_finalize(res);

		sqlite3_close(db);
	}
	else{
		debug_out(F("Store2DB: DB opening error."),						DEBUG_ERROR, 1);
	}
}


void ClearDB(){
	if (!db_open(DB_PATH, &db))
	{
		/*
		"CREATE TABLE IF NOT EXISTS measGPS (datetime, sendGS BOOL, sendAD BOOL, lat REAL, lon REAL);"
		"CREATE TABLE IF NOT EXISTS measSDS (datetime, sendGS BOOL, sendAD BOOL, PM025 REAL, PM100 REAL);"
		"CREATE TABLE IF NOT EXISTS measPMS (datetime, sendGS BOOL, sendAD BOOL, PM010 REAL, PM025 REAL, PM100 REAL);"
		"CREATE TABLE IF NOT EXISTS measBME (datetime, sendGS BOOL, sendAD BOOL, temp REAL, press REAL, humid REAL);"
		*/

		String query = "";

		query  = "DELETE FROM measBME;";

		rc = db_exec(db, query.c_str());
		if (rc != SQLITE_OK) {
			debug_out(F("Table 'measBME' not updated"),			DEBUG_ERROR, 1);
		}

		query  = "DELETE FROM measSDS;";

		rc = db_exec(db, query.c_str());
		if (rc != SQLITE_OK) {
			debug_out(F("Table 'measSDS' not updated"),			DEBUG_ERROR, 1);
		}

		query  = "DELETE FROM measPMS;";

		rc = db_exec(db, query.c_str());
		if (rc != SQLITE_OK) {
			debug_out(F("Table 'measPMS' not updated"),			DEBUG_ERROR, 1);
		}

		query  = "DELETE FROM measGPS;";

		rc = db_exec(db, query.c_str());
		if (rc != SQLITE_OK) {
			debug_out(F("Table 'measGPS' not updated"),			DEBUG_ERROR, 1);
		}

		rec_count = 0;

		sqlite3_close(db);
	}
	else{
		debug_out(F("ClearDB: DB opening error."),				DEBUG_ERROR, 1);
	}
}

#endif
