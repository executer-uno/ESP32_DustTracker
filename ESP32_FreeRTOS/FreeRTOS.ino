
// Energu savings recomendations https://www.savjee.be/2019/12/esp32-tips-to-increase-battery-life/
// And here https://randomnerdtutorials.com/esp32-timer-wake-up-deep-sleep/



#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 	1
#define SECOND_CORE 			0

#endif

	// Config functionality
	#define CFG_BME280
	#define CFG_LCD
	#define CFG_GPS
	#define CFG_SQL
	#define CFG_GSHEET


// check git branchiing

#include <Arduino.h>
#ifdef CFG_GPS
	//#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino
	#include "SoftwareSerial.h"
#endif
#include "html-content.h"



// *********************** Conditional includes ****************************

#ifdef CFG_LCD
	// https://www.winstar.com.tw/products/oled-module/graphic-oled-display/color-oled-display.html
	#include "oledfont.h"							// avoids including the default Arial font, needs to be included before SSD1306.h
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

#ifdef CFG_GSHEET
	#include <esp_wifi.h>
	//#include <WiFi.h>
	#include "time.h"

	//#include <esp_wifi.h>						 	// must be first
	//#include <WiFiClient.h>
	//#include <WiFiClientSecure.h>
	#include "lib/HTTPSRedirect.h"

	struct struct_wifiInfo *wifiInfo;
	uint8_t count_wifiInfo;

	const char *GScriptId = GSHEET_ID;



	const char* ntpServer = "pool.ntp.org";
	const long  gmtOffset_sec = 3600*2;
	const int   daylightOffset_sec = 3600;



#endif


#ifdef CFG_SQL


	#include <sqlite3.h>
	#include <SPI.h>
	#include <FS.h>
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
	#define DB_FILE 	   "/DB_portable04.db"
	#define DB_PATH 	"/sd/DB_portable04.db"

	sqlite3 	*db;
	int 		rc;
	sqlite3_stmt *res;
	int 		rec_count 	= 0;
	int64_t 	RID 		= 0;

#endif

#include "Definitions.h"
#include "Credentials.h" // Use 'Credentials_template.h' file as template
#include "Sensors.h"
#include <rom/rtc.h>
#include "api.h"

#include "AQI_Calculation.h"
#include "esp_deep_sleep.h"

// ***************************** Variables *********************************

HardwareSerial 	serialSDS(1);
HardwareSerial 	serialPMS(2);

SoftwareSerial 	serialPMS_EXT;

#ifdef CFG_GPS
	SoftwareSerial 	Serial; 		//Object for Bluetooth
	HardwareSerial 	serialGPS(0);
#else
	HardwareSerial 	Serial(0);		// HW serial debug
#endif



#ifdef CFG_LCD
	/*****************************************************************
	 * Display definitions																					 *
	 *****************************************************************/
	SSD1306 display(0x3c, I2C_PIN_SDA, I2C_PIN_SCL); // OLED_ADDRESS (128x64 pixels)

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

	double last_value_GPS_lat = GPS_UNDEF;
	double last_value_GPS_lon = GPS_UNDEF;
	double last_value_GPS_alt = GPS_UNDEF;

#endif
#ifdef CFG_GSHEET
	// For google spreadsheets:

	String esp_chipid = "- - -";

	const char* host = "script.google.com";
	const int 	httpsPort = 443;
	const char* fingerprint = "";


	String url =  String("/macros/s/") + GScriptId + "/exec?value=Hello";		// Write to Google Spreadsheet
	String url2 = String("/macros/s/") + GScriptId + "/exec?cal";				// Fetch Google Calendar events for 1 week ahead
	String url3 = String("/macros/s/") + GScriptId + "/exec?read";				// Read from Google Spreadsheet
	String payload_base =	"{\"command\": \"appendRow\", \"sheet_name\": \"DATA\", \"values\": ";

	HTTPSRedirect* client = nullptr;

	const char data_first_part[] PROGMEM = "{\"software_version\": \"{v}\", \"sensordatavalues\":[";

#endif

namespace cfg {
	int	debug 			= DEBUG;

	bool sds_read 		= SDS_READ;
	bool pms_read 		= PMS_READ;
	bool bme280_read 	= BME280_READ;
	bool gps_read 		= GPS_READ;
}

time_t now;	// Time variable

int	 debugPrev			= 0;
long next_display_count = 0;

bool BUT_A_PRESS=false;
bool BUT_B_PRESS=false;
bool BUT_C_PRESS=false;

uint32_t AnyButtonPressed = 0;

bool BUT_DB_CLEAR_FLAG=false;

RTC_DATA_ATTR RecMode	Mode = RecMode::NoGPS;		// default mode without GPS // Stored in RTC low power memory

bool inWindow = false;				// Check if sensor is in anonimizing rectangle coordinates


PMmeas SDSmeasPM025;
PMmeas SDSmeasPM100;
PMmeas PMSmeasPM010;
PMmeas PMSmeasPM025;
PMmeas PMSmeasPM100;

PMmeas PMSEmeasPM010;
PMmeas PMSEmeasPM025;
PMmeas PMSEmeasPM100;

PMmeas BMEmeasP;
PMmeas BMEmeasT;
PMmeas BMEmeasH;

// define tasks
void TaskBlink(			void *pvParameters );
void TaskReadSensors( 	void *pvParameters );
void TaskDiagLevel( 	void *pvParameters );
void TaskKeyboard( 		void *pvParameters );
void TaskArchiveMeas( 	void *pvParameters );
void TaskDisplay( 		void *pvParameters );
void TaskWiFi( 			void *pvParameters );

static TaskHandle_t xTaskDisplay_handle = NULL;
static TaskHandle_t xTaskReadSensors_handle = NULL;
static TaskHandle_t xTaskArchiveMeas_handle = NULL;

UBaseType_t uxHighWaterMark_TaskBlink;
UBaseType_t uxHighWaterMark_TaskReadSensors;
UBaseType_t uxHighWaterMark_TaskDiagLevel;
UBaseType_t uxHighWaterMark_TaskKeyboard;
UBaseType_t uxHighWaterMark_TaskArchiveMeas;
UBaseType_t uxHighWaterMark_TaskDisplay;
UBaseType_t uxHighWaterMark_TaskWiFi;

SemaphoreHandle_t I2C_mutex;	// Mutex to access to I2C interface
SemaphoreHandle_t Serial_mutex;	// Mutex to access to Serial RS232 interface
SemaphoreHandle_t SQL_mutex;	// Mutex to access to SQLite database
SemaphoreHandle_t WiFi_mutex;	// Mutex to access to SQLite database


// the setup function runs once when you press reset or power the board
void setup() {
	I2C_mutex 		= xSemaphoreCreateMutex();
	Serial_mutex 	= xSemaphoreCreateMutex();
	SQL_mutex		= xSemaphoreCreateMutex();
	WiFi_mutex		= xSemaphoreCreateMutex();

	cfg::debug 		= DEBUG_ALWAYS;

	serialSDS.begin(9600, SERIAL_8N1, PM_SERIAL_RX,  PM_SERIAL_TX);			 		// for HW UART SDS
	serialPMS.begin(9600, SERIAL_8N1, PM2_SERIAL_RX, PM2_SERIAL_TX);			 	// for HW UART PMS

	SDS_cmd(&serialSDS, PmSensorCmd::Stop);
	PMS_cmd(&serialPMS, PmSensorCmd::Stop);

	// Drop power consumption in case we are out of battery
	//WiFi.disconnect(true);
	//WiFi.mode(WIFI_OFF);
	//btStop();

	// Configure buttons
	pinMode(BUT_1, INPUT_PULLUP);
	pinMode(BUT_2, INPUT_PULLUP);
	pinMode(BUT_3, INPUT_PULLUP);

	// Configure device supply self-control. High = Hold supply on, Low = Power Off
	pinMode(SUPPLY, OUTPUT);
	digitalWrite(SUPPLY, LOW);  // Check if it was manual switching on. If non - most likely battery is low, and brownout restart.
	vTaskDelay(500); 			// one tick delay (1ms) in between reads for stability
	digitalWrite(SUPPLY, HIGH); // Check if it was manual switching on. If non - most likely battery is low, and brownout restart.


	#ifdef CFG_LCD
		/*****************************************************************
		 * Init OLED display																						 *
		 *****************************************************************/
		display.init();
		display.displayOff();
	#endif

 	// initialize digital LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);    // turn the LED ON by making the voltage HIGH

	char MAC_chars[15]; //Create a Unique AP from MAC address
	uint64_t chipid= ESP.getEfuseMac();			//The chip ID is essentially its MAC address(length: 6 bytes).
	uint16_t chiph = (uint16_t)(chipid>>32);	//High 	2 bytes
	uint32_t chipl = (uint32_t)(chipid);		//Low	4 bytes

	snprintf(MAC_chars,15,  "%04X", chiph);
	snprintf(MAC_chars+4,15,"%08X", chipl);
	esp_chipid = String(MAC_chars);			//50E1F1BF713C

	#ifdef CFG_GPS
		serialGPS.begin(9600, SERIAL_8N1, GPS_SERIAL_RX, GPS_SERIAL_TX);			// for HW UART GPS

	#else
		Serial.begin(115200, SERIAL_8N1, DEB_RX, DEB_TX);			 				// for HW UART GPS
	#endif

	Serial.enableIntTx(false);		// For test
	Serial.begin(9600, SWSERIAL_8N1, DEB_RX, DEB_TX, false, 200, 110);
	Serial.printf("ESP: 01 Min level of free heap: %u\n", ESP.getMinFreeHeap());

	serialPMS_EXT.enableIntTx(false);		// For test
	serialPMS_EXT.begin(9600, SWSERIAL_8N1, PM3_SERIAL_RX, PM3_SERIAL_TX, false, 400, 220);

	PMS_cmd(&serialPMS_EXT, PmSensorCmd::Stop);

	time(&now);
	debug_out("Undefined raw time:" + String(now), DEBUG_ALWAYS, 1);

	#ifdef CFG_LCD
		/*****************************************************************
		 * Turn on OLED display																						 *
		 *****************************************************************/
		display.resetOrientation();
		//display.flipScreenVertically();
		//display.mirrorScreen();				// Adapt for new device enclosure and board
		display.displayOn();
	#endif

	Serial.printf("ESP: 02 Min level of free heap: %u\n", ESP.getMinFreeHeap());

	#ifdef CFG_GPS

		display.setTextAlignment(TEXT_ALIGN_CENTER);
		display.drawString(64, 0, "WiFi SmartConfig");
		display.drawString(64, 13, "press button to start");

		// time to start WiFi setup
		String progress = " .";
		for(int del=0; del<42; del++){

			digitalWrite(LED_BUILTIN, HIGH);    // turn the LED ON by making the voltage HIGH
			vTaskDelay(100); // one tick delay (1ms) in between reads for stability
			digitalWrite(LED_BUILTIN, LOW );
			vTaskDelay(100); // one tick delay (1ms) in between reads for stability
			progress += " .";

			#ifdef CFG_LCD
				display.drawString(0, 37, progress);
				display.display();
			#endif

			bool BUT_A = !digitalRead(BUT_1);	// no internal pullup
			bool BUT_B = !digitalRead(BUT_2);
			bool BUT_C = !digitalRead(BUT_3);

			if(BUT_A || BUT_B || BUT_C){

				// Display WiFi config
				xSemaphoreTake(I2C_mutex, portMAX_DELAY);
				display.clear();
				display.displayOn();
				display.setTextAlignment(TEXT_ALIGN_CENTER);
				display.drawString(64, LINE1,  "START ESP TOUCH SC");
				display.drawString(64, LINE2, "on your smartphone");
				display.display();

				//Init WiFi as Station, start SmartConfig
				WiFi.mode(WIFI_AP_STA);
				WiFi.beginSmartConfig();

				//Wait for SmartConfig packet from mobile
				debug_out("Waiting for SmartConfig",						DEBUG_MIN_INFO, 1);

				while (!WiFi.smartConfigDone()) {
					Serial.print(".");


					digitalWrite(LED_BUILTIN, HIGH);    // turn the LED ON by making the voltage HIGH
					vTaskDelay(100);  // one tick delay (1ms) in between reads for stability
					digitalWrite(LED_BUILTIN, LOW );
					vTaskDelay(500);  // one tick delay (1ms) in between reads for stability


				}

				//Wait for WiFi to connect to AP
				debug_out("Waiting for WiFi",								DEBUG_MIN_INFO, 1);
				digitalWrite(LED_BUILTIN, LOW );

				while (WiFi.status() != WL_CONNECTED) {

					display.drawString(64, LINEM, "SmartConfig received");
					display.display();

					vTaskDelay(500);  // one tick delay (1ms) in between reads for stability
					Serial.print(".");


					digitalWrite(LED_BUILTIN, HIGH);    // turn the LED ON by making the voltage HIGH
					vTaskDelay(20);  // one tick delay (1ms) in between reads for stability
					digitalWrite(LED_BUILTIN, LOW );
					vTaskDelay(100);  // one tick delay (1ms) in between reads for stability


				}

				display.drawString(64, LINE3, "IP: " + WiFi.localIP().toString());
				display.drawString(64, LINE4, "restart me now");
				digitalWrite(LED_BUILTIN, LOW );


				display.display();

				while(true){
					digitalWrite(SUPPLY, LOW );
					vTaskDelay(500); // one tick delay (1ms) in between reads for stability
				}

			}
		}
	#endif

	Serial.printf("ESP: 03 Min level of free heap: %u\n", ESP.getMinFreeHeap());

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

	Serial.printf("ESP: 04 Min level of free heap: %u\n", ESP.getMinFreeHeap());

	#ifdef CFG_GPS
		initGPS();
	#endif

	Serial.printf("ESP: 05 Min level of free heap: %u\n", ESP.getMinFreeHeap());


	#ifdef CFG_SQL

		yield();

		if(!SD.begin()){
			debug_out(F("Card Mount Failed."),													DEBUG_ALWAYS, 1);
		}
		uint8_t cardType = SD.cardType();

		if(cardType == CARD_NONE){
			debug_out(F("No SD card attached."),												DEBUG_ALWAYS, 1);
		}
		else
		{
			debug_out(F("SD Card Type: "),														DEBUG_ALWAYS, 0);

			if(cardType == CARD_MMC){
				debug_out(F("MMC"),																DEBUG_ALWAYS, 1);
			} else if(cardType == CARD_SD){
				debug_out(F("SDSC"),															DEBUG_ALWAYS, 1);
			} else if(cardType == CARD_SDHC){
				debug_out(F("SDHC"),															DEBUG_ALWAYS, 1);
			} else {
				debug_out(F("UNKNOWN"),															DEBUG_ALWAYS, 1);
			}
			uint64_t cardSize = SD.cardSize() / (1024 * 1024);

			debug_out("SD Card Size: "+ String(int(cardSize)) 						+ "MB",		DEBUG_ALWAYS, 1);
			debug_out("Total space:  "+ String(int(SD.totalBytes() / (1024 * 1024)))+ "MB",		DEBUG_ALWAYS, 1);
			debug_out("Used space:   "+ String(int(SD.usedBytes() / (1024)))		+ "kB",		DEBUG_ALWAYS, 1);

			sqlite3_initialize();

			if (!db_open(DB_PATH, &db))
				{

				rc = db_exec(db, "PRAGMA page_size = 512;");
				if (rc != SQLITE_OK) {
					debug_out(F("PRAGMA page_size set failure"),								DEBUG_ERROR, 1);
				}

				rc = db_exec(db, "PRAGMA default_cache_size = 200; PRAGMA cache_size = 200;");
				if (rc != SQLITE_OK) {
					debug_out(F("PRAGMA default_cache_size set failure"),						DEBUG_ERROR, 1);
				}



				rc = db_exec(db, "CREATE TABLE IF NOT EXISTS timestamps (Id integer PRIMARY KEY, datetime integer, sendGS BOOL, sendAD BOOL);");
				if (rc != SQLITE_OK) {
					 sqlite3_close(db);
					 debug_out(F("Table 'timestamps' creation failure"),						DEBUG_ERROR, 1);
					 return;
				}

				rc = db_exec(db, "CREATE TABLE IF NOT EXISTS measBME (Id integer PRIMARY KEY, temp TEXT, press TEXT, humid TEXT);");
				if (rc != SQLITE_OK) {
					 sqlite3_close(db);
					 debug_out(F("Table measurements 'measBME' creation failure"),			DEBUG_ERROR, 1);
					 return;
				}

				rc = db_exec(db, "CREATE TABLE IF NOT EXISTS measPMS (Id integer PRIMARY KEY, PM010 TEXT, PM025 TEXT, PM100 TEXT);");
				if (rc != SQLITE_OK) {
					 sqlite3_close(db);
					 debug_out(F("Table measurements 'measPMS' creation failure"),			DEBUG_ERROR, 1);
					 return;
				}

				rc = db_exec(db, "CREATE TABLE IF NOT EXISTS measPMSE (Id integer PRIMARY KEY, PM010 TEXT, PM025 TEXT, PM100 TEXT);");
				if (rc != SQLITE_OK) {
					 sqlite3_close(db);
					 debug_out(F("Table measurements 'measPMS' creation failure"),			DEBUG_ERROR, 1);
					 return;
				}

				rc = db_exec(db, "CREATE TABLE IF NOT EXISTS measSDS (Id integer PRIMARY KEY, PM025 TEXT, PM100 TEXT);");
				if (rc != SQLITE_OK) {
					 sqlite3_close(db);
					 debug_out(F("Table measurements 'measSDS' creation failure"),			DEBUG_ERROR, 1);
					 return;
				}

				rc = db_exec(db, "CREATE TABLE IF NOT EXISTS measGPS (Id integer PRIMARY KEY, lat REAL, lon REAL);");
				if (rc != SQLITE_OK) {
					 sqlite3_close(db);
					 debug_out(F("Table measurements 'measGPS' creation failure"),			DEBUG_ERROR, 1);
					 return;
				}

				const char *sql = "SELECT COUNT(*) FROM timestamps WHERE sendGS IS NULL";
				int64_t rec_count64;

				if(!GetDB_Count( sql , rec_count64)){
					rec_count = rec_count64;
					debug_out("Count records waiting = " + String(rec_count),				DEBUG_ALWAYS, 1);
				}
				else{
					debug_out("Error getting records count",								DEBUG_ERROR, 1);
				}

				sqlite3_close(db);
			}

		}



	#endif

	Serial.printf("ESP: 06 Min level of free heap: %u\n", ESP.getMinFreeHeap());

	// Set time zone once, not in loop.
    setTimeZone(gmtOffset_sec,daylightOffset_sec);

	connectWifi();

	Serial.printf("ESP: 07 Min level of free heap: %u\n", ESP.getMinFreeHeap());

    // Now set up two tasks to run independently.
	xTaskCreatePinnedToCore(
		TaskReadSensors
		,  "ReadSDSPMS"
		,  1024*3 	// Stack size
		,  NULL
		,  2  		// Priority
		,  &xTaskReadSensors_handle
		,  ARDUINO_RUNNING_CORE);

	Serial.printf("ESP: 00 Min level of free heap: %u\n", ESP.getMinFreeHeap());

	xTaskCreatePinnedToCore(
		TaskDiagLevel
		,  "DiagLevel"
		,  1024  // Stack size
		,  NULL
		,  2  // Priority
		,  NULL
		,  ARDUINO_RUNNING_CORE);

	Serial.printf("ESP: 00 Min level of free heap: %u\n", ESP.getMinFreeHeap());

	xTaskCreatePinnedToCore(
		TaskKeyboard
		,  "Keyboard"
		,  1024  // Stack size
		,  NULL
		,  3  // Priority
		,  NULL
		,  ARDUINO_RUNNING_CORE);

	Serial.printf("ESP: 00 Min level of free heap: %u\n", ESP.getMinFreeHeap());

	xTaskCreatePinnedToCore(
		TaskArchiveMeas
		,  "CyclicArcive"
		,  1024*5 // Stack size
		,  NULL
		,  1  // Priority
		,  &xTaskArchiveMeas_handle
		,  ARDUINO_RUNNING_CORE);

	Serial.printf("ESP: 00 Min level of free heap: %u\n", ESP.getMinFreeHeap());

	xTaskCreatePinnedToCore(
		TaskDisplay
		,  "Display"
		,  1024*4  // Stack size
		,  NULL
		,  3  // Priority
		,  &xTaskDisplay_handle
		,  ARDUINO_RUNNING_CORE);

	Serial.printf("ESP: 00 Min level of free heap: %u\n", ESP.getMinFreeHeap());

	xTaskCreatePinnedToCore(
		TaskBlink
		,  "Blink"   // A name just for humans
		,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
		,  NULL
		,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
		,  NULL
		,  SECOND_CORE);	//SECOND_CORE);

	Serial.printf("ESP: 00 Min level of free heap: %u\n", ESP.getMinFreeHeap());

	xTaskCreatePinnedToCore(
		TaskWiFi
		,  "WiFi_reconnect"   // A name just for humans
		,  1024*2  	// This stack size can be checked & adjusted by reading the Stack Highwater
		,  NULL
		,  1  		// Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
		,  NULL
		,  SECOND_CORE);	//SECOND_CORE);

	Serial.printf("ESP: 00 Min level of free heap: %u\n", ESP.getMinFreeHeap());

/*	xTaskCreatePinnedToCore(
		TaskPush2WWW
		,  "Internet"   // A name just for humans
		,  1024*90  // This stack size can be checked & adjusted by reading the Stack Highwater
		,  NULL
		,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
		,  NULL
		,  SECOND_CORE);	//SECOND_CORE);

	Serial.printf("ESP: 00 Min level of free heap: %u\n", ESP.getMinFreeHeap());
*/

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.

	cfg::debug 		= DEBUG;

}

void loop()
{
#ifdef CFG_GSHEET
#ifdef CFG_SQL

	String  sql				= "";
	String  data			= "";
	String  dataOSM			= "";

	String  RowID			= "";
	String  TEMP1 			= "";
	String  TEMP2 			= "";

	int64_t	rec_count64		= 0;


	xSemaphoreTake(SQL_mutex, portMAX_DELAY);
	data	="";
	dataOSM ="";

	if (WiFi.isConnected()) {

		bool	GSdata			= false;
		bool	OSdata			= false;
		bool	GSsavedone		= false;
		bool	OSsavedone		= false;

		String BME280_P;
		String BME280_T;
		String BME280_H;

		String SDS_100;
		String SDS_025;

		String PMS_100;
		String PMS_025;
		String PMS_010;


		if (!db_open(DB_PATH, &db)){

			// Check if anything in database to be sent?
			sql = F("SELECT COUNT(*) FROM timestamps WHERE sendGS IS NULL OR sendAD IS NULL");

			if(!GetDB_Count(sql.c_str(), rec_count64)){
				rec_count = rec_count64;
				debug_out("WWW: Count measSDS records = " + String(rec_count),									DEBUG_MIN_INFO, 1);
				if(rec_count){

					String DateTime 	= "";

					String MeasSDS		= "";
					String MeasPMS		= "";
					String MeasGPS		= "";
					String MeasBME		= "";

					sql = F("SELECT datetime, Id, sendGS, sendAD FROM timestamps WHERE sendGS IS NULL OR sendAD IS NULL ORDER BY datetime ASC LIMIT 1");
					GetDB_Data(sql.c_str(), DateTime	, TEMP1);

					debug_out("WWW: From timestamps DB: TEMP1= " + TEMP1,														DEBUG_ALWAYS, 1);

					RowID			=	StrSplitItem(TEMP1, ',', 1);
					GSdata			=	StrSplitItem(TEMP1, ',', 2).length() == 0;
					OSdata			=	StrSplitItem(TEMP1, ',', 3).length() == 0;

					RowID.trim();

					debug_out("WWW: From timestamps DB: RowID= " + RowID + "; DateTime= " + DateTime,			DEBUG_MED_INFO, 1);

					if(RowID){

						sql = "SELECT Id, PM100, PM025        FROM measSDS WHERE Id='" + RowID + "' LIMIT 1";
						GetDB_Data(sql.c_str(), TEMP1	, MeasSDS);

						sql = "SELECT Id, PM100, PM025, PM010 FROM measPMS WHERE Id='" + RowID + "' LIMIT 1";
						GetDB_Data(sql.c_str(), TEMP1	, MeasPMS);

						sql = "SELECT Id, temp, humid, press  FROM measBME WHERE Id='" + RowID + "' LIMIT 1";
						GetDB_Data(sql.c_str(), TEMP1	, MeasBME);

#ifdef CFG_GPS
						sql = "SELECT Id, lat, lon            FROM measGPS WHERE Id='" + RowID + "' LIMIT 1";
						GetDB_Data(sql.c_str(), TEMP1	, MeasGPS);
#endif

						debug_out(("WWW: From measSDS DB: ") + MeasSDS,											DEBUG_MED_INFO, 1);
						debug_out(("WWW: From measPMS DB: ") + MeasPMS,											DEBUG_MED_INFO, 1);
						debug_out(("WWW: From measBME DB: ") + MeasBME,											DEBUG_MED_INFO, 1);
						debug_out(("WWW: From measGPS DB: ") + MeasGPS,											DEBUG_MED_INFO, 1);

						vTaskDelay(100);  // one tick delay (1ms) in between reads for stability

						String GPS_lat 		= 	StrSplitItem(MeasGPS, ',', 1);
						String GPS_lon 		= 	StrSplitItem(MeasGPS, ',', 2);

						BME280_P			=	StrSplitItem(MeasBME, ',', 3);
						BME280_T			=	StrSplitItem(MeasBME, ',', 1);
						BME280_H			=	StrSplitItem(MeasBME, ',', 2);

						SDS_100				=	StrSplitItem(MeasSDS, ',', 1);
						SDS_025				=	StrSplitItem(MeasSDS, ',', 2);

						PMS_100				=	StrSplitItem(MeasPMS, ',', 1);
						PMS_025				=	StrSplitItem(MeasPMS, ',', 2);
						PMS_010				=	StrSplitItem(MeasPMS, ',', 3);


						if(GSdata){
							// GPS data
							debug_out("WWW: Prepare JSON.",															DEBUG_MED_INFO, 1);

							data = FPSTR(data_first_part);
							data.replace("{v}", SOFTWARE_VERSION);

							data += Var2Json(F("datetime"),						DateTime);
							data += Var2Json(F("GPS_lat"),						GPS_lat);
							data += Var2Json(F("GPS_lon"),						GPS_lon);

							data += Var2Json(F("BME280_pressure"),				BME280_P);
							data += Var2Json(F("BME280_temperature"), 			BME280_T);
							data += Var2Json(F("BME280_humidity"), 				BME280_H);

							data += Var2Json(F("SDS_P1"),						SDS_100); // PM10.0
							data += Var2Json(F("SDS_P2"),						SDS_025); // PM 2.5

							data += Var2Json(F("PMS_P1"),						PMS_100); // PM10.0
							data += Var2Json(F("PMS_P2"),						PMS_025); // PM 2.5
							data += Var2Json(F("PMS_P3"),						PMS_010); // PM 1.0

							data += "]}";

							// prepare fo gscript
							data.remove(0, 1);
							data = "{\"espid\": \"" + esp_chipid + "\"," + data + "}";
							data.replace("\"sensordatavalues\":[", "\"sensordatavalues\":{");
							data.replace("}","");
							data.replace("]","");
							data += "}}}";
							data.replace(",}}}","}}}");

							data = payload_base + data;

							debug_out(F("WWW: Send from buffer to spreadsheet. Payload prepared:"), 				DEBUG_MED_INFO, 1);
							debug_out(data, 																		DEBUG_MED_INFO, 1);
						}
						if(OSdata){
							if(GPS_lat.length() > 3 && GPS_lon.length()>3){

								time_t TStamp;
								TStamp = DateTime.toInt();

								TStamp -= 3600*3;
								// TODO!! TStamp correction should depends on dayLight save period

								struct tm timeinfo;
								localtime_r(&TStamp, &timeinfo);
								if(timeinfo.tm_year > (2016 - 1900)){
									char timeStringBuff[50]; //50 chars should be enough
									strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);	//"%04d-%02d-%02dT%02d:%02d:%02dZ"

									String TimeStamp(timeStringBuff);

									//TimeStamp = TimeStamp.substring(0, TimeStamp.length() - 2);
									//TimeStamp += ":00";


									// Take out AVG values
									BME280_P			=	StrSplitItem(BME280_P, ':', 2);
									BME280_T			=	StrSplitItem(BME280_T, ':', 2);
									BME280_H			=	StrSplitItem(BME280_H, ':', 2);

									SDS_100				=	StrSplitItem(SDS_100, ':', 2);
									SDS_025				=	StrSplitItem(SDS_025, ':', 2);

									PMS_100				=	StrSplitItem(PMS_100, ':', 2);
									PMS_025				=	StrSplitItem(PMS_025, ':', 2);
									PMS_010				=	StrSplitItem(PMS_010, ':', 2);

									// Prepare JSON packages
									SDS_100  = ValueLocated2Json(TimeStamp, GPS_lat, GPS_lon, SDS_100);
									SDS_025  = ValueLocated2Json(TimeStamp, GPS_lat, GPS_lon, SDS_025);
									PMS_100  = ValueLocated2Json(TimeStamp, GPS_lat, GPS_lon, PMS_100);
									PMS_025  = ValueLocated2Json(TimeStamp, GPS_lat, GPS_lon, PMS_025);
									PMS_010  = ValueLocated2Json(TimeStamp, GPS_lat, GPS_lon, PMS_010);
									BME280_P = ValueLocated2Json(TimeStamp, GPS_lat, GPS_lon, BME280_P);
									BME280_T = ValueLocated2Json(TimeStamp, GPS_lat, GPS_lon, BME280_T);
									BME280_H = ValueLocated2Json(TimeStamp, GPS_lat, GPS_lon, BME280_H);

									TEMP2	= "{\"sensor\":\"" + String(ID_SENSOR_SDS_100) + "\" , ";
									SDS_100.replace("{", TEMP2);

									TEMP2	= "{\"sensor\":\"" + String(ID_SENSOR_SDS_025) + "\" , ";
									SDS_025.replace("{", TEMP2);

									TEMP2	= "{\"sensor\":\"" + String(ID_SENSOR_PMS_100) + "\" , ";
									PMS_100.replace("{", TEMP2);

									TEMP2	= "{\"sensor\":\"" + String(ID_SENSOR_PMS_025) + "\" , ";
									PMS_025.replace("{", TEMP2);

									TEMP2	= "{\"sensor\":\"" + String(ID_SENSOR_PRESS) + "\" , ";
									BME280_P.replace("{", TEMP2);

									TEMP2	= "{\"sensor\":\"" + String(ID_SENSOR_TEMP) + "\" , ";
									BME280_T.replace("{", TEMP2);

									TEMP2	= "{\"sensor\":\"" + String(ID_SENSOR_HUMID) + "\" , ";
									BME280_H.replace("{", TEMP2);

									dataOSM  = "[";
									dataOSM +=  SDS_100 + ",";
									dataOSM +=  SDS_025 + ",";
									dataOSM +=  PMS_100 + ",";
									dataOSM +=  PMS_025 + ",";
									dataOSM += BME280_P + ",";
									dataOSM += BME280_T + ",";
									dataOSM += BME280_H;
									dataOSM += "]";

								}
								else{
									OSsavedone = true; // no need to transfer data, just mark it out as done
								}
							}
							else {
								OSsavedone = true; // no need to transfer data, just mark it out as done
							}
						}
					}
				}
			}
		}

		sqlite3_close(db);

		vTaskDelay(50);  // one tick delay (1ms) in between reads for stability


		if(GSdata){

			// Connect to spreadsheet

			client = new HTTPSRedirect(httpsPort);
			client->setPrintResponseBody(false);
			client->setContentTypeHeader("application/json");

			vTaskDelay(50);  // one tick delay (1ms) in between reads for stability
			debug_out(F("WWW: Client object created"), 											DEBUG_MED_INFO, 1);

			if (client != nullptr){
				if (!client->connected()){

					// Try to connect for a maximum of 1 times
					for (int i=0; i<1; i++){

						debug_out(F("WWW: Calling Client->connect"), 							DEBUG_MED_INFO, 1);
						vTaskDelay(50);

						int retval = client->connect(host, httpsPort);
						if (retval == 1) {
							 break;
						}
						else {
							debug_out(F("Connection failed. Retrying..."), 						DEBUG_WARNING, 1);
							vTaskDelay(50);
							Serial.println(client->getResponseBody() );
						}
					}
				}
			}
			else{
				debug_out(F("Error creating client object!"), 									DEBUG_ERROR, 1);
			}
			if (!client->connected()){
				debug_out(F("Connection failed. Stand by till next period"), 					DEBUG_ERROR, 1);
			}
			else
			{
				debug_out(F("WWW: Client object requests to Spreadsheet"), 						DEBUG_MED_INFO, 1);

				if(client->POST(url2, host, data)){
					debug_out(F("Spreadsheet updated"), DEBUG_MIN_INFO, 1);
					GSsavedone	=	 true;
				}
				else{
					debug_out(F("Spreadsheet update fails: "), DEBUG_MIN_INFO, 1);
				}
			}

			// delete HTTPSRedirect object
			delete client;
			client = nullptr;

			debug_out(F("WWW: Client object deleted"), 											DEBUG_MED_INFO, 1);
		}
		if(OSdata && !OSsavedone){
			debug_out(F("OsemApi: Transfer start"), 											DEBUG_MED_INFO, 1);

			// Send to opensensemap
			OsemApi api = OsemApi();

			debug_out("OsemApi: dataOSM=" + dataOSM, 											DEBUG_MED_INFO, 1);

//			OSsavedone = true;
//			OSsavedone &= api.postMeasurement(BME280_T, 	ID_SENSOR_TEMP);
//			OSsavedone &= api.postMeasurement(BME280_P, 	ID_SENSOR_PRESS);
//			OSsavedone &= api.postMeasurement(BME280_H, 	ID_SENSOR_HUMID);
//			OSsavedone &= api.postMeasurement(PMS_100, 		ID_SENSOR_PMS_100);
//			OSsavedone &= api.postMeasurement(PMS_025, 		ID_SENSOR_PMS_025);
//			OSsavedone &= api.postMeasurement(SDS_100, 		ID_SENSOR_SDS_100);
//			OSsavedone &= api.postMeasurement(SDS_025, 		ID_SENSOR_SDS_025);

			OSsavedone  = api.postMeasurement(dataOSM);


			if(!OSsavedone){
				debug_out(F("OsemApi: data transfer error"), 									DEBUG_ERROR, 1);
			}
			else{
				debug_out(F("OsemApi: transfer OK"),		 									DEBUG_MIN_INFO, 1);
			}

		}

		if(GSsavedone || OSsavedone){

			// Data sent successfully. Remove record from DB
			if (!db_open(DB_PATH, &db)){

				sql = "UPDATE timestamps SET ";

				if(GSsavedone){
					sql += "sendGS = 1";
					if(OSsavedone){
						sql += ", ";
					}
				}
				if(OSsavedone){
					sql += "sendAD = 1";
				}
				sql += " WHERE Id='" + RowID + "';";

				debug_out("SQL UPDATE: " + sql, 											DEBUG_MIN_INFO, 1);

				GetDB_Data(sql.c_str(), TEMP1	, TEMP2);		// Mark record as sended
				debug_out(F("Spreadsheet updated successfully. Data row marked"), 			DEBUG_MED_INFO, 1);

			}
			sqlite3_close(db);
		}
	}
	else{
		debug_out("WWW: WiFi not connected. Cycle skipped",										DEBUG_ERROR, 1);
	}


	xSemaphoreGive(SQL_mutex);

#endif
#endif


	vTaskDelay(6000);  // one tick delay (1ms) in between reads for stability
}


bool GetDB_Count(const char *sql, int64_t &count){

	bool result = true;

	// get actual number of records to variable
	if (sqlite3_prepare_v2(db, sql, -1, &res, NULL) != SQLITE_OK) {
		String resp = "Failed to fetch data: ";
		resp += sqlite3_errmsg(db);
		debug_out(resp,																			DEBUG_ERROR, 1);
		sqlite3_finalize(res);
	}
	else {
		if (sqlite3_step(res) != SQLITE_ROW) {
			String resp = "Step failure: ";
			resp += sqlite3_errmsg(db);
			debug_out(resp,																		DEBUG_ERROR, 1);
			sqlite3_finalize(res);
		}
		else {
			count = sqlite3_column_int64(res, 0);
			result = false;
			debug_out("GetDB_Count returns: " + String((int)count),								DEBUG_MED_INFO, 1);

			sqlite3_finalize(res);
		}
	}

	return result;
}

bool GetDB_Data(const char *sql, String &firstCol, String &otherCol){

	bool result = true;

	if (sqlite3_prepare_v2(db, sql, -1, &res, NULL) != SQLITE_OK) {
		String resp = "Failed to fetch data: ";
		resp += sqlite3_errmsg(db);
		debug_out(resp,																			DEBUG_ERROR, 1);
		sqlite3_finalize(res);
	}
	else {
		if (sqlite3_step(res) != SQLITE_ROW) {
			String resp = "Step failure: ";
			resp += sqlite3_errmsg(db);
			debug_out(resp,																		DEBUG_MED_INFO, 1);
			sqlite3_finalize(res);
		}
		else {

			firstCol = sqlite3_column_int(res, 0);

			otherCol  = "";
			otherCol += String((char*)sqlite3_column_text(res, 1)) + ",";
			otherCol += String((char*)sqlite3_column_text(res, 2)) + ",";
			otherCol += String((char*)sqlite3_column_text(res, 3)) + ",";
			otherCol += String((char*)sqlite3_column_text(res, 4));

			result = false;
			sqlite3_finalize(res);
		}
	}

	return result;
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

  for (;;){ // A Task shall never return or exit.

	uxHighWaterMark_TaskBlink = uxTaskGetStackHighWaterMark( NULL );

	// Mark task begin by LED

	digitalWrite(LED_BUILTIN, HIGH);   	// turn the LED on (HIGH is the voltage level)
    vTaskDelay(  1);  					// one tick delay (1ms) in between reads for stability
    digitalWrite(LED_BUILTIN, LOW);    	// turn the LED off by making the voltage LOW
    vTaskDelay(100);  					// one tick delay (1ms) in between reads for stability

	if (WiFi.isConnected()) {
		digitalWrite(LED_BUILTIN, HIGH);   	// turn the LED on (HIGH is the voltage level)
		vTaskDelay(  1);  					// one tick delay (1ms) in between reads for stability
		digitalWrite(LED_BUILTIN, LOW);    	// turn the LED off by making the voltage LOW
	}
    vTaskDelay(100);  					// one tick delay (1ms) in between reads for stability



    if(SDSmeasPM025.status == SensorSt::ok && PMSmeasPM025.status == SensorSt::ok){
    	if(BUT_DB_CLEAR_FLAG){
    		vTaskDelay(100 );  // one tick delay (1ms) in between reads for stability
    	}
    	else{
    		vTaskDelay(2000);  // one tick delay (1ms) in between reads for stability
    	}
    }
    else{
    	vTaskDelay(500);  // one tick delay (1ms) in between reads for stability
    }

  }
  vTaskDelete( NULL );
}





void TaskDiagLevel(void *pvParameters)  // This is a task.
{
	(void) pvParameters;

	for (;;){ // A Task shall never return or exit.
		xSemaphoreTake(Serial_mutex, portMAX_DELAY);
		if (Serial.available()){ //Check if we receive anything from Bluetooth
			int incoming;
			incoming = Serial.read(); //Read what we receive

			if (incoming >= 49 && incoming <= 53 ){
				incoming -= 48;
				cfg::debug = incoming;
			}
		}
		xSemaphoreGive(Serial_mutex);

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

		  debug_out(F("ARCH"), DEBUG_MIN_INFO, 1);

		  SDSmeasPM025.ArchPush();
		  SDSmeasPM100.ArchPush();
		  PMSmeasPM010.ArchPush();
		  PMSmeasPM025.ArchPush();
		  PMSmeasPM100.ArchPush();

		  PMSEmeasPM010.ArchPush();
		  PMSEmeasPM025.ArchPush();
		  PMSEmeasPM100.ArchPush();

		  BMEmeasH.ArchPush();
		  BMEmeasT.ArchPush();
		  BMEmeasP.ArchPush();

		  time(&now);
		  if(now > 1577836800){					// check if time was set
			  Store2DB();						// no reason to store values without timestamp

			  if(Mode == RecMode::NoGPS_Slow){
				  if(SDSmeasPM025.ArchMeas.avg[1]>-1.0){
						// Two measurements recorded
						// go to sleep till next cycle

						debug_out(F("Stand by: Sleep till next cycle"), 					DEBUG_MIN_INFO, 1);


						vTaskDelete(xTaskReadSensors_handle);
						vTaskDelete(xTaskDisplay_handle);


						WiFi.disconnect(true);
						WiFi.mode(WIFI_OFF);
						btStop();
						//	esp_wifi_stop(); // fails

						SDS_cmd(&serialSDS, PmSensorCmd::Stop);
						PMS_cmd(&serialPMS, PmSensorCmd::Stop);
						PMS_cmd(&serialPMS_EXT, PmSensorCmd::Stop);

						display.displayOff();

						// Try to switch off GPS
						// https://github.com/JuniorIOT/GPS-Lora-Balloon-rfm95-TinyGPS/blob/master/Balloon-rfm95/Balloon-rfm95.ino
						gps_SetMode_gpsOff();

						// Configure the timer to wake us up!
						esp_sleep_enable_timer_wakeup(15 * 60L * 1000000L);

						vTaskDelay(2000);  // one tick delay (1ms) in between reads for stability
						// Go to sleep! Zzzz
						esp_deep_sleep_start();

				  }
			  }
		  }
		  else{
			  debug_out(F("Time was not set. No DB push"), 									DEBUG_WARNING, 1);
		  }
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
			debug_out(F("DISPLAY: refresh by Notification"), 				DEBUG_WARNING, 1);
		}
		else
		{
			/* The call to ulTaskNotifyTake() timed out. */
			debug_out(F("DISPLAY: refresh by timeout"), 					DEBUG_MAX_INFO, 1);
		}

		vTaskDelay(100);  // one tick delay (1ms) in between reads for stability

		uxHighWaterMark_TaskDisplay = uxTaskGetStackHighWaterMark( NULL );
	}
	vTaskDelete( NULL );
}



void TaskWiFi(void *pvParameters)  // This is a task.
{
	(void) pvParameters;

	for (;;) // A Task shall never return or exit.
	{

		if(Mode != RecMode::NoGPS_Slow){

			xSemaphoreTake(SQL_mutex, portMAX_DELAY);

			if (!WiFi.isConnected()) {

				// Attempt to reconnect
				waitForWifiToConnect(6);

				if (!WiFi.isConnected()) {

					debug_out(F("WiFi in cycle unable to reconnect"), 			DEBUG_ERROR, 1);
				}
			}
			else {
				debug_out(F("WiFi connected at IP address: "), 					DEBUG_MED_INFO, 0);
				debug_out(WiFi.localIP().toString(), 							DEBUG_MED_INFO, 1);

				//init and get the time
				configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

				time(&now);
				debug_out("UNIX time: " + String(now), DEBUG_ALWAYS, 1);

				//printLocalTime();
			}
			xSemaphoreGive(SQL_mutex);

		}
	    // Wait for the next cycle.
		vTaskDelay(5000);  // one tick delay (1ms) in between reads for stability

		uxHighWaterMark_TaskWiFi = uxTaskGetStackHighWaterMark( NULL );
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
		xTaskNotifyGive(xTaskDisplay_handle);
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

	if(BUT_A_PRESS||BUT_B_PRESS||BUT_C_PRESS){
		AnyButtonPressed++;
	}
	else{
		AnyButtonPressed = 0;
	}
	if(AnyButtonPressed > xFrequency*2){
		digitalWrite(SUPPLY, LOW);
	}
	else{
		digitalWrite(SUPPLY, HIGH);
	}



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
	sensorPMS(&serialPMS, &PMSmeasPM010, &PMSmeasPM025, &PMSmeasPM100);
    vTaskDelay(100);  // one tick delay (1ms) in between reads for stability

    sensorPMS(&serialPMS_EXT, &PMSEmeasPM010, &PMSEmeasPM025, &PMSEmeasPM100);
    vTaskDelay(100);  // one tick delay (1ms) in between reads for stability

    sensorSDS(&serialSDS, &PMSmeasPM025, &PMSmeasPM100);
    vTaskDelay(100);  // one tick delay (1ms) in between reads for stability
    sensorBME280();
    vTaskDelay(150);  // one tick delay (1ms) in between reads for stability

#ifdef CFG_GPS
    sensorGPS();
    vTaskDelay(150);  // one tick delay (1ms) in between reads for stability
#endif

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

	int screens[9];
	int screen_count = 0;

	int AQI_value=0;

	screens[screen_count++] = 9;		// Record mode select screen

	if (cfg::pms_read || cfg::sds_read ) {
		screens[screen_count++] = 1;
	}

	if (cfg::bme280_read) {
		screens[screen_count++] = 3;
	}

	#ifdef CFG_GPS
	if (cfg::gps_read) {
		screens[screen_count++] = 4;
	}
	#endif

	screens[screen_count++] = 5;	// Wifi info
	screens[screen_count++] = 6;	// chipID, firmware and count of measurements
	screens[screen_count++] = 11;	// Trend, GPS, Values

	if(next_display_count<0){		// Fix bug with previous display of 0 screen
		next_display_count = screen_count-1;
	}


	xSemaphoreTake(I2C_mutex, portMAX_DELAY);
	display.clear();
	display.displayOn();

	switch (screens[next_display_count % screen_count]) {

	case (1):


		display_header =  F("PM (µg/m³)");

		display.setTextAlignment(TEXT_ALIGN_RIGHT);
		display.drawString(50, LINE1, "PM 0.1:");
		display.drawString(50, LINE2, "PM 2.5:");
		display.drawString(50, LINE3, "PM 10.0:");
		display.drawString(50, LINE4, "AQI:");

		display.setTextAlignment(TEXT_ALIGN_LEFT);
		display.drawString(55, LINE1, check_display_value(PMSmeasPM010.ArchMeas.avg[0], -1.0, 1, 6));
		display.drawString(55, LINE2, check_display_value(PMSmeasPM025.ArchMeas.avg[0], -1.0, 1, 6));
		display.drawString(55, LINE3, check_display_value(PMSmeasPM100.ArchMeas.avg[0], -1.0, 1, 6));

		display.drawString(85, LINE1, ";");
		display.drawString(85, LINE2, ";");
		display.drawString(85, LINE3, ";");

		display.drawString(90, LINE2, check_display_value(SDSmeasPM025.ArchMeas.avg[0], -1.0, 1, 6));
		display.drawString(90, LINE3, check_display_value(SDSmeasPM100.ArchMeas.avg[0], -1.0, 1, 6));

		AQI_value = (int)max(getAQI( false, SDSmeasPM025.ArchMeas.avg[0] ),getAQI( true, SDSmeasPM100.ArchMeas.avg[0] ));
		display.drawString(55, LINE4, String(AQI_value) + " (" + updateAQIDisplay(AQI_value) + ")");

		break;

	case (3):
		display_header = F("Air");

		display.setTextAlignment(TEXT_ALIGN_RIGHT);
		display.drawString(40, LINE1, "Temp.:");
		display.drawString(40, LINE2, "Hum.:");
		display.drawString(40, LINE3, "Pres.:");

		display.drawString(95, LINE1, check_display_value(BMEmeasT.ArchMeas.avg[0] , -1.0				, 1, 6));
		display.drawString(95, LINE2, check_display_value(BMEmeasH.ArchMeas.avg[0] , -1.0				, 1, 6));
		display.drawString(95, LINE3, check_display_value(BMEmeasP.ArchMeas.avg[0]  / 100, (-1 / 100.0), 1, 6));

		display.setTextAlignment(TEXT_ALIGN_LEFT);
		display.drawString(100, LINE1, "°C");
		display.drawString(100, LINE2, "%");
		display.drawString(100, LINE3, "hPa");

		break;

	case (4):
		display_header = F("GPS");

		display.setTextAlignment(TEXT_ALIGN_RIGHT);
		display.drawString(40, LINE1, "Lat:");
		display.drawString(40, LINE2, "Lon:");
		display.drawString(40, LINE3, "Alt:");

		display.drawString(100, LINE1, check_display_value(last_value_GPS_lat , GPS_UNDEF, 6, 10));
		display.drawString(100, LINE2, check_display_value(last_value_GPS_lon , GPS_UNDEF, 6, 10));
		display.drawString(100, LINE3, check_display_value(last_value_GPS_alt , GPS_UNDEF, 2, 10));

		break;

	case (5):
		display_header = F("Stack free");

		display.setTextAlignment(TEXT_ALIGN_RIGHT);
		display.drawString(25, LINE1, "Blink");
		display.drawString(25, LINE2, "Diag");
		display.drawString(25, LINE3, "Arch");
		display.drawString(25, LINE4, "WiFi");

		display.drawString(60, LINE1, check_display_value(uxHighWaterMark_TaskBlink		, 0, 0, 6)+";");
		display.drawString(60, LINE2, check_display_value(uxHighWaterMark_TaskDiagLevel	, 0, 0, 6)+";");
		display.drawString(60, LINE3, check_display_value(uxHighWaterMark_TaskArchiveMeas	, 0, 0, 6)+";");
		display.drawString(60, LINE4, check_display_value(uxHighWaterMark_TaskWiFi			, 0, 0, 6)+";");

		display.drawString(90, LINE1, "Sens");
		display.drawString(90, LINE2, "Keyb");
		display.drawString(90, LINE3, "Disp");
		display.drawString(90, LINE4, "");

		display.drawString(125, LINE1, check_display_value(uxHighWaterMark_TaskReadSensors	, 0, 0, 6));
		display.drawString(125, LINE2, check_display_value(uxHighWaterMark_TaskKeyboard	, 0, 0, 6));
		display.drawString(125, LINE3, check_display_value(uxHighWaterMark_TaskDisplay		, 0, 0, 6));

		break;
	case (6):


		display_header = F("Info [M=CLR]");

		display.setTextAlignment(TEXT_ALIGN_RIGHT);
		display.drawString(30, LINE1, "Recs:");
		display.drawString(90, LINE1, "RID:");
		display.drawString(30, LINE2, "SSID:");
		display.drawString(30, LINE3, "IP:");

		display.setTextAlignment(TEXT_ALIGN_LEFT);
		display.drawString(40, LINE1, String(rec_count));
		display.drawString(100,LINE1, String((int)RID));

		display.drawString(40, LINE2, "\"" + WiFi.SSID() + " \" (" + String(calcWiFiSignalQuality(WiFi.RSSI())) + "%)");
		display.drawString(40, LINE3, WiFi.localIP().toString());

		if(BUT_B_PRESS){
			BUT_DB_CLEAR_FLAG = true;
		}

		break;

	case (9):

		if(BUT_B_PRESS){
			//Mode++;
			Mode = (Mode == RecMode::GPS_Slow) ? RecMode::NoGPS : static_cast<RecMode>(static_cast<int>(Mode)+1);
		}

		display_header = F("Mode");

		display.setTextAlignment(TEXT_ALIGN_RIGHT);
		display.drawString(45,  LINE2, "No GPS");
		display.drawString(45,  LINE3, "GPS");

		display.setTextAlignment(TEXT_ALIGN_LEFT);
		display.drawString(50,  LINE1, "NORM    SLOW");

		display.drawString(60,  LINE2, String(Mode == RecMode::NoGPS? "O":""));
		display.drawString(60,  LINE3, String(Mode == RecMode::GPS  ? "O":""));

		display.drawString(110, LINE2, String(Mode == RecMode::NoGPS_Slow? "O":""));
		display.drawString(110, LINE3, String(Mode == RecMode::GPS_Slow  ? "O":""));

		break;

	case (11):
		display_header = "Measurements";

		break;
	}



	if(screens[next_display_count % screen_count] < 10){
		display.setTextAlignment(TEXT_ALIGN_LEFT);

		String StMode = "";
		switch (Mode) {
		case (RecMode::NoGPS):
			StMode = "N";
			break;
		case (RecMode::NoGPS_Slow):
			StMode = "N+";
			break;
		case (RecMode::GPS):
			if(last_value_GPS_lat == GPS_UNDEF){
				StMode = "G";
			}
			else{
				StMode = "[G]";
			}
			break;
		case (RecMode::GPS_Slow):
			if(last_value_GPS_lat == GPS_UNDEF){
				StMode = "G+";
			}
			else{
				StMode = "[G+]";
			}
			break;
		}

		display.drawString(100, LINEM, printLocalTime("%H:%M"));
		display.drawString(80,  LINEM, StMode);

		if(inWindow){
			display.drawString(60,  LINEM, "[H]");
		}

		display.drawString(0,  LINEM, display_header);


	}
	else{
		for(int i=1; i<display.getWidth(); i++){
			int16_t Y=0;

			Y = int(PMSmeasPM025.ArchMeas.avg[i]/4);

			Y = 64-18-Y;
			Y = (Y<1 ? 1 : Y);
			display.setPixel(i, Y);
			display.setPixel(i, 64-17);
		}

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
		debug_out(FPSTR(DBG_TXT_TEMPERATURE)	, DEBUG_MED_INFO, 0);
		debug_out(Float2String(t) + " C"		, DEBUG_MED_INFO, 1);
		debug_out(FPSTR(DBG_TXT_HUMIDITY)		, DEBUG_MED_INFO, 0);
		debug_out(Float2String(h) + " %"		, DEBUG_MED_INFO, 1);
		debug_out(FPSTR(DBG_TXT_PRESSURE)		, DEBUG_MED_INFO, 0);
		debug_out(Float2String(p / 100) + " hPa", DEBUG_MED_INFO, 1);

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

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
	for(int i=0; i<len; i++) {
		serialGPS.write(MSG[i]);
	}
}

// https://github.com/JuniorIOT/GPS-Lora-Balloon-rfm95-TinyGPS/blob/master/Balloon-rfm95/Balloon-rfm95.ino
// A lot of commands there:
void gps_SetMode_gpsOff() {

  ////Set GPS to backup mode (sets it to never wake up on its own) minimal current draw <5mA, loses all settings
  //uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
  ////Restart GPS
  //uint8_t GPSon[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};

  byte arrCommand[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
  sendUBX(arrCommand, sizeof(arrCommand)/sizeof(uint8_t));
  // after this command no gps output is available
}
void gps_SetMode_gpsOn() {

  byte arrCommand[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
  sendUBX(arrCommand, sizeof(arrCommand)/sizeof(uint8_t));
}


/*****************************************************************
 * read GPS sensor values																				*
 *****************************************************************/

void sensorGPS() {

	debug_out(String(FPSTR(DBG_TXT_START_READING)) + "GPS", DEBUG_MED_INFO, 1);

	while (serialGPS.available() > 0) {
		if (gps.encode(serialGPS.read())) {
			if (gps.location.isValid()) {
				last_value_GPS_lat = gps.location.lat();
				last_value_GPS_lon = gps.location.lng();

				inWindow  = last_value_GPS_lat < max(GPS_HOME_LAT_1,GPS_HOME_LAT_2);
				inWindow &= last_value_GPS_lat > min(GPS_HOME_LAT_1,GPS_HOME_LAT_2);
				inWindow &= last_value_GPS_lon < max(GPS_HOME_LON_1,GPS_HOME_LON_2);
				inWindow &= last_value_GPS_lon > min(GPS_HOME_LON_1,GPS_HOME_LON_2);

			} else {
				last_value_GPS_lat = GPS_UNDEF;
				last_value_GPS_lon = GPS_UNDEF;
				debug_out(F("Lat/Lng INVALID"), DEBUG_MAX_INFO, 1);

				inWindow = false;


			}
			if (gps.altitude.isValid()) {
				last_value_GPS_alt = gps.altitude.meters();
				String gps_alt = Float2String(last_value_GPS_alt, 2);
			} else {
				last_value_GPS_alt = GPS_UNDEF;
				debug_out(F("Altitude INVALID"), DEBUG_MAX_INFO, 1);
			}
		}
	}

	if(gps.location.age()>5000){

		last_value_GPS_lat = GPS_UNDEF;
		last_value_GPS_lon = GPS_UNDEF;
		inWindow = false;

	}


	// gps.time.hour() resets Updated flag!
	if(gps.time.isUpdated() && gps.time.isValid()){

		  if(gps.date.year() > 2020){									// check if time was set from satelite

				// Set time from GPS
				time_t t_of_day;
				struct tm t;

				timeval epoch;
				const timeval *tv = &epoch;

				timezone utc = {0, 0};									// {gmtOffset_sec/60, daylightOffset_sec/60};
				const timezone *tz = &utc;

				t.tm_year = gps.date.year()  - 1900;
				t.tm_mon  = gps.date.month() - 1;   					// Month, 0 - jan
				t.tm_mday = gps.date.day();         					// Day of the month

				t.tm_hour = gps.time.hour() + gmtOffset_sec/3600; 		// somewhy not timezone utc nor setTimeZone works, always +0 hours

				t.tm_min  = gps.time.minute();
				t.tm_sec  = gps.time.second();

				debug_out("GPS Time: " + String(gps.time.hour())+":" + String(gps.time.minute()),	DEBUG_MED_INFO, 1);

				t_of_day  = mktime(&t);

				epoch = {t_of_day, 0};

				settimeofday(tv, tz);

				debug_out(F("GPS: GPS Time set"), 													DEBUG_MED_INFO, 1);

		  }
		  else
		  {
				debug_out(F("GPS: GPS Time was not receieved from SAT yet"), 						DEBUG_MED_INFO, 1);
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
	char *zErrMsg = 0;

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
				 debug_out(F("Can't open database: "), 			DEBUG_ERROR, 0);
				 debug_out(String(sqlite3_errmsg(*db)), 		DEBUG_ERROR, 1);
				 return rc;
		 } else {
				 debug_out(F("Opened database successfully"), 	DEBUG_MED_INFO, 1);
		 }
		 return rc;
	}

	int db_exec(sqlite3 *db, const char *sql) {
		 debug_out("db_exec: " + String(sql), 					DEBUG_MED_INFO, 1);

		 int rc = sqlite3_exec(db, sql, callback, (void*)data, &zErrMsg);
		 if (rc != SQLITE_OK) {
				 debug_out(F("SQL error: "), 					DEBUG_ERROR, 0);
				 debug_out(String(zErrMsg), 					DEBUG_ERROR, 1);
				 sqlite3_free(zErrMsg);
		 } else {
				 debug_out(F("Operation done successfully"), 	DEBUG_MED_INFO, 1);
		 }

		 return rc;
	}


void Store2DB(){

	xSemaphoreTake(SQL_mutex, portMAX_DELAY);

	if (!db_open(DB_PATH, &db))
	{
		/*
			"CREATE TABLE IF NOT EXISTS timestamps (Id integer PRIMARY KEY, datetime integer, sendGS BOOL, sendAD BOOL);");
			"CREATE TABLE IF NOT EXISTS measBME    (Id integer PRIMARY KEY, temp REAL, press REAL, humid REAL);");
			"CREATE TABLE IF NOT EXISTS measPMS    (Id integer PRIMARY KEY, PM010 REAL, PM025 REAL, PM100 REAL);");
			"CREATE TABLE IF NOT EXISTS measSDS    (Id integer PRIMARY KEY, PM025 REAL, PM100 REAL);");
			"CREATE TABLE IF NOT EXISTS measGPS    (Id integer PRIMARY KEY, lat REAL, lon REAL);");
		*/

		String query = "";

		time(&now);

		query  = "INSERT INTO timestamps (datetime) VALUES ('" + String(now) + "')";
		rc = db_exec(db, query.c_str());
		if (rc != SQLITE_OK) {
			debug_out(F("Table 'timestamps' not updated"),			DEBUG_ERROR, 1);
		}
		else
		{

			query = "SELECT last_insert_rowid();";
			if(!GetDB_Count(query.c_str(), RID)){

				query  = "INSERT INTO measBME (Id, temp, press, humid) VALUES ('" + String((int)RID) + "',";
				query += "'" + Float2String(BMEmeasT.ArchMeas.min[0]) 		+":" + Float2String(BMEmeasT.ArchMeas.avg[0]) 		+ ":" + Float2String(BMEmeasT.ArchMeas.max[0]) + "',";
				query += "'" + Float2String(BMEmeasP.ArchMeas.min[0]) 		+":" + Float2String(BMEmeasP.ArchMeas.avg[0]) 		+ ":" + Float2String(BMEmeasP.ArchMeas.max[0]) + "',";
				query += "'" + Float2String(BMEmeasH.ArchMeas.min[0]) 		+":" + Float2String(BMEmeasH.ArchMeas.avg[0]) 		+ ":" + Float2String(BMEmeasH.ArchMeas.max[0]) + "')";

				rc = db_exec(db, query.c_str());
				if (rc != SQLITE_OK) {
					debug_out(F("Table 'measBME' not updated"),			DEBUG_ERROR, 1);
				}

				query  = "INSERT INTO measSDS (Id, PM025, PM100) VALUES ('" + String((int)RID) + "',";
				query += "'" + Float2String(SDSmeasPM025.ArchMeas.min[0]) 	+":" + Float2String(SDSmeasPM025.ArchMeas.avg[0]) 	+ ":" + Float2String(SDSmeasPM025.ArchMeas.max[0]) + "',";
				query += "'" + Float2String(SDSmeasPM100.ArchMeas.min[0]) 	+":" + Float2String(SDSmeasPM100.ArchMeas.avg[0]) 	+ ":" + Float2String(SDSmeasPM100.ArchMeas.max[0]) + "')";

				rc = db_exec(db, query.c_str());
				if (rc != SQLITE_OK) {
					debug_out(F("Table 'measSDS' not updated"),			DEBUG_ERROR, 1);
				}

				query  = "INSERT INTO measPMS (Id, PM010, PM025, PM100) VALUES ('" + String((int)RID) + "',";
				query += "'" + Float2String(PMSmeasPM010.ArchMeas.min[0]) 	+":" + Float2String(PMSmeasPM010.ArchMeas.avg[0]) 	+ ":" + Float2String(PMSmeasPM010.ArchMeas.max[0]) + "',";
				query += "'" + Float2String(PMSmeasPM025.ArchMeas.min[0]) 	+":" + Float2String(PMSmeasPM025.ArchMeas.avg[0]) 	+ ":" + Float2String(PMSmeasPM025.ArchMeas.max[0]) + "',";
				query += "'" + Float2String(PMSmeasPM100.ArchMeas.min[0]) 	+":" + Float2String(PMSmeasPM100.ArchMeas.avg[0]) 	+ ":" + Float2String(PMSmeasPM100.ArchMeas.max[0]) + "')";

				rc = db_exec(db, query.c_str());
				if (rc != SQLITE_OK) {
					debug_out(F("Table 'measPMS' not updated"),			DEBUG_ERROR, 1);
				}


				if(PMSEmeasPM010.status == SensorSt::ok){
					query  = "INSERT INTO measPMSE (Id, PM010, PM025, PM100) VALUES ('" + String((int)RID) + "',";
					query += "'" + Float2String(PMSEmeasPM010.ArchMeas.min[0]) 	+":" + Float2String(PMSEmeasPM010.ArchMeas.avg[0]) 	+ ":" + Float2String(PMSEmeasPM010.ArchMeas.max[0]) + "',";
					query += "'" + Float2String(PMSEmeasPM025.ArchMeas.min[0]) 	+":" + Float2String(PMSEmeasPM025.ArchMeas.avg[0]) 	+ ":" + Float2String(PMSEmeasPM025.ArchMeas.max[0]) + "',";
					query += "'" + Float2String(PMSEmeasPM100.ArchMeas.min[0]) 	+":" + Float2String(PMSEmeasPM100.ArchMeas.avg[0]) 	+ ":" + Float2String(PMSEmeasPM100.ArchMeas.max[0]) + "')";
					rc = db_exec(db, query.c_str());
					if (rc != SQLITE_OK) {
						debug_out(F("Table 'measPMS' not updated"),			DEBUG_ERROR, 1);
					}
				}

#ifdef CFG_GPS
				// Store GPS only if its alive and enabled by RecordMode selected
				if(last_value_GPS_lat != GPS_UNDEF && (Mode==RecMode::GPS || Mode==RecMode::GPS_Slow )){

					if(!inWindow){
						query  = "INSERT INTO measGPS (Id, lat, lon) VALUES ('" + String((int)RID) + "',";
						query += Float2String(last_value_GPS_lat, 6) + ",";
						query += Float2String(last_value_GPS_lon, 6) + ")";

						rc = db_exec(db, query.c_str());
						if (rc != SQLITE_OK) {
							debug_out(F("Table 'measGPS' not updated"),			DEBUG_ERROR, 1);
						}
					}
				}

#endif


			}
		}
		sqlite3_close(db);
	}
	else{
		debug_out(F("Store2DB: DB opening error."),						DEBUG_ERROR, 1);
	}
	xSemaphoreGive(SQL_mutex);
}


void ClearDB(){

	xSemaphoreTake(SQL_mutex, portMAX_DELAY);
	xSemaphoreTake(I2C_mutex, portMAX_DELAY);



	if(SD.exists(DB_FILE)){
		debug_out(F("DB file exists"),									DEBUG_ALWAYS, 1);

		if(SD.remove(DB_FILE)){
			debug_out(F("DB file deleted"),								DEBUG_ALWAYS, 1);
		}

		vTaskDelay(500);  // one tick delay (1ms) in between reads for stability

		SD.end();
		display.clear();
		display.displayOff();
		display.end();

		vTaskDelay(500);  // one tick delay (1ms) in between reads for stability

		ESP.restart();

	}
	else {
		debug_out(F("DB file not exists"),								DEBUG_ALWAYS, 1);
	}

	xSemaphoreGive(I2C_mutex);
	xSemaphoreGive(SQL_mutex);

}

#endif




/*****************************************************************
 * WiFi auto connecting script																	 *
 *****************************************************************/

static void waitForWifiToConnect(int maxRetries) {
	if (!WiFi.isConnected())
	{
		int retryCount = 0;

		WiFi.disconnect(false);
		WiFi.mode(WIFI_STA);
		WiFi.begin(); // Start WiFI
		vTaskDelay(500);

		while ((WiFi.status() != WL_CONNECTED) && (retryCount <	maxRetries)) {


			vTaskDelay(1000);  // one tick delay (1ms) in between reads for stability
			debug_out(".", 									DEBUG_ALWAYS, 0);

			WiFi.begin(); // Start WiFI

			++retryCount;
		}
		debug_out("", 										DEBUG_ALWAYS, 1);
	}
}


void connectWifi() {

	debug_out(F("Connecting to wifi"), 						DEBUG_ALWAYS, 1);

	WiFi.begin(); // Start WiFI
	WiFi.mode(WIFI_STA);

	debug_out(WiFi.SSID(),									DEBUG_ALWAYS, 1);

}


static int32_t calcWiFiSignalQuality(int32_t rssi) {
	if (rssi > -50) {
		rssi = -50;
	}
	if (rssi < -100) {
		rssi = -100;
	}
	return (rssi + 100) * 2;
}


String StrSplitItem(const String& toSplit, const char Delim, int Index) {

	String s="";
	int pos1 = 0;
	int pos2 = 0;
	int count = 1;

	if(Index<1){
		return s;
	}

	while(count <= Index){

		pos1 = pos2;
		pos2 = toSplit.indexOf(Delim, pos1+1);

		count++;
	}

	pos1 ? pos1++ : 0;

	if(pos2 < 0){
		s = toSplit.substring(pos1);
	}
	else{
		s = toSplit.substring(pos1, pos2);
	}

	return s;

}

