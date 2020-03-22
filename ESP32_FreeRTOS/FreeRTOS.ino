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

struct PMmeas SDSmeas;
struct PMmeas PMSmeas;

// define two tasks for Blink & AnalogRead
void TaskBlink( void *pvParameters );
void TaskReadPMSensors( void *pvParameters );
void TaskDiagLevel( void *pvParameters );
void TaskKeyboard( void *pvParameters );

void PMmeas::NewMeas(float inPm010, float inPm025, float inPm100){

	this->pm010.sum += inPm010;
	this->pm025.sum += inPm025;
	this->pm100.sum += inPm100;

	this->pm010.max = (this->pm010.max < inPm010 ? inPm010 : this->pm010.max);
	this->pm025.max = (this->pm025.max < inPm025 ? inPm025 : this->pm025.max);
	this->pm100.max = (this->pm100.max < inPm100 ? inPm100 : this->pm100.max);

	this->pm010.min = (this->pm010.min > inPm010 ? inPm010 : this->pm010.min);
	this->pm025.min = (this->pm025.min > inPm025 ? inPm025 : this->pm025.min);
	this->pm100.min = (this->pm100.min > inPm100 ? inPm100 : this->pm100.min);

	this->count++;
}
void PMmeas::CRCError(){
	this->CRCerr++;
}
void PMmeas::ArchPush(){

	// move all older values to archive's end
	for(int i=200; i>1; i--){

		// Pm 1.0
		this->ArchPm010.avg[i] = this->ArchPm010.avg[i-1];
		this->ArchPm010.max[i] = this->ArchPm010.max[i-1];
		this->ArchPm010.min[i] = this->ArchPm010.min[i-1];

		// Pm 2.5
		this->ArchPm025.avg[i] = this->ArchPm025.avg[i-1];
		this->ArchPm025.max[i] = this->ArchPm025.max[i-1];
		this->ArchPm025.min[i] = this->ArchPm025.min[i-1];

		// Pm 10.0
		this->ArchPm100.avg[i] = this->ArchPm100.avg[i-1];
		this->ArchPm100.max[i] = this->ArchPm100.max[i-1];
		this->ArchPm100.min[i] = this->ArchPm100.min[i-1];
	}

	// Add fresh measurements
	// Pm 1.0
	this->ArchPm010.avg[0] = this->pm010.sum / this->count;
	this->ArchPm010.max[0] = this->pm010.max;
	this->ArchPm010.min[0] = this->pm010.min;

	// Pm 2.5
	this->ArchPm025.avg[0] = this->pm025.sum / this->count;
	this->ArchPm025.max[0] = this->pm025.max;
	this->ArchPm025.min[0] = this->pm025.min;

	// Pm 10.0
	this->ArchPm100.avg[0] = this->pm100.sum / this->count;
	this->ArchPm100.max[0] = this->pm100.max;
	this->ArchPm100.min[0] = this->pm100.min;

	// CRC errors

	this->CRCerrRate += this->CRCerr / this->count;
	this->CRCerrRate /= 2;

	// prepare for next measurements
	this->CRCerr =0;
	this->count  =0;

	this->pm010.sum = 0.0;
	this->pm010.max = 0.0;
	this->pm010.min = 999999999.9;

	this->pm025.sum = 0.0;
	this->pm025.max = 0.0;
	this->pm025.min = 999999999.9;

	this->pm100.sum = 0.0;
	this->pm100.max = 0.0;
	this->pm100.min = 999999999.9;

}
void PMmeas::Init(){

	this->status	= SensorSt::raw;

	this->CRCerr 	= 0;
	this->count  	= 0;

	this->pm010.sum = -1.0;
	this->pm010.max = -1.0;
	this->pm010.min = 999999999.9;

	this->pm025.sum = -1.0;
	this->pm025.max = -1.0;
	this->pm025.min = 999999999.9;

	this->pm100.sum = -1.0;
	this->pm100.max = -1.0;
	this->pm100.min = 999999999.9;

	// init all values of archive
	for(int i=0; i<200; i++){

		// Pm 1.0
		this->ArchPm010.avg[i] = -1.0;
		this->ArchPm010.max[i] = -1.0;
		this->ArchPm010.min[i] = -1.0;

		// Pm 2.5
		this->ArchPm025.avg[i] = -1.0;
		this->ArchPm025.max[i] = -1.0;
		this->ArchPm025.min[i] = -1.0;

		// Pm 10.0
		this->ArchPm100.avg[i] = -1.0;
		this->ArchPm100.max[i] = -1.0;
		this->ArchPm100.min[i] = -1.0;
	}
}

void PMmeas::PrintDebug(){

	if(this->count){
		String strDebug = "";

		if(cfg::debug <= DEBUG_MIN_INFO ){

			if(this->pm100.sum > 0.0){
				strDebug  = "PM10.0=" + Float2String(this->pm100.sum / this->count,1 , 7) + ",";
			}
			if(this->pm025.sum > 0.0){
				strDebug += "PM2.5="  + Float2String(this->pm025.sum / this->count,1 , 7) + ",";
			}
			if(this->pm010.sum > 0.0){
				strDebug += "PM1.0="  + Float2String(this->pm010.sum / this->count,1 , 7) + "";
			}
			debug_out(strDebug, DEBUG_MIN_INFO, 1);
		}

		strDebug  = "PM10.0 : [" + 	Float2String(this->pm100.min,1 , 7) + " : ";
		strDebug += 				Float2String(this->pm100.sum / this->count,1 , 7) + " : ";
		strDebug += 				Float2String(this->pm100.max,1 , 7) + " ]";
		debug_out(strDebug, DEBUG_MED_INFO, 1);

		strDebug  = "PM2.5 :  [" + 	Float2String(this->pm025.min,1 , 7) + " : ";
		strDebug += 				Float2String(this->pm025.sum / this->count,1 , 7) + " : ";
		strDebug += 				Float2String(this->pm025.max,1 , 7) + " ]";
		debug_out(strDebug, DEBUG_MED_INFO, 1);

		strDebug  = "PM1.0 :  [" + 	Float2String(this->pm010.min,1 , 7) + " : ";
		strDebug += 				Float2String(this->pm010.sum / this->count,1 , 7) + " : ";
		strDebug += 				Float2String(this->pm010.max,1 , 7) + " ]";
		debug_out(strDebug, DEBUG_MED_INFO, 1);

		debug_out(F("CRC errors rate: "), 									DEBUG_MED_INFO , 0);
		debug_out(Float2String(this->CRCerr/(this->CRCerr+this->count)),	DEBUG_MED_INFO , 1);
		debug_out(F("Count: "), 											DEBUG_MED_INFO , 0);
		debug_out(Float2String(this->count),								DEBUG_MED_INFO , 1);
	}
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
    vTaskDelay(50);  // one tick delay (1ms) in between reads for stability
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

    if(cfg::debug == debugPrev){
    	vTaskDelay(950);  // one tick delay (1ms) in between reads for stability
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

		SDSmeas.ArchPush();
		PMSmeas.ArchPush();

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
	debug_out(F(""), DEBUG_MIN_INFO, 1);
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

					SDSmeas.NewMeas(-0.1, (float)pm025_serial/10.0, (float)pm100_serial/10.0);

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

String Float2String(const double value, uint8_t digits, uint8_t size) {

	String s = Float2String(value, digits);

	s = String("               ").substring(1, size - s.length()) + s;

	return s;
}
