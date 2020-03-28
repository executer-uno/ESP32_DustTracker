/*
 * Definitions.h
 *
 *  Created on: Mar 17, 2020
 *      Author: E_CAD
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

	// Config functionality

	#define CFG_BME280
	#define CFG_LCD
	#define CFG_GPS
	#define CFG_SQL
	#define CFG_GSHEET

	#define STUP_TIME 15000ULL

	// Build in LED
	#define LED_BUILTIN 2

	// Definition der Debuglevel
	#define DEBUG_ALWAYS 	0
	#define DEBUG_ERROR 	1
	#define DEBUG_WARNING 	2
	#define DEBUG_MIN_INFO 	3
	#define DEBUG_MED_INFO 	4
	#define DEBUG_MAX_INFO 	5
	// Wieviele Informationen sollenber die serielle Schnittstelle ausgegeben werden?
	#define DEBUG DEBUG_ERROR

	// BasicAuth config
	#define WWW_USERNAME "admin"
	#define WWW_PASSWORD "feinstaub"
	#define WWW_BASICAUTH_ENABLED 0

	// Sensor Wifi config (config mode)
	#define FS_SSID "sensor_cfg"
	#define FS_PWD  ""

	// Initial configuration settings ***********
	// GPS, bevorzugt Neo-6M
	#define GPS_READ 		1
	#define GPS_API_PIN 	9
	// BME280, temperature, humidity, pressure
	#define BME280_READ 	1
	#define BME280_API_PIN 	11
	// SDS011, der etwas teuerere Feinstaubsensor
	#define SDS_READ 		1
	#define SDS_API_PIN 	1
	// PMS1003, PMS300, 3PMS5003, PMS6003, PMS7003
	#define PMS_READ 		1
	#define PMS_API_PIN 	1

	// Pinout definition ************************
	#define PM_SERIAL_RX    35	// SDS
	#define PM_SERIAL_TX    32	// SDS

	#define PM2_SERIAL_RX   25  // PMS
	#define PM2_SERIAL_TX   26	// PMS

	#define GPS_SERIAL_RX   16	// GPS
	#define GPS_SERIAL_TX   17	// GPS

	#define DEB_RX    		3	// Debug UART pins
	#define DEB_TX    		1

	#define BUT_1 			34	// no pullup!
	#define BUT_2 			33
	#define BUT_3 			27

	// define pins for I2C
	#define OLED_CLASS_OBJ  SSD1306Wire
	#define OLED_ADDRESS    0x3C
	#define OLED_SDA    21
	#define OLED_SCL    22
	#define OLED_RST    -1

	#define I2C_PIN_SCL OLED_SCL
	#define I2C_PIN_SDA OLED_SDA



	enum class SensorSt {
		raw,
		wait,
		ok,
		hold,
		timeout
	};


	struct measure {
		float 	max;
		float 	min;
		float 	sum;
	};
	struct measureArchive {
		float 	max[200];
		float 	min[200];
		float 	avg[200];
	};

	// Single sensor measurements object
	class PMmeas {

	private:
		uint32_t 		count; 		// number of measurements in accumulator
		uint32_t 		CRCerr;		// number of CRC errors

		struct measure 	Measurements;

		SemaphoreHandle_t meas_mutex;		// Mutex to access to Measurements accumulators

	public:
		SensorSt		status;				// Sensor initialized and started
		float			CRCerrRate = 0.0;	// Rate of CRC errors

		struct measureArchive 	ArchMeas;

		void NewMeas(float Measure); // Add new measurement to accumulator
		void ArchPush();	// Push measured values to archive
		void CRCError();	// Increase CRC error counter
		String DebugAvg();	// Returns actual average value for debug
		String DebugRange();// Returns string in MIN:MAX format
		String DebugCRC();	// Returns CRC errors rate and measurements count
		PMmeas();
	};

	String Float2String(const double value);
	String Float2String(const double value, uint8_t digits);
	String Float2String(const double value, uint8_t digits, uint8_t size);
	String check_display_value(double value, double undef, uint8_t len, uint8_t str_len);

	void display_values();
	void debug_out(const String& text, const int level, const bool linebreak);

#endif /* DEFINITIONS_H_ */
