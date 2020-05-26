/*
 * Definitions.h
 *
 *  Created on: Mar 17, 2020
 *      Author: E_CAD
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_



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
	#define DEBUG DEBUG_MED_INFO

	#define OSM_API_ENDPOINT "ingress.opensensemap.org"				//"ingress.osem.vo1d.space"


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
	#define PM_SERIAL_RX    25	//35	// SDS	// Swapped As-Build in new device enclosure
	#define PM_SERIAL_TX    26	//32	// SDS

	#define PM2_SERIAL_RX   35	//25  	// PMS	// Swapped As-Build in new device enclosure
	#define PM2_SERIAL_TX   32	//26	// PMS

	#define GPS_SERIAL_RX   16	// GPS
	#define GPS_SERIAL_TX   17	// GPS

	#define DEB_RX    		3	// Debug UART pins
	#define DEB_TX    		1

	#define BUT_1 			27	// no pullup!   // LEFT				// reordered for new enclosure
	#define BUT_2 			34					// MID
	#define BUT_3 			33					// RIGHT

	#define SUPPLY			12			// Device supply self-control


	#define LINE1			0
	#define LINE2			13
	#define LINE3			25
	#define LINE4			37
	#define LINEM			49



	// define pins for I2C
	#define OLED_CLASS_OBJ  SSD1306Wire
	#define OLED_ADDRESS    0x3C
	#define OLED_SDA    21
	#define OLED_SCL    22
	#define OLED_RST    -1

	#define I2C_PIN_SCL OLED_SCL
	#define I2C_PIN_SDA OLED_SDA

	#define	GPS_UNDEF -999.0

	#define SOFTWARE_VERSION "PORT2XBMEGPS01"



	enum class RecMode {
		NoGPS,				// no GPS record
		GPS,				// GPS records
		NoGPS_Slow,			// Slow collection with no GPS data
		GPS_Slow			// Slow collection with GPS data recording
	};

	RecMode& operator++(RecMode& d);
	RecMode& operator--(RecMode& d);

	enum class SensorSt {
		raw,
		wait,
		ok,
		hold,
		timeout
	};

	struct struct_wifiInfo {
		char ssid[35];
		uint8_t encryptionType;
		int32_t RSSI;
		int32_t channel;
		bool isHidden;
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

	String Var2Json(const String& name, const String& value);
	String Var2Json(const String& name, const bool value);
	String Var2Json(const String& name, const int value);
	String Var2Json(const String& name, const double value);
	String ValueLocated2Json(const String& timestamp, const String& lat, const String& lng, const String& value);

	String printLocalTime(const char* format);
	int setUnixtime(int32_t unixtime);
	void setTimeZone(long offset, int daylight);

#endif /* DEFINITIONS_H_ */
