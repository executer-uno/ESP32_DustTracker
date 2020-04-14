
#ifndef Credentials_H_
#define Credentials_H_

#ifdef CFG_GSHEET
	// Replace with your own script id to make server side changes
	#define GSHEET_ID "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
#endif


	// OpenSenseMap
	#define ID_BOX 				"xxxxxxxxxxxxxxxxxxxxxxxx"


	#define ID_SENSOR_TEMP  	"xxxxxxxxxxxxxxxxxxxxxxxx"
	#define ID_SENSOR_HUMID  	"xxxxxxxxxxxxxxxxxxxxxxxx"
	#define ID_SENSOR_PRESS		"xxxxxxxxxxxxxxxxxxxxxxxx"
	#define ID_SENSOR_PMS_100 	"xxxxxxxxxxxxxxxxxxxxxxxx"
	#define ID_SENSOR_PMS_025 	"xxxxxxxxxxxxxxxxxxxxxxxx"
	#define ID_SENSOR_SDS_100 	"xxxxxxxxxxxxxxxxxxxxxxxx"
	#define ID_SENSOR_SDS_025 	"xxxxxxxxxxxxxxxxxxxxxxxx"

	// Home area coordinates for anonimizing
	// Useful site for that https://www.gps-coordinates.net/
	#define GPS_HOME_LAT_1		00.0000
	#define GPS_HOME_LAT_2		00.0010

	#define GPS_HOME_LON_1		00.0000
	#define GPS_HOME_LON_2		00.0010

#endif
