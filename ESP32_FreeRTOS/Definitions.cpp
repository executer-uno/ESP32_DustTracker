/*
 * Definitions.cpp
 *
 *  Created on: Mar 23, 2020
 *      Author: E_CAD
 */

#include <Arduino.h>
#include <WiFi.h>

#include "Definitions.h"
//#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino


namespace cfg {
	extern int	debug;
	extern char wlanssid[35];
	extern char wlanpwd[65];
}

extern HardwareSerial		Serial;
extern SemaphoreHandle_t 	Serial_mutex;


void PMmeas::NewMeas(float Measure){

	xSemaphoreTake(meas_mutex, portMAX_DELAY);

	this->Measurements.sum += Measure;

	this->Measurements.max = (this->Measurements.max < Measure ? Measure : this->Measurements.max);

	this->Measurements.min = (this->Measurements.min > Measure ? Measure : this->Measurements.min);

	this->count++;

	xSemaphoreGive(meas_mutex);

}
void PMmeas::CRCError(){
	this->CRCerr++;
}
void PMmeas::ArchPush(){

	if(this->count > 0){

		// move all older values to archive's end
		for(int i=199; i>0; i--){

			this->ArchMeas.avg[i] = this->ArchMeas.avg[i-1];
			this->ArchMeas.max[i] = this->ArchMeas.max[i-1];
			this->ArchMeas.min[i] = this->ArchMeas.min[i-1];

		}

		xSemaphoreTake(meas_mutex, portMAX_DELAY);

		// Add fresh measurements
		this->ArchMeas.avg[0] = this->Measurements.sum / this->count;
		this->ArchMeas.max[0] = this->Measurements.max;
		this->ArchMeas.min[0] = this->Measurements.min;

		// CRC errors
		this->CRCerrRate += (float) this->CRCerr / (float) this->count;
		this->CRCerrRate /= 2.0;

		xSemaphoreGive(meas_mutex);

	}

	xSemaphoreTake(meas_mutex, portMAX_DELAY);

	// prepare for next measurements
	this->CRCerr =0;
	this->count  =0;

	this->Measurements.sum = 0.0;
	this->Measurements.max = -1.0;
	this->Measurements.min = 999999999.9;

	xSemaphoreGive(meas_mutex);

}

PMmeas::PMmeas(){

	this->status	= SensorSt::raw;

	this->CRCerr 	= 0;
	this->count  	= 0;

	this->Measurements.sum = 0.0;
	this->Measurements.max = -999999999.9;
	this->Measurements.min = +999999999.9;


	// init all values of archive
	for(int i=0; i<200; i++){

		this->ArchMeas.avg[i] = -1.0;
		this->ArchMeas.max[i] = -1.0;
		this->ArchMeas.min[i] = -1.0;

	}

	meas_mutex = xSemaphoreCreateMutex();
}
String PMmeas::DebugAvg(){
	String strDebug = "";
	if(this->count > 0){
		strDebug = Float2String(this->Measurements.sum / this->count,1 , 7);
	}
	return strDebug;
}

String PMmeas::DebugRange(){
	String strDebug = "";
	if(this->count > 0){
		strDebug  = Float2String(this->Measurements.min,1 , 7) + " : ";
		strDebug += Float2String(this->Measurements.max,1 , 7);
	}
	return strDebug;
}

String PMmeas::DebugCRC(){
	String strDebug = "";
	if(this->count > 0){
		strDebug  =	F("CRC errors rate: ");
		strDebug += Float2String(this->CRCerr/(this->CRCerr + this->count));
		strDebug += F("\nCount: ");
		strDebug += Float2String( (float) this->count);
	}
	return strDebug;
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

	s = String("               ").substring(1, size - s.length() +1 ) + s;

	return s;
}

/*****************************************************************
 * Debug output																									*
 *****************************************************************/
void debug_out(const String& text, const int level, const bool linebreak) {
	if (level <= cfg::debug) {
		xSemaphoreTake(Serial_mutex, portMAX_DELAY);

		if (linebreak) {
			Serial.println(text);
		} else {
			Serial.print(text);
		}
		xSemaphoreGive(Serial_mutex);
	}
}



/*****************************************************************
 * check display values, return '-' if undefined								 *
 *****************************************************************/
String check_display_value(double value, double undef, uint8_t digits, uint8_t str_len) {
	String s = (value != undef ? Float2String(value, digits, str_len) : "-");
	return s;
}



/*****************************************************************
 * convert value to json string																	*
 *****************************************************************/
String Value2Json(const String& type, const String& value) {
	String s = F("{\"value_type\":\"{t}\",\"value\":\"{v}\"},");
	s.replace("{t}", type);
	s.replace("{v}", value);
	return s;
}

/*****************************************************************
 * convert string value to json string													 *
 *****************************************************************/
String Var2Json(const String& name, const String& value) {
	String s = F("\"{n}\":\"{v}\",");
	String tmp = value;
	tmp.replace("\\", "\\\\"); tmp.replace("\"", "\\\"");
	s.replace("{n}", name);
	s.replace("{v}", tmp);
	return s;
}

/*****************************************************************
 * convert boolean value to json string													*
 *****************************************************************/
String Var2Json(const String& name, const bool value) {
	String s = F("\"{n}\":\"{v}\",");
	s.replace("{n}", name);
	s.replace("{v}", (value ? "true" : "false"));
	return s;
}

/*****************************************************************
 * convert integer value to json string													*
 *****************************************************************/
String Var2Json(const String& name, const int value) {
	String s = F("\"{n}\":\"{v}\",");
	s.replace("{n}", name);
	s.replace("{v}", String(value));
	return s;
}

/*****************************************************************
 * convert double value to json string													*
 *****************************************************************/
String Var2Json(const String& name, const double value) {
	String s = F("\"{n}\":\"{v}\",");
	s.replace("{n}", name);
	s.replace("{v}", String(value));
	return s;
}




void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
	  debug_out(F("Failed to obtain time"), 			DEBUG_ERROR, 1);
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

//
//
//// Network connection
//
////connectWifi();
//		  //connect to WiFi
//
//	debug_out("Connecting to " + String(cfg::wlanssid),		DEBUG_ALWAYS, 1);
//
//
//		  WiFi.begin(cfg::wlanssid, cfg::wlanpwd); // Start WiFI
//
//			int retryCount = 0;
//			while ((WiFi.status() != WL_CONNECTED) && (retryCount <	20)) {
//				delay(500);  									// one tick delay (1ms) in between reads for stability
//				debug_out(".", 									DEBUG_ALWAYS, 0);
//				++retryCount;
//			}
//
//			if (WiFi.status() != WL_CONNECTED) {
//				debug_out(F("WiFi not connected"), 				DEBUG_ALWAYS, 1);
//			}
//			else {
//				debug_out(F("WiFi connected\nIP address: "), 	DEBUG_ALWAYS, 0);
//				debug_out(WiFi.localIP().toString(), 			DEBUG_ALWAYS, 1);
//
//				//init and get the time
//				configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
//
//			}
//
//
//		  printLocalTime();
//




