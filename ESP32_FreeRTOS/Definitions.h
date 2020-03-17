/*
 * Definitions.h
 *
 *  Created on: Mar 17, 2020
 *      Author: E_CAD
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

	#ifndef LED_BUILTIN
		#define LED_BUILTIN 2
	#endif

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

#endif /* DEFINITIONS_H_ */
