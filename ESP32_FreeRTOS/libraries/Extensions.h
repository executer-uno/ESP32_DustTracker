/*
 * Extensions.h
 *
 *  Created on: Mar 17, 2020
 *      Author: E_CAD
 */

#ifndef LIBRARIES_EXTENSIONS_H_
#define LIBRARIES_EXTENSIONS_H_


	#include "SoftwareSerial.h"
	extern SoftwareSerial Serial;



	/*****************************************************************
	 * Debug output																									*
	 *****************************************************************/
	void debug_out(const String& text, const bool linebreak);



#endif /* LIBRARIES_EXTENSIONS_H_ */
