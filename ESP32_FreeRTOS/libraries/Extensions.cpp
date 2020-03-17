/*
 * Extensions.cpp
 *
 *  Created on: Mar 17, 2020
 *      Author: E_CAD
 */

#include "Extensions.h"



/*****************************************************************
 * Debug output																									*
 *****************************************************************/
void debug_out(const String& text, const bool linebreak) {
	if (true){ 								//(level <= cfg::debug) {
		if (linebreak) {
			Serial.println(text);
		} else {
			Serial.print(text);
		}
	}
}
