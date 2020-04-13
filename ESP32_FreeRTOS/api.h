#pragma once
#include <WiFiClient.h>
#include "Credentials.h"
#include "streampipe.h"

class OsemApi {
  protected:
  WiFiClient client;

  public:
  bool postMeasurement(String measurement, String sensorID) {

    if (!client.connect(OSM_API_ENDPOINT, 80)){
    	debug_out("postMeasurement client connection fails", 									DEBUG_ERROR, 1);
    	return false;
    }

	vTaskDelay(4000);  // one tick delay (1ms) in between reads for stability

    client << String("POST ") << "/boxes/" << ID_BOX << "/" << sensorID << " HTTP/1.1" << EOL;
    client << "Host: " << OSM_API_ENDPOINT << EOL;
    client << "content-type: application/json" << EOL;
    client << "Connection: close" << EOL;
    client << "Content-Length: " << measurement.length() << EOL << EOL;
    client << measurement;

    // read response
    if (!client.connected()){

    	debug_out("postMeasurement client connection broken", 									DEBUG_ERROR, 1);
    	return false;
    }

    String line = client.readStringUntil('\r');

    if (line != "HTTP/1.1 201 Created"){
    	debug_out("postMeasurement unexpected returns: " + line, 								DEBUG_ERROR, 1);
    	return false;
    }

    return true;
  }
};

