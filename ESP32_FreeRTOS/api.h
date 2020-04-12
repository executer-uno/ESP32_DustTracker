#pragma once
#include <WiFiClient.h>
#include "Credentials.h"
#include "streampipe.h"

class OsemApi {
  protected:
  WiFiClient client;

  public:
  bool postMeasurement(String measurement, String sensorID) {
    if (!client.connect(OSM_API_ENDPOINT, 80)) return false;

    client << String("POST ") << "/boxes/" << ID_BOX << "/" << sensorID << " HTTP/1.1" << EOL;
    client << "Host: " << OSM_API_ENDPOINT << EOL;
    client << "Content-Type: application/json" << EOL;
    client << "Connection: close" << EOL;
    client << "Content-Length: " << measurement.length() << EOL << EOL;
    client << measurement;

    // read response
    if (!client.connected()) return false;
    String line = client.readStringUntil('\r');
    if (line != "HTTP/1.1 201 Created") return false;

    return true;
  }
};

