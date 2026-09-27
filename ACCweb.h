// ACCweb.h
// BEWARE: you must upload your data folder to the ESP8266.  If using Visual Studio vMicro
// set option 15: Filesystem uploadtool:LittleFS it might default to SPIFFS which is obsolete

/*
We need to pass wsUri to the client browser so that it can set up a websocket on this ip and port combo
serveral ways to do this. One is to add a cookie to the server response containing the wsUri, the other
is to have the client page call /hardware as a GET and return a JSON string. This allows a bigger payload.

*/

#ifndef _ACCWEB_h
#define _ACCWEB_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <ESP8266WebServer.h>  //not using this
//#include <ESPAsyncWebServer.h> // The core async web server library  NOT USING
#include <ArduinoJson.h>  //what version is this
#include <LittleFS.h>
#include <WebSockets.h>  //from arduino library manager. Markus Sattler v2.1
#include <WebSocketsServer.h>

namespace nsACCweb {


	void startWebServices(void);
	void loopWebServices(void);

	void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
	void sendJson(JsonObject& out);
	void sendJson(JsonDocument out);
	void processHardware(JsonDocument &doc);
	void processBank(JsonDocument& doc);
	std::string getWsUri();
}

#endif

