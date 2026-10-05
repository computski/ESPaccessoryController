// ACCweb.h
// BEWARE: you must upload your data folder to the ESP8266.  If using Visual Studio vMicro
// set option 15: Filesystem uploadtool:LittleFS it might default to SPIFFS which is obsolete

/*
We need to pass wsUri to the client browser so that it can set up a websocket on this ip and port combo
serveral ways to do this. One is to add a cookie to the server response containing the wsUri, the other
is to have the client page call /hardware as a GET and return a JSON string. This allows a bigger payload.

We are using ESPAsyncTCP and ESPAsyncWebServer.  This also includes a websocket handler.  The websocket is on the same port as the webserver, which is 80.
The websocket is on ws://address/ws.  No longer using Markus Sattler WebSockets library.
*/

#ifndef _ACCWEB_h
#define _ACCWEB_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <ESPAsyncWebServer.h> // The core async web server library 
#include <ESPAsyncTCP.h>  //also needed for async web
#include <ArduinoJson.h>  //what version is this
#include <LittleFS.h>

namespace nsACCweb {


	void startWebServices(void);
	void loopWebServices(void);
	void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);
	void sendJson(JsonObject& out);
	void sendJson(JsonDocument out);
	
		
	void processHardware(JsonDocument &doc);
	void processBank(JsonDocument& doc);
	void sendState(void);
	
	std::string getWsUri();
}

#endif

