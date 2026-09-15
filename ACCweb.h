// ACCweb.h

#ifndef _ACCWEB_h
#define _ACCWEB_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
//#include <ESP8266WebServer-impl.h>
#include <ESP8266WebServer.h>
//#include <ESP8266WebServerSecure.h>
//#include <Parsing-impl.h>
//#include <Uri.h>
#include <ArduinoJson.h>
//#include <ArduinoJson.hpp>


namespace nsACCweb {
//dont use a namespace its a pain in the ass

	void startWebServices(void);
	void loopWebServices(void);
	//void sendJson(JsonObject& out);   
	//void sendJson(JsonDocument out);

}

#endif

