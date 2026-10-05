// 
// 
// 

#include "ACCweb.h"
#include "ESPaccessory.h"
#include <ESP8266mDNS.h>

/*ESP8266WebServer replaced with AsyncWebServer which is more robust.  The problem is that the ESP8266WebServer library has a bug in the streamFile() function 
that causes it to fail when serving files larger than 21kb.  The AsyncWebServer library does not have this limitation and can serve larger files without issues.
The AsyncWebServer library also has better performance and can handle multiple connections more efficiently.  It also handles content types automatically*/






using namespace nsACCweb;

//create an instance of the AsyncWebServer class on port 80
AsyncWebServer web(80);

AsyncWebSocket ws("/ws");

void handleFormSubmit(AsyncWebServerRequest* request) {
	if (request->hasParam("username", true)) { // true = check POST body
		const AsyncWebParameter* p = request->getParam("username", true);  //need to use const
		Serial.printf("username=%s\n", p->value().c_str());
	}
	request->send(200, "text/plain", "Form Received");
}


void listRootFiles() {
	Serial.println(F("LittleFS files in root directory:"));
	Dir root = LittleFS.openDir("/");
	while (root.next()) {
		Serial.print("File: ");
		Serial.print(root.fileName());
		Serial.print(" | Size: ");
		Serial.println(root.fileSize());
	}
}

void getHardware(AsyncWebServerRequest* request) {
	//used to return the wsUri string.  Note the websocket is hardcoded to port 81
	JsonDocument doc;
	doc["wsUri"] = getWsUri();
	doc["cpu"] = ESP.getCpuFreqMHz();
	doc["hasPCA"] = nsESPaccessory::bootController.hasPCA9685modules;
	doc["heap"] = ESP.getFreeHeap();
	

	//We can avoid a String class, but need to guesstimate a useful buffer size
	char jsonChar[512];
	serializeJsonPretty(doc, jsonChar, sizeof(jsonChar));
	//request->send(200, "text/json", jsonChar);
	request->send(200, "application/json", jsonChar);  //AI says use application

}



//call regularly from main loop
void nsACCweb::loopWebServices(void) {
	// Cleanup disconnected websocket clients safely from memory
	ws.cleanupClients();
	//2026-07-26 keep refreshing the MDNS
	MDNS.update();
}


void nsACCweb::startWebServices() {
	Serial.print("start ACCweb");

	// Start the Flash Files System
	LittleFS.begin();
	listRootFiles();

	// Serve static files directly from LittleFS (fixes >21KB serving issue automatically)
	//web.serveStatic("/", LittleFS, "/").setDefaultFile("index.htm");
	web.serveStatic("/", LittleFS, "/").setDefaultFile("hardware.htm");

	// Special GET/POST handlers
	web.on("/hardware", HTTP_GET, [](AsyncWebServerRequest* request) {
		getHardware(request);
		});

	web.on("/submit", HTTP_POST, [](AsyncWebServerRequest* request) {
		handleFormSubmit(request);
		});

	// 404 Handler
	web.onNotFound([](AsyncWebServerRequest* request) {
		request->send(404, "text/plain", "404: Not Found");
		});

	// Attach the WebSocket handler to the Web Server instance
	ws.onEvent(onWsEvent);
	web.addHandler(&ws);

	web.begin();
	Serial.println(F("HTTP server started. Websockets started"));

	/*
	// Start the websocket server, hardcoded to port 81
	webSocket = new WebSocketsServer(81);
		
	//ping clients every 8 seconds.  If no pong response after 2 retries (3 seconds apart), forcefully close the slot.
	webSocket->enableHeartbeat(8000, 3000, 2); // ping 8 sec, pong 3, 2 retry
	webSocket->begin();
	webSocket->onEvent(webSocketEvent); // if there's an incoming websocket message, go to function 'webSocketEvent'
	Serial.println("WebSocket start");
	*/


	//2026-07-26 allow user to find device via http://ACC_ESP.local rather than using the assigned IP address
	//note that browsing to ESP_ACC gives a connection refused error as we are not running a web server

	if (!MDNS.begin(nsESPaccessory::bootController.MDNS)) {
		Serial.println("Error setting up MDNS responder!");
	}
}






#pragma region WEBSOCKET_routines
//These relate to interactions on the web pages
// Unified Asynchronous WebSocket Event Handler
void nsACCweb::onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
	switch (type) {
	case WS_EVT_CONNECT: {
		Serial.printf("[%u] Connected WS from IP: %s\n", client->id(), client->remoteIP().toString().c_str());
		break;
	}
	case WS_EVT_DISCONNECT: {
		Serial.printf("[%u] Disconnected WS\n", client->id());
		break;
	}
	case WS_EVT_DATA: {
		AwsFrameInfo* info = (AwsFrameInfo*)arg;
		if (info->opcode == WS_TEXT) {
			// Null-terminate the data packet safe safely
			data[len] = 0;

			JsonDocument doc;
			DeserializationError err = deserializeJson(doc, (char*)data);
			if (err) {
				Serial.println(F("parseObject() failed"));
				Serial.println(err.c_str());
				return;
			}
			
			//reboot is supported from any page
			const char* action = doc["action"];
			if ((action != nullptr) && strcmp(action, "reboot") == 0) {
				Serial.println(F("REBOOTING...\n\n"));
				Serial.flush();
				ESP.restart();
				return;
			}
			
			const char* sPage = doc["page"];
			if (sPage == nullptr) return;

			if (strcmp(sPage, "hardware") == 0) {
				processHardware(doc);
				return;
			}
			if (strcmp(sPage, "bank0") == 0 || strcmp(sPage, "bank1") == 0 || strcmp(sPage, "bank2") == 0) {
				processBank(doc);
				return;
			}
			if (strcmp(sPage, "state") == 0) {
				sendState();
				return;
			}
		}
		break;
	}
	case WS_EVT_PONG:
	case WS_EVT_ERROR:
		break;
	}
}

//Note, seems like we have to use the String class
//using char payload[5000] results in hangups.
void nsACCweb::sendJson(JsonObject& out) {
	String payload;
	serializeJson(out, payload);
	ws.textAll(payload); // Broadcasts cleanly via Async dynamically allocated frame chunks
}

void nsACCweb::sendJson(JsonDocument out) {
	String payload;
	serializeJson(out, payload);
	ws.textAll(payload);
}


//Avoid String class - this seems to cause a crash
/*
void nsACCweb::sendJson(JsonObject& out) {
	// Allocate a safe static block on the stack
	char payload[5000];

	// Serialize directly into our character array
	size_t len = serializeJson(out, payload, sizeof(payload));

	// Send raw buffer data across all clients asynchronously
	if (len > 0 && len < sizeof(payload)) {
		ws.textAll((uint8_t*)payload, len);
	}
}

void nsACCweb::sendJson(JsonDocument out) {
	// Allocate a safe static block on the stack
	char payload[5000];

	// Serialize directly into our character array
	size_t len = serializeJson(out, payload, sizeof(payload));

	// Send raw buffer data across all clients asynchronously
	if (len > 0 && len < sizeof(payload)) {
		ws.textAll((uint8_t*)payload, len);
	}
}

*/



void nsACCweb::processHardware(JsonDocument& doc) {
	using namespace nsESPaccessory;
	//maybe pass by ref and save memory?
	//Strictly we should validate the IP address and no port can be set to 0
	//the client should only send up the fields that were changed.
	// 
//can we use std::string


	const char* v = doc["action"];
	Serial.printf("processHW %s\n", v);

	//reboot?
	if (strcmp(v, "reboot") == 0) {
		Serial.println(F("REBOOTING...\n\n"));
		Serial.flush();
		ESP.restart();
		return;
	}



	JsonDocument out;
	out["page"] = "hardware";
	out["action"] = "poll";


	//valid inbound actions are poll|write
	//valid outbound actions are poll|ok|fail

	if (strcmp(v, "write") == 0) {
		Serial.println("write");
		v = doc["AP_SSID"];
		if (v != nullptr) {
			strncpy(bootController.AP_SSID, v, sizeof(bootController.AP_SSID));
			//cannot set the AP SSID to null
			if (bootController.AP_SSID[0] == '\0') {
				strncpy(bootController.AP_SSID, "DCC_ESP\0", sizeof(bootController.AP_SSID));
				bootController.isDirty = true;
			}

		}

		v = doc["AP_pwd"];
		if (v != nullptr) {
			//"none" is used explicity to instruct system to set a null pwd
			if (strcmp(v, "none") == 0) { memset(bootController.AP_pwd, '\0', sizeof(bootController.AP_pwd)); }
			else {
				//length of password must be 8+ chars
				if (strlen(v) >= 7) {
					strncpy(bootController.AP_pwd, v, sizeof(bootController.AP_pwd));
					bootController.isDirty = true;
				}
			}
		}

		v = doc["mDNS"];
		if (v != nullptr) {
			Serial.printf("mDNS %d\n", strlen(v));
			if (strlen(v) > 4) {
				//only change mDNS if supplied string is >4 chrs
				strncpy(bootController.MDNS, v, sizeof(bootController.MDNS));
				bootController.isDirty = true;
			}
		}

		v = doc["STA_SSID"];
		if (v != nullptr) {
			strncpy(bootController.STA_SSID, v, sizeof(bootController.STA_SSID));
			//writing null is ok.
			bootController.isDirty = true;
		}

		v = doc["STA_pwd"];
		if (v != nullptr) {
			//"none" is used explicity to instruct system to set a null pwd
			if (strcmp(v, "none") == 0) { memset(bootController.STA_pwd, '\0', sizeof(bootController.STA_pwd)); }
			else {
				//only save passwords 8+ char
				if (strlen(v) >= 7) strncpy(bootController.STA_pwd, v, sizeof(bootController.STA_pwd));
			}
			bootController.isDirty = true;
		}

		v = doc["AP_IP"];
		if (v != nullptr) {
			//ip address is stored as dot separated. really we should validate this
			strncpy(bootController.AP_IP, v, sizeof(bootController.AP_IP));
			bootController.isDirty = true;
		}

		v = doc["loconetIP"];
		if (v != nullptr) {
			//ip address is stored as dot separated. really we should validate this
			strncpy(bootController.tcpIP, v, sizeof(bootController.tcpIP));
			bootController.isDirty = true;
		}

		v = doc["loconetPort"];
		if (v != nullptr) {
			if (strtol(v, NULL, 10) > 0 && strtol(v, NULL, 10) <= 65535) {
				bootController.isDirty = true;
				bootController.tcpPort = strtol(v, NULL, 10);
			}
		}

		v = doc["mode"];
		if (v != nullptr) {
			bootController.Mode = (strcmp(v, "Controller") == 0) ? 'S' : 'C';
			bootController.isDirty = true;
		}

		v = doc["I2C"];
		if (v != nullptr) {
			bootController.hasPCA9685modules = (strcmp(v, "true") == 0) ? true : false;
			bootController.isDirty = true;
		}

		out["action"] = bootController.isDirty ? "ok" : "fail";
		if (bootController.isDirty)	eePutSettings();


	}//end write






	out["wsUri"] = getWsUri();
	out["mDNS"] = bootController.MDNS;

	out["AP_IP"] = bootController.AP_IP;
	out["AP_SSID"] = bootController.AP_SSID;
	out["AP_pwd"] = bootController.AP_pwd[0] == '\0' ? "none" : "*****";
	char buffer[30];
	byte mac[6];
	WiFi.macAddress(mac);
	snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
	out["mac"] = buffer;

	out["mode"] = bootController.Mode == 'C' ? "Client" : "Controller";
	out["version"] = bootController.softwareVersion;
	out["loconetIP"] = bootController.tcpIP;
	out["loconetPort"] = bootController.tcpPort;
	out["STA_SSID"] = bootController.STA_SSID;
	out["STA_pwd"] = bootController.STA_pwd[0] == '\0' ? "none" : "*****";
	out["networkIP"] = WiFi.localIP().toString();  //when connected to a router
	out["I2C"] = bootController.hasPCA9685modules;
	
	sendJson(out);

}


/* MY OG CODE
void nsACCweb::processBank(JsonDocument& doc) {
	using namespace nsESPaccessory;
	
	//default response is a poll
	JsonDocument out;
	out["page"] = doc["page"];
	out["action"] = "poll";
	// Create the pins[] array inside the main object
	JsonArray pins = out["pins"].to<JsonArray>();


	//actions are poll|write
		
	//I come from the lambda down under.  Define a lambda routine to process changes passed up in [doc]
	//note that the capture variable [doc] is by default read only.
	
	auto localHelperIn = [doc](VIRTUALSERVO& vs) {
		bool fail = true;
		VIRTUALSERVO vsParse=vs;

		//Directly assign the array reference
		//were passed [doc] as a const, so need to search it as const
		JsonArrayConst pinsArray = doc["pins"];

		//Loop through to find pin 0 using the modern V7 syntax
		JsonObjectConst matchedPin;

		for (JsonObjectConst item : pinsArray) {
			// item["pin"].is<int>() checks if the key exists AND is an integer
			if (item["pin"].is<int>() && item["pin"].as<int>() == vs.pin) {
				matchedPin = item;
				break;
			}
		}

		//exit now if no match
		if (matchedPin.isNull()) return;
		
		//copy data from matchedPin to vsParse and if validation is good, write this back to vs
		//Parse device-specific fields
	
		if ( "Servo"== matchedPin["mode"]) {
			//servo specific fields	
			vsParse.deviceType = DEVICE_SERVO;
			uint8_t swing = matchedPin["swing"];
			if (swing < 91) { vsParse.swing = swing;fail = false; }

			int8_t rate = matchedPin["rate"];
			if ((rate > -10) && (rate < 10)) { vsParse.rate = rate;fail = false; }
		}

		if ("MAS" == matchedPin["mode"]) {
			vsParse.deviceType = DEVICE_MAS;
			//expect the MASarray to be 64chars each should be 0-9 A-F
			std::string MASarray = matchedPin["MASarray"];

			Serial.printf("MASarray in %s\n", MASarray.c_str());  //debug

			//lamda function
			bool is_valid = std::all_of(MASarray.begin(), MASarray.end(), [](unsigned char c) {
				return std::isxdigit(c);
				});

			if ((MASarray.size() == 64) && is_valid) {
				//need to parse the hex chars into array of uint8

				for (int i = 0;i < 32;i++) {
					//advance through MSarray two hex chrs at a time and convert to uint8_t
					vsParse.aspectParameters[i] = std::stoi(MASarray.substr(i * 2, 2), nullptr, 16);
				}
				fail = false;
			}
		}

		if ("Aspect"== matchedPin["mode"]) { vsParse.deviceType = DEVICE_ASPECT;fail = false; }
		if ("Sensor"== matchedPin["mode"]) { vsParse.deviceType = DEVICE_SENSOR;fail = false; }
		if ("SensorW"== matchedPin["mode"]) { vsParse.deviceType = DEVICE_SENSOR_WPU;fail = false; }

		//any other modes such as I2C or interstitial states are ignored and leave fail=true
		if (!fail) {
			//all modes, update dcc address and boolean fields
			//sending a string or neg number will result in addr 0 which disables that pin
			uint16_t addr = matchedPin["addr"];
			if (addr > 2048) fail = true;
			vsParse.address = addr;
			vsParse.invert = matchedPin["invert"];
			vsParse.continuous = matchedPin["cont"];
		}

		//if validation passes, we write back
		if (!fail) {
			vs = vsParse;
			bootController.isDirty = true;
			eePutSettings();
		}
		Serial.printf("pin update %d result=%d\n", vs.pin, fail);

	};


	//Lambda function to generate output. use &vs to save creating a copy of vs on every call
	//and use &pins in the [] capture, so that we can build this out in doc
	auto localHelperOutput = [&pins](VIRTUALSERVO& vs) {
			
		// Add a "pin" object
		const std::string devType[6] = { "Servo","Aspect","MAS","SensorW","Sensor","I2C" };
		const std::string servoState[13] = { "Neutral","x","Thrown","x","Closed","x","Thrown","Closed","MAS","High","Low","x","x" };
		JsonObject pin = pins.add<JsonObject>();
		
		if (vs.deviceType<size(devType)) pin["mode"] = devType[vs.deviceType];
		pin["pin"] = vs.pin;
		pin["addr"] = vs.address;
		pin["invert"] = vs.invert;
		pin["cont"] = vs.continuous;
		pin["swing"] = vs.swing;
		pin["rate"] = vs.rate;
		//pin["notSupported"] = vs.ignorePowerParameter;

		pin["MASstate"] = vs.MASstate;  //vs.state
		if (vs.state<size(servoState)) pin["state"] = servoState[vs.state];
				
		//vs.aspectParameters is a 32 byte array. we convert it here to a hex string of 64 char
		char buffer[70];
		char* ptr = buffer;
		for (int i = 0;i < sizeof(vs.aspectParameters);i++) {
			//we know we are creating 2 chars with a trailling null
			//and we know 70 chars is sufficient without need to track end of buffer, hence use 3
			snprintf(ptr, 3, "%02X", vs.aspectParameters[i]);
			ptr += 2;
		}
		pin["MASarray"] = buffer;

		};//end lambda


	//DO THE PROCESSING
	if ("write" == doc["action"]) {
		if ("bank0" == doc["page"]) { for (auto& vs : virtualservoCollection) { localHelperIn(vs); } };
		if ("bank1" == doc["page"]) { for (auto& vs : virtualservoCollectionBank1) { localHelperIn(vs); } };
		if ("bank2" == doc["page"]) { for (auto& vs : virtualservoCollectionBank2) { localHelperIn(vs); } };
	}

	//then whether write or poll, we roll into returning the updated out object		
	//Caution: bad eeprom values will cause a crash if they are outside of the arrabounds when used as an index. test them.
	//also after calling the helper and sending data you need to clear pins array by redeclaring it.
	//JsonArray pins = out["pins"].to<JsonArray>();  do not use .clear() as you will cause a memory leak
	
//	WARNING: ESP8266 TCP frame size is 1460 bytes max.  Sending 7+ pins will usually exceed this causing the websocket
	//transmission to cut short, trigger an error on the client and reset the websocket.  To ensure reliably operation
	//never transmit more than 6 pins at a time.
	


	if ("bank0" == doc["page"]) {
		for (uint8_t r = 0;r < 5;r++) {	localHelperOutput(virtualservoCollection[r]);}
		sendJson(out);
		JsonArray pins = out["pins"].to<JsonArray>();
		for (uint8_t r = 5;r < 10;r++) {localHelperOutput(virtualservoCollection[r]);}
	}

	if ("bank1" == doc["page"]) {
		for (uint8_t r = 0;r < 5;r++) {	localHelperOutput(virtualservoCollectionBank1[r]);}
		sendJson(out);
		JsonArray pins = out["pins"].to<JsonArray>();
		for (uint8_t r = 5;r < 10;r++) {localHelperOutput(virtualservoCollectionBank1[r]);}
		sendJson(out);
		pins = out["pins"].to<JsonArray>();
		for (uint8_t r = 10;r < 16;r++) { localHelperOutput(virtualservoCollectionBank1[r]); }

	}

	if ("bank2" == doc["page"]) {
		for (uint8_t r = 0;r < 5;r++) { localHelperOutput(virtualservoCollectionBank2[r]); }
		sendJson(out);
		JsonArray pins = out["pins"].to<JsonArray>();
		for (uint8_t r = 5;r < 10;r++) { localHelperOutput(virtualservoCollectionBank2[r]); }
		sendJson(out);
		pins = out["pins"].to<JsonArray>();
		for (uint8_t r = 10;r < 16;r++) { localHelperOutput(virtualservoCollectionBank2[r]); }
	}
			
	sendJson(out);
}
*/

void nsACCweb::processBank(JsonDocument& doc) {
	//AI fixed code.  note that not all arduino compilers support std::size(something) so it has used literal sizes.
	
	using namespace nsESPaccessory;

	// Default response is a poll
	JsonDocument out;
	out["page"] = doc["page"];
	out["action"] = "poll";
	// Create the pins[] array inside the main object
	JsonArray pins = out["pins"].to<JsonArray>();

	// Actions are poll|write

	// I come from the lambda down under. Define a lambda routine to process changes passed up in [doc]
	auto localHelperIn = [doc](VIRTUALSERVO& vs) {
		bool fail = true;
		VIRTUALSERVO vsParse = vs;

		// Directly assign the array reference
		JsonArrayConst pinsArray = doc["pins"];

		// Loop through to find pin using modern V7 syntax
		JsonObjectConst matchedPin;

		for (JsonObjectConst item : pinsArray) {
			if (item["pin"].is<int>() && item["pin"].as<int>() == vs.pin) {
				matchedPin = item;
				break;
			}
		}

		// Exit now if no match
		if (matchedPin.isNull()) return;

		// Parse device-specific fields
		if ("Servo" == matchedPin["mode"]) {
			vsParse.deviceType = DEVICE_SERVO;
			uint8_t swing = matchedPin["swing"];
			if (swing < 91) { vsParse.swing = swing; fail = false; }

			int8_t rate = matchedPin["rate"];
			if ((rate > -10) && (rate < 10)) { vsParse.rate = rate; fail = false; }
		}

		if ("MAS" == matchedPin["mode"]) {
			vsParse.deviceType = DEVICE_MAS;
			std::string MASarray = matchedPin["MASarray"];

			Serial.printf("MASarray in %s\n", MASarray.c_str());

			bool is_valid = std::all_of(MASarray.begin(), MASarray.end(), [](unsigned char c) {
				return std::isxdigit(c);
				});

			if ((MASarray.size() == 64) && is_valid) {
				for (int i = 0; i < 32; i++) {
					vsParse.aspectParameters[i] = std::stoi(MASarray.substr(i * 2, 2), nullptr, 16);
				}
				fail = false;
			}
		}

		if ("Aspect" == matchedPin["mode"]) { vsParse.deviceType = DEVICE_ASPECT; fail = false; }
		if ("Sensor" == matchedPin["mode"]) { vsParse.deviceType = DEVICE_SENSOR; fail = false; }
		if ("SensorW" == matchedPin["mode"]) { vsParse.deviceType = DEVICE_SENSOR_WPU; fail = false; }

		if (!fail) {
			uint16_t addr = matchedPin["addr"];
			if (addr > 2048) fail = true;
			vsParse.address = addr;
			vsParse.invert = matchedPin["invert"];
			vsParse.continuous = matchedPin["cont"];
		}

		if (!fail) {
			vs = vsParse;
			bootController.isDirty = true;
		}
		Serial.printf("pin update %d result=%d\n", vs.pin, fail);
		};

	// Lambda function to generate output
	auto localHelperOutput = [&pins](VIRTUALSERVO& vs) {
		//not all compilers support std::size(something)
		const size_t devTypeSize = 6;
		const size_t servoStateSize = 13;

		const std::string devType[devTypeSize] = { "Servo","Aspect","MAS","SensorW","Sensor","I2C" };
		//note servos have interstitial states such as servo-to-thrown.  Report these as thrown|closed because that is the ultimate state.
		const std::string servoState[servoStateSize] = { "Neutral","Thrown","Thrown","Closed","Closed","x","Thrown","Closed","MAS","High","Low","x","x" };
		JsonObject pin = pins.add<JsonObject>();

		if (vs.deviceType < devTypeSize) pin["mode"] = devType[vs.deviceType];
		pin["pin"] = vs.pin;
		pin["addr"] = vs.address;
		pin["invert"] = vs.invert;
		pin["cont"] = vs.continuous;
		pin["swing"] = vs.swing;
		pin["rate"] = vs.rate;

		pin["MASstate"] = vs.MASstate;
		pin["w"] = vs.state;
		if (vs.state < servoStateSize) pin["state"] = servoState[vs.state];

		char buffer[70];
		char* ptr = buffer;
		for (size_t i = 0; i < sizeof(vs.aspectParameters); i++) {
			snprintf(ptr, 3, "%02X", vs.aspectParameters[i]);
			ptr += 2;
		}
		pin["MASarray"] = buffer;
		};

	//DO THE PROCESSING
	if ("write" == doc["action"]) {
		if ("bank0" == doc["page"]) { for (auto& vs : virtualservoCollection) { localHelperIn(vs); } }
		if ("bank1" == doc["page"]) { for (auto& vs : virtualservoCollectionBank1) { localHelperIn(vs); } }
		if ("bank2" == doc["page"]) { for (auto& vs : virtualservoCollectionBank2) { localHelperIn(vs); } }
		eePutSettings();
	}

	// OUTPUT GENERATION 
	if ("bank0" == doc["page"]) {
		// Fix: Bank0 originally only sent 10 pins chunked into groups of 5
		for (uint8_t r = 0; r < 5; r++) { localHelperOutput(virtualservoCollection[r]); }
		sendJson(out);
		pins = out["pins"].to<JsonArray>();
		for (uint8_t r = 5; r < 10; r++) { localHelperOutput(virtualservoCollection[r]); }
	}

	if ("bank1" == doc["page"]) {
		for (uint8_t r = 0; r < 5; r++) { localHelperOutput(virtualservoCollectionBank1[r]); }
		sendJson(out);
		pins = out["pins"].to<JsonArray>();
		for (uint8_t r = 5; r < 10; r++) { localHelperOutput(virtualservoCollectionBank1[r]); }
		sendJson(out);
		pins = out["pins"].to<JsonArray>();
		for (uint8_t r = 10; r < 16; r++) { localHelperOutput(virtualservoCollectionBank1[r]); }
	}

	if ("bank2" == doc["page"]) {
		for (uint8_t r = 0; r < 5; r++) { localHelperOutput(virtualservoCollectionBank2[r]); }
		sendJson(out);
		pins = out["pins"].to<JsonArray>();
		for (uint8_t r = 5; r < 10; r++) { localHelperOutput(virtualservoCollectionBank2[r]); }
		sendJson(out);
		pins = out["pins"].to<JsonArray>();
		for (uint8_t r = 10; r < 16; r++) { localHelperOutput(virtualservoCollectionBank2[r]); }
	}

	sendJson(out);
}











/// <summary>
/// Send websocket giving status updates to all banks.  Call this after processing a turnout or MAS command, or a sensor change.
/// </summary>
void nsACCweb::sendState(void) {
	
	using namespace nsESPaccessory;
	Serial.println("sendState");  //debug
	JsonDocument out;
	out.clear();
	out["page"] = "bank0";
	out["action"] = "state";
	// Create the pins[] array inside the main object
	JsonArray pins = out["pins"].to<JsonArray>();

	auto localHelperOutput = [&pins](VIRTUALSERVO& vs) {
		// Add a "pin" object
		//note servos have interstitial states such as servo-to-thrown.  Report these as thrown|closed because that is the ultimate state.
		const size_t servoStateSize = 13;
		const std::string servoState[servoStateSize] = { "Neutral","Thrown","Thrown","Closed","Closed","x","Thrown","Closed","MAS","High","Low","x","x" };
		JsonObject pin = pins.add<JsonObject>();
		pin["pin"] = vs.pin;
		pin["MASstate"] = vs.MASstate;  //vs.state
		if (vs.state < servoStateSize) pin["state"] = servoState[vs.state];

		};

	// the websocket output should be small enough we can send all pins in each back in one go.
	for (auto& vs : virtualservoCollection) { localHelperOutput(vs); }
	sendJson(out);
	pins = out["pins"].to<JsonArray>();
	out["page"] = "bank1";
	for (auto& vs : virtualservoCollectionBank1) { localHelperOutput(vs); }
	sendJson(out);
	pins = out["pins"].to<JsonArray>();
	out["page"] = "bank2";
	for (auto& vs : virtualservoCollectionBank2) { localHelperOutput(vs); }
	sendJson(out);

}






std::string nsACCweb::getWsUri() {
	std::string wsUri = "ws://";
	//format for the new websocket handler is, and this is harded to same port as the http server, which is 80.  The websocket is on /ws
	//"ws://192.168.6.1/ws"


	if (WiFi.getMode() == WIFI_AP) {
		//send the AP default gateway
		wsUri.append(nsESPaccessory::bootController.AP_IP);
	}
	else {
		wsUri.append(WiFi.localIP().toString().c_str());
	}
	//add trailing /ws to the uri.  The websocket is hardcoded to /ws
	wsUri.append("/ws");
	return wsUri;
}
#pragma endregion
