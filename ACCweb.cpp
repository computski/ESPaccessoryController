// 
// 
// 

#include "ACCweb.h"
#include "ESPaccessory.h"
#include <ESP8266mDNS.h>

/*Known issue. ESP8266WebServer has a problem with serving a file > 21kb.  My bank0 page is 21,856 (with 8 TR) but 22,592 with 9 TR
and at that point i get a ERR_CONTENT_LENGTH_MISMATCH on the browser.*/






using namespace nsACCweb;

//reate a web server on port 80.  problems https://github.com/esp8266/Arduino/issues/4085
ESP8266WebServer web(80);
//AsyncWebServer web(80);

//declare as a pointer, we need to instantate once wsPort is pulled from eeprom
WebSocketsServer* webSocket;

void testToAccessVirtualServo() {
	nsESPaccessory::VIRTUALSERVO banana;
	//call out to get a copy of a VS from nsESPaccessory
//struct is 48 bytes x 16.  the RAM is 80kb and we are building a JSON string here plus running Wifi etc.  so and extra 0.5kb for a copy of 
//each VScollection is incidental

}


void handleRoot() {
	//web.send(200, "text/html", "<h1>You are connected to ACC ESP!</h1>");

	if (LittleFS.exists("/index.htm")) {
		File file = LittleFS.open("/index.htm", "r");

		//old ESP8266webserver only
		//web.sendHeader("Set-Cookie", "ESPSESSIONID=1");  //experiment, can we send a value via cookie
		//web.sendHeader("Set-Cookie", getWsUri().c_str());

		size_t sent = web.streamFile(file, "text/html");  //we know its html!
		file.close();
	}
	else {
		Serial.println(F("cannot find /index.htm"));
	}

}

String getContentType(String filename) { // convert the file extension to the MIME type
	if (filename.endsWith(".htm")) return "text/html";
	else if (filename.endsWith(".css")) return "text/css";
	else if (filename.endsWith(".js")) return "application/javascript";
	else if (filename.endsWith(".ico")) return "image/x-icon";
	return "text/plain";
}

bool handleFileRead(String path) { // send the right file to the client (if it exists)
	if (path.endsWith("/")) path += "index.htm";         // If a folder is requested, send the index file
	String contentType = getContentType(path);            // Get the MIME type

	if (LittleFS.exists(path)) {                            // If the file exists
		File file = LittleFS.open(path, "r");                 // Open it
		size_t sent = web.streamFile(file, contentType);	// And send it to the client
		file.close();                                       // Then close the file again
		return true;
	}

	Serial.println("\tFile Not Found " + path);
	return false;                                         // If the file doesn't exist, return false

}

void handleFormSubmit() {
	if (web.hasArg("username")) {
		Serial.printf("username=%s\n", web.arg("username"));
		//args works off the tag name, not the id
	}

	/*
	<form action = "/submit" method = "POST">
		<label for = "user">Username:< / label>
		<input type = "text" id = "user" name = "username"><br><br>

		<label for = "pass">Password : < / label>
		<input type = "password" id = "pass" name = "password"><br><br>

		<input type = "submit" value = "Submit Data">
		< / form>
		*/



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

void getHardware() {
	//used to return the wsUri string.  Note the websocket is hardcoded to port 81
	JsonDocument doc;
	doc["wsUri"] = getWsUri();
	doc["cpu"] = ESP.getCpuFreqMHz();
	doc["hasPCA"] = nsESPaccessory::bootController.hasPCA9685modules;

	//We can avoid a String class, but need to guesstimate a useful buffer size
	char jsonChar[512];
	serializeJsonPretty(doc, jsonChar, sizeof(jsonChar));
	web.send(200, "text/json", jsonChar);
}



//call regularly from main loop
void nsACCweb::loopWebServices(void) {
	web.handleClient();
	webSocket->loop();
	//2026-07-26 keep refreshing the MDNS
	MDNS.update();
}

void nsACCweb::startWebServices() {
	Serial.print("start ACCweb");

	web.on("/", handleRoot);

	web.onNotFound([]() {                              // If the client requests any URI
		if (!handleFileRead(web.uri()))                  // send it if it exists
			web.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
		});

	//special GET handlers
	//https://forum.arduino.cc/index.php?topic=476291.0
	web.on("/hardware", HTTP_GET, []() {getHardware(); });
	web.on("/submit", HTTP_POST, handleFormSubmit);

	/* Define route with inline lambda request handler.  asyncwebserver
	web.on("/hardware", HTTP_GET, [](AsyncWebServerRequest* request) {
		getHardware(request);
		});
		*/



	web.begin();
	Serial.println(F("HTTP server started."));
	// Start the  Flash Files System
	LittleFS.begin();
	listRootFiles();


	// start the websocket server, hardcoded to port 81.  Cannot be port 80 as this is used by the webserver
	webSocket = new WebSocketsServer(81);
	//2025-01-31 enableHeatbeat added to fix bug where Wifi is lost but the websocket does not disconnect causing all WS services to slow down
	webSocket->enableHeartbeat(8000, 3000, 2); //ping 8 sec, pong 3, 2 retry
	webSocket->begin();
	webSocket->onEvent(webSocketEvent);          // if there's an incomming websocket message, go to function 'webSocketEvent'

	Serial.println("WebSocket start");

	//2026-07-26 allow user to find device via http://ACC_ESP.local rather than using the assigned IP address
	//note that browsing to ESP_ACC gives a connection refused error as we are not running a web server

	if (!MDNS.begin(nsESPaccessory::bootController.MDNS)) {
		Serial.println("Error setting up MDNS responder!");
	}

}

#pragma region WEBSOCKET_routines
//These relate to interactions on the web pages

void nsACCweb::webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) { // When a WebSocket message is received
	switch (type) {
	case WStype_DISCONNECTED:             // if the websocket is disconnected
		break;
	case WStype_CONNECTED: {              // if a new websocket connection is established
		IPAddress ip = webSocket->remoteIP(num);
		Serial.printf("[%u] Connected WS %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
	}
						 break;
	case WStype_TEXT:                     // if new text data is received
		//Serial.printf("[%u] get Text: %s\n", num, payload);
		//Serial.printf("\nfrom WS: %s\n", payload);


			//2024-4-25 we are expecting inbound websocket data to be a json string
			//JSON 7
		JsonDocument doc;
		DeserializationError err = deserializeJson(doc, payload);
		if (err) {
			Serial.println(F("parseObject() failed"));
			Serial.println(err.c_str());
			return;
		}


		const char* sPage = doc["page"];
		if (sPage == nullptr) return;

		if (strcmp(sPage, "hardware") == 0) {
			//callout to hardware page routine
			processHardware(doc);
			return;
		}

		if (strcmp(sPage, "bank0") == 0) {
			//callout to bank0 routine
			processBank(doc);
			return;
		}

		if (strcmp(sPage, "bank1") == 0) {
			//callout to bank0 routine
			processBank(doc);
			return;
		}

	}//end switch

}//end websocket event


void nsACCweb::sendJson(JsonObject& out) {
	//We can avoid a String class, but need to guesstimate a useful buffer size
	char payload[5000];
	//JSON 7
	serializeJson(out, payload, sizeof(payload));
	webSocket->broadcastTXT(payload);
}

void nsACCweb::sendJson(JsonDocument out) {
	//WARNING: ESP8266 TCP frame size is 1460 bytes max.  payload[5000] payload works, 3000 does not, crashes.
	char payload[5000];
	serializeJson(out, payload, sizeof(payload));
	webSocket->broadcastTXT(payload);
	

}


void nsACCweb::processHardware(JsonDocument& doc) {
	using namespace nsESPaccessory;
	//maybe pass by ref and save memory?
	//Strictly we should validate the IP address and no port can be set to 0
	//the client should only send up the fields that were changed.
	// 
//can we use std::string
	const char* v = doc["action"];
	Serial.printf("processHW %s\n", v);

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
		}

		out["action"] = bootController.isDirty ? "ok" : "fail";
		if (bootController.isDirty)	eePutSettings();

	}//end write






	out["wsUri"] = getWsUri();
	out["mDNS"] = nsESPaccessory::bootController.MDNS;

	out["AP_IP"] = nsESPaccessory::bootController.AP_IP;
	out["AP_SSID"] = nsESPaccessory::bootController.AP_SSID;
	out["AP_pwd"] = nsESPaccessory::bootController.AP_pwd[0] == '\0' ? "none" : "*****";

	char buffer[30];
	byte mac[6];
	WiFi.macAddress(mac);
	snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
	out["mac"] = buffer;

	out["mode"] = nsESPaccessory::bootController.Mode == 'C' ? "Client" : "Controller";
	out["version"] = nsESPaccessory::bootController.softwareVersion;
	out["loconetIP"] = nsESPaccessory::bootController.tcpIP;
	out["loconetPort"] = nsESPaccessory::bootController.tcpPort;
	out["STA_SSID"] = nsESPaccessory::bootController.STA_SSID;
	out["STA_pwd"] = nsESPaccessory::bootController.STA_pwd[0] == '\0' ? "none" : "*****";
	out["networkIP"] = WiFi.localIP().toString();  //when connected to a router


	sendJson(out);



}



void nsACCweb::processBank(JsonDocument& doc) {
	using namespace nsESPaccessory;
	
	//default response is a poll
	JsonDocument out;
	out.clear();
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



	/*
	auto localHelperInput = [doc](VIRTUALSERVO& vs) {
		//we copy existing item to vsParse, update it there and only write back to original item if validation is good
		VIRTUALSERVO vsParse;
		vsParse = vs;
		bool fail = true;

		//Parse device-specific fields
		std::string mode = doc["pins"][vs.pin]["mode"];

		//Assume that if we index a pin that does not exist in pins[] that we return "" rather than crashing...
		//not all pins will be present in the input

		if (mode == "Servo") {
			//servo specific fields	
			vsParse.deviceType = DEVICE_SERVO;
			uint8_t swing = doc["pins"][vs.pin]["swing"];
			if (swing < 91) { vsParse.swing = swing;fail = false; }

			int8_t rate = doc["pins"][vs.pin]["rate"];
			if ((rate > -10) && (rate < 10)) { vsParse.rate = rate;fail = false; }
		}

		if (mode == "MAS") {
			vsParse.deviceType = DEVICE_MAS;
			//expect the MASarray to be 64chars each should be 0-9 A-F
			std::string MASarray = doc["pins"][vs.pin]["MASarray"];

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

		if (mode == "Aspect") { vsParse.deviceType = DEVICE_ASPECT;fail = false; }
		if (mode == "Sensor") { vsParse.deviceType = DEVICE_SENSOR;fail = false; }
		if (mode == "SensorW") { vsParse.deviceType = DEVICE_SENSOR_WPU;fail = false; }

		//any other modes such as I2C or interstitial states are ignored and leave fail=true
		if (!fail) {
			//all modes, update dcc address and boolean fields
			//sending a string or neg number will result in addr 0 which disables that pin
			uint16_t addr = doc["pins"][vsParse.pin]["addr"];
			if (addr > 2048) fail = true;
			vsParse.address = addr;
			vsParse.invert = doc["pins"][vs.pin]["invert"];
			vsParse.continuous = doc["pins"][vs.pin]["cont"];
		}

		//if validation passes, we write back
		if (!fail) {
			vs = vsParse;
			bootController.isDirty = true;
			eePutSettings();
		}
		Serial.printf("pin update %d %s result=%d\n", vs.pin, mode.c_str(), fail);

		};//end lambda
	*/

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


	/*DO THE PROCESSING*/
	if ("write" == doc["action"]) {
		if ("bank0" == doc["page"]) { for (auto& vs : virtualservoCollection) { localHelperIn(vs); } };
		if ("bank1" == doc["page"]) { for (auto& vs : virtualservoCollectionBank1) { localHelperIn(vs); } };
		if ("bank2" == doc["page"]) { for (auto& vs : virtualservoCollectionBank2) { localHelperIn(vs); } };
	}

	//then whether write or poll, we roll into returning the updated out object		
	/*Caution: bad eeprom values will cause a crash if they are outside of the arrabounds when used as an index. test them.
	also after calling the helper and sending data you need to clear pins array by redeclaring it.
	JsonArray pins = out["pins"].to<JsonArray>();  do not use .clear() as you will cause a memory leak
	
	WARNING: ESP8266 TCP frame size is 1460 bytes max.  Sending 7+ pins will usually exceed this causing the websocket
	transmission to cut short, trigger an error on the client and reset the websocket.  To ensure reliably operation
	never transmit more than 6 pins at a time.
	*/


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
		for (uint8_t r = 0;r < 5;r++) { localHelperOutput(virtualservoCollectionBank1[r]); }
		sendJson(out);
		JsonArray pins = out["pins"].to<JsonArray>();
		for (uint8_t r = 5;r < 10;r++) { localHelperOutput(virtualservoCollectionBank2[r]); }
		sendJson(out);
		pins = out["pins"].to<JsonArray>();
		for (uint8_t r = 10;r < 16;r++) { localHelperOutput(virtualservoCollectionBank2[r]); }
	}
			
	sendJson(out);
}



std::string nsACCweb::getWsUri() {
	std::string wsUri = "ws://";
	//"ws://192.168.6.1:81


	if (WiFi.getMode() == WIFI_AP) {
		//send the AP default gateway
		wsUri.append(nsESPaccessory::bootController.AP_IP);
	}
	else {
		wsUri.append(WiFi.localIP().toString().c_str());
	}
	//add the port which hardcoded to 81
	wsUri.append(":81");
	return wsUri;
}
#pragma endregion
