// 
// 
// 

#include "ACCweb.h"
#include "ESPaccessory.h"
#include <ESP8266mDNS.h>





using namespace nsACCweb;

//reate a web server on port 80.  problems https://github.com/esp8266/Arduino/issues/4085
ESP8266WebServer web(80);

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
			//web.sendHeader("Set-Cookie", "ESPSESSIONID=1");  //experiment, can we send a value via cookie
			web.sendHeader("Set-Cookie", getWsUri().c_str());
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

	void nsACCweb::webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) { // When a WebSocket message is received
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
				//nsDCCweb::DCCwebWS(doc);
				return;
			}


		}//end switch
	
	}//end websocket event


	void nsACCweb::sendJson(JsonObject & out) {
		//We can avoid a String class, but need to guesstimate a useful buffer size
		char payload[800];
		//JSON 7
		serializeJson(out, payload, sizeof(payload));
		webSocket->broadcastTXT(payload);
	}

	void nsACCweb::sendJson(JsonDocument out) {
		//We can avoid a String class, but need to guesstimate a useful buffer size
		char payload[800];
		serializeJson(out, payload, sizeof(payload));
		webSocket->broadcastTXT(payload);

	}


	void nsACCweb::processHardware(JsonDocument &doc) {
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
					if (strlen(v) > 4){
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

