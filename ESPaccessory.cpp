// 
// 
// 


/*Commands, uppercase set the wifi and TCP server params

S=set SSID
W=set SSID pwd
T=set TCP server IP
P=set TCP server port
D=dump param buffer
R=reboot
M=mode;
X=dump WIFI and server params
V = verbose

commands from servo
SERVO SET-UP command is: s pin,addr,swing,invert,[continuous] 
pin is 4-12, addr is 1-2048, swing is 5-180 and invert is 0|1 where 1 inverts the direction. 
Optional continuous (default 0) will keep drive active after movement has ceased.

ASPECT SET-UP command is: a pin,addr,invert,[ignorePower] pin is 4-12, addr is 1-204, invert is 0|1
and optional ignorePower (default 1) will ignore DCC power parameter and instead is always powered on.

PIN command is: p pin, c|t|n|T pin is 4-12, c|t|n|T correspond to closed, thrown, neutral, Toggle.
Once a pin is mapped to a DCC address, it will respond to that DCC turnout.

RATE command is: r pin,rate pin is 4-12, rate is -10 to +10 where negative values retard rate of servo rotation. 
Command has no effect on aspects.

DCC EMULATION command is:d addr t|c|T|n, [power] Simulates a DCC command from loconet.
A dcc controller itself can only send t|c, whereas T|n are for our convenience when setting up providing Toggle and neutral. 
Optional power will power/depower a signal aspect

DUMP command is: x, this will dump current settings to serial

VERBOSE command is: v, this will dump loconet messages to serial. reverts to verbose OFF at boot.

*/





#include "ESPaccessory.h"

extern "C" {
#include <osapi.h>
#include <os_type.h>
}


#define TRACE

#ifndef TRACE
#define trace(traceCodeBlock) ;
#else
#define trace(traceCodeBlock) traceCodeBlock
#endif


using namespace nsESPaccessory;

//version control and capture of some system defaults for new compilations
///note, IP addresses are stored as a string to allow more easy editing in a web window or serial
struct CONTROLLER
{
	long softwareVersion = 20260407;  //yyyymmdd captured as an integer
	char AP_SSID[21] = "ESPACC";   //local SSID
	char AP_pwd[21] = "";
	char AP_IP[17] = "192.168.6.1\0";   //note the actual setting requires comma separators
	char STA_SSID[21] = "Ossonet\0";  //SSID when running as a station on an external WiFi network
	char STA_pwd[21] = "11223344AA\0";			//pwd for station
	char tcpIP[17] = "192.168.1.118\0";   //target IP of CONTROLLER to connect to
	uint16_t tcpPort = 1234;       //tcp port
	char relayMode[3] = "R1";  //R denotes relay, K kiosk. 1|0 denotes watchdog enabled|off
	bool isDirty = false;  //will be true if EEPROM needs to be written
};



CONTROLLER bootController;
IPAddress TCPserverIP;
AsyncClient* clientX; //points to currently active client

static std::vector<std::string> messages;


enum RELAY {
	S_BOOT,
	S_FAULT,
	S_TCP_CONNECTED,
	S_TCP_RECONNECT,
	S_TCP_PENDING_CONNECT
};

unsigned long previousMillis;
unsigned long interval;
bool ledState = true;

static os_timer_t intervalTimer;

static os_timer_t watchdogTimer;
#define WATCHDOG_TIMEOUT 120000  //two minutes

//os_timer_disarm(&watchdogTimer);   we need some code that will periodically reset the WDT before it times out
//os_timer_arm(&watchdogTimer, WATCHDOG_TIMEOUT, true);


static uint8_t deviceState = S_BOOT;


//2024-03-22 ballID handling
//void payloadTimerCallback(void* pArg);


static os_timer_t payloadTimer;





//and if you put it here and use a declaration above you get a odd linker reference error... hmmm
//https://stackoverflow.com/questions/625799/resolve-build-errors-due-to-circular-dependency-amongst-classes
void payloadTimerCallback(void* pArg)
{
	Serial.println("payloadTimer Event");
	os_timer_disarm(&payloadTimer);

	//more stuff..

}


//local function declarations, not visible outside of this module
static void heartbeat(void* arg);
static void handleData(void* arg, AsyncClient* client, void* data, size_t len);
void onConnect(void* arg, AsyncClient* client);
void onDisconnect(void* arg, AsyncClient* client);
void bootTCP(void);
void stringIPtoArray(char* s, uint8_t* myIP);
void eeGetSettings(void);
void eePutSettings(void);
void checkSerial(void);
static void sendEnqueuedMessages();
static void watchdogFail(void);


bool verbose;

//5,4 are used for SCL,SDA on I2C

//ESP-12 pinout
//SCK 14 
//MOSI 13
//MISO 12
//CSN 15
//CE 2


#define TRACE

#ifndef TRACE
#define trace(traceCodeBlock) ;
#else
#define trace(traceCodeBlock) traceCodeBlock
#endif



bool nsESPaccessory::queueMessage(std::string s) {
	if (s.empty()) return false;
	messages.push_back(s);
	return true;
}



void nsESPaccessory::ESPaccessorySetup() {

	verbose = false;

	eeGetSettings();

	pinMode(16, OUTPUT);

	deviceState = S_BOOT;
	Serial.begin(115200);
	delay(20);

	//to protect against Wifi hangups and loss of TCP, start watchdog now

	//https://sub.nanona.fi/esp8266/hello-world.html
	os_timer_setfn(&watchdogTimer, (os_timer_func_t*)watchdogFail, NULL);
	os_timer_arm(&watchdogTimer, WATCHDOG_TIMEOUT, true);


thing:
	// connects to access point
	WiFi.mode(WIFI_STA);
	WiFi.setHostname("ESPACC");

	Serial.println(F("For help use ?"));
	Serial.printf("Connecting to Wifi %s", bootController.STA_SSID);

	WiFi.begin(bootController.STA_SSID, bootController.STA_pwd);

	long dotTimer = millis();

	while (WiFi.status() != WL_CONNECTED) {
		checkSerial();
		if (millis() - dotTimer >= 500) {
			Serial.print('.');
			dotTimer = millis();
		}
	}
	//want the ESP to auto reconnect the WiFi if it fails
	WiFi.setAutoReconnect(true);
	Serial.print("Wifi Connected, IP address: ");
	Serial.println(WiFi.localIP().toString());
	bootTCP();


}



void nsESPaccessory::ESPaccessoryLoop() {

	unsigned long currentMillis = millis();



	//TCP connection state engine
	switch (deviceState) {
	case S_FAULT:
		break;
	case S_TCP_CONNECTED:
		//flash led 0.2 sec on, 5 off
		if (currentMillis - previousMillis >= interval) {
			digitalWrite(16, ledState);  //led is active low, false=on
			interval = ledState ? 5000 : 200;
			ledState = !ledState;
			previousMillis = currentMillis;
		}
		break;

	case S_TCP_RECONNECT:
		bootTCP();
		deviceState = S_TCP_PENDING_CONNECT;
		break;

	case S_TCP_PENDING_CONNECT:
		//we kicked off ONE new attempt at a connect
		//flash led 1 sec on/off
		if (currentMillis - previousMillis >= 1000) {
			digitalWrite(16, ledState);
			ledState = !ledState;
			previousMillis = currentMillis;

			//NOTE: this technique of only allowing one pending connect at a time seems to work.  The asyncTCP object does have a timeout on it, and will call disconnect
			//after about 60 sec.  At this point we will retry the connection.
			//There is a risk that old instances of the client remain in memory and after about 30 instances are created the ESP will run out of resources and crassh, i.e. 
			//its a memory leak.

		}
		break;

	}



	/*	//Otherwise, get first message in queue and transmit
		if (!messages.empty()) {
			auto it = messages.begin();

			if (clientX != nullptr) {
				//have an active TCP link to send over.  Test if it is free to send
				if (clientX->space() > 32 && clientX->canSend()) {
					clientX->add((char*)it->msg, it->bytes);
					clientX->send();
					//flag the message as sent
					it->sent = true;
				}
			}
		}
	*/


	//serial, we need to define some commands, then read an entire input block then act on it
	//note that modifying the Wifi AP SSID might not take effect until a reboot.

	checkSerial();


	sendEnqueuedMessages();

}// end main loop


#pragma region "...SUPPORT ROUTINES..."




/// <summary>
/// Read incoming serial commands and action
/// </summary>
void checkSerial(void) {
	if (Serial.available()) {
		char command[32];
		Serial.readString().toCharArray(command, 32);

		if (command[0] == 'D') {
			/*
						//dump current parameter values.
						//http://arduinomania.com/posts/sprintf-function/

						//dump param buffer. first 3 bytes are structured
						Serial.printf("Params %d bytes\n", parameterBufferLen);
						Serial.printf("marker %02X\n", parameterBuffer[0]);
						Serial.printf("next_id %02X%02X\n", parameterBuffer[1], parameterBuffer[2]);
						//now dump as hex
						for (uint8_t i = 3;i < parameterBufferLen;i++) {
							Serial.printf("%02X ", parameterBuffer[i]);
						}

						//dump reset buffer
						Serial.printf("\nreset %d bytes\n", resetBufferLen);
						Serial.printf("marker %02X\n", resetBuffer[0]);
						//now dump as hex
						for (uint8_t i = 1;i < resetBufferLen;i++) {
							Serial.printf("%02X ", resetBuffer[i]);
						}
						Serial.println("\n\n");

						*/
		}

		if (command[0] == 'T') {  //set TCPserver IP
			//set server address.  expect next params to be a series of integers
			const char s[2] = ".";
			char* token;
			uint8_t myIP[4];

			/* get the first token */
			token = strtok(command + 1, s);

			uint8_t i = 0;

			/* walk through other tokens */
			while (token != NULL) {
				//printf(" %s\n", token);
				myIP[i++] = atoi(token);
				token = strtok(NULL, s);
			}
			//expect i=4
			if (i != 4) { Serial.println("bad IP address\n"); }
			else
			{//use this IP
				char dump[80];
				//actually i  want to just use the string, if its valid
				//wow, C is a pain in the ass with strings
				//and we want to ensure 
				sprintf(dump, "%d.%d.%d.%d", myIP[0], myIP[1], myIP[2], myIP[3]);

				//save this to bootController and update eeprom
				//need to restart server with it.
				//note, garbage like 192.abc.3.4 will convert to 192.0.3.4 and any ints over 255 will be modulus
				//negative ints will be expressed as inverse e.g. -8 becomes 248
				//still you can see what it has done through the X dump command


				//need a routine to restart the tcp client
				Serial.printf("Server IP set as %s\n", dump);
				Serial.println(F("Now you must REBOOT\n\n"));
				strncpy(bootController.tcpIP, dump, sizeof(bootController.tcpIP));
				bootController.isDirty = true;
				eePutSettings();
			}
		}


		if (command[0] == 'P') {
			//set tcp port
			uint16_t port = atoi(command + 1);

			//need a routine to restart the tcp client
			Serial.printf("Server port set as %d\n", port);
			Serial.println(F("Now you must REBOOT\n\n"));
			bootController.tcpPort = port;
			bootController.isDirty = true;
			eePutSettings();
		}

		if (command[0] == 'X') {
			//dump IP set up
			Serial.printf("\nSoftware ver %d\n", bootController.softwareVersion);
			Serial.print("MAC ");
			Serial.println(WiFi.macAddress());  // %s in printf does not work
			Serial.printf("Network SSID %s\n", bootController.STA_SSID);
			Serial.printf("Server IP %s\n", bootController.tcpIP);
			Serial.printf("Server port %d\n\n", bootController.tcpPort);
			
			/*
			if (bootController.relayMode[0] == 'K') {
				Serial.println(F("KIOSK mode\n"));
			}
			else {
				Serial.println(F("RELAY mode\n"));
			}


			if (bootController.relayMode[1] == '1') {
				Serial.println(F("WATCHDOG ON\n\n"));
			}
			else {
				Serial.println(F("WATCHDOG OFF\n\n"));
			}
			*/

		}

		if (command[0] == 'S') {
			//set Station SSID
			char buffer[20];  //full of nulls
			strncpy(buffer, command + 1, 19);
			memset(bootController.STA_SSID, '\0', sizeof(bootController.STA_SSID));
			strncpy(bootController.STA_SSID, buffer, sizeof(bootController.STA_SSID));
			Serial.printf("STA SSID set to %s\n", buffer);
			Serial.println(F("Now you must REBOOT\n\n"));
			bootController.isDirty = true;
			eePutSettings();
		}

		if (command[0] == 'W') {
			//set Station SSID
			char buffer[21];  //full of nulls
			strncpy(buffer, command + 1, 20);  //max ssid or pwd is 20 char
			//if keyword none is given, wipe the pwd
			if (strcmp(buffer, "none") == 0) { memset(bootController.STA_pwd, '\0', sizeof(bootController.STA_pwd)); }
			else {
				//copy password
				strncpy(bootController.STA_pwd, buffer, sizeof(bootController.STA_pwd));
			}

			Serial.printf("STA password set to %s\n", buffer);
			Serial.println(F("Password must be min 8 chars"));
			Serial.println(F("Now you must REBOOT\n\n"));
			bootController.isDirty = true;
			eePutSettings();
		}

		if (command[0] == '?') {
			Serial.println(F("S=set SSID"));
			Serial.println(F("W=set SSID pwd"));
			Serial.println(F("T=set TCP server IP"));
			Serial.println(F("P=set TCP server port"));
			Serial.println(F("D=dump param buffer"));
			Serial.println(F("R=reboot"));
			Serial.println(F("M=mode"));
			Serial.println(F("X=dump params\n"));
			Serial.println(F("H=dump incoming\n"));
		}

		if (command[0] == 'R') {
			Serial.println(F("REBOOTING...\n\n"));
			ESP.restart();
		}

		if (command[0] == 'M') {
			//switch modes
			if (command[1] == 'K') {
				bootController.relayMode[0] = 'K';
				Serial.println(F("KIOSK mode set\n"));
			}
			else {
				bootController.relayMode[0] = 'R';
				Serial.println(F("RELAY mode set\n"));
			}

			//optional 1 | 0 to enable | disable watchdog
			switch (command[2]) {
			case '1':
				bootController.relayMode[1] = '1';
				Serial.println(F("watchdog ON\n"));
				break;
			case '0':
				bootController.relayMode[1] = '0';
				Serial.println(F("watchdog OFF\n"));
				break;
			default:
				Serial.println(F("watchdog no change\n"));
			}

			Serial.println(F("Now you must REBOOT\n\n"));
			bootController.isDirty = true;
			eePutSettings();
		}


		if ((command[0] == 'V')||(command[0] == 'v')) {
			//toggle verbose
			verbose = !verbose;
			if (verbose) {
				Serial.println(F("verbose ON\n"));
			}
			else {
				Serial.println(F("verbose OFF\n"));
			}

		}


	}//serial available


}

void stringIPtoArray(char* s, uint8_t* myIP) {
	//IPAddress class requires the address to be provided as 4 octets
	//uint8_t myIP[4];
	char* p = nullptr;
	char ipBoot[17];
	strcpy(ipBoot, s);

	//strtok modifies its arguement, have to use a copy.
	p = strtok((char*)ipBoot, ",.");
	int i = 0;
	while (p != NULL) {
		myIP[i] = atoi(p);
		i++;
		if (i == 4) break;
		//more data?
		p = strtok(NULL, ",.");
	}
	return;
}

/// <summary>
/// restores settings from EEPROM. If the software version has changed, we overwrite the eeprom with defaults
/// we also need to clear certain values on boot. max EEPROM we can use is 4096
/// </summary>
void eeGetSettings(void) {
	CONTROLLER defaultController;  //grab defaults as per DCCcore.h
	EEPROM.begin(1024);
	int eeAddr = 0;
	EEPROM.get(eeAddr, bootController);
	if (defaultController.softwareVersion != bootController.softwareVersion) {
		/*need to re-initiatise eeprom with factory defaults*/
		EEPROM.put(0, defaultController);
		eeAddr += sizeof(bootController);
		EEPROM.commit();

		//Now populate our structs with EEprom values
		eeAddr = 0;
		EEPROM.get(eeAddr, bootController);
		eeAddr += sizeof(bootController);
	}

}

/// <summary>
/// save settings to EEPROM, we save the bootController struct
/// </summary>
void eePutSettings(void) {
	if (bootController.isDirty == false) { return; }
	int eeAddr = 0;
	EEPROM.put(eeAddr, bootController);
	eeAddr += sizeof(bootController);


	EEPROM.commit();
	bootController.isDirty = false;
	//trace(Serial.printf("EEPROM commit, bytes %d\r\n", eeAddr);)
}


static void watchdogFail(void) {
	Serial.println(F("watchdog TIMEOUT\n"));
	delayMicroseconds(1000);
	//called if there is a wdt timeout, will invoke a reboot
	if (bootController.relayMode[1] == '1') {
		//WDT timeout, reset the unit
		ESP.restart();
	}

	//the wdt is disabled, so disarm the timer
	//to renable it a reboot is required
	os_timer_disarm(&watchdogTimer);
}

#pragma endregion



#pragma region "...TCP..."

//called every 5 sec from intervalTimer
//it sends a heartbeat message
static void heartbeat(void* arg) {
	AsyncClient* client = reinterpret_cast<AsyncClient*>(arg);

	queueMessage("ESPACC\n\0");

	return;
	// send reply to server
	if (client->space() > 32 && client->canSend()) {
		char message[] = "ESPACC\n\0";
		client->add(message, strlen(message));
		client->send();
	}
}

//event callback for TCP inbound data
static void handleData(void* arg, AsyncClient* client, void* data, size_t len) {
	if (verbose) {
		//do stuff

	}
	
	Serial.printf("\n%d bytes from server %s \n", len, client->remoteIP().toString().c_str());
	Serial.print("IN: ");
	Serial.write((uint8_t*)data, len);  //write the bytes as received

	//os_timer_arm(&intervalTimer, 5000, true); // schedule for reply to server at next 5s


	uint8_t* ptr = (uint8_t*)data;



}

void onConnect(void* arg, AsyncClient* client) {
	//Serial.printf("\n client has been connected to %s on port %d \n", SERVER_HOST_NAME, TCP_PORT);
	Serial.printf("\n client has been connected to %s on port %d \n", TCPserverIP.toString().c_str(), bootController.tcpPort);
	deviceState = S_TCP_CONNECTED;
	heartbeat(client);
	clientX = client;

	os_timer_arm(&intervalTimer, 5000, true); // schedule for reply to server at next 5s


}

void onDisconnect(void* arg, AsyncClient* client) {
	Serial.printf("\n client has disconnected\n");
	deviceState = S_TCP_RECONNECT;
	os_timer_disarm(&intervalTimer);
	clientX = nullptr;
}

void bootTCP(void) {
	uint8_t myIP[4];
	stringIPtoArray(bootController.tcpIP, myIP);
	TCPserverIP = IPAddress(myIP);




	Serial.printf("Attempt connect to TCP server %s on port %d\n", TCPserverIP.toString().c_str(), bootController.tcpPort);

	AsyncClient* client = new AsyncClient;
	client->onData(&handleData, client);
	client->onConnect(&onConnect, client);

	//client->onError(&onError, client);
	client->onDisconnect(&onDisconnect, client);

	client->connect(TCPserverIP, bootController.tcpPort);

	//does it reconnect automatically? NO.  You need to destroy object by setting a nullptr;
	//onDisconnect, then you can open a new client which will sit and wait for the server to reappear


	os_timer_disarm(&intervalTimer);
	os_timer_setfn(&intervalTimer, &heartbeat, client);
	//this timer is not triggered yet, see on_timer_arm()
	//make sense as there is no point arming it until we are connected



}

#pragma endregion


void sendEnqueuedMessages() {
	if (messages.size() == 0) return;
	std::string jumboMessage;

	for (auto m : messages) {
		jumboMessage.append(m);
	}


	if (jumboMessage.size() > 0) {
		char* data = new char[jumboMessage.size() + 1];
		copy(jumboMessage.begin(), jumboMessage.end(), data);
		data[jumboMessage.size()] = '\0';

		if (clientX->space() > sizeof(data) && clientX->canSend()) {
			clientX->add(data, strlen(data));
			clientX->send();

			if (verbose) {}//needs to be verbose and capture the serial lines
			Serial.print("OUT: ");
			Serial.write(jumboMessage.c_str());
			messages.clear();
		}
		//we used new to create *data.  delete now else you create a memory leak
		delete data;
	}

}







