// 
// 
// 

/*2026-05-17 bug. when updating the software version we are not writing bank1 and bank2 to eeprom, and/or when reading them
we get junk data in them.  e.g. device type is 234, power is 255 instead of 1=true.  The eeprom is 2202 which is big enough as
system says it needs 2160.  FIXED, i forgot to write those banks to eeprom duh.

processor load: the fp math is slow. when a servo is moving we keep recalculating its posn, when we could instead calculate a step size once based on 
the angle to move and the rate.  also once a servo is closed|thrown its position value does not change, so even if continous is enabled we should
not recalculate the same end position over and over.

even if we fix all this, with 32 servos we might have a math overload if we try to move all of them at once.
*/


/*Commands, uppercase set the wifi and TCP server params

S=set SSID
W=set SSID pwd
T=set TCP server IP
P=set TCP server port
D=dump param buffer
R=reboot
M=mode;  //toggles client or server
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




/*
2026-05-04 adding I2C.  will use pin0 for SCL and pin1 for SDA.






good idea to use BANKS, with bank 0 is the NodeMCU, bank1 is the first PCA and bank2 the second PCA
each PCA can then have 16 pins

NodeMCU hardware SZDOIT board
https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
D0 GPIO16 (also on-board LED active low)
D1 GPIO5 PWMA
D2 GPIO4 PWMB
D3 GPIO0 dir A (WPU, flash button, boot fails if low)
D4 GPIO2 dir B (WPU, boot fails if low. Also on-module LED, active low)
D5 GPIO14 
D6 GPIO12
D7 GPIO13
D8 GPIO15 (WPD, boot fails if hi)

Assuming you use active pull down sensors, then all pins bar D8 can host a sensor
The A0 analog input could also house a sensor.

So BANK0 CAN HAVE 9 io pins, we will map them as 0 through 8 to mirror the nodemcu naming
on the PCA9685 device it will be pins 0-15



*/

#include "ESPaccessory.h"
#include "ESPservo.h"
#include <stdint.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/examples/servo/servo.ino

const uint8_t NodeMCUmap[9] = {16,5,4,0,2,14,12,13,15};
//5,4 are commonly used for SCL,SDA on I2C
void processServo(void);


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
	long softwareVersion = 20260511;  //yyyymmdd captured as an integer
	char AP_SSID[21] = "ESPACC";   //local SSID when operating as a stand alone LocoNet server
	char AP_pwd[21] = "";
	char AP_IP[17] = "192.168.6.2\0";   //note the actual setting requires comma separators
	char STA_SSID[21] = "Ossonet\0";  //SSID when running as a station on an external WiFi network
	char STA_pwd[21] = "11223344AA\0";			//pwd for station
	char tcpIP[17] = "192.168.1.118\0";   //target IP of CONTROLLER to connect to
	uint16_t tcpPort = 1234;       //tcp port
	char Mode = 'S';  //C denotes client, S server and L as standalone wifi server
	bool hasPCA9685modules = false; //denotes PCA modules are present
	uint16_t PCAservoMin = 150;
	uint16_t PCAservoMax = 600;
	bool isDirty = false;  //will be true if EEPROM needs to be written
};

CONTROLLER bootController;
uint8_t bankSelect = 0;

/*Modes;
* C: connect via wifi to STA SSID and act as a client 
* S: connect via wifi to STA SSID and act as a loconet server
* L: act as a standalone wifi AP and act as a loconet server
*/



//++++++++++++ TCP IP ++++++++++++++++++++++++++++++++++++++++++++
IPAddress TCPserverIP;
AsyncClient* clientX; //points to currently active client
static std::vector<std::string> messages;

enum RELAY {
	S_BOOT_WIFI_STA_LOCONET_HOST,
	S_BOOT_WIFI_STA_LOCONET_CLIENT,
	S_BOOT_WIFI_AP_LOCONET_HOST,
	S_FAULT,
	S_TCP_CONNECTED_AS_CLIENT,
	S_TCP_RECONNECT_AS_CLIENT,
	S_TCP_PENDING_CONNECT_AS_CLIENT
};


bool heartbeatLEDstate = true;
bool MASledState = true;

static os_timer_t hearbeatTimer;

static os_timer_t servoTimer;
#define SERVO_TIMEOUT 15  //15ms


static uint8_t deviceState = S_BOOT_WIFI_AP_LOCONET_HOST;



//void payloadTimerCallback(void* pArg);
static os_timer_t payloadTimer;



//++++++++++++++++++++ TURNOUTS, SIGNALS AND SENSORS ++++++++++++++++++++++++++++++++++++++++++++++
#define ESP_TOTAL_PINS 9
#define ESP_BASE_PIN 0
#define ASPECT_PARAMETER_SIZE	8	//# of parameters in each MAS parameter array
#define MAS_EMPTY_VAL 255			//char which denotes a MAS parameter is not-set


enum DEVICE_TYPES : uint8_t{
	DEVICE_SERVO,
	DEVICE_ASPECT,
	DEVICE_MAS,
	DEVICE_SENSOR,
	DEVICE_SENSOR_WPU,
	DEVICE_I2C
};



bool MAScommandSync;

/*servo control.  VIRTUALSERVO is each virtualised device with its params.  Commanded over serial for testing
or DCC in normal operation. VIRTUALSERVO objects support both mechanical servos and LED aspect signals */
enum SERVOSTATE : uint8_t{
	SERVO_NEUTRAL,
	SERVO_TO_THROWN,
	SERVO_THROWN,
	SERVO_TO_CLOSED,
	SERVO_CLOSED,
	SERVO_BOOT,
	ASPECT_THROWN,
	ASPECT_CLOSED,
	ASPECT_MULTIPLE,
	SENSOR_HIGH,
	SENSOR_LOW,
	HEARTBEAT_LOW,
	HEARTBEAT_HIGH
};

struct VIRTUALSERVO {
	uint8_t bank;
	uint8_t pin;
	uint16_t address;
	uint8_t swing;
	bool invert;
	bool continuous;
	bool power;
	bool ignorePowerParameter;
	DEVICE_TYPES deviceType;
	SERVOSTATE state;
	uint8_t position;  //0-180 degrees
	int8_t rate;  //+ve values speed up movement, -ve slow it down
	int8_t timeDelay;  //working register, loaded negative and counts up to zero
	uint8_t aspectParameters[ASPECT_PARAMETER_SIZE * 4];
	uint8_t MASstate;  //Multiple Aspect Signal commanded state
};

//virtual servo objects, there are 9 in Bank 0 (the ESP12) and then 16 in each of Bank 1 and 2 which are PCA drivers
VIRTUALSERVO virtualservoCollection[ESP_TOTAL_PINS];
VIRTUALSERVO virtualservoCollectionBank1[16];
VIRTUALSERVO virtualservoCollectionBank2[16];

Adafruit_PWMServoDriver PCAbank1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver PCAbank2 = Adafruit_PWMServoDriver(0x41);



//function declaraions
uint8_t parseBracketedParameters(char* token, uint8_t* result);
bool validatePin(int p);
bool validateAddress(int address);
uint8_t assertMASoutput(VIRTUALSERVO vs);







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
void bootTCPclient(void);
void bootTCPserver(void);
void stringIPtoArray(char* s, uint8_t* myIP);
void eeGetSettings(void);
void eePutSettings(void);
void checkSerial(void);
static void sendEnqueuedMessages();
//void PCAservoAttach(VIRTUALSERVO vs, uint8_t bank, bool attach);
//void PCAservoWrite(VIRTUALSERVO vs, uint8_t bank);
void PCAservoWrite(VIRTUALSERVO *vs, uint8_t bank, bool attach);

//cannot put this in the header and then expect to use it in another namespace as it will cause a compiler error
//instead we have to return its value via a function
bool verbose;




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
	Serial.begin(115200);
	delay(200);
	Serial.setTimeout(500);

	verbose = false;
	eeGetSettings();
	
	//DEBUG
	verbose = true;

	Serial.println(F("For help use ?"));


	//choose boot mode
	switch (bootController.Mode) {
	case 'C':
		//boot as a client on building wifi, to interwork with my ESP_DCC_Controller project
		deviceState = S_BOOT_WIFI_STA_LOCONET_CLIENT;
		WiFi.mode(WIFI_STA);
		WiFi.setHostname("ESPACC");
		Serial.printf("Connecting to Wifi %s", bootController.STA_SSID);
		WiFi.begin(bootController.STA_SSID, bootController.STA_pwd);
		break;
	case 'L':
		//boot as a complete standalone AP running a loconet server
		{//scope block
		deviceState = S_BOOT_WIFI_AP_LOCONET_HOST;
		WiFi.mode(WIFI_AP);
		WiFi.setHostname("ESPACC");
		//wait for the softAP to start, then set the ip address
		delayMicroseconds(500);
		//IPAddress class requires the address to be provided as 4 octets
		uint8_t apIP[4];
		stringIPtoArray(bootController.AP_IP, apIP);
		IPAddress Ip(apIP[0], apIP[1], apIP[2], apIP[3]);
		IPAddress NMask(255, 255, 255, 0);
		WiFi.softAPConfig(Ip, Ip, NMask);  //static IP, gateway, subnet
		WiFi.softAP(bootController.AP_SSID, bootController.AP_pwd);
		Serial.print(F("AP started "));
		Serial.println(WiFi.softAPIP().toString());
		}
		break;
	case 'S':
	default:
		//boot as a loconet server on the building wifi
		deviceState = S_BOOT_WIFI_AP_LOCONET_HOST;
		WiFi.mode(WIFI_STA);
		WiFi.setHostname("ESPACC");
		Serial.printf("Connecting to Wifi %s", bootController.STA_SSID);
		WiFi.begin(bootController.STA_SSID, bootController.STA_pwd);
	}

	//for wifi client modes, wait for connection

	switch (bootController.Mode) {
	case 'L':  //standalone system
		bootTCPserver();
		break;

	case 'C':	//loconet client on home Wifi
	case 'S':   //loconet server on home Wifi
	default:    //loconet server on home Wifi
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
		if (bootController.Mode == 'C') {
			bootTCPclient();
		}
		else {
			bootTCPserver();
		}
		break;
		
	}
	

	//https://sub.nanona.fi/esp8266/hello-world.html
	os_timer_setfn(&servoTimer, (os_timer_func_t*)processServo, NULL);
	os_timer_arm(&servoTimer, SERVO_TIMEOUT, true);
	


}



void nsESPaccessory::ESPaccessoryLoop() {
	static unsigned long previousMillis;
	static unsigned long interval;
	unsigned long currentMillis = millis();

	//TCP connection state engine and heatbeat indicator state engine
	switch (deviceState) {
	case S_FAULT:
		break;
	case S_TCP_CONNECTED_AS_CLIENT:
		//LED: flash led 0.2 sec on, 5 off
		if (currentMillis - previousMillis >= interval) {
			//digitalWrite(16, ledState);  //led is active low, false=on    DEBUG disable
			interval = heartbeatLEDstate ? 5000 : 200;
			heartbeatLEDstate = !heartbeatLEDstate;
			previousMillis = currentMillis;
		}
		break;

	case S_TCP_RECONNECT_AS_CLIENT:
		bootTCPclient();
		deviceState = S_TCP_PENDING_CONNECT_AS_CLIENT;
		break;

	case S_TCP_PENDING_CONNECT_AS_CLIENT:
		//we kicked off ONE new attempt at a connect
		//LED: 1 sec on/off
		if (currentMillis - previousMillis >= 1000) {
			heartbeatLEDstate = !heartbeatLEDstate;
			previousMillis = currentMillis;

			//NOTE: this technique of only allowing one pending connect at a time seems to work.  The asyncTCP object does have a timeout on it, and will call disconnect
			//after about 60 sec.  At this point we will retry the connection.
			//There is a risk that old instances of the client remain in memory and after about 30 instances are created the ESP will run out of resources and crassh, i.e. 
			//its a memory leak.

		}
		break;

	case S_BOOT_WIFI_AP_LOCONET_HOST:
	case S_BOOT_WIFI_STA_LOCONET_HOST:
	case S_BOOT_WIFI_STA_LOCONET_CLIENT:
		//refactor. if we have a client we must be connected to wifi also
		if (clientX) {
			//LED: yes, we have a client. flash 0.2 then 5 sec
			if (currentMillis - previousMillis >= interval) {
				interval = heartbeatLEDstate ? 5000 : 200;  //active low
				heartbeatLEDstate = !heartbeatLEDstate;
				previousMillis = currentMillis;
			}
		}
		else {
			//no client but is Wifi active?
			if ((deviceState == S_BOOT_WIFI_AP_LOCONET_HOST) || (WiFi.isConnected())) {
				//LED: slow flash 1s
					if (currentMillis - previousMillis >= 1000) {
					heartbeatLEDstate = !heartbeatLEDstate;
					previousMillis = currentMillis;
				}
			}
			else {
				//LED: urgent flash 0.2 duty
				if (currentMillis - previousMillis >= 200) {
					heartbeatLEDstate = !heartbeatLEDstate;
					previousMillis = currentMillis;
				}
			}
		}
		break;

	}


	checkSerial();
	sendEnqueuedMessages();

}// end main loop

#pragma region "...TCP..."

//called every 10 sec from intervalTimer
//it sends a heartbeat message when acting as a TCP client
static void heartbeat(void* arg) {
	if (deviceState != S_TCP_CONNECTED_AS_CLIENT) return;
	AsyncClient* client = reinterpret_cast<AsyncClient*>(arg);
	queueMessage("ESPACC\n\0");
}

//event callback for TCP inbound data whether we are in server or client mode
static void handleData(void* arg, AsyncClient* client, void* data, size_t len) {
	if (verbose) {
		trace(Serial.printf("\n%d bytes from server %s \n", len, client->remoteIP().toString().c_str());)
			Serial.print("IN: ");
		Serial.write((uint8_t*)data, len);  //write the bytes as received
	}

	//uint8_t* ptr = (uint8_t*)data;
	nsLOCONETaccessoryProcessor::handleLocoNet(arg, client, data, len);
}

void onConnect(void* arg, AsyncClient* client) {
	//Serial.printf("\n client has been connected to %s on port %d \n", SERVER_HOST_NAME, TCP_PORT);
	//in client mode, this is us connecting to the host
	//in host mode, this is a new client connecting to us.


	if (bootController.Mode == 'C') {
		Serial.printf("\nclient connected to %s on port %d \n", TCPserverIP.toString().c_str(), bootController.tcpPort);
		deviceState = S_TCP_CONNECTED_AS_CLIENT;
		heartbeat(client);
		clientX = client;
		os_timer_arm(&hearbeatTimer, 10000, true); // schedule for reply to server at next 10s

	}
	else {
		//we are the host, and the client has a remoteIP
		Serial.printf("\nclient connection from %s\n", client->remoteIP());;
		clientX = client;
	}

}

void onDisconnect(void* arg, AsyncClient* client) {
	Serial.printf("\nclient disconnected\n");
	if (bootController.Mode == 'C') {
		deviceState = S_TCP_RECONNECT_AS_CLIENT;
		os_timer_disarm(&hearbeatTimer);
		clientX = nullptr;
	}
	else {
		//we are the host so no need to nullptr clientX which is our server
		Serial.printf("\nremote client disconnected\n");
		os_timer_disarm(&hearbeatTimer);
		clientX = nullptr;
	}

}

void bootTCPclient(void) {
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


	os_timer_disarm(&hearbeatTimer);
	os_timer_setfn(&hearbeatTimer, &heartbeat, client);
	//this timer is not triggered yet, see on_timer_arm()
	//make sense as there is no point arming it until we are connected

}



#pragma region "...TCP SERVER..."
static void handleNewClient(void* arg, AsyncClient* client) {
	if (verbose) { Serial.printf("\nnew client ip: %s", client->remoteIP().toString().c_str()); }

	// register events
	client->onData(&handleData, NULL);   //can use the same handle data routine as when we are a client
	clientX = client;  //this should work
	//client->onError(&handleError, NULL);
	//client->onDisconnect(&handleDisconnect, NULL);
	//client->onTimeout(&handleTimeOut, NULL);
}

void bootTCPserver(void) {
	//hold on, what's the wifi IP, so if we connect as 114 then the port below should be the same.
	//how did we do this in the controller? Actually you don't provide an IP, you provide a port.

	AsyncServer* server = new AsyncServer(bootController.tcpPort); // start listening on tcp port
	server->onClient(&handleNewClient, server);
	server->begin();
	Serial.println(F("boot as LOCONET server"));
}
#pragma endregion


void sendEnqueuedMessages() {
	if (messages.size() == 0) return;
	std::string jumboMessage;

	for (auto m : messages) {
		jumboMessage.append(m);
	}

	//2026-04-22 bug fix, if you queue messages but there's no client, the block below will cause a crash
	if (!clientX) {
		messages.clear();
		return;
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


void stringIPtoArray(char* s, uint8_t* myIP) {
	//IPAddress class requires the address to be provided as 4 octets
	char* p = nullptr;
	char ipBoot[17];
	strcpy(ipBoot, s);

	//strtok modifies its arguement, have to use a copy.
	p = strtok((char*)ipBoot, ",.");
	int i = 0;
	while (p != NULL) {
		myIP[i] = strtol(p, NULL, 10);
		i++;
		if (i == 4) break;
		//more data?
		p = strtok(NULL, ",.");
	}
	return;
}


#pragma endregion

#pragma region "...SERIAL ROUTINES..."

/// <summary>
/// Read incoming serial commands and action
/// </summary>
void checkSerial(void) {
	
	if (!Serial.available()) return;
	char SerialBuffer[64];
	Serial.readString().toCharArray(SerialBuffer, 64);

	//Note: its a quirk of readString but ParseBracketedParameters won't work with the SerialBuffer reliably
	//need to fix this in ParseBracketedParameters itself


	//need a temporary virtualservo object
	VIRTUALSERVO vsParse;
	//also need a pointer to a servo in virtualservoCollection
	VIRTUALSERVO* vsPointer = nullptr;


	//+++ TCP-IP and WIFI COMMANDS +++

	if (SerialBuffer[0] == 'T') {  //set TCPserver IP
			//set server address.  expect next params to be a series of integers
			const char s[2] = ".";
			char* token;
			uint8_t myIP[4];

			/* get the first token */
			token = strtok(SerialBuffer + 1, s);

			uint8_t i = 0;

			/* walk through other tokens */
			while (token != NULL) {
				//printf(" %s\n", token);
				myIP[i++] = strtol(token,NULL,10);
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

	if (SerialBuffer[0] == 'B') {
		//BE or BX enables/disables I2C communication to PCA modules
		//B0 B1 B2 selects servo banks, with B1+ as PCA
		//B? gives current bank.

		switch (SerialBuffer[1]){
		
		case '?':
			if (bootController.hasPCA9685modules) {
				Serial.printf("Bank %d active\n", bankSelect);
			}
			else {
				Serial.println(F("I2C not active, no PCA modules"));
			}
			break;
		case 'E':
			Serial.println(F("I2C activated. Now reboot."));
			bootController.hasPCA9685modules = true;
			bootController.isDirty = true;
			eePutSettings();
			break;
		case 'X':
			Serial.println(F("I2C deactivated. Now reboot."));
			bootController.hasPCA9685modules = false;
			bootController.isDirty = true;
			eePutSettings();
			break;
		case '0':
		case '1':
		case '2':
			//subtract 0x30
			bankSelect = SerialBuffer[1] - 0x30;
			Serial.printf("Bank %d active\n", bankSelect);
			break;
		}
	
	}




		if (SerialBuffer[0] == 'P') {
			//set tcp port
			uint16_t port = strtol(SerialBuffer + 1,NULL,10);

			//need a routine to restart the tcp client
			Serial.printf("Server port set as %d\n", port);
			Serial.println(F("Now you must REBOOT\n\n"));
			bootController.tcpPort = port;
			bootController.isDirty = true;
			eePutSettings();
		}

		if (SerialBuffer[0] == 'X') {
			//wifi and IP configs
			Serial.printf("\nSoftware ver %d\n", bootController.softwareVersion);
			Serial.print("MAC ");
			Serial.println(WiFi.macAddress());  // %s in printf does not work

			switch (bootController.Mode) {
			case 'S':
				Serial.printf("Network SSID %s\n", bootController.STA_SSID);
				Serial.println(F("Running as LocoNet HOST"));
				Serial.print(F("Loconet server IP "));
				Serial.println(WiFi.localIP().toString());
				Serial.printf("Loconet port %d\n\n", bootController.tcpPort);
				break;
			case 'L':
				Serial.println(F("Running as standalone LocoNet HOST"));
				Serial.printf("SSID %s\n", bootController.AP_SSID);
				Serial.print(F("LocoNet server IP "));
				Serial.println(WiFi.softAPIP().toString());
				Serial.printf("Loconet port %d\n\n", bootController.tcpPort);
				break;
			case  'C':
				Serial.printf("Network SSID %s\n", bootController.STA_SSID);
				Serial.println(F("Running as LocoNet CLIENT"));
				Serial.printf("Loconet server IP %s\n", bootController.tcpIP);
				Serial.printf("Loconet port %d\n\n", bootController.tcpPort);
			}

			Serial.printf("PCA min %d\n\n", bootController.PCAservoMin);
			Serial.printf("PCA max %d\n\n", bootController.PCAservoMax);
		}

		if (SerialBuffer[0] == 'S') {
			//set Station SSID
			char buffer[20];  //full of nulls
			strncpy(buffer, SerialBuffer + 1, 19);
			memset(bootController.STA_SSID, '\0', sizeof(bootController.STA_SSID));
			strncpy(bootController.STA_SSID, buffer, sizeof(bootController.STA_SSID));
			Serial.printf("STA SSID set to %s\n", buffer);
			Serial.println(F("Now you must REBOOT\n\n"));
			bootController.isDirty = true;
			eePutSettings();
		}

		if (SerialBuffer[0] == 'W') {
			//set Station SSID
			char buffer[21];  //full of nulls
			strncpy(buffer, SerialBuffer + 1, 20);  //max ssid or pwd is 20 char
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

		if (SerialBuffer[0] == '?') {
			Serial.println(F("S=set SSID"));
			Serial.println(F("W=set SSID pwd"));
			Serial.println(F("T=set TCP server IP"));
			Serial.println(F("P=set TCP server port"));
			Serial.println(F("D=dump param buffer"));
			Serial.println(F("R=reboot"));
			Serial.println(F("M=mode"));
			Serial.println(F("X=dump WIFI params\n\n"));
			Serial.println(F("x=dump servo params"));
			Serial.println(F("s=setup servo"));
			Serial.println(F("a=setup aspect"));
			Serial.println(F("A=setup MAS aspect"));
			Serial.println(F("k=setup a sensor"));
			Serial.println(F("p=command a pin"));
			Serial.println(F("r=set servo rate"));
			Serial.println(F("d or D=emulate DCC command"));
		}

		if (SerialBuffer[0] == 'R') {
			Serial.println(F("REBOOTING...\n\n"));
			ESP.restart();
		}

		if (SerialBuffer[0] == 'M') {
			//switch modes MS ML MC or M? to query current mode
			bootController.isDirty = true;

			switch (SerialBuffer[1])
			{
			case 'S':
				bootController.Mode = 'S';
				Serial.println(F("LocoNet HOST on home Wifi set\n"));
				break;	
			case 'L':
				bootController.Mode = 'L';
				Serial.println(F("Stand alone LocoNet HOST set\n"));
				break;
			case 'C':
				bootController.Mode = 'C';
				Serial.println(F("LocoNet CLIENT on home Wifi set\n"));
				break;
			case '?':
			default:
				bootController.isDirty = false;
				switch (bootController.Mode) {
				case 'S':
					Serial.println(F("S: LocoNet HOST on home Wifi\n"));
					break;
				case'L':
					Serial.println(F("L: Stand alone LocoNet HOST\n"));
					break;
				case'C':
					Serial.println(F("C: LocoNet CLIENT on home Wifi\n"));
				}
 			}

			if (bootController.isDirty) {
				Serial.println(F("Now you must REBOOT\n\n"));
				eePutSettings();
			}
		}


		if ((SerialBuffer[0] == 'V')||(SerialBuffer[0] == 'v')) {
			//toggle verbose
			verbose = !verbose;
			if (verbose) {
				Serial.println(F("verbose ON\n"));
			}
			else {
				Serial.println(F("verbose OFF\n"));
			}

		}

		//+++ SERVO SIGNAL AND SENSOR RELATED commands +++

		//ASPECT set-up command. Usage: a pin,addr,invert,[ignorePower]
		//ignorePower is default true and will ignore dcc power off commands
		if (SerialBuffer[0] == 'a') {
			bool resolved = true;

			//detokenize
			char* pch;
			int i = 0;
			pch = strtok(SerialBuffer, " ,");

			while (pch != NULL) {
				switch (i++) {
				case 1:
					vsParse.pin = strtol(pch, NULL, 10);
					resolved = validatePin(vsParse.pin);
					break;
				case 2:
					vsParse.address = strtol(pch, NULL, 10);
					//if out of range 1-2048 then throw an error
					resolved = validateAddress(vsParse.address);
					break;

				case 3:
					vsParse.invert = strtol(pch, NULL, 10) == 0 ? false : true;
					vsParse.ignorePowerParameter = true;  //default
					break;

				case 4:
					vsParse.ignorePowerParameter = strtol(pch, NULL, 10) == 0 ? false : true;
					break;
				}

				pch = strtok(NULL, " ,");
				if (!resolved) break;
			}

			if (((i == 4) || (i == 5)) && resolved) {
				Serial.println("OK");
				//match to a pin member of servoslot and copy it over  
				for (auto& vs : virtualservoCollection) {
					if (vs.pin == vsParse.pin) {
						//clear vsParse.aspectParameters to MAS_EMPTY_VAL, as this array is only used by multi aspect signals
						vs = vsParse;  //copy over from vsParse
						memset(vs.aspectParameters, MAS_EMPTY_VAL, 4 * ASPECT_PARAMETER_SIZE * sizeof(int8_t));
						vs.deviceType = DEVICE_ASPECT;
						vs.state = ASPECT_CLOSED;
						vs.ignorePowerParameter = true;
						vs.continuous = false;  //default setting
						vs.bank = 0;
						vs.rate = 0;
						ESPservoAttach(vs.pin, false);
						//write to EEPROM
						bootController.isDirty = true;
						eePutSettings();
						break;
					}
				}

			}
			else {
				Serial.println(F("bad command. usage a pin,addr,invert,[ignorePower]"));
			}

		}


		//SERVO set-up command. Usage: s pin, addr, swing, invert, [continuous]
		if (SerialBuffer[0] == 's') {
			//default settings
			vsParse.deviceType = DEVICE_SERVO;
			vsParse.ignorePowerParameter = true;
			vsParse.continuous = false; 
			vsParse.bank = 0;
			vsParse.position = 90;
			vsParse.rate = 0;
			vsParse.state = SERVO_TO_CLOSED;

			//detokenize
			bool resolved = true;
			char* pch;
			int i = 0;
			pch = strtok(SerialBuffer, " ,");
			
			while (pch != NULL) {
				switch (i++) {
				case 1:
					vsParse.pin = strtol(pch, NULL, 10);
					resolved = validatePin(vsParse.pin);
					break;
				case 2:
					vsParse.address = strtol(pch, NULL, 10);
					resolved = validateAddress(vsParse.address);
					break;
				case 3:
					vsParse.swing = strtol(pch, NULL, 10);
					if (vsParse.swing > 90) {
						resolved = false;
						Serial.println("bad swing range");
					}
					break;
				case 4:
					vsParse.invert = strtol(pch, NULL, 10) == 0 ? false : true;
					break;
				case 5:
					//this param is optional
					vsParse.continuous = strtol(pch, NULL, 10) == 0 ? false : true;
					break;
				}
				//zero is the s char

				pch = strtok(NULL, " ,");
				if (!resolved) break;
			}

			if (resolved && ((i == 6) || (i == 5))) {
				//[continuous] is optional, accept 5 || 6
				Serial.println("OK");
				//match to a pin member of virtualservoCollection and copy it over

				for (auto& vs : virtualservoCollection) {
					if (vs.pin != vsParse.pin) continue;
					//copy servoParse to vs
					vs = vsParse;
					memset(vs.aspectParameters, MAS_EMPTY_VAL, 4 * ASPECT_PARAMETER_SIZE * sizeof(int8_t));

					//write to EEPROM
					bootController.isDirty = true;
					eePutSettings();
					break;
				}
			}
			else
			{
				Serial.println("bad command. usage s pin,addr,swing,invert,[continuous]");
			}

		}

		//PIN ACTION. Usage: p pin, c|t|T|n , [power]
		// where closed|thrown|TOGGLE|neutral
		//power is 1|0 and only affects aspects
		if (SerialBuffer[0] == 'p') {
			//detokenize
			char* pch;
			int i = 0;
			pch = strtok(SerialBuffer, " ,");
			int p = -1;
			bool resolved = true;

			while (pch != NULL) {
				switch (i++) {
				case 1:
					//pin
					p = strtol(pch, NULL, 10);
					resolved = validatePin(p);
					if (!resolved) break;

					//p is valid, use this to lookup the servoslot
					for (auto& vs : virtualservoCollection) {
						if (vs.pin != p) continue;
						//use a pointer because we subsequently want to modify the collection item, not copy data to it
						vsPointer = (VIRTUALSERVO*)&vs;

						//2026-03-09 if this is a MAS signal, this command will not work
						if (vs.deviceType == DEVICE_MAS) {
							Serial.println(F("Cannot use command on MAS aspect"));
							resolved = false;
						}

						if (vs.deviceType == DEVICE_SENSOR) {
							Serial.println(F("Cannot use command on a sensor"));
							resolved = false;
						}
					}
					break;

				case 2:
					if (vsPointer == nullptr) { resolved = false;break; }

					if (vsPointer->deviceType==DEVICE_SERVO) {
						switch (pch[0]) {
						case 'c':
							vsPointer->state = SERVO_TO_CLOSED;
							break;
						case 't':
							vsPointer->state = SERVO_TO_THROWN;
							break;
						case 'n':
							vsPointer->state = SERVO_NEUTRAL;
							break;
						case 'T':
							vsPointer->state = vsPointer->state == SERVO_CLOSED ? SERVO_TO_THROWN : SERVO_TO_CLOSED;
						}
					}
					else if (vsPointer->deviceType == DEVICE_ASPECT) {
						//signal aspect. Only supports thrown or closed states
						switch (pch[0]) {
						case 't':
							vsPointer->state = ASPECT_THROWN;
							break;
						case 'T':
							vsPointer->state = (vsPointer->state == ASPECT_THROWN) ? ASPECT_CLOSED : ASPECT_THROWN;
							break;
						default:
							vsPointer->state = ASPECT_CLOSED;
							break;
						}
					}
					break;
				case 3:
					//optional [power] param for signal aspects
					vsPointer->power = pch[0] == '1' ? true : false;
				}

				pch = strtok(NULL, " ,");
				if (!resolved) break;
			}

			if (resolved && ((i == 3) || (i == 4))) {
				Serial.println("OK");
			}
			else
			{
				Serial.println("bad command. usage p pin,t|c|n|T,[power]");
			}

		}


		//EMULATE a dcc command.  This will affect all servos/aspects at a given dcc address
		//this code block can support T=toggle and n=neutral, which are not themselves a DCC command
		//usage: d addr,t|n|T|c,[power]
		if (SerialBuffer[0] == 'd') {
			char* pch;
			int i = 0;
			pch = strtok(SerialBuffer, " ,");
			int p = -1;
			int address = -1;
			bool resolved = true;

			while (pch != NULL) {
				switch (i++) {
				case 1:
					//resolve address
					address = strtol(pch, NULL, 10);
					resolved = validateAddress(address);
					break;

				case 2:
					//command.  Iterate all servos and execute on all matching addresses
					//the command only operates on DEVICE_SERVO and DEVICE_ASPECT
					for (auto& vs : virtualservoCollection) {
						if (vs.address != address) continue;
						if ((vs.deviceType != DEVICE_SERVO) && (vs.deviceType != DEVICE_ASPECT)) continue;


						switch (pch[0]) {
						case 't':
							vs.state = vs.deviceType==DEVICE_SERVO ? SERVO_TO_THROWN : ASPECT_THROWN;
							break;
						case 'n':
							vs.state = vs.deviceType == DEVICE_SERVO ? SERVO_NEUTRAL : ASPECT_CLOSED;
							break;
						case 'T':
							if (vs.deviceType == DEVICE_SERVO) {
								vs.state = (vs.state == SERVO_CLOSED) ? SERVO_TO_THROWN : SERVO_TO_CLOSED;
							}
							else {
								vs.state = (vs.state == ASPECT_THROWN) ? ASPECT_CLOSED : ASPECT_THROWN;
							}
							break;
						default: //also covers closed
							vs.state = vs.deviceType == DEVICE_SERVO ? SERVO_TO_CLOSED : ASPECT_CLOSED;
						}
					}
					break;
				case 3:
					//power.  Iterate all servos and execute on all matching addresses
					for (auto& vs : virtualservoCollection) {
						if (vs.address != address) continue;
						//power is only applicable to servo and aspect
						if ((vs.deviceType != DEVICE_SERVO) && (vs.deviceType != DEVICE_ASPECT)) continue;
						vs.power = pch[0] == '1' ? true : false;
					}
					break;
				}
				pch = strtok(NULL, " ,");
				if (!resolved) break;
			}

			if (resolved && ((i == 3) || (i == 4))) {
				Serial.println("OK");
			}
			else
			{
				Serial.println("bad command. usage d address,t|c|T|n,[power]");
			}

		}

		//EMULATE a dcc command for Multi Aspect Signal.  This will affect all MAS at a given dcc address
		//usage: D addr,state
		if (SerialBuffer[0] == 'D') {
			char* pch;
			int i = 0;
			pch = strtok(SerialBuffer, " ,");
			int address = -1;
			bool resolved = true;

			while (pch != NULL) {
				switch (i++) {
				case 1:
					//resolve address
					address = strtol(pch, NULL, 10);
					resolved = validateAddress(address);
					break;
				case 2:
					//state command.  Iterate all MAS and execute on all matching addresses
					uint8_t state = strtol(pch, NULL, 10);

					for (auto& vs : virtualservoCollection) {
						if (vs.address != address) continue;
						if (vs.deviceType != DEVICE_MAS) continue;
						vs.MASstate = state;
						MAScommandSync = false;
						assertMASoutput(vs);
						if (verbose) Serial.printf("Mas to %d\n",state);
					}
					break;
				}
				pch = strtok(NULL, " ,");
				if (!resolved) break;
			}

			if (resolved && (i >= 3)) Serial.println("OK MAS");
		}

		//SENSOR command. sets up a sensor on a given pin, by default will be WPU a zero param is given
		//some pins have pulldowns on the board and the WPU may not be enough to overcome these.
		//usage k pin address [wpu]
		if (SerialBuffer[0] == 'k') {
			char* pch;
			int i = 0;
			pch = strtok(SerialBuffer, " ,");
			int p = -1;
			int address = -1;
			bool resolved = true;
			//default setting
			vsParse.deviceType = DEVICE_SENSOR;
			vsParse.ignorePowerParameter = true;
			vsParse.continuous = false; 
			vsParse.position =0;
			vsParse.state = SERVO_BOOT;

			while (pch != NULL) {
				switch (i++) {
				case 1:
					p = strtol(pch, NULL, 10);
					resolved = validatePin(p);
					if (!resolved) break;
					vsParse.pin = p;
					break;

				case 2:
					//resolve address
					address = strtol(pch, NULL, 10);
					resolved = validateAddress(address);
					vsParse.address = address;
					break;

				case 3:
					//optional WPU
					if (strtol(pch, NULL, 10) == 0) break;
					vsParse.deviceType = DEVICE_SENSOR_WPU;
					break;
				}
			
				pch = strtok(NULL, " ,");
				if (!resolved) break;
			}

			if (resolved && (i >= 2)) {
				Serial.println("OK");
				for (auto& vs : virtualservoCollection) {
					if (vs.pin != vsParse.pin) continue;
					//copy servoParse to vs
					vs = vsParse;
					memset(vs.aspectParameters, MAS_EMPTY_VAL, 4 * ASPECT_PARAMETER_SIZE * sizeof(int8_t));
					ESPservoAttach(vs.pin, false);
					//write to EEPROM
					bootController.isDirty = true;
					eePutSettings();
					break;
				}

			}
			else
			{
				Serial.println("bad command. usage k pin address [wpu]");
			}

		}

		

		//RATE command. sets a positive or negative rate on the servo swing.
		//usage r pin rate, where rate is + or -ve integer, useful values are -10 to +10
		if (SerialBuffer[0] == 'r') {
			char* pch;
			int i = 0;
			pch = strtok(SerialBuffer, " ,");
			int p = -1;
			int rate = 0;
			bool resolved = true;

			while (pch != NULL) {
				switch (i++) {
				case 1:
					p = strtol(pch, NULL, 10);
					resolved = validatePin(p);
					if (!resolved) break;

					//p is valid, use this to lookup the servoslot
					for (auto& vs : virtualservoCollection) {
						if (vs.pin != p) continue;
						//use a pointer because we subsequently want to modify the collection item, not copy data to it
						vsPointer = (VIRTUALSERVO*)&vs;
					}
					break;

				case 2:
					//resolve rate
					rate = strtol(pch, NULL, 10);
					if (rate > 10) rate = 10;
					if (rate < -10) rate = -10;
					//note improperly formed numbers such as -7.7 or 'three' will resolve to zero
					break;
				}

				pch = strtok(NULL, " ,");
				if (!resolved) break;
			}

			if (resolved && (i == 3)) {
				Serial.println("OK");
				vsPointer->rate = rate;
				bootController.isDirty = true;
				eePutSettings();
			}
			else
			{
				Serial.println("bad command. usage r pin rate");
			}
		}

		if (SerialBuffer[0] == 'j') {
		//DEBUG
			Serial.printf("power %d\n", virtualservoCollectionBank1[0].power);
			Serial.printf("cont %d\n", virtualservoCollectionBank1[0].continuous);
			Serial.printf("posn %d\n", virtualservoCollectionBank1[0].position);
			Serial.printf("state %d\n", virtualservoCollectionBank1[0].state);
			Serial.printf("device %d\n\n", virtualservoCollectionBank1[0].deviceType);
			Serial.printf("power %d\n", virtualservoCollectionBank1[1].power);
			Serial.printf("cont %d\n", virtualservoCollectionBank1[1].continuous);
			Serial.printf("posn %d\n", virtualservoCollectionBank1[1].position);
			Serial.printf("state %d\n", virtualservoCollectionBank1[1].state);
			Serial.printf("device %d\n", virtualservoCollectionBank1[1].deviceType);
		}


		//DUMP all servo/aspect information
		if (SerialBuffer[0] == 'x') {

			for (auto vs : virtualservoCollection) {
				//rewrite as switch
				switch (vs.deviceType) {
				case DEVICE_SENSOR:
				case DEVICE_SENSOR_WPU:
					Serial.print(F("sensor  pin "));
					Serial.print(vs.pin, DEC);
					Serial.print(F("  address "));
					Serial.print(vs.address, DEC);
					if (vs.deviceType == DEVICE_SENSOR) break;
					Serial.print(F(" WPU "));
					break;

				case DEVICE_SERVO:
					//special case for pin 0
					if (vs.pin == 0) {
						Serial.print(F("heartbeat LED pin 0"));
						break;
					}

					Serial.print(F("servo  pin "));
					Serial.print(vs.pin, DEC);
					Serial.print(F("  address "));
					Serial.print(vs.address, DEC);
					Serial.print(F("  swing "));
					Serial.print(vs.swing, DEC);
					Serial.print(F("  invert "));
					Serial.print(vs.invert, DEC);
					Serial.print(F("  continuous "));
					Serial.print(vs.continuous, DEC);
					Serial.print(F("  rate "));
					Serial.print(vs.rate, DEC);
					break;

				case DEVICE_ASPECT:
					Serial.print(F("aspect pin "));
					Serial.print(vs.pin, DEC);
					Serial.print(F("  address "));
					Serial.print(vs.address, DEC);
					Serial.print(F("  invert "));
					Serial.print(vs.invert, DEC);
					Serial.print(F("  power "));
					if (vs.ignorePowerParameter) { Serial.print("x"); }
					else { Serial.print(vs.power, DEC); }
					break;
				case DEVICE_I2C:
					Serial.print(F("I2C pin "));
					break;
				case DEVICE_MAS:
					Serial.print(F("MAS pin "));
					Serial.print(vs.pin, DEC);
					Serial.print(F("  address "));
					Serial.print(vs.address, DEC);
					Serial.print(F("  invert "));
					Serial.print(vs.invert, DEC);

					Serial.print(F("  output "));
					switch (assertMASoutput(vs)) {
					case 0:
						Serial.print("0 ");
						break;
					case 1:
						Serial.print("1 ");
						break;

					default:
						Serial.print("tristate ");
					}

					//now the param arrays
					bool noSpace = true;
					for (uint8_t a = 0;a < 32;a++) {
						if (a == 0) {
							Serial.print(" hi[");
							noSpace = true;
						}
						if (a == ASPECT_PARAMETER_SIZE - 1) {
							Serial.print("] lo[");
							noSpace = true;
						}
						if (a == (2 * ASPECT_PARAMETER_SIZE) - 1) {
							Serial.print("] hi-flash[");
							noSpace = true;
						}
						if (a == (3 * ASPECT_PARAMETER_SIZE) - 1) {
							Serial.print("] lo-flash[");
							noSpace = true;
						}

						if (vs.aspectParameters[a] == MAS_EMPTY_VAL) continue;
						if (!noSpace) Serial.print(" ");
						Serial.print(vs.aspectParameters[a], DEC);
						noSpace = false;
					}
					Serial.print("]");
					break;
				
				}//end switch


				
					//dump output state
					switch (vs.state) {
					case SENSOR_HIGH:
						Serial.print(F(" hi" ));
						break;
					case SENSOR_LOW:
						Serial.print(F(" lo "));
						break;
					case ASPECT_MULTIPLE:
						Serial.print(F(" state "));
						Serial.print(vs.MASstate, DEC);
						break;
					case ASPECT_THROWN:
					case SERVO_THROWN:
					case SERVO_TO_THROWN:
						Serial.print(F(" thrown"));
						break;
					default:
						Serial.print(F(" closed"));
						break;
					}
				
					//DEBUG state attach status
					//if (ESPservoIsAttached(vs.pin)) {
						//Serial.print(" AT ");
					//}else{ Serial.print(" U "); }


				Serial.print("\n");
			}
		}


		//Advanced aspect signalling.  A pin addr invert [hi array] [low array] [hi flash array] [low flash array]
		if (SerialBuffer[0] == 'A') {
			//make use of vsParse to hold params as we verify them
			vsParse.pin = -1;
			vsParse.address = -1;
			char* pch;
			uint8_t i = 0;   //i is a state engine, its not auto incremented in the switch statement because there's a sub-state engine in case 4,5,6,7
			pch = strtok(SerialBuffer, " ,");
			bool resolved = true;
			uint8_t bufOffset = 0;
			//default is all output drivers are tri-state, await first dcc command
			vsParse.deviceType = DEVICE_MAS;
			vsParse.state = ASPECT_MULTIPLE;
			vsParse.power = false;  
			vsParse.ignorePowerParameter = true;
			vsParse.invert=false;
			vsParse.MASstate = 127; //default


			while (pch != NULL) {
				switch (i) {
				case 0:
					//ignore A character at start of command
					i++;
					bufOffset = 0;
					break;

				case 1:
					i++;
					vsParse.pin = strtol(pch, NULL, 10);
					resolved = validatePin(vsParse.pin);
					break;

				case 2:
					//resolve address
					i++;
					vsParse.address = strtol(pch, NULL, 10);
					resolved = resolved && validateAddress(vsParse.address);
					break;

				case 3:
					vsParse.invert = strtol(pch, NULL, 10) == 0 ? false : true;
					i++;
					break;

				case 4:
				case 5:
				case 6:
				case 7:
					//expect 4 sets of [bracketed params]
				{//scope A
				//we keep looking at tokens until we find a [ start then we import to ] and declare end (2).
					switch (parseBracketedParameters(pch, vsParse.aspectParameters + bufOffset))
					{
					case 2:
						bufOffset += ASPECT_PARAMETER_SIZE;
						i++;
						break;

					case 4:
						//parser error in bracketed params
						resolved = false;
						break;
					default:
						break;
					}

				}//end scope A
				break;

				case 8://optional flash rate param
					i++;
					break;
				}//switch


				pch = strtok(NULL, " ,");
				if (!resolved) 	break;

			}//while

			if (i < 8) resolved = false;

			//next we expect [a b c] where a b c can be [] through to 8 sets of digits
			if (resolved) {
				//all params were captured into vsParse

				for (auto& vs : virtualservoCollection) {
					if (vs.pin != vsParse.pin) continue;
					//copy received data to the item
					vs = vsParse;
					memcpy(vs.aspectParameters, vsParse.aspectParameters, 4 * ASPECT_PARAMETER_SIZE * sizeof(int8_t));
					ESPservoAttach(vs.pin, false);
					break;
				}
				bootController.isDirty = true;
				eePutSettings();
				Serial.println("OK");
			}
			else {
				Serial.println(F("Error parsing command. Usage A pin addr invert [hi] [lo] [hi-flash] [low-flash]"));
			}

		}





}  //end checkSerial()






#pragma region ".... SERVOS SIGNALS AND SENSORS ....."

//helper functions
	/// <summary>
	/// validates a given pin is valid on this device
	/// </summary>
	/// <param name="p">pin number</param>
	/// <returns>true if valid</returns>
bool validatePin(int p) {
	if ((p < ESP_BASE_PIN) || (p >= ESP_BASE_PIN + ESP_TOTAL_PINS)) {
		Serial.printf("bad pin %d\n",p);
		return false;
	}
	return true;
}

/// <summary>
/// validates if a DCC accessory address is in the range 1 to 2048
/// </summary>
/// <param name="address">DCC address</param>
/// <returns>true if valid</returns>
bool validateAddress(int address) {
	//valid DCC addresses are in the range 1 to 2048
	if ((address < 1) || (address > 2048)) {
		Serial.println("bad address");
		return false;
	}
	return true;
}


/*
/// <summary>
/// test if the vs object is a Multiple Aspect Signal device
/// </summary>
/// <param name="vs">target vs</param>
/// <returns>true if a MAS device</returns>
bool isMASdevice(VIRTUALSERVO vs) {
	//Note; could make this a member function of VIRTUALSERVO - but it will increase struct size by a pointer var.
	//if all aspectParameter arrays are full of MAS_EMPTY_VAL then the device is not a MAS device
	for (int a = 0;a < ASPECT_PARAMETER_SIZE * 4; a++) {
		if (vs.aspectParameters[a] != MAS_EMPTY_VAL) return true;
	}
	return false;
}
*/


/// <summary>
/// call from a strtok loop, repeatedly passing in tokens. It will parse looking for [aa bb cc]
/// max of 8 parameters, min no params.  params must be enclosed in square brackets.
/// params values can be 0-254.  255 is a reserved value.
/// </summary>
/// <param name="token">null terminated char string</param>
/// <param name="result">array to receive the numeric parameters recovered</param>
/// <returns>2 when the entire set of max 8 tokens is parsed.  non-values are represented as 255 </returns>
uint8_t parseBracketedParameters(char* token, uint8_t* result) {
	//known bug: you can pass in params values >254 or negative and they will be stored/truncated into the parameter arrays with
	//unpredictable outcomes.  They are not validated here.

	//bug fix, run through token and replace any cr/lf null.  Serial.readString will capture a cr or lf as its last character
	//and this causes the parser to fail on an otherwise valid string.
	for (int j = strlen(token) - 1;j > 0;j--) {
		if (token[j] == '\n') token[j] = '\0';
		if (token[j] == '\r') token[j] = '\0';
	}

	static int8_t arr[ASPECT_PARAMETER_SIZE];
	static uint8_t c = 0;
	static uint8_t parserState = 0;
	char* endptr;
	/*0 not in block
	1 in block
	2 block ended, valid result
	4 format error
	if you call again for state other than 1, it will revert to case 0
	*/

	//self reset
	if (parserState != 1) parserState = 0;
	bool foundStart = token[0] == '[' ? true : false;
	bool foundEnd = token[strlen(token) - 1] == ']' ? true : false;


	switch (parserState) {
	case 0:  //not in a [] block
		if (!foundStart) break;
		parserState = 1;
		//fill arr[] with MAS_EMPTY_VAL
		for (c = 0;c < 8;c++) {
			arr[c] = MAS_EMPTY_VAL;
		}
		c = 0;

		//[] is a special case
		if ((strlen(token) == 2) && foundEnd) {
			parserState = 2;
			break;
		}

		arr[c++] = strtol(++token, &endptr, 10);

		//if the conversion worked and used entire string, *endptr==NULL
	//more useful if endptr==p then nothing was converted
		if (endptr == token) {
			parserState = 4;  //declare fail
			break;
		}

		if ((*endptr != '\0') && (*endptr != ']')) {
			parserState = 4;  //declare fail
			break;
		}

		//its possible this is [x]
		if (foundEnd) {
			parserState = 2;
		}
		//note that ]] results in no start found and [[ results in 1
		break;



	case 1:  //in a [] block
		if (foundStart) {
			//re start is an error
			parserState = 4;
			break;
		}

		arr[c++] = strtol(token, &endptr, 10);
		if (endptr == token) {
			parserState = 4;  //FAIL
			break;
		}

		if ((*endptr != '\0') && (*endptr != ']')) {
			parserState = 4;  //FAIL
			break;
		}


		if (c > ASPECT_PARAMETER_SIZE) {
			//too many params
			parserState = 4;
			break;
		}

		if (foundEnd) {
			parserState = 2;
		}
		break;

	}

	if (parserState == 2) {
		//copy static array to result.  If state=fail, result is left untouched
		//WARNING: Array Decay: When an array is passed as an argument to a function, it decays into a pointer to its first element.
		//this means sizeof tests can yeild unpredctable behaviour.  The safer approach is to use ASPECT_PARAMETER_SIZE

		for (int a = 0;a < ASPECT_PARAMETER_SIZE; a++) {
			result[a] = arr[a];
		}
	}

	return parserState;
};


/// <summary>
/// Asserts the pin output state based on the MASstate of the vs object.
/// Default state, including boot (which sets MASstate=127) or no-match on the MASstate code will be tristate.
/// If MASstate code does appear in one/more of the parameter arrays, it will drive the pin active hi|low or the flash variants thereof.
/// </summary>
/// <param name="vs">target virtual servo</param>
/// <returns>pin output state just set: 0 low, 1 hi, 2 tristate.</returns>
uint8_t assertMASoutput(VIRTUALSERVO vs) {
	/*All MAS pins will boot as power=off, i.e. tristate
	* if no MAS command code is reserved, we exit tristate
	* flash-lo and flash-hi, if resolved are gated with the LEDstate. This allows a pin to go low-high flashing if the same command
	* is present in [hi-flash] [hi-flash] e.g. A pin addr 0 [] [] [7] [7] will alternate hi-lo on the output
	* if [hi] [lo] codes also match, then these take precedence over flash, and high takes precedence over low
	* e.g. A pin addr 0 [7] [7] [7] [7] will always drive hi on resolving a 7
	* invert is applied to the final pin state after the brackets are resolved
	*/


	//note, arduino.h defines constants of LOW=0, HIGH=1 but no tristate
	//this is why we use our own states here
	enum output {
		LO,
		HI,
		TRISTATE,
	};

	uint8_t outputState = TRISTATE;

	for (int a = (ASPECT_PARAMETER_SIZE * 4) - 1;a > -1;a--) {
		switch (int(a / ASPECT_PARAMETER_SIZE)) {
		case 3:
			//flash low
			if (vs.aspectParameters[a] == vs.MASstate) {
				//assert low gated with ledState == low, this allows flash lo/hi to work on the same pin
				if (!MASledState) outputState = LO;
			}
			break;

		case 2:
			//flash high
			if (vs.aspectParameters[a] == vs.MASstate) {
				//assert hi gated with ledState == high, this allows flash lo/hi to work on the same pin
				if (MASledState) outputState = HI;
			}
			break;

		case 1:
			//solid low
			if (vs.aspectParameters[a] == vs.MASstate) {
				outputState = LO;
			}
			break;

		case 0: //solid hi, which has highest precedence
			if (vs.aspectParameters[a] == vs.MASstate) {
				outputState = HI;
			}
			break;
		}

	}

	uint8_t gpioPin = NodeMCUmap[vs.pin];
	
	//assert the output pin, also process invert
	switch (outputState) {
	case TRISTATE:
		pinMode(gpioPin, INPUT);
		break;

	case LO:
		outputState = vs.invert ? HI : LO;
		pinMode(gpioPin, OUTPUT);
		digitalWrite(gpioPin, outputState == LO ? LOW : HIGH);
		break;

	case HI:
		outputState = vs.invert ? LO : HI;
		pinMode(gpioPin, OUTPUT);
		digitalWrite(gpioPin, outputState == LO ? LOW : HIGH);
		break;

	}

	return outputState;
}





void nsESPaccessory::commandTurnout(int16_t addr, bool thrown) {
	//find all aspects and servos with addr, and assert thrown state
	for (auto& vs : virtualservoCollection) {
		if (vs.address != addr) continue;
		switch (vs.deviceType) {
		case DEVICE_SERVO:
			vs.state = thrown ? SERVO_TO_THROWN : SERVO_TO_CLOSED;
			break;
		case DEVICE_ASPECT:
			vs.state = thrown ? ASPECT_THROWN : ASPECT_CLOSED;
			break;
		}
	}
	if (verbose) {
		Serial.printf("address %d ", addr);
		if (thrown) {
			Serial.println("thrown");
		}
		else {
			Serial.println("closed");
		}
	}
}

void nsESPaccessory::commandMAS(int16_t addr, uint8_t state) {
	//find all MAS with addr, and assert new state
	for (auto& vs : virtualservoCollection) {
		if (vs.address != addr) continue;
		if (vs.deviceType != DEVICE_MAS) continue;
		//set the new state, processServo() will act on this
		vs.MASstate = state;
	}
}


bool nsESPaccessory::pollSensor(int16_t addr) {
	for (auto vs : virtualservoCollection) {
		if (vs.address != addr) continue;
		if (vs.deviceType != DEVICE_SENSOR) continue;
		return digitalRead(NodeMCUmap[vs.pin]);
	}
	
	return true; 
}






#pragma endregion




const int EEOFFSET = 192;


/// <summary>
/// If booting a new system, set virtual servos to their default settings
/// </summary>
/// <param name="vsc">virtual servo collection</param>
/// <param name="pinCount">total pins</param>
/// <param name="isPCAbank">true if a PCA bank</param>
void setVirtualServoDefaults(VIRTUALSERVO vsc[], uint8_t pinCount, bool isPCAbank) {
	//cpp arrays decay to a pointer to the first element and size is lost
	//cannot use auto, we just have to iterate array
	static uint8_t totalPins;

	for (uint8_t p = 0;p < pinCount;p++) {
		totalPins++;
		auto& vs = vsc[p];
		//initialise the pin assignments move all servos and aspects to closed position
		vs.pin = p;
		vs.invert = 0;
		vs.position = 90;
		vs.swing = 25;
		vs.continuous = 0;
		vs.state = SERVO_BOOT;
		vs.power = false;
		vs.ignorePowerParameter = true;
		vs.deviceType = DEVICE_SERVO;
		vs.rate = 0;
		memset(vs.aspectParameters, MAS_EMPTY_VAL, 4 * ASPECT_PARAMETER_SIZE * sizeof(int8_t));
	}

	//debug
	Serial.printf("total pins %d\n", totalPins);
}


/*
void setVirtualServoDefaults(VIRTUALSERVO(*ptrToVSC)[16], uint8_t pinCount, bool isPCAbank) {
	uint8_t p = 0;
	for (auto& vs : *ptrToVSC) {
		vs.pin = p++;
		vs.invert = 0;
		vs.position = 90;
		vs.swing = 25;
		vs.continuous = 0;
		vs.state = SERVO_BOOT;
		vs.power = true;
		vs.ignorePowerParameter = true;
		vs.deviceType = DEVICE_SERVO;
		vs.rate = 0;
		memset(vs.aspectParameters, MAS_EMPTY_VAL, 4 * ASPECT_PARAMETER_SIZE * sizeof(int8_t));
	}
}
*/









/// <summary>
/// boot a virtual servo on power up from its programmed settings
/// </summary>
/// <param name="vsc">virtual servo collection</param>
/// <param name="pinCount">total pins</param>
/// <param name="isPCAbank">true if a PCA bank</param>
void bootVirtualServo(VIRTUALSERVO vsc[], uint8_t pinCount, bool isPCAbank) {
	for (uint8_t p = 0;p < pinCount;p++) {
		auto& vs=vsc[p];  //we need to modify vsc, not modify a copy of the element!
		vs.pin = p;
		vs.state = SERVO_BOOT;
		vs.MASstate = 127;
		//vs.position = 90;  //neutral
		/*minimum useful swing is 5 degrees*/
		if ((vs.swing < 5) || (vs.swing > 90)) vs.swing = 5;

		//calculate closed posn (we may be inverted) then back off 5 degrees and set that as posn
		if (vs.invert) {
			//max position
			vs.position = 90 + vs.swing - 5;
		}
		else {
			//min position, normal for closed
			vs.position = 90 - vs.swing + 5;
		}

		//2026-02-13 for some reason, address data is corrupt when reading back pin 3 parameters.
		//try masking out irrelevant bits.  FFF allows 2048d, whereas 7FF allows upto 2047d
		//vs.address &= 0xFFF;
			

		if (!bootController.hasPCA9685modules) continue;
		if (isPCAbank) continue; 
		if (p > 0) continue;
		//initialise Wire on bank0, pins 0 (SCL) and 1 (SDA)
		//nah, use D4 and D5.   D0 is an RTC pin and prob does not work with Wire.
		Wire.begin(NodeMCUmap[4], NodeMCUmap[5]);  //sda,scl
		vsc[4].deviceType = DEVICE_I2C;
		vsc[5].deviceType = DEVICE_I2C;

		/*I2C device scan*/
		byte error, address;
		int nDevices;

		Serial.println(F("I2C scanning..."));
		nDevices = 0;
		for (address = 1; address < 127; ++address)
		{
			// The i2c_scanner uses the return value of
			// the Write.endTransmisstion to see if
			// a device did acknowledge to the address.
			Wire.beginTransmission(address);
			error = Wire.endTransmission();

			if (error == 0)
			{
				Serial.print("I2C device found at address 0x");
				if (address < 16)
					Serial.print("0");
				Serial.print(address, HEX);
				Serial.println(" !");
				++nDevices;
			}
			else if (error == 4)
			{
				Serial.print(F("Unknown error at address 0x"));
				if (address < 16)
					Serial.print("0");
				Serial.println(address, HEX);
			}
		}

		if (nDevices == 0) {
			Serial.println(F("No I2C devices found\n"));
		}
		else {
			if (bootController.hasPCA9685modules) {
				//IMPORTANT: Wire must be started before calling pwm.begin
				PCAbank1.begin();
				PCAbank1.setPWMFreq(50);
				delay(10);
				PCAbank2.begin();
				PCAbank2.setPWMFreq(50);
				Serial.println(F("boot PCA modules"));
				
				//debug, this works
				//PCAbank1.setPin(0, 4000, false);
				//PCAbank1.setPin(15, 1024, true);


			}


		}
		//there's no means to check a device is a PCA device as these do not have ID codes





	}
}




/// <summary>
/// restores settings from EEPROM. If the software version has changed, we overwrite the eeprom with defaults
/// we also need to clear certain values on boot. max EEPROM we can use is 4096
/// </summary>
void eeGetSettings(void) {
	CONTROLLER defaultController;  //grab defaults
	//EEPROM.begin(1024);  //ESP does not have dedicated eeprom and must be allocated from Flash
	EEPROM.begin(2200);  //ESP does not have dedicated eeprom and must be allocated from Flash.  Need 2200 for 3 banks
	int eeAddr = 0;
	bool factory = false;
	EEPROM.get(eeAddr, bootController);
	if (defaultController.softwareVersion != bootController.softwareVersion) {
		/*need to re-initiatise eeprom with factory defaults*/
		/*If software version has changed, we need to re-initiatise eeprom with factory defaults*/
		Serial.println("restore factory defaults");
		factory = true;
		EEPROM.put(0, defaultController);
		//eeAddr += sizeof(defaultController);
		eeAddr = EEOFFSET; //bug fix

		//set defaults for all virtual servo groups

		setVirtualServoDefaults(virtualservoCollection, ESP_TOTAL_PINS, false);
		setVirtualServoDefaults(virtualservoCollectionBank1, 16, true);
		setVirtualServoDefaults(virtualservoCollectionBank2, 16, true);

		
		/*write back default values*/
		EEPROM.put(eeAddr, virtualservoCollection);
		eeAddr += sizeof(virtualservoCollection);
		//and banks 1 & 2
		EEPROM.put(eeAddr, virtualservoCollectionBank1);
		eeAddr += sizeof(virtualservoCollectionBank1);
		EEPROM.put(eeAddr, virtualservoCollectionBank2);
		eeAddr += sizeof(virtualservoCollectionBank2);

	}
		/*either way, now populate our structs with EEprom values*/
		eeAddr = 0;
		EEPROM.get(eeAddr, bootController);
		//eeAddr += sizeof(bootController);
		eeAddr = EEOFFSET; //bug fix
		EEPROM.get(eeAddr, virtualservoCollection);
		eeAddr += sizeof(virtualservoCollection);
		//and banks 1 & 2
		EEPROM.get(eeAddr, virtualservoCollectionBank1);
		eeAddr += sizeof(virtualservoCollectionBank1);
		EEPROM.get(eeAddr, virtualservoCollectionBank2);
		eeAddr += sizeof(virtualservoCollectionBank2);
		
		Serial.print(F("EEPROM="));
		Serial.println(eeAddr, DEC);
		bankSelect = 0;  //select bank 0 on startup

		//initialise the pin assignments move all servos and aspects to closed position
				
		bootVirtualServo(virtualservoCollection, ESP_TOTAL_PINS,false);
		bootVirtualServo(virtualservoCollectionBank1, 16, true);
		bootVirtualServo(virtualservoCollectionBank2, 16, true);
			

		//2026-04-05 for some reason bootVS means we no longer see the boot 0,1,2,3 etc messages
		//how nothing to do with PCA because I am not calling setVSD

		Serial.print(F("\nsofware version "));
		Serial.println(bootController.softwareVersion, DEC);
		if (factory) Serial.println(F("factory reset"));

}




/// <summary>
/// save settings to EEPROM, we save the bootController struct
/// </summary>
void eePutSettings(void) {
	if (bootController.isDirty == false) { return; }
	int eeAddr = 0;
	EEPROM.put(eeAddr, bootController);
	eeAddr += sizeof(bootController);  //is approx 128 bytes
	eeAddr = EEOFFSET;  //bug fix, to be safe, move to a higher block

	EEPROM.put(eeAddr, virtualservoCollection);
	eeAddr += sizeof(virtualservoCollection);
//and banks 1 and 2
	EEPROM.put(eeAddr, virtualservoCollectionBank1);
	eeAddr += sizeof(virtualservoCollectionBank1);
	EEPROM.put(eeAddr, virtualservoCollectionBank2);
	eeAddr += sizeof(virtualservoCollectionBank2);


	EEPROM.commit();
	trace(Serial.printf("EEPROM commit, bytes %d\r\n", eeAddr);)
	bootController.isDirty = false;
}

/*
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
*/

#pragma endregion



bool nsESPaccessory::getVerbose(void) {
	return verbose;
}





/// <summary>
/// PROCESS SERVO POSITIONS AND SIGNAL ASPECTS.  Call every 15mS from a timer.
/// </summary>
void processServo(void) {
	static VIRTUALSERVO* vsBoot = nullptr;
	static uint8_t bootTimer = 0;
	static uint8_t tick;
	
	//IMPORTANT: vs.pin does represent the GPIO pin and not the ordinal in the virtualServoCollection


	if (tick++ > 33) {
		//0.5sec counter used to toggle MAS leds and also sync to the edge
		tick = 0;
		MAScommandSync = true;
		MASledState = !MASledState;
	}


	

	//in normal non-invert mode, minPosition is turnout closed, and maxPosition is turnout thrown
	//for signal aspects, we move from ASPECT_CLOSED or ASPECT_THROWN straight to the antiphase, and we heed the .power parameter

	for (auto& vs : virtualservoCollection) {
		if (vs.pin > 8) continue;  //invalid pin

		uint8_t maxPosition = vs.swing + 90;
		uint8_t minPosition = 90 - vs.swing;
		uint8_t gpioPin = NodeMCUmap[vs.pin];

		//Servo rotation rates. +ve rate values will speed up movement by increasing the movement increment above 1
		uint8_t increment = vs.rate > 0 ? vs.rate : 1;
		//.timeDelay is used for -ve rate values
		vs.timeDelay += vs.timeDelay < 0 ? 1 : 0;
		

		switch (vs.state) {
		case SERVO_NEUTRAL:
			vs.position = 90;
			ESPservoAttach(vs.pin,true);
			break;

		case SERVO_TO_CLOSED:
			//-ve rotation rate values will slow down movement
			if (vs.timeDelay != 0) break;
			vs.timeDelay = vs.rate < 0 ? vs.rate : 0;

			//swing toward minPosition, unless invert is true
			if (vs.invert) {
				vs.position += vs.position < maxPosition ? increment : 0;
			}
			else {
				vs.position -= vs.position > minPosition ? increment : 0;
			}

			if ((vs.position >= maxPosition) || (vs.position <= minPosition)) {
				vs.state = SERVO_CLOSED;
			}

			ESPservoAttach(vs.pin, true);
			break;

		case SERVO_TO_THROWN:
			//-ve roation rate values will slow down movement
			if (vs.timeDelay != 0) break;
			vs.timeDelay = vs.rate < 0 ? vs.rate : 0;

			//swing toward maxPosition unless invert is true
			if (vs.invert) {
				vs.position -= vs.position > minPosition ? increment : 0;
			}
			else {
				vs.position += vs.position < maxPosition ? increment : 0;
			}

			if ((vs.position >= maxPosition) || (vs.position <= minPosition)) {
				vs.state = SERVO_THROWN;
			}

			ESPservoAttach(vs.pin, true);
			break;

		case SERVO_THROWN:
			vs.position = vs.invert ? minPosition : maxPosition;
			if (!vs.continuous) ESPservoAttach(vs.pin, false);
			break;

		case SERVO_CLOSED:
			vs.position = vs.invert ? maxPosition : minPosition;
			if (!vs.continuous) ESPservoAttach(vs.pin, false);
			break;

		case ASPECT_CLOSED:
			if ((vs.power) || (vs.ignorePowerParameter)) {
				//actively drive the pin. Thrown is a high state unless invert is active
				pinMode(gpioPin, OUTPUT);
				digitalWrite(gpioPin, vs.invert ? HIGH : LOW);
			}
			else {
				//pin to tri-state
				pinMode(gpioPin, INPUT);
			}
			break;

		case ASPECT_THROWN:
			if ((vs.power) || (vs.ignorePowerParameter)) {
				//actively drive the pin. Thrown is a high state unless invert is active
				pinMode(gpioPin, OUTPUT);
				digitalWrite(gpioPin, vs.invert ? LOW : HIGH);
			}
			else {
				//pin to tri-state
				pinMode(gpioPin, INPUT);
			}
			break;


		case ASPECT_MULTIPLE:
			// MAScommandSync is set by an LEDstate edge, thus synchronising all changes with the master LED clock.
			// this prevents short-duration flashses when changing to one of the MAS flashing aspects
			if (MAScommandSync) assertMASoutput(vs);
			break;


		case SENSOR_HIGH:
		case SENSOR_LOW:
			//for sensors we repurpose the position parameter to record the input state every 15ms
			//and this in turn is used to debounce the input 8 zeros or 1s must be seen
			if ((vs.deviceType != DEVICE_SENSOR) && (vs.deviceType != DEVICE_SENSOR_WPU)) break;
			vs.position = vs.position << 1;
			vs.position += digitalRead(gpioPin);  
			

			//2026-04-21 found the sensor bug.  If you queue a TCP message before you are connected then this caused a crash
			//this is now fixed in sendEnqueueMessages().  No need to suspend &servoTimer

			if ((vs.state == SENSOR_HIGH) && (vs.position == 0)) {
					//declare low
					nsLOCONETaccessoryProcessor::sensorEvent(vs.address,false);
					vs.state = SENSOR_LOW;
			}

			if ((vs.state == SENSOR_LOW) && (vs.position == 0xFF)){
					nsLOCONETaccessoryProcessor::sensorEvent(vs.address, true);
					vs.state = SENSOR_HIGH;
			}
			break;

		case HEARTBEAT_HIGH:
			//convention is heartbeatLEDstate is activelow
			if (heartbeatLEDstate) break;
			digitalWrite(gpioPin, HIGH);
			vs.state = HEARTBEAT_LOW;
			break;
		case HEARTBEAT_LOW:
			if (!heartbeatLEDstate) break;
			digitalWrite(gpioPin, LOW);
			vs.state = HEARTBEAT_HIGH;
			break;

		case SERVO_BOOT:
			if (vsBoot == nullptr) {
				//handle next-up servo to boot. servos are booted in the CLOSED position
				//and aspects are booted with POWER=off
				vsBoot = (VIRTUALSERVO*)&vs;
				bootTimer = 34;
	
				//refactor as switch.  vs and vsBoot are the same item
				switch (vs.deviceType) {
				case DEVICE_SENSOR:
				case DEVICE_SENSOR_WPU:
					ESPservoAttach(vs.pin, false);
					if  (vs.deviceType== DEVICE_SENSOR_WPU)
					{ pinMode(gpioPin,INPUT_PULLUP); }
					else{ pinMode(gpioPin, INPUT); }
					vs.position = 0xFF;  //spoof a high input state
					vs.state = SENSOR_HIGH;
					vsBoot = nullptr;
					bootTimer = 0;
					break;

				case DEVICE_SERVO:
					//special case for pin0. This cannot operate as a servo so instead make it a heartbeat indicator
					if (vs.pin == 0) {
						vs.state = HEARTBEAT_HIGH;   //DEBUG DISABLE
						pinMode(gpioPin, OUTPUT);
						vsBoot = nullptr;
						bootTimer = 0;
						//break;
					}

					vs.position = vs.invert ? maxPosition : minPosition;
					ESPservoAttach(vs.pin, true);
					ESPservoWrite(vs.pin, vs.position);
					break;

				case DEVICE_MAS:
					vs.MASstate = 127;
				case DEVICE_ASPECT:
					vs.power = false;
					vs.state = vs.deviceType==DEVICE_MAS ? ASPECT_MULTIPLE : ASPECT_CLOSED;
					vsBoot = nullptr;
					bootTimer = 0;
					ESPservoAttach(vs.pin, false);
					break;
				case DEVICE_I2C:
					vsBoot = nullptr;
					bootTimer = 0;
					break;

				}


			}
			else if (vsBoot == (VIRTUALSERVO*)&vs) {
				//This is the current boot-servo. Decrement bootTimer
				bootTimer -= bootTimer > 0 ? 1 : 0;

				//timed out?
				if (bootTimer == 0) {
					vs.state = SERVO_CLOSED;
					Serial.print(F("pin booted "));
					Serial.println(vs.pin, DEC);
					//release for next vs to boot
					vsBoot = nullptr;
				}
			}
			break;
		}

		//update servo positions every 15ms
		if (vs.deviceType==DEVICE_SERVO) ESPservoWrite(vs.pin, vs.position);

		//2026-05-11 need to modify this to handle PCA servos or aspects or MAS

	} //end of auto virtualServoCollection, aka BANK 0

	
	if (!bootController.hasPCA9685modules) return;

	for (uint8_t activeBank = 1;activeBank < 3;activeBank++) {

		//debug, we might run out of time.  We are seeing Soft WDT resets after bank2 boots
		//yup, exiting before we process bank2 prevents the WDT.  yield() just causes and immedate crash
		//i think i either need to speed up this loop (it does call float math a lot) or slow down calls to the loop
		//and loose resolution on servo movements.

		if (activeBank == 2) break;  //temp fix for WDT resets
	
	VIRTUALSERVO(*ptrToVSC)[16] = &virtualservoCollectionBank1;  //this points to entire array and does not decay to first element
	if (activeBank == 2) ptrToVSC = &virtualservoCollectionBank2;

		for (auto& vs : *ptrToVSC) {
			if (vs.pin > 15) continue;  //invalid pin
			uint8_t maxPosition = vs.swing + 90;
			uint8_t minPosition = 90 - vs.swing;
			//Servo rotation rates. +ve rate values will speed up movement by increasing the movement increment above 1
			uint8_t increment = vs.rate > 0 ? vs.rate : 1;
			//.timeDelay is used for -ve rate values
			vs.timeDelay += vs.timeDelay < 0 ? 1 : 0;

			switch (vs.state) {
			case SERVO_BOOT:
				if (vsBoot == nullptr) {
					//handle next-up servo to boot. servos are booted in the CLOSED position
					//and aspects are booted with POWER=off
					vsBoot = (VIRTUALSERVO*)&vs;
					bootTimer = 34;

					//refactor as switch.  vs and vsBoot are the same item
					switch (vs.deviceType) {
					case DEVICE_SERVO:
						vs.position = vs.invert ? maxPosition : minPosition;  //means we boot at 0 or 180 even though we might normally operate at say 90+-10....
						//attaches the driver and commands to a specific position immediately
						PCAservoWrite(&vs, activeBank,true);
						break;

					case DEVICE_ASPECT:
						vs.power = false;
						vs.state = vs.deviceType == DEVICE_MAS ? ASPECT_MULTIPLE : ASPECT_CLOSED;
						vsBoot = nullptr;
						bootTimer = 0;
						//ESPservoAttach(vs.pin, false);  //no need to attach
						break;
					}
				}
				else if (vsBoot == (VIRTUALSERVO*)&vs) {
					//This is the current boot-servo. Decrement bootTimer
					bootTimer -= bootTimer > 0 ? 1 : 0;

					//timed out?
					if (bootTimer == 0) {
						vs.state = SERVO_CLOSED;
						Serial.print(F("pin booted "));
						Serial.println(vs.pin, DEC);
						//release for next vs to boot
						vsBoot = nullptr;
					}
				}

				break;

			case SERVO_NEUTRAL:
				vs.position = 90;
				PCAservoWrite(&vs, activeBank,true);
				break;

			case SERVO_TO_CLOSED:
				//-ve rotation rate values will slow down movement
				if (vs.timeDelay != 0) break;
				vs.timeDelay = vs.rate < 0 ? vs.rate : 0;

				//swing toward minPosition, unless invert is true
				if (vs.invert) {
					vs.position += vs.position < maxPosition ? increment : 0;
				}
				else {
					vs.position -= vs.position > minPosition ? increment : 0;
				}

				if ((vs.position >= maxPosition) || (vs.position <= minPosition)) {
					vs.state = SERVO_CLOSED;
				}

				PCAservoWrite(&vs, activeBank, true);
				break;

			case SERVO_TO_THROWN:
				//-ve roation rate values will slow down movement
				if (vs.timeDelay != 0) break;
				vs.timeDelay = vs.rate < 0 ? vs.rate : 0;

				//swing toward maxPosition unless invert is true
				if (vs.invert) {
					vs.position -= vs.position > minPosition ? increment : 0;
				}
				else {
					vs.position += vs.position < maxPosition ? increment : 0;
				}

				if ((vs.position >= maxPosition) || (vs.position <= minPosition)) {
					vs.state = SERVO_THROWN;
				}

				PCAservoWrite(&vs, activeBank, true);
				break;

			case SERVO_THROWN:
				vs.position = vs.invert ? minPosition : maxPosition;
				PCAservoWrite(&vs, activeBank, vs.continuous);
				break;

			case SERVO_CLOSED:
				vs.position = vs.invert ? maxPosition : minPosition;
				PCAservoWrite(&vs, activeBank, vs.continuous);

				break;


			}//end switch


			//update servo positions every 15ms
			//if (vs.deviceType == DEVICE_SERVO) PCAservoWrite(vs, activeBank);

		}

	}//for loop



}












/*PCA notes
PCAbank1.setPWM(vs.pin, whenOn, whenOff);  whenOn is usually 0, whenOff is when 0-4096 that output goes low
if both are zero then presumably there is no high pulse

PCAbank1.setPin(pin, pulse value, invert);  pulse value is 0-4096 where 0 is completely OFF unless invert is true
PCAbank1.setOutputMode(totem);  true for totem outputs, false for open drain. this is global and all outputs will be of same type.

*/



/// <summary>
/// accepts a virtual servo member, processes it and modifies it
/// </summary>
/// <param name="vs">target vs</param>
/// <param name="bank">bank 1 or 2</param>
/// <param name="attach">true if pwm active</param>
void PCAservoWrite(VIRTUALSERVO *vs, uint8_t bank, bool attach) {
	if (vs->pin > 15) return;
	if (attach) {
		float d = vs->position / 180.0;
		d *= (bootController.PCAservoMax - bootController.PCAservoMin);  //full range
		d += bootController.PCAservoMin;  //add min pulse offset
		if (bank == 1) {
			PCAbank1.setPWM(vs->pin, 0, (uint16_t)d);
		}
		else {
			PCAbank2.setPWM(vs->pin, 0, (uint16_t)d);
		}
		vs->power = true;
		return;
	}
	//detach, pull output low
		if (!vs->power) return;
		if (bank == 1) {
			PCAbank1.setPin(vs->pin, 0, false);
		}
		else {
			PCAbank2.setPin(vs->pin, 0, false);
		}
		vs->power = false;
}


//delay test for PCA processing.  if we have a bit that we take high when we enter the 15mS block and low when we exit, we should see how long the processing takes
//

