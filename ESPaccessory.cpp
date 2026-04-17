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




/*
BUGS
the thing hits a soft WDT even before Wifi connects.  possibly this is servo.h
causing a problem
Yes, Servo.h works with the ESP8266/ESP-12, but it requires the dedicated ESP8266 board package,
not the standard Arduino AVR library. Use PWM-capable GPIO pins (like D2 or D4)

And not all ESP pins can support PWM. GPIO 4 (D2) or GPIO 2 (D4) are generally recommended. 
Avoid GPIO 0, 1, and 15, which are used for booting.

So... when it comes to PWM support, maybe I don't use Servo.h and instead use the PCA breakout board 
this way we reserve the ESP for sensor inputs and tristate led drivers



good idea to use BANKS, with bank 0 is the NodeMCU, bank1 is the first PCA and bank2 the second PCA
each PCA can then have 16 pins

NodeMCU hardware SZDOIT board
https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
D0 GPIO16
D1 GPIO5 PWMA
D2 GPIO4 PWMB
D3 GPIO0 dir A (WPU, flash button, boot fails if low)
D4 GPIO2 dir B (WPU, boot fails if low)
D5 GPIO14 
D6 GPIO12
D7 GPIO13
D8 GPIO15 (WPD, boot fails if hi)

Assuming you use active pull down sensors, then all pins bar D8 can host a sensor
The A0 analog input could also house a sensor.

So BANK0 CAN HAVE 9 io pins, we will map them as 0 through 8 to mirror the nodemcu naming
on the PCA9685 device it will be pins 0-15



*/


const char NodeMCUmap[9] = {16,5,4,0,2,14,12,13,15};
void processServo(void);






#include "ESPaccessory.h"
//#include <Servo.h>  //not using it

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
	long softwareVersion = 20260404;  //yyyymmdd captured as an integer
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


//++++++++++++ TCP IP ++++++++++++++++++++++++++++++++++++++++++++
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

static os_timer_t hearbeatTimer;

static os_timer_t servoTimer;
#define SERVO_TIMEOUT 15  //15ms


static uint8_t deviceState = S_BOOT;


//2024-03-22 ballID handling
//void payloadTimerCallback(void* pArg);
static os_timer_t payloadTimer;



//++++++++++++++++++++ TURNOUTS, SIGNALS AND SENSORS ++++++++++++++++++++++++++++++++++++++++++++++
#define TOTAL_PINS 9
#define BASE_PIN 0
#define ASPECT_PARAMETER_SIZE	8	//# of parameters in each MAS parameter array
#define MAS_EMPTY_VAL 255			//char which denotes a MAS parameter is not-set

bool MAScommandSync;

/*servo control.  VIRTUALSERVO is each virtualised device with its params.  Commanded over serial for testing
or DCC in normal operation. VIRTUALSERVO objects support both mechanical servos and LED aspect signals */
enum servoState {
	SERVO_NEUTRAL,
	SERVO_TO_THROWN,
	SERVO_THROWN,
	SERVO_TO_CLOSED,
	SERVO_CLOSED,
	SERVO_BOOT,
	ASPECT_THROWN,
	ASPECT_CLOSED,
	ASPECT_MULTIPLE
};

struct VIRTUALSERVO {
	uint8_t pin;
	uint16_t address;
	uint8_t swing;
	bool invert;
	bool continuous;
	bool power;
	bool ignorePowerParameter;
	bool isServo;  //servo or aspect
	uint8_t state;
	uint8_t position;
	int8_t rate;  //+ve values speed up movement, -ve slow it down
	int8_t timeDelay;  //working register, loaded negative and counts up to zero
	uint8_t aspectParameters[ASPECT_PARAMETER_SIZE * 4];
	//Servo* thisDriver;
	uint8_t MASstate;  //Multiple Aspect Signal commanded state
};

//virtual servo objects
VIRTUALSERVO virtualservoCollection[TOTAL_PINS];

//servo drivers. This library creates a servo driver (pulse posn modulation) for each of the IO pins
//Servo servoDriver[TOTAL_PINS];

//function declaraions
uint8_t parseBracketedParameters(char* token, uint8_t* result);
bool isMASdevice(VIRTUALSERVO vs);
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
void bootTCP(void);
void stringIPtoArray(char* s, uint8_t* myIP);
void eeGetSettings(void);
void eePutSettings(void);
void checkSerial(void);
static void sendEnqueuedMessages();

//cannot put this in the header and then expect to use it in another namespace as it will cause a compiler error
//instead we have to return its value via a function
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
	Serial.begin(115200);
	delay(200);

	verbose = false;
	eeGetSettings();
	

	//DEBUG
	verbose = true;


	//pinMode(16, OUTPUT);  LED sits on D0

	deviceState = S_BOOT;
	
	
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


	//https://sub.nanona.fi/esp8266/hello-world.html
	os_timer_setfn(&servoTimer, (os_timer_func_t*)processServo, NULL);
	os_timer_arm(&servoTimer, SERVO_TIMEOUT, true);

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
	//		digitalWrite(16, ledState);  //led is active low, false=on    DEBUG disable
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
			//digitalWrite(16, ledState);
			ledState = !ledState;
			previousMillis = currentMillis;

			//NOTE: this technique of only allowing one pending connect at a time seems to work.  The asyncTCP object does have a timeout on it, and will call disconnect
			//after about 60 sec.  At this point we will retry the connection.
			//There is a risk that old instances of the client remain in memory and after about 30 instances are created the ESP will run out of resources and crassh, i.e. 
			//its a memory leak.

		}
		break;

	}


	/*
		//Otherwise, get first message in queue and transmit
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


#pragma region "...SERIAL ROUTINES..."




/// <summary>
/// Read incoming serial commands and action
/// </summary>
void checkSerial(void) {
	if (!Serial.available()) return;
	char SerialBuffer[32];
	Serial.readString().toCharArray(SerialBuffer, 32);

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
			Serial.println(F("X=dump params\n"));
			Serial.println(F("H=dump incoming\n"));
		}

		if (SerialBuffer[0] == 'R') {
			Serial.println(F("REBOOTING...\n\n"));
			ESP.restart();
		}

		if (SerialBuffer[0] == 'M') {
			//switch modes
			if (SerialBuffer[1] == 'K') {
				bootController.relayMode[0] = 'K';
				Serial.println(F("KIOSK mode set\n"));
			}
			else {
				bootController.relayMode[0] = 'R';
				Serial.println(F("RELAY mode set\n"));
			}

			//optional 1 | 0 to enable | disable watchdog
			switch (SerialBuffer[2]) {
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
			vsParse.isServo = false;
			vsParse.state = ASPECT_CLOSED;
			bool resolved = true;

			//detokenize
			char* pch;
			int i = 0;
			pch = strtok(SerialBuffer, " ,");

			while (pch != NULL) {
				switch (i++) {
				case 1:
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
						memset(vsParse.aspectParameters, MAS_EMPTY_VAL, 4 * ASPECT_PARAMETER_SIZE * sizeof(int8_t));
	//					vsParse.thisDriver = vs.thisDriver;
						vs = vsParse;  //copy over from vsParse
		//				if (vs.thisDriver->attached()) vs.thisDriver->detach();   
												
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
			vsParse.isServo = true;
			vsParse.ignorePowerParameter = true;
			vsParse.continuous = false;  //default setting
			memset(vsParse.aspectParameters, MAS_EMPTY_VAL, 4 * ASPECT_PARAMETER_SIZE * sizeof(int8_t));

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
					//first copy servo-driver pointer to servoParse
				//	vsParse.thisDriver = vs.thisDriver;
					//then copy servoParse to vs
					vs = vsParse;
					vs.position = 90;
					vs.isServo = true;
					vs.state = SERVO_TO_CLOSED;
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
						if (isMASdevice(vs)) {
							Serial.println(F("Cannot use command on MAS aspect"));
							resolved = false;
						}
					}
					break;

				case 2:
					if (vsPointer == nullptr) { resolved = false;break; }

					if (vsPointer->isServo) {
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
					else {
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
					for (auto& vs : virtualservoCollection) {
						if (vs.address != address) continue;

						switch (pch[0]) {
						case 't':
							vs.state = vs.isServo ? SERVO_TO_THROWN : ASPECT_THROWN;
							break;
						case 'n':
							vs.state = vs.isServo ? SERVO_NEUTRAL : ASPECT_CLOSED;
							break;
						case 'T':
							if (vs.isServo) {
								vs.state = (vs.state == SERVO_CLOSED) ? SERVO_TO_THROWN : SERVO_TO_CLOSED;
							}
							else {
								vs.state = (vs.state == ASPECT_THROWN) ? ASPECT_CLOSED : ASPECT_THROWN;
							}
							break;
						default:
							vs.state = vs.isServo ? SERVO_TO_CLOSED : ASPECT_CLOSED;
						}
					}
					break;
				case 3:
					//power.  Iterate all servos and execute on all matching addresses
					for (auto& vs : virtualservoCollection) {
						if (vs.address != address) continue;
						if (!isMASdevice(vs)) continue;
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
						if (!isMASdevice(vs)) continue;
						vs.MASstate = state;
						MAScommandSync = false;
						if (verbose) Serial.println(assertMASoutput(vs), DEC);
					}
					break;
				}
				pch = strtok(NULL, " ,");
				if (!resolved) break;
			}

			if (resolved && (i >= 3)) Serial.println("OK");
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
				Serial.println(i, DEC);
			}
		}

		//DUMP all servo/aspect information
		if (SerialBuffer[0] == 'x') {

			for (auto vs : virtualservoCollection) {
				//dump this pin
				if (vs.isServo) {
					Serial.print("servo  pin ");
					Serial.print(vs.pin, DEC);
					Serial.print("  address ");
					Serial.print(vs.address, DEC);
					Serial.print("  swing ");
					Serial.print(vs.swing, DEC);
					Serial.print("  invert ");
					Serial.print(vs.invert, DEC);
					Serial.print("  continuous ");
					Serial.print(vs.continuous, DEC);
					Serial.print("  rate ");
					Serial.print(vs.rate, DEC);

				}
				else {
					//dump signal aspects
					if (isMASdevice(vs)) {
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
					}
					else
					{
						Serial.print(F("aspect pin "));
						Serial.print(vs.pin, DEC);
						Serial.print(F("  address "));
						Serial.print(vs.address, DEC);
						Serial.print(F("  invert "));
						Serial.print(vs.invert, DEC);
						Serial.print(F("  power "));
						if (vs.ignorePowerParameter) { Serial.print("x"); }
						else { Serial.print(vs.power, DEC); }
					}
				}

				//if (vs.thisDriver == nullptr) {
					if (false){
					Serial.print(" pointer bad");
				}
				else
				{
					//dump output state
					switch (vs.state) {
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

				}
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
					vs.address = vsParse.address;
					vs.isServo = false;
					vs.power = false;  //default is all output drivers are tri-state, await first dcc command
					vs.ignorePowerParameter = true;
					vs.invert = vsParse.invert;

					memcpy(vs.aspectParameters, vsParse.aspectParameters, 4 * ASPECT_PARAMETER_SIZE * sizeof(int8_t));
					vs.state = ASPECT_MULTIPLE;
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
	if ((p < BASE_PIN) || (p >= BASE_PIN + TOTAL_PINS)) {
		Serial.println("bad pin");
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
				if (!ledState) outputState = LO;
			}
			break;

		case 2:
			//flash high
			if (vs.aspectParameters[a] == vs.MASstate) {
				//assert hi gated with ledState == high, this allows flash lo/hi to work on the same pin
				if (ledState) outputState = HI;
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

	//assert the output pin, also process invert
	switch (outputState) {
	case TRISTATE:
		pinMode(vs.pin, INPUT);
		break;

	case LO:
		outputState = vs.invert ? HI : LO;
		pinMode(vs.pin, OUTPUT);
		digitalWrite(vs.pin, outputState == LO ? LOW : HIGH);
		break;

	case HI:
		outputState = vs.invert ? LO : HI;
		pinMode(vs.pin, OUTPUT);
		digitalWrite(vs.pin, outputState == LO ? LOW : HIGH);
		break;

	}

	return outputState;
}









void nsESPaccessory::commandTurnout(int16_t addr, bool thrown) {}
void nsESPaccessory::commandMAS(int16_t addr, uint8_t state) {}
bool nsESPaccessory::pollSensor(int16_t addr) { return true; }






#pragma endregion




const int EEOFFSET = 192;

/// <summary>
/// restores settings from EEPROM. If the software version has changed, we overwrite the eeprom with defaults
/// we also need to clear certain values on boot. max EEPROM we can use is 4096
/// </summary>
void eeGetSettings(void) {
	CONTROLLER defaultController;  //grab defaults
	EEPROM.begin(1024);  //ESP does not have dedicated eeprom and must be allocated from Flash
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

		//set defaults
		int p = BASE_PIN;
		for (auto& vs : virtualservoCollection) {
			vs.pin = p;
			vs.invert = 0;
			vs.position = 90;
			vs.swing = 25;
			vs.continuous = 0;
			vs.state = SERVO_BOOT;
			vs.power = false;
			vs.ignorePowerParameter = true;
			vs.isServo = true;
			vs.rate = 0;
			memset(vs.aspectParameters, MAS_EMPTY_VAL, 4 * ASPECT_PARAMETER_SIZE * sizeof(int8_t));
			++p;
		}
		/*write back default values*/
		EEPROM.put(eeAddr, virtualservoCollection);
	}
		/*either way, now populate our structs with EEprom values*/
		eeAddr = 0;
		EEPROM.get(eeAddr, bootController);
		//eeAddr += sizeof(bootController);
		eeAddr = EEOFFSET; //bug fix
		EEPROM.get(eeAddr, virtualservoCollection);
		eeAddr += sizeof(virtualservoCollection);
		Serial.print(F("EEPROM="));
		Serial.println(eeAddr, DEC);

		//initialise the pin assignments move all servos and aspects to closed position
		int i = 0;
		int p = BASE_PIN;
		
		for (auto& vs : virtualservoCollection) {
			vs.pin = p;
			vs.state = SERVO_BOOT;
			vs.MASstate = 127;
			//s.position = 90;  //neutral
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
			vs.address &= 0xFFF;


			//Servo.h for the ESP12 needs to be fed the GPIO reference, all we are doing here is associating 
			//each servoDriver with its virtualServo parent
			//servoDriver[i].detach();
			//vs.thisDriver = &servoDriver[i];
			++p;
			++i;
		}
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



#pragma region "...TCP..."

//called every 10 sec from intervalTimer
//it sends a heartbeat message
static void heartbeat(void* arg) {
	AsyncClient* client = reinterpret_cast<AsyncClient*>(arg);
	queueMessage("ESPACC\n\0");

}

//event callback for TCP inbound data
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
	Serial.printf("\n client has been connected to %s on port %d \n", TCPserverIP.toString().c_str(), bootController.tcpPort);
	deviceState = S_TCP_CONNECTED;
	heartbeat(client);
	clientX = client;

	os_timer_arm(&hearbeatTimer, 10000, true); // schedule for reply to server at next 10s

}

void onDisconnect(void* arg, AsyncClient* client) {
	Serial.printf("\n client has disconnected\n");
	deviceState = S_TCP_RECONNECT;
	os_timer_disarm(&hearbeatTimer);
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


	os_timer_disarm(&hearbeatTimer);
	os_timer_setfn(&hearbeatTimer, &heartbeat, client);
	//this timer is not triggered yet, see on_timer_arm()
	//make sense as there is no point arming it until we are connected
	

}




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
		myIP[i] = strtol(p, NULL, 10);
		i++;
		if (i == 4) break;
		//more data?
		p = strtok(NULL, ",.");
	}
	return;
}


#pragma endregion

bool nsESPaccessory::getVerbose(void) {
	return verbose;
}




//PROCESS SERVO POSITIONS AND SIGNAL ASPECTS

void processServo(void) {
	static VIRTUALSERVO* vsBoot = nullptr;
	static uint8_t bootTimer = 0;
	
	//IMPORTANT: vs.pin does represent the GPIO pin and not the ordinal in the virtualServoCollection


	//1 sec count block here
	MAScommandSync = true;

	//in normal non-invert mode, minPosition is turnout closed, and maxPosition is turnout thrown
	//for signal aspects, we move from ASPECT_CLOSED or ASPECT_THROWN straight to the antiphase, and we heed the .power parameter

	for (auto& vs : virtualservoCollection) {
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
		//	if (!vs.thisDriver->attached()) vs.thisDriver->attach(gpioPin);
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

		//	if (!vs.thisDriver->attached()) vs.thisDriver->attach(gpioPin);
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

			//if (!vs.thisDriver->attached()) vs.thisDriver->attach(gpioPin);
			break;

		case SERVO_THROWN:
			vs.position = vs.invert ? minPosition : maxPosition;
		//	if ((vs.thisDriver->attached()) && (!vs.continuous)) {
			//	vs.thisDriver->detach();
			//}
			break;
		case SERVO_CLOSED:
			vs.position = vs.invert ? maxPosition : minPosition;
		//	if ((vs.thisDriver->attached()) && (!vs.continuous)) {
				//vs.thisDriver->detach();
		//	}
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

		case SERVO_BOOT:
			if (vsBoot == nullptr) {
				//handle next-up servo to boot. servos are booted in the CLOSED position
				//and aspects are booted with POWER=off
				vsBoot = (VIRTUALSERVO*)&vs;
				bootTimer = 34;
				if (vs.isServo) {
					vs.position = vs.invert ? maxPosition : minPosition;
					//if (!vs.thisDriver->attached()) vs.thisDriver->attach(gpioPin);
					//vs.thisDriver->write(vs.position);
				}
				else {
					//aspect. Immediately go to closed state with power off
					vs.power = false;
					vs.state = isMASdevice(vs) ? ASPECT_MULTIPLE : ASPECT_CLOSED;
					vsBoot = nullptr;
					bootTimer = 0;
					//vs.thisDriver->detach();
				}

			}
			else if (vsBoot == (VIRTUALSERVO*)&vs) {
				//if this is the current boot-servo, then decrement bootTimer
				bootTimer -= bootTimer > 0 ? 1 : 0;

				//timed out?
				if (bootTimer == 0) {
					vs.state = SERVO_CLOSED;
					Serial.print(F("pin booted"));
					Serial.println(vs.pin, DEC);
					//release for next vs to boot
					vsBoot = nullptr;
				}
			}
			break;
		}

		//update the servo position every 15mS
		//if (vs.thisDriver) vs.thisDriver->write(vs.position);    ///BUG writing this causes a WDT soft reset.  I think it is likely interfering with the WiFi
	}

	//NOW we have a wdt reset; we generate PWM and connect to wifi but after pin booted0 appears, the unit locks up and hits a wdt reset and does not respond to 
	// serial either.
	


}


//2026-04-14 give up on servo support.   The nodeMCU has 3v3 outputs, and so the breakoutboard servo pin connectors are all 3v3
//there's not enough power available from the 3v3 regulator.
//and it seems the Servo library will cause soft WDT timeouts if you write(vs.position) for all the servos every 15mS.
//I think signals with tristate is still viable and infact on 3v3 is a good thing.  But servos are off the agenda as the SZDOIT board does not 
//put 5v on the JST connector common rail.   I could write my own low-level int PWM code, but this does not fix the SZDOIT board problem

