// ESPaccessory.h

#ifndef _ESPACCESSORY_h
#define _ESPACCESSORY_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <string>  //required for std::string
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <EEPROM.h>
#include <vector>
#include "LocoNetAccessoryProcessor.h"



namespace nsESPaccessory {

	void ESPaccessoryLoop();
	void ESPaccessorySetup();
	bool queueMessage(std::string s);
	bool getVerbose(void);

	void commandTurnout(int16_t addr, bool thrown);
	void commandMAS(int16_t addr, uint8_t state);
	//bool pollSensor(int16_t addr);
	bool getSensorState(int16_t addr);

}

#endif

