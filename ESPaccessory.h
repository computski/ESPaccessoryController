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
	#define ASPECT_PARAMETER_SIZE	8	//# of parameters in each MAS parameter array
	#define MAS_EMPTY_VAL 255			//char which denotes a MAS parameter is not-set

	//this comment line is required to stop the compiler throwing warnings in the cpp file whereever VIRTUALSERVO is used :-)
	
	enum DEVICE_TYPES : uint8_t {
		DEVICE_SERVO,
		DEVICE_ASPECT,
		DEVICE_MAS,
		DEVICE_SENSOR,
		DEVICE_SENSOR_WPU,
		DEVICE_I2C
	};

	enum SERVOSTATE : uint8_t {
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


	struct AppConfig {
		int maxUsers;
		bool debugMode;
	};

	// Declared and initialized in the header safely:
	inline AppConfig globalConfig{ 100, true };



	void ESPaccessoryLoop();
	void ESPaccessorySetup();
	bool queueMessage(std::string s);
	bool getVerbose(void);
	bool isLoconetHost(void);

	void commandTurnout(int16_t addr, bool thrown);
	void commandMAS(int16_t addr, uint8_t state);
	//bool pollSensor(int16_t addr);
	int8_t getSensorState(int16_t addr);
	void dumpX(std::string& s);
	void replaceAll(std::string& src, const std::string& from, const std::string& to);
	std::string getWsUri();

}

#endif

