// ESPservo.h

#ifndef _ESPSERVO_h
#define _ESPSERVO_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

//struct to model servo GPIO posn and IO state
struct SERVO {
	uint16_t hiPulseLen;  //calculated high pulse len in ticks
	uint8_t gpioPin;	//the assigned GPIO
	bool isAttached;	//attach/detach
};


void ESPservoInit();
void ESPservoDebug();
void ESPservoWrite(uint8_t pin, uint8_t position);
void ESPservoAttach(uint8_t pin, bool isAttached);
bool ESPservoIsAttached(uint8_t pin);
#endif

