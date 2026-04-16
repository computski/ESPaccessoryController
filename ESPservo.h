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
	int8_t position;     //swing angle -90 to +90
	uint16_t hiPulseLen;  //calculated high pulse len in ticks
	uint8_t gpioPin;	//the assigned GPIO
	bool isAttached;	//attach/detach
};

volatile static SERVO servoPool[9];  //9 pins

void ESPservoInit();


#endif

