
/*
	Name:       ESPaccessoryController
	Created:	2026-06-07
	Author:     J.Ossowski
	Hardware:	NodeMCU EXP8266

	This project implements a LocoNet over TCP accessory controller.  It is intended to be used with JMRI Panel Pro.

	There are three configuration options, set with the M command;

	1.LocoNet HOST on home Wifi set
	2.LocoNet CLIENT on home Wifi
	3.Stand alone LocoNet HOST 
	
	1 & 2 will connect to the home Wifi, same with JRMI Decoder Pro.  With 1, this device acts as a LocoNet host and JRMI will connect to 
	this device's IP and port.  With 2, this device acts as a client, connecting to another LocoNet host such as my ESP_DCC with railcom DCC controller project.  
	JRMI also needs to connect to that same LocoNet host.

	With 3, you do not need a home Wifi network.  The JRMI laptop will connect to this device (which acts as a hotspot) and this device will run as a LocoNet host 
	on its own IP and port.
	

*/



#include "ESPservo.h"
#include "ESPaccessory.h"

/*Because the Arduino IDE is rather basic and requires an .INO file, this creates issues because
modules cannot see functions declared in that .ino file. For this reason, the .INO is used as a shell
and all the code is moved into modules with their own cpp and hpp headers*/


using namespace nsESPaccessory;

void setup() {
	//set system frequency from the IDE
	//system_update_cpu_freq(160);  //this will double the clock speed from 80MHz
	ESPservoInit();
	ESPaccessorySetup();

}

void loop() {
	ESPaccessoryLoop();
}

