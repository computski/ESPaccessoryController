
/*
	Name:       ESPaccessoryController
	Created:	2026-04-07
	Author:     J.Ossowski
	Hardware:	NodeMCU

	Note; if this device just acts as a TCP client, then we cannot use it in a system with just it and Panel Pro.
	It must instead be used as a client on port 1234 with a separate dcc controller acting as a loconet server on 2560
	alternatively it can connect to the ESP controller directly on port 2560
	This is because Panel Pro expects to connect as a client when booting.  We'd need to spin up a TCP server here.
	And even if we did, that's good for only one ESPA, as the second one would need to connect as a client on 1234.

*/



#include "ESPservo.h"
#include "ESPaccessory.h"
//#include "LocoNetAccessoryProcessor.h"


/*Because the Arduino IDE is rather basic and requires an .INO file, this creates issues because
modules cannot see functions declared in that .ino file. For this reason, the .INO is used as a shell
and all the code is moved into modules with their own cpp and hpp headers*/


using namespace nsESPaccessory;

void setup() {
	ESPservoInit();
	ESPaccessorySetup();
}

void loop() {
	ESPaccessoryLoop();
}

