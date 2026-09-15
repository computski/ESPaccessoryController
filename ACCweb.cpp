// 
// 
// 

#include "ACCweb.h"
#include "ESPaccessory.h"


/*notes
challenge is we need to read the various virtualServoCollections inside nsESPaccessory
so it prob makes more sense to move these routines into that namespace especially as it already fires up TCP servers and Wifi connections

else, a call to handleRoot here could call a routine in nsESPaccessory that returns a JSON string.
but ultimately we 

*/



using namespace nsACCweb;

//reate a web server on port 80.  problems https://github.com/esp8266/Arduino/issues/4085
ESP8266WebServer web(80);


void testToAccessVirtualServo() {
	nsESPaccessory::VIRTUALSERVO banana;
		//call out to get a copy of a VS from nsESPaccessory
	//struct is 48 bytes x 16.  the RAM is 80kb and we are building a JSON string here plus running Wifi etc.  so and extra 0.5kb for a copy of 
	//each VScollection is incidental

}


void handleRoot() {
	web.send(200, "text/html", "<h1>You are connected to ACC ESP!</h1>");

}


void handleFormSubmit() {
	if (web.hasArg("username")) {
		Serial.printf("username=%s\n", web.arg("myText"));
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

void getHardware() {
	//used to return the wsUri string
	JsonDocument doc;
	doc["wsUri"] = nsESPaccessory::getWsUri();
	doc["maxUsers"]= nsESPaccessory::globalConfig.maxUsers;

	
	//We can avoid a String class, but need to guesstimate a useful buffer size
	char jsonChar[512]; 
	serializeJsonPretty(doc, jsonChar, sizeof(jsonChar));
	web.send(200, "text/json", jsonChar);
}



//call regularly from main loop
void nsACCweb::loopWebServices(void) {
	web.handleClient();
}

void nsACCweb::startWebServices() {
	Serial.print("start ACCweb");

	web.on("/", handleRoot);
	web.onNotFound([]() {                              // If the client requests any URI
			web.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
		});

	//special GET handlers
	//https://forum.arduino.cc/index.php?topic=476291.0
	web.on("/hardware", HTTP_GET, []() {getHardware(); });

	web.on("/submit", HTTP_POST, handleFormSubmit);

	web.begin();



}