// ACCweb.h
// BEWARE: you must upload your data folder to the ESP8266.  If using Visual Studio vMicro
// set option 15: Filesystem uploadtool:LittleFS it might default to SPIFFS which is obsolete

/*
We need to pass wsUri to the client browser so that it can set up a websocket on this ip and port combo
serveral ways to do this. One is to add a cookie to the server response containing the wsUri, the other
is to have the client page call /hardware as a GET and return a JSON string. This allows a bigger payload.

*/

#ifndef _ACCWEB_h
#define _ACCWEB_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <ESP8266WebServer.h>  //using this
//#include <ESPAsyncWebServer.h> // The core async web server library  NOT USING
#include <ArduinoJson.h>  //what version is this
#include <LittleFS.h>
#include <WebSockets.h>  //from arduino library manager. Markus Sattler v2.1
#include <WebSocketsServer.h>
//- DWEBSOCKETS_MAX_DATA_SIZE = 32768;
//https://www.google.com/search?q=Markus+Sattler+arduino+websocket+max+length&sca_esv=2d6c5a1e6a3d194c&rlz=1C1CHBF_enAU1138AU1138&sxsrf=APpeQnvvKr1pp4gB0OpbfXCee82T_dzqpg%3A1790570786277&ei=wfC5atfwOrjR1e8PlYqvoAU&biw=1536&bih=695&uact=5&sclient=gws-wiz-serp&fbs=ABfTbFVyMZGZf1hfvX9uKjN_-G8c4u0nXx4bEIpwm1lnNH832SMIiTl3t-JZ4hGJOxPbHYSIu8Q64jU5EwQ-803VaKbd8XGNh2EAGT96nVa30badWeLcMOEWQcU9WGmTgQb8mz6mPncLV6BMXrUfQD5ftu4HKiVJOOiqhNYT8Keh_15lhVLohIhM8kHnw_tixWHDpdAHWCTimc1GO2znH-0EdfLD74c9fw&aep=10&ntc=1&mstk=AUtExfBZpPwLHhQzKiDTtON3-A9FtU0YDJgkkN5hFY9yCljMTxDGFybne_Njkr0CDpiqMnToI-8eSdPTWquuJBszkBkww_HOSWix8jUWh-OD8l5Kk3AW85oF0WtOh-hnTeTJ2RhuVsf8ibliPlINLFegj5-cDdEYAgsRWqK9IV7LmWxCXNvNmtjtg3dqUtbxfSSkMiD-YKMDSVhtHbwwHghVagSKhM8-mjMVDojyDp6XhTT9Gj7imlsPnrSi0d_fe_ylM02fC1c1FdWqNq878PbB2f5SSeHGnmlyI8cds_5gfSNIEzojKcUb13noJ0JCd5nq488ps6UuY7hD1Q&aioh=3&csuir=1&cs=0&udm=50&mtid=mPG5ap_8C-Dt1e8PuaaWiAQ


namespace nsACCweb {


	void startWebServices(void);
	void loopWebServices(void);

	void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
	void sendJson(JsonObject& out);
	void sendJson(JsonDocument out);
	void processHardware(JsonDocument &doc);
	void processBank(JsonDocument& doc);
	
	std::string getWsUri();
}

#endif

