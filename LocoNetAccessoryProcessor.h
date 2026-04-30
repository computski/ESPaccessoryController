// LocoNetAccessoryProcessor.h

#ifndef _LocoNetAccessoryProcessor_h
#define _LocoNetAccessoryProcessor_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#include <ESPAsyncTCP.h>  //Github me-no-dev/ESPAsyncTCP
#include <string>   //required if you wish to compile in arduino IDE, this is the std::string library

//https://wiki.rocrail.net/doku.php?id=loconet:ln-pe-en
// 
// 2 byte message opcodes
#define OPC_IDLE	0x85
#define OPC_GPON	0x83
#define PC_GPOFF	0x82
#define OPC_BUSY	0x81

//4 byte message opcodes
#define	OPC_LOCO_ADR	0xBF
#define OPC_SW_ACK		0xBD
#define OPC_SW_STATE	0xBC
#define OPC_RQ_SL_DATA	0xBB
#define OPC_MOVE_SLOTS	0xBA
#define OPC_LINK_SLOTS	0xB9
#define OPC_UNLINK_SLOTS	0xB8
#define OPC_CONSIST_FUNC	0xB6
#define OPC_SLOT_STAT1	0xB5
#define OPC_LONG_ACK	0xB4
#define OPC_INPUT_REP	0xB2	
#define OPC_SW_REP		0xB1
#define OPC_SW_REQ		0xB0
#define OPC_LOCO_SND	0xA2
#define OPC_LOCO_DIRF	0xA1
#define OPC_LOCO_SPD	0xA0

//variable length message opcodes
#define OPC_WR_SL_DATA 0xEF
#define OPC_SL_RD_DATA 0xE7
#define OPC_IMM_PACKET 0xED




namespace nsLOCONETaccessoryProcessor {
	void handleLocoNet(void* arg, AsyncClient* client, void* data, size_t len);
	void tokenProcessor(char* msg, AsyncClient* client);
	void sensorEvent(uint16_t address, bool event);
	bool pollSensor(int16_t addr); 
	void commandTurnout(int16_t addr, bool thrown);
	void commandMAS(int16_t addr, uint8_t state);


}








#endif

