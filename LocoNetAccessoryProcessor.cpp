// 
// 
// 

#define TRACE

#ifndef TRACE
#define trace(traceCodeBlock) ;
#else
#define trace(traceCodeBlock) traceCodeBlock
#endif


#include "LocoNetAccessoryProcessor.h"
#include "ESPaccessory.h"

using namespace nsLOCONETaccessoryProcessor;

void nsLOCONETaccessoryProcessor::handleLocoNet(void* arg, AsyncClient* client, void* data, size_t len) {
	trace(Serial.printf("\nLOCOnet %s \n", client->remoteIP().toString().c_str());)

		//malloc gives more efficient memory usage than a fixed buffer - remember to use free
		char* buffer;
	buffer = (char*)malloc(len + 1);
	//incoming *data was void, need to cast to const char
	strncpy(buffer, (const char*)data, len);
	buffer[len] = '\0';  //terminate with a null

	//it appears incoming messages may include SEND-space as a prefix and cr multiple times over in 
	//a single message.  Use SEND-space as a token separator.  Look ahead through buffer to find all tokens
	char* ptr = strstr(buffer, "SEND ");
	char* ptrNext;

	while (ptr != nullptr) {
		ptrNext = strstr(ptr + 5, "SEND ");

		if (ptrNext != nullptr) {
			//sneaky; temporarily put a null terminator at ptrNext position
			//so that we don't send the entire rest of string
			ptrNext[0] = '\0';
			tokenProcessor(ptr + 5, client);  //send part after "SEND "
			ptrNext[0] = 'S';  //revert to S
		}
		else {
			//last token
			tokenProcessor(ptr + 5, client);  //send part after "SEND "
			break;
		}
		ptr = ptrNext;
	}

	

	free(buffer);
}




/// <summary>
/// Loconet token processor. The LocoNet over TCP protocol requries that every incoming loconet SEND XX YY message
/// be echoed out as RECEIVE XX YY followed by SENT OK.  Then the processor can actually process the message content.
/// This processor only handles turnout, aspect and sensor messages
/// </summary>
/// <param name="msg">A single incoming SEND message</param>
/// <param name="client">TCP client</param>
void nsLOCONETaccessoryProcessor::tokenProcessor(char* msg, AsyncClient* client) {
	//its easier to break on spaces using strtok and build a vector of ints
	//and use strtoul just for conversion of a single token rather than iteration of msg
	//https://wiki.rocrail.net/doku.php?id=loconet:ln-pe-en


	std::vector<std::uint8_t> tokens;
#define BUFSIZE 25
	char buf[BUFSIZE];

	trace(Serial.printf("In: %s\r\n", msg);)

		//Loconet over TCP requires that we echo the message prefixed by RECEIVE
		//and follow this with SENT OK
		std::string m;
	m.append("RECEIVE ");
	m.append(msg);
	m.append("\nSENT OK\n");

	//retain a copy of the loconet message, but prefix with SEND
	std::string msgCopy;
	msgCopy = "SEND ";
	msgCopy.append(msg);

	char* tokenSplit = strtok(msg, " ");
	while (tokenSplit != NULL) {
		//tokens.push_back(strtoul(tokenSplit, nullptr, 16));
		//emplace does not create copies of things and is more efficient
		tokens.emplace_back(strtoul(tokenSplit, nullptr, 16));
		tokenSplit = strtok(NULL, " ");
	}

	uint8_t checksum;
	//XOR all tokens, should give 0xFF
	for (auto t : tokens) { checksum ^= t; }

	//if checksum fails, ignore the message
	if (checksum != 0xFF) {
		trace(Serial.println("checksum fail");)
			return;
	}


	nsESPaccessory::queueMessage(m);
	m.clear();

	//TWO-TOKEN messages not supported
	//OPC_IDLE:
	//OPC_GPON:
	//PC_GPOFF:

	

	//FOUR-TOKEN messages
	if (tokens.size() == 4) {

		switch (tokens[0]) {
			
		case OPC_SW_REP://request a sensor status, decays to 0xB2 handler.  Panel Pro sends B2 instead of B1
		case OPC_INPUT_REP:  //report a sensor status
			/* <0xB2>,<SN1>,<SN2>,<CHK>*/
			/* <0xB1>,<SN1>,<SN2>,<CHK> SENSOR state REPORT  NO feedback
<SN1> =<0,A6,A5,A4- A3,A2,A1,A0>, 7 ls adr bits. A1,A0 select 1 of 4 input pairs in a DS54
<SN2> =<0,1,I,L- A10,A9,A8,A7> Report/status bits and 4 MS adr bits.
 this <B1> opcode encodes input levels for turnout feedback
"I" =0 for "aux" inputs (normally not feedback), 1 for "switch" input used for turnout
feedback for DS54 ouput/turnout # encoded by A0-A10
"L" = 0 for this input 0V (LO), 1= this input > +6V (HI)
alternately;
<SN2> =<0,0,C,T- A10,A9,A8,A7>
Report/status bits and 4 MS adr bits.
this <B1> opcode encodes current OUTPUT levels
"C"= 0 if "Closed" ouput line is OFF, 1="closed" output line is ON (sink current)
"T"=0 if "Thrown" output line is OFF, 1="thrown" output line is ON (sink I)

*/

			{//scope block
				//2026-03-30 new
				//nsWiThrottle::relayLocoNetMessage(msgCopy);


				//Digitrax DS54 address logic. SN1,2 hold A10-A0, left shift these and append SN2<5> as lsb A0, giving 12 bits
				//this logic needs to go into the ESPACC

				uint16_t addr = ((tokens[2] & 0b1111)<< 7) + tokens[1];
				addr = addr << 1;
				addr += (tokens[2] & 0b100000) == 0 ? 0 : 1;
				addr++;  //DCC addresses start at 1, range 1-4096
				trace(Serial.printf("sensor a=%d\n", addr);)

					
			}

			//DO SOMETHING...
			//Remember we also want to send RECIEVE B2 messages on sensor changes


			break;


		case OPC_SW_REQ:
			/*Command a turnout.
			<0xB0>,<SW1>,<SW2>,<CHK>
			<SW1> =<0,A6,A5,A4- A3,A2,A1,A0>, 7 ls adr bits. A1,A0 select 1 of 4 input pairs in a DS54
			<SW2> =<0,0,DIR,ON- A10,A9,A8,A7> Control bits and 4 MS adr bits.
			DIR=1 for Closed,/GREEN, =0 for Thrown/RED
			ON=1 for Output ON, =0 FOR output OFF
			Note-,Immediate response of <0xB4><30><00> if command failed, otherwise no response

			JRMI Panel Pro sends turnouts as 0-2047 which maps to 1-2048 in the 'real world'. It sends a turnout command twice.  First with power on, then with power off.
			e.g. throw+poweron then throw+poweroff
			*/


			uint16_t addr = tokens[1];
			addr += (tokens[2] & 0x0F) << 8;
			/*2026-02-05 deprecated, we now use actionAccessoryFromLocoNet()
			bool closed = (tokens[2] & 0b00100000) == 0 ? false : true;
			bool onState = (tokens[2] & 0b00010000) == 0 ? false : true;
			writeTurnout(addr, closed, onState);  //DEPRECATED
			*/

			//actionAccessoryFromLocoNet(addr, (tokens[2] & 0b00100000) == 0 ? true : false, (tokens[2] & 0b00010000) == 0 ? false : true);

		}
		return;
	}//end 4 token block

	//VARIABLE-LENGTH token messages
	switch (tokens[0]) {

	case OPC_IMM_PACKET:
		//used for multi-aspect signalling (MAS).  the DCC payload itself is captured in the tokens
		//<0xED>,<0B>,<7F>,<REPS>,<DHI>,<IM1>,<IM2>,<IM3>,<IM4>,<IM5>,<CHK>
		//<REPS> D4,5,6=#IM bytes,D3=0(reserved); D2,1,0=repeat CNT
		//<DHI >= <0, 0, 1, IM5.7 - IM4.7, IM3.7, IM2.7, IM1.7>
		//LACK=<B4>,<7D>,<7F>,<chk> if CMD ok
		//example from Panel Pro, note that it violates DHI coding as <5> should be 1
		//[ED 0B 7F 30 01 01 77 03 00 00 22]  Extended Accessory Decoder Set Digitrax Address 8 (NMRA Address 4) to Aspect 3.

		//there's no need to decode this message in full.  We will pass on the DCC payload to a routine that directly puts DCC packets into the
		//packet engine for transmission.  IM1,IM2 are the encoded address and IM3 is the command.  Note that IMs are 7 bits, their <7> is held in
		//DHI

	{//scope block
		//expect 11 tokens
		
		
		if (tokens[1] != 0x0B) return;

		uint8_t payloadLength = (tokens[3] & 0b111000) >> 4;
		uint8_t repeats = tokens[3] & 0b11;  //zero means no repeats, i.e. one transmission
		//now build a payload array
		uint8_t payload[5];
		for (int i = 0;i < 5;i++) {
			payload[i] = tokens[5 + i];
			//add <7> for each IM as indicated in DHI byte
			payload[i] += (tokens[4] & (1 << i)) != 0 ? 0x80 : 0;
		}

		//actionDCCpacketFromLocoNet(payload, payloadLength, repeats);

		//note that the Extended Accessory Decoder specification is found here, and is updated as of 2025
		//https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9.2.1_dcc_extended_packet_formats.pdf
		//{preamble} 0 10AAAAAA 0 0BBB0AA1 0 XXXXXXXX 0 EEEEEEEE 1
		//byte 1 is A<7-2> byte 2 BBB=A<10-8> 1s compliment, AA=A<1-0>  and byte 3 is a full 8-bit payload

		

		/*2026-03-30 deprecated
				//addr is wrong!  33 dcc (offset checked) generates 36 in addr var
				//ED 0B 7F 32 01   09 71 15 00 00 38   should give 33 true dcc.
				//we do need to decode it to send as a JSON message to the ESPaccessory controller
				uint16_t addr = payload[1]>>4;  //BBB part
				addr ^= 0b111; //1's compliment
				addr = addr << 6; //move to <10-8> posn
				addr += (payload[0] & 0b111111); //add <7-2>
				addr = addr << 2;
				addr += ((payload[1] & 0b110) >> 1);  //add <1-0>
				//%.0f is decimal no dp, %02X is hex
				snprintf(buf, 5, "%02X", payload[2]);
				//nsDCCweb::broadcastLocoNetCommand("MAS", addr, buf);
				trace(Serial.printf("MAS %d %s\n", addr, buf);)
				//Serial.println("booya");
				*/

				//final respone is LACK=<B4>,<7D>,<7F>,<chk> if CMD ok
		nsESPaccessory::queueMessage("RECEIVE 0xB4 0x7D 0x7F 0x49\n");
	}
	break;
	
	}  //end tokens[0] block

}

