#include <stdint.h>
#include <pru_cfg.h>
#include "resource_table_empty.h"

/*
	Inputs:
		WDat	P8_45	R31_0
		P0/REQ	P8_46	R31_1
		P1		P8_43	R31_2
		P2		P8_44	R31_3
		P3		P8_41	R31_4

	Outputs:
		OUTEN-	P8_42	R30_5	0=LS367 enabled, RDAT active
		RDat	P8_39	R30_6
		ACK		P8_40	R30_7	1=ready to send/receive
		LED		P8_27	R30_8
		TEST	P8_29	R30_9

	Memory Locations shared with Controller:
		STATUS		0x300
		Bus ID 1	0x301
		Bus ID 2	0x302

		Received packet start	0x400	1024
		Sent packet start		0x800	2048
		Init response #1 start	0xC00	3072
		Init response #2 start	0xE00	3584
*/

#define PRU0_DRAM		0x00000			// Offset to DRAM
// First 0x200 bytes of DRAM are STACK & HEAP
volatile unsigned char *PRU1_RAM = (unsigned char *) PRU0_DRAM;

// Fixed PRU Memory Locations
#define RCVD_PACKET_ADR		0x0400		// 1048, command or data from A2
#define RCVD_DEST_ADR		0x0407		// Destination ID offset
#define RCVD_CMD_ADR		0x040F		// CMD number offset

#define RESP_PACKET_ADR		0x0800		// 2024, status or data, all responses except Inits
#define INIT_RESP_1_ADR		0x0C00		// 3072
#define INIT_RESP_2_ADR		0x0E00		// 3584

#define STATUS				0x0300
#define BUS_ID_1			0x0301
#define BUS_ID_2			0x0302

volatile register uint32_t __R30;
volatile register uint32_t __R31;

// Globals
uint32_t WDAT, REQ, P1, P2, P3;			// inputs
uint32_t OUTEN, RDAT, ACK, LED, TEST;	// outputs
unsigned char inited, busID1, busID2;

// enum pruStatuses {eIDLE, eRESET, eENABLED, eRCVDPACK, eSENDING, eWRITING, eUNKNOWN};
#define eIDLE			0x00
#define eRESET			0x01
#define eENABLED		0x02
#define eRCVDPACK		0x03
#define eSENDING		0x04
#define eWRITING		0x05
#define eUNKNOWN		0x06

// Bus statuses - REVISIT ALL STATUSES
typedef enum
{
	eUnknown,
	eBusIdle,
	eBusReset,
	eBusEnabled
} eBusState;

void		HandleReset(void);
eBusState	GetBusState(void);
void		ReceivePacket(void);
void		ProcessPacket(void);
void		SendInit(void);

//____________________
int main(int argc, char *argv[])
{
	eBusState previousBusState, currentBusState;
	previousBusState = eUnknown;

	// Set I/O constants
	WDAT  = 0x1<<0;		// P8_45 input
	REQ   = 0x1<<1;		// P8_46 input
	P1    = 0x1<<2;		// P8_43 input
	P2    = 0x1<<3;		// P8_44 input
	P3    = 0x1<<4;		// P8_41 input
	OUTEN = 0x1<<5;		// P8_42 output
	RDAT  = 0x1<<6;		// P8_39 output
	ACK   = 0x1<<7;		// P8_40 output
	LED   = 0x1<<8;		// P8_27 output
	TEST  = 0x1<<9;		// P8_29 output

	// Clear SYSCFG[STANDBY_INIT] to enable OCP master port
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	HandleReset();

	while (1)
	{
		currentBusState = GetBusState();
		if (currentBusState != previousBusState)
		{
			switch (currentBusState)
			{
				case eBusIdle:
					__R30 &= ~LED;		// LED off
					PRU1_RAM[STATUS] = eIDLE;
					break;

				case eBusReset:
					PRU1_RAM[STATUS] = eRESET;
					HandleReset();
					break;

				case eBusEnabled:
//					__R30 &= ~TEST;		// TEST=0
					__R30 |= LED;		// LED on
					PRU1_RAM[STATUS] = eENABLED;

					__R30 |= ACK;		// set ACK, ready to receive
					ReceivePacket();

					ProcessPacket();
					break;

				default:
					PRU1_RAM[STATUS] = eUNKNOWN;
			}
			previousBusState = currentBusState;
		}
	}
}

//____________________
void HandleReset(void)
{
	// Reset outputs and SP parameters

//	__R30 |= TEST;		// TEST=1
//	__R30 &= ~TEST;		// TEST=0
	__R30 &= ~ACK;		// clear ACK, we are not ready to send/receive yet
	__R30 |=  OUTEN;	// tri-state RDAT
	__R30 &= ~LED;		// LED off

	inited = 0;		// not initialized so send first Init reply
	busID1 = 0xFF;	// set bus IDs to uninitialized values
	busID2 = 0xFE;

	PRU1_RAM[BUS_ID_1] = busID1;	// reset bus IDs for Controller
	PRU1_RAM[BUS_ID_2] = busID2;
}

//____________________
eBusState GetBusState(void)
{
	// Looks at four phase signals and returns bus state
	uint32_t allPhases;

	allPhases = (__R31 & (REQ | P1 | P2 | P3))>>1;
	switch (allPhases)
	{
		case 0x0A:
		case 0x0B:
		case 0x0E:
		case 0x0F:
			return eBusEnabled;

		case 0x05:
			return eBusReset;

		default:
			return eBusIdle;
	}
}

//____________________
void ReceivePacket(void)
{
	unsigned char lastWDAT, byteInProcess, bitCnt, packetNotDone;
	unsigned char currentWDAT, WDAT_xor;
	unsigned int memoryPtr;

	lastWDAT = 1;
	byteInProcess = 0;
	bitCnt = 0;
	packetNotDone = 1;
	memoryPtr = RCVD_PACKET_ADR;

	// Wait for WDATA to go low, our t0
	while((__R31 & WDAT) == WDAT);		// Can hang here!?

	__delay_cycles(400);	// 2 us to sample in bit cell center

//	__R30 |= TEST;		// TEST=1

	while (packetNotDone == 1)
	{
		__R30 |= TEST;			// TEST=1
		currentWDAT = __R31 & WDAT;
		WDAT_xor = lastWDAT ^ currentWDAT;
		lastWDAT = currentWDAT;

		if (WDAT_xor == 0)
			byteInProcess &= 0xFE;
		else
			byteInProcess |= 0x01;

		if (bitCnt == 7)		// done with this byte?
		{
			PRU1_RAM[memoryPtr] = byteInProcess;
			memoryPtr++;
			if (byteInProcess == 0x00)	// WDAT hasn't changed in 32 us
				packetNotDone = 0;
			else
				bitCnt = 0;
		}
		else					// not done with this byte
		{
			byteInProcess = byteInProcess<<1;		// shift bits left
			bitCnt++;
		}
		__delay_cycles(360);	// 1.8 us
		__R30 &= ~TEST;			// TEST=0
		__delay_cycles(400);	// 2.0 us
	}

//	__R30 &= ~TEST;		// TEST=0

	PRU1_RAM[STATUS] = eRCVDPACK;
}

//____________________
void ProcessPacket(void)
{
	unsigned char cmd;

	cmd = PRU1_RAM[RCVD_CMD_ADR];

	if (cmd == 0x85)
		SendInit();
	else
	{
//		Tell Controller and wait for go-ahead
	}

}

//____________________
void SendInit(void)
{
	// Send first or second Init packet, based on inited
	
	
	

}
