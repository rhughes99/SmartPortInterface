/*	SmartPort PRU
	Emulates two devices
	Modern OS, shared memory

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
		Wait flag	0x303

		Received packet start	0x400	1024
		Sent packet start		0x800	2048
		Init response #1 start	0xC00	3072
		Init response #2 start	0xE00	3584

	03/03/2020
*/
#include <stdint.h>
#include <pru_cfg.h>
#include "resource_table_empty.h"

// First 0x200 bytes of PRU RAM are STACK & HEAP
#define PRU0_DRAM		0x00000			// Offset to Data RAM
volatile unsigned char *PRU1_RAM = (unsigned char *) PRU0_DRAM;

// Fixed PRU Memory Locations
#define STATUS_ADR			0x0300		// address of eBusState
#define BUS_ID_1_ADR		0x0301		// address location of ID1
#define BUS_ID_2_ADR		0x0302		// address location of ID2
#define WAIT_ADR			0x0303		// address of WAIT, for Controller to tell us to continue
#define WAIT_SET			0x00		// PRU -> Controller: waiting
#define WAIT_GO				0x01		// Controller -> PRU: send response
#define WAIT_SKIP			0x02		// Controller -> PRU: continue without sending response

#define RCVD_PACKET_ADR		0x0400		// 1048, command or data from A2
#define RCVD_PBEGIN_ADR		0x0406		// Packet Begin
#define RCVD_DEST_ADR		0x0407		// Destination ID offset
#define RCVD_CMD_ADR		0x040F		// CMD number offset

#define RESP_PACKET_ADR		0x0800		// 2024, status or data, all responses except Inits
#define INIT_RESP_1_ADR		0x0C00		// 3072
#define INIT_RESP_2_ADR		0x0E00		// 3584

volatile register uint32_t __R30;
volatile register uint32_t __R31;

// Globals
uint32_t WDAT, REQ, P1, P2, P3;			// inputs
uint32_t OUTEN, RDAT, ACK, LED, TEST;	// outputs
unsigned char initCnt, busID1, busID2;

// Must be identical to SmartPortController.c
typedef enum
{
	eIDLE, eRESET, eENABLED, eRCVDPACK, eSENDING, eWRITING, eUNKNOWN, eERROR
} eBusState;

void			HandleReset(void);
eBusState		GetBusState(void);
unsigned char	WaitForReq(void);
void			ReceivePacket(void);
void			InsertBit(signed char bit);
void			ProcessPacket(void);
void			SendInit(unsigned char dest);
void			SendPacket(char initFlag, unsigned int memPtr);

//____________________
int main(int argc, char *argv[])
{
	eBusState busState;

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
		busState = GetBusState();
		switch (busState)
		{
			case eIDLE:
			{
				PRU1_RAM[STATUS_ADR] = eIDLE;
				__R30 &= ~LED;			// LED off
//				__R30 |=  OUTEN;		// float RDAT
				__R30 |=  ACK;			// ACK = 1, ready to receive
				break;
			}
			case eRESET:
			{
				PRU1_RAM[STATUS_ADR] = eRESET;
				HandleReset();
				break;
			}
			case eENABLED:
			{
				PRU1_RAM[STATUS_ADR] = eENABLED;
				__R30 |= LED;			// LED on
				__R30 |= ACK;			// ACK = 1, ready to receive

				if (WaitForReq())		// 1 = REQ set, 0 = bus disabled
				{
					ReceivePacket();	// receive packet & store in memory
					ProcessPacket();	// either send Init or wait for Controller
				}
				break;
			}
			default:
				PRU1_RAM[STATUS_ADR] = eUNKNOWN;
		}
	}
}

//____________________
unsigned char WaitForReq(void)
{
	// Wait for REQ = 1 or bus disabled
	// Return: 1 = REQ set, 0 = bus disabled
	unsigned char done, result;

	done = 0;
	while (!done)
	{
		if (GetBusState() != eENABLED)
		{
			result = 0;
			done = 1;
		}

		if ((__R31 & REQ) == REQ)
		{
			result = 1;
			done = 1;
		}
	}
	return result;
}

//____________________
void HandleReset(void)
{
	// Reset outputs and SP parameters

	__R30 &= ~TEST;		// TEST = 0
	__R30 &= ~ACK;		// ACK = 0, we are not ready to send/receive yet
	__R30 |=  OUTEN;	// float RDAT
	__R30 &= ~LED;		// LED off

	initCnt = 0;		// not initialized so send Init reply
	busID1 = 0xFF;		// set bus IDs to uninitialized values
	busID2 = 0xFF;

	PRU1_RAM[BUS_ID_1_ADR] = busID1;	// reset bus IDs for Controller
	PRU1_RAM[BUS_ID_2_ADR] = busID2;
	PRU1_RAM[WAIT_ADR] = 0x00;				// don't wait for Controller input
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
			return eENABLED;

		case 0x05:
			return eRESET;

		default:
			return eIDLE;
	}
}

//____________________
void ReceivePacket(void)
{
	unsigned char count, lastWDAT;

	// Set up InsertBit()
	InsertBit(-1);

	while ((__R31 & WDAT) == WDAT);		// wait for WDAT to go low

	while (1)
	{
		count = 0;
		lastWDAT = __R31 & WDAT;
		while ((__R31 & WDAT) == lastWDAT)
		{
			count++;
			if (count > 65)	// was 64
				return;

			__delay_cycles(100);	// 0.5 us
		}

		// Convert  count into bit(s)
		if (count < 10)			// 1
		{
			InsertBit(1);
		}
		else if (count < 17)	// 01
		{
			InsertBit(0);
			InsertBit(1);
		}
		else if (count < 24)	// 001
		{
			InsertBit(0);
			InsertBit(0);
			InsertBit(1);
		}
		else if (count < 31)	// 0001
		{
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(1);
		}
		else if (count < 38)	// 0 0001
		{
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(1);
		}
		else if (count < 45)	// 00 0001
		{
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(1);
		}
		else if (count < 52)	// 000 0001
		{
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(1);
		}
		else					// 0000 0001
		{
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(0);
			InsertBit(1);
		}
	}
}

//____________________
void InsertBit(signed char bit)
{
	// Insert bit into byteInProcess and put in RAM when byte completed
	// If bit = -1, reset parameters (start of packet)

	static unsigned char bitCnt, byteInProcess;
	static unsigned int memoryPtr;

	if (bit == -1)
	{
		bitCnt = 1;
		byteInProcess = 0x02;		// we miss first 1 in byte 0
		memoryPtr = RCVD_PACKET_ADR;
	}
	else
	{
		if (bit == 0)
			byteInProcess &= 0xFE;	// clear LSB
		else
			byteInProcess |= 0x01;	// set LSB

		if (bitCnt == 7)
		{
			PRU1_RAM[memoryPtr] = byteInProcess;
			memoryPtr++;
			bitCnt = 0;
		}
		else
		{
			byteInProcess = byteInProcess<<1;	// shift bits left
			bitCnt++;
		}
	}
}

/*
//____________________
void ReceivePacket(void)
{
	// Receive packet from A2 and put into data memory, starting at RCVD_PACKET_ADR

	unsigned char lastWDAT, byteInProcess, bitCnt, packetNotDone;
	unsigned char currentWDAT, WDAT_xor, timingToggle;
	unsigned int memoryPtr;

	lastWDAT = 1;
	byteInProcess = 0;
	bitCnt = 0;
	packetNotDone = 1;
	timingToggle = 0;
	memoryPtr = RCVD_PACKET_ADR;

	// Wait for WDATA to go low, our t0
	while ((__R31 & WDAT) == WDAT);

	__delay_cycles(500);	// ~n us to sample in bit cell center

	while (packetNotDone == 1)
	{
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
			byteInProcess = byteInProcess<<1;	// shift bits left
			bitCnt++;
		}

		// Timing appears to be dependent on number of cases (!?)
		//  Starting with 6 + default
		// ~762, on average?
		switch (timingToggle)
		{
			case 0:
				__delay_cycles(759);
				timingToggle++;
				break;

			case 1:
				__delay_cycles(760);
				timingToggle++;
				break;

			case 2:
				__delay_cycles(759);
				timingToggle++;
				break;

			case 3:
				__delay_cycles(760);
				timingToggle++;
				break;

			case 4:
				__delay_cycles(759);
				timingToggle++;
				break;

			case 5:
				__delay_cycles(760);
				timingToggle++;
				break;

			case 6:
				__delay_cycles(759);
				timingToggle++;
				break;

			case 7:
				__delay_cycles(759);
				timingToggle++;
				break;

			case 8:
				__delay_cycles(759);
				timingToggle++;
				break;

			default:
				__delay_cycles(759);
				timingToggle = 0;
		}
	}
	PRU1_RAM[STATUS_ADR] = eRCVDPACK;
}
*/
//____________________
void ProcessPacket(void)
{
	// If packet is Init, immediately send Init response
	// Otherwise, tell Controller and wait for instructions
	// Do we assume packet is always for us?

	unsigned char cmd, dest;

	cmd  = PRU1_RAM[RCVD_CMD_ADR];
	dest = PRU1_RAM[RCVD_DEST_ADR];		// this is our assigned ID in INIT packets

	if ((cmd == 0x85) || (cmd == 0xF0))	// WHY???
//	if (cmd == 0x85)
		SendInit(dest);
	else if (PRU1_RAM[RCVD_PBEGIN_ADR] == 0xC3)	// quick sanity check
	{
		PRU1_RAM[STATUS_ADR] = eRCVDPACK;		// tell Controller packet received

		__R30 &= ~ACK;			// ACK = 0, to tell A2 we are responding

		PRU1_RAM[WAIT_ADR] = WAIT_SET;			// tell Controller we are
		while(PRU1_RAM[WAIT_ADR] == WAIT_SET)	// waiting for go-ahead
			__delay_cycles(1600);

		if (PRU1_RAM[WAIT_ADR] == WAIT_GO)
			SendPacket(0, RESP_PACKET_ADR);
	}
	else
		PRU1_RAM[STATUS_ADR] = eERROR;	// tell Controller something is fishy
}

//____________________
void SendInit(unsigned char dest)
{
	// Send first or second Init packet, based on initCnt

	unsigned char finalCheckSum, checkSumA, checkSumB;

	__R30 &= ~ACK;		// ACK = 0, to tell A2 we are responding

	if (initCnt == 0)
	{
		PRU1_RAM[INIT_RESP_1_ADR+8] = dest;	// put ID in our response

		// Compute checksum; Controller started
		finalCheckSum = PRU1_RAM[INIT_RESP_1_ADR+19] ^ dest;

		checkSumA = finalCheckSum | 0xAA;
		checkSumB = (finalCheckSum >> 1) | 0xAA;

		PRU1_RAM[INIT_RESP_1_ADR+19] = checkSumA;
		PRU1_RAM[INIT_RESP_1_ADR+20] = checkSumB;

		initCnt++;
		SendPacket(1, INIT_RESP_1_ADR);
		PRU1_RAM[BUS_ID_1_ADR] = dest;		// for Controller
	}
	else if (initCnt == 1)
	{
		PRU1_RAM[INIT_RESP_2_ADR+8] = dest;	// put ID in our response

		// Compute checksum; Controller started
		finalCheckSum = PRU1_RAM[INIT_RESP_2_ADR+19] ^ dest;

		checkSumA = finalCheckSum | 0xAA;
		checkSumB = (finalCheckSum >> 1) | 0xAA;

		PRU1_RAM[INIT_RESP_2_ADR+19] = checkSumA;
		PRU1_RAM[INIT_RESP_2_ADR+20] = checkSumB;

		initCnt++;
		SendPacket(1, INIT_RESP_2_ADR);
		PRU1_RAM[BUS_ID_2_ADR] = dest;		// for Controller
	}
}

//____________________
void SendPacket(char initFlag, unsigned int memPtr)
{
	// Send packet starting at memPtr, ending with 0x00
	// initFlag == 1, we are sending init and handle ending differently

	unsigned char byteInProgress, bitMask, sendDone;

	PRU1_RAM[STATUS_ADR] = eSENDING;	// for Controller

	while((__R31 & REQ) == REQ);	// wait for A2 to finish its send cycle, REQ = 0

	// Set up outputs
	__R30 |= ACK;		// ACK = 1, ready to send
	__R30 |= RDAT;		// set to know value, 1
	__R30 &= ~OUTEN;	// clear OUTEN to enable RDAT

	// Set up parameters
	bitMask = 0x80;		// we send msb first
	sendDone = 0;		// 1 = done

	while ((__R31 & REQ) == 0);		// wait for A2 to indicate ready to receive, ~60 us

	while (sendDone == 0)
	{
		byteInProgress = PRU1_RAM[memPtr];
		if (byteInProgress == 0x00)		// end of packet marker
			sendDone = 1;

		byteInProgress &= bitMask;
		if (byteInProgress == bitMask)	// we have a 1
			__R30 &= ~RDAT;		// RDAT = 0
		else
			__R30 |= RDAT;		// RDAT still 1, for timing

		__delay_cycles(350);

		__R30 |= RDAT;			// RDAR = 1

		if (bitMask == 1)		// we just sent lsb so time for next byte
		{
			memPtr++;
			bitMask = 0x80;
		}
		else
			bitMask = bitMask >> 1;

//		__delay_cycles(420);
		__delay_cycles(410);	// starting point
//		__delay_cycles(400);
//		__delay_cycles(390);
	}

	__R30 &= ~ACK;			// ACK = 0, tell A2 we are done with this packet
	__R30 |= OUTEN;			// float RDAT

	if (initFlag == 1)
		__delay_cycles(5000);	// 25 us

	else
		while ((__R31 & REQ) == REQ);	// wait for REQ = 0
}
