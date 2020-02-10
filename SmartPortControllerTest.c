/*	SmartPort Controller TEST
	Emulates two devuces
	Shared memory example
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <sys/mman.h>

void myShutdown(int sig);
void myDebug(int sig);
void loadDiskImages(const char *image1, const char *image2);
void saveDiskImage(unsigned char image, const char *fileName);

void encodeInitReplyPackets(void);
void encodeStdStatusReplyPacket(unsigned char srcID, unsigned char dataStat);
void encodeStdDibStatusReplyPacket(unsigned char srcID, unsigned char dataStat);
void encodeHandshakeReplyPacket(void);
void encodeDataPacket(unsigned char srcID, unsigned char dataStat, unsigned char device, unsigned int block);

char decodeDataPacket(void);
char checkCmdChecksum(void);
void printPacket(unsigned char id);
void debugDataPacket(void);


// PRU Memory Locations
#define PRU_ADDR		0x4A300000		// Start of PRU memory Page 163 am335x TRM
#define PRU_LEN			0x80000			// Length of PRU memory
#define PRU1_DRAM		0x02000
//#define PRU_SHAREDMEM	0x10000			// Offset to shared memory

//unsigned int *pru1DRAM_32int_ptr;		// Points to the start of PRU 1 usable RAM
unsigned char *pru1DRAM_char_ptr;
//unsigned int *prusharedMem_32int_ptr;	// Points to the start of shared memory


static unsigned char *pruStatusPtr;						// PRU -> Controller
static unsigned char *rcvdPacketPtr;					// start of what A2 sent us
static unsigned char *respPacketPtr;					// start of what we send to A2
static unsigned char *initResp1Ptr;						// start of Init response 1
static unsigned char *initResp2Ptr;						// start of Init response 2


unsigned char running;
#define NUM_BLOCKS	65536
unsigned char theImages[2][NUM_BLOCKS][512];				// [device][block][byte]
unsigned char tempBuffer[512];								// holds data from A2 till verified

// IDs provided by A2
unsigned char spID1, spID2, imageToggle;	// we seem to assume spID2 > spID1

//____________________
int main(int argc, char *argv[])
{
	unsigned char destID, destDevice, type, cmdNum, statCode;
	unsigned char msbs, blkNumLow, blkNumMid, blkNumHi;
	unsigned char diskImage1Changed, diskImage2Changed;		// 1 = image changed since loading
	unsigned int i, resetCnt, loopCnt, blkNum, readCnt1, writeCnt1, readCnt2, writeCnt2;
	char saveName[64], imageName[64];
	size_t length;

	enum pruStatuses {eIDLE, eRESET, eENABLED, eRCVDPACK, eSENDING, eWRITING, eUNKNOWN};
	enum pruStatuses pruStatus, lastPruStatus;

	enum cmdNums {eSTATUS=0x80, eREADBLK, eWRITEBLK, eFORMAT, eCONTROL, eINIT, eOPEN, eCLOSE, eREAD, eWRITE};
	enum extCmdNums {eEXTSTATUS=0xC0, eEXTREADBLK, eEXTWRITEBLK, eEXTFORMAT, eEXTCONTROL, eEXTINIT, eEXTOPEN, eEXTCLOSE, eEXTREAD, eEXTWRITE};

	unsigned char *pru;		// start of PRU memory
	int	fd;

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1)
	{
		printf("*** ERROR: could not open /dev/mem.\n");
		return EXIT_FAILURE;
	}
	pru = mmap(0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);
	if (pru == MAP_FAILED)
	{
		printf("*** ERROR: could not map memory.\n");
		return EXIT_FAILURE;
	}
	close(fd);

	// Set memory pointers
	pru1DRAM_char_ptr  = pru + PRU1_DRAM + 0x200;
//	prusharedMem_32int_ptr = pru + PRU_SHAREDMEM/4;			// Points to start of shared memory

	pruStatusPtr	= pru1DRAM_char_ptr + 0x0100;			// 0x200 + 0x100 =  768
	rcvdPacketPtr	= pru1DRAM_char_ptr + 0x0200;			// 0x200 + 0x200 = 1024
	respPacketPtr	= pru1DRAM_char_ptr + 0x0600;			// 0x200 + 0x600 = 2048
	initResp1Ptr	= pru1DRAM_char_ptr + 0x0A00;			// 0x200 + 0xA00 = 3072
	initResp2Ptr	= pru1DRAM_char_ptr + 0x0C00;			// 0x200 + 0xC00 = 3584

	diskImage1Changed = 0;
	diskImage2Changed = 0;

	(void) signal(SIGINT,  myShutdown);					// ^c = graceful shutdown
	(void) signal(SIGTSTP, myDebug);					// ^z

	lastPruStatus = eUNKNOWN;
	spID1 = 0xFF;										// we are not inited yet
	spID2 = 0xFF;
	resetCnt = 0;
	readCnt1 = 0;
	readCnt2 = 0;
	writeCnt1 = 0;
	writeCnt2 = 0;
	loopCnt = 0;										// do something every n times around the loop
	running = 1;
	imageToggle = 0;

	encodeInitReplyPackets();							// put two Init reply packets in PRU ram

	printf("\n--- SmartPortIF running\n");
	do
	{
		usleep(100);
		pruStatus = *pruStatusPtr;
		switch (pruStatus)
		{
			case eIDLE:
			{
				if (pruStatus != lastPruStatus)
				{
					printf("Idle\n");
					lastPruStatus = pruStatus;
				}
				break;
			}
			case eRESET:
			{
				if (pruStatus != lastPruStatus)
				{
					printf("--- Reset %d \n", resetCnt);
					spID1 = *(pruStatusPtr + 1);
					spID2 = *(pruStatusPtr + 2);
					printf("spID1=0x%X spID2=0x%X\n", spID1, spID2);

					readCnt1 = 0;
					writeCnt1 = 0;
					readCnt2 = 0;
					writeCnt2 = 0;
					resetCnt++;
					lastPruStatus = pruStatus;
				}
				break;
			}
			case eENABLED:
			{
				if (pruStatus != lastPruStatus)
				{
					printf("Enabled\n");
					lastPruStatus = pruStatus;
				}
				break;
			}
			case eRCVDPACK:
			{	// PRU has a packet, command or data
				printf("Received packet\n");

				destID = *(rcvdPacketPtr + 7);					// with msb = 1
				type   = *(rcvdPacketPtr + 9);					// 0x80=Cmd, 0x81=Status, 0x82=Data
				cmdNum = *(rcvdPacketPtr + 15);

				printf("\tdestID = 0x%X\n", destID);
				printf("\ttype   = 0x%X\n", type);
				printf("\tcmdNm  = 0x%X\n", cmdNum);

				destDevice = destID - spID1;					// theImages[0] or [1]

				printPacket(destID);
				break;
			}
			case eSENDING:
			{
				if (pruStatus != lastPruStatus)
				{
					printf("Sending...\n");
					lastPruStatus = eSENDING;
				}
				break;
			}
			case eWRITING:
			{
				if (pruStatus != lastPruStatus)
				{
					printf("Writing...\n");
					lastPruStatus = eWRITING;
				}
				break;
			}
			default:
				printf("*** Unexpected pruStatus: %d\n", pruStatus);
		}
	} while (running);

	printf ("---Shutting down...\n");

	if(munmap(pru, PRU_LEN))
		printf("*** ERROR: munmap failed at Shutdown\n");

	return EXIT_SUCCESS;
}

//____________________
void myShutdown(int sig)
{
	// ctrl-c
	printf("\n");
	running = 0;
	(void) signal(SIGINT, SIG_DFL);		// reset signal handling of SIGINT
}

//____________________
void myDebug(int sig)
{
	// ctrl-z
	unsigned int i;

	printf("\n");
//	for (i=0; i<16; i++)
//		printf("%d 0x%X\n", i, *(rcvdPacketPtr + i));

	printf("0 %X [FF]\n", *rcvdPacketPtr);
	printf("1 %X [3F]\n", *(rcvdPacketPtr+1));
	printf("2 %X [CF]\n", *(rcvdPacketPtr+2));
	printf("3 %X [F3]\n", *(rcvdPacketPtr+3));
	printf("4 %X [FC]\n", *(rcvdPacketPtr+4));
	printf("5 %X [FF]\n", *(rcvdPacketPtr+5));
	printf("6 %X [C3]\n", *(rcvdPacketPtr+6));

	printf("25 %X [BB]\n", *(rcvdPacketPtr+25));
	printf("26 %X [BE]\n", *(rcvdPacketPtr+26));
	printf("27 %X [C8]\n", *(rcvdPacketPtr+27));
}

//____________________
void loadDiskImages(const char *image1, const char *image2)
{
}

//____________________
void saveDiskImage(unsigned char image, const char *fileName)
{
}

//____________________
void encodeStdStatusReplyPacket(unsigned char srcID, unsigned char dataStat)
{
}

//____________________
void encodeInitReplyPackets(void)
{
	// Puts two Init reply packets in PRU.
	// Source IDs filled in by PRU.
	// This routine computes checksum for all elements except source ID
	//  and puts it in Ptr+19. PRU completes calculation and puts
	//  result in Ptr+19 & Ptr+20
	// First device is big HD, second is 800k HD
	unsigned char checksum = 0;
	unsigned int i;

	// Device 1
	*(initResp1Ptr     ) = 0xFF;				// sync bytes
	*(initResp1Ptr +  1) = 0x3F;
	*(initResp1Ptr +  2) = 0xCF;
	*(initResp1Ptr +  3) = 0xF3;
	*(initResp1Ptr +  4) = 0xFC;
	*(initResp1Ptr +  5) = 0xFF;

	*(initResp1Ptr +  6) = 0xC3;				// packet begin
	*(initResp1Ptr +  7) = 0x80;				// destination
	*(initResp1Ptr +  8) = 0x00;				// source, filled in by PRU
	*(initResp1Ptr +  9) = 0x81;				// packet Type: 1 = status
	*(initResp1Ptr + 10) = 0x80;				// aux type: 0 = standard packet
	*(initResp1Ptr + 11) = 0x80;				// data status: 0 = not last device on bus
	*(initResp1Ptr + 12) = 0x84;				// odd byte count: 4
	*(initResp1Ptr + 13) = 0x80;				// groups-of-7 count: 0

	for (i=7; i<14; i++)
		checksum ^= *(initResp1Ptr+i);

												// 32 MB  - 0x010000 (or 0x00FFFF ???)
	*(initResp1Ptr + 14) = 0xC0;				// odd MSBs: 100 0000
	*(initResp1Ptr + 15) = 0xF0;				// device status: 1111 0000, read/write
	checksum ^= 0xF0;
	*(initResp1Ptr + 16) = 0x80;				// block size low byte: 0x00
	*(initResp1Ptr + 17) = 0x80;				// block size mid byte: 0x00
	*(initResp1Ptr + 18) = 0x81;				// block size high byte: 0x01
	checksum ^= 0x01;

//	*(initResp1Ptr + 19) =  checksum	   | 0xAA;	// 1 C6 1 C4 1 C2 1 C0
//	*(initResp1Ptr + 20) = (checksum >> 1) | 0xAA;	// 1 C7 1 C5 1 C3 1 C1
	*(initResp1Ptr + 19) = checksum;
	*(initResp1Ptr + 20) = 0x00;

	*(initResp1Ptr + 21) = 0xC8;				// PEND
	*(initResp1Ptr + 22) = 0x00;				// end of packet marker in memory

	// Device 2
	*(initResp2Ptr     ) = 0xFF;				// sync bytes
	*(initResp2Ptr +  1) = 0x3F;
	*(initResp2Ptr +  2) = 0xCF;
	*(initResp2Ptr +  3) = 0xF3;
	*(initResp2Ptr +  4) = 0xFC;
	*(initResp2Ptr +  5) = 0xFF;

	*(initResp2Ptr +  6) = 0xC3;				// packet begin
	*(initResp2Ptr +  7) = 0x80;				// destination
	*(initResp2Ptr +  8) = 0x00;				// source, filled in by PRU
	*(initResp2Ptr +  9) = 0x81;				// packet Type: 1 = status
	*(initResp2Ptr + 10) = 0x80;				// aux type: 0 = standard packet
	*(initResp2Ptr + 11) = 0xFF;				// data status: FF = last device on bus
	*(initResp2Ptr + 12) = 0x84;				// odd byte count: 4
	*(initResp2Ptr + 13) = 0x80;				// groups-of-7 count: 0

	checksum = 0;
	for (i=7; i<14; i++)
		checksum ^= *(initResp2Ptr+i);

												// 800kB  - 0x000640
	*(initResp2Ptr + 14) = 0xC0;				// odd MSBs: 100 0000
	*(initResp2Ptr + 15) = 0xF0;				// device status: 1111 0000, read/write
	checksum ^= 0xF0;
	*(respPacketPtr + 16) = 0xC0;				// block size low byte: 0x40
	checksum ^= 0x40;
	*(respPacketPtr + 17) = 0x86;				// block size mid byte: 0x06
	checksum ^= 0x06;
	*(respPacketPtr + 18) = 0x80;				// block size high byte: 0x00

//	*(initResp2Ptr + 19) =  checksum	   | 0xAA;	// 1 C6 1 C4 1 C2 1 C0
//	*(initResp2Ptr + 20) = (checksum >> 1) | 0xAA;	// 1 C7 1 C5 1 C3 1 C1
	*(initResp2Ptr + 19) = checksum;
	*(initResp2Ptr + 20) = 0x00;

	*(initResp2Ptr + 21) = 0xC8;				// PEND
	*(initResp2Ptr + 22) = 0x00;				// end of packet marker in memory
}

//____________________
void encodeStdDibStatusReplyPacket(unsigned char srcID, unsigned char dataStat)
{
}

//____________________
void encodeHandshakeReplyPacket(void)
{
}

//____________________
void encodeDataPacket(unsigned char srcID, unsigned char dataStat, unsigned char device, unsigned int block)
{
}

//____________________
char decodeDataPacket(void)
{
	return 'a';
}

//____________________
char checkCmdChecksum(void)
{
	return 'a';
}

//____________________
void printPacket(unsigned char id)
{
}

//____________________
void debugDataPacket(void)
{
}
