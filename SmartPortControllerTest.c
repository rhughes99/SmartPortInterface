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
//#define PRU0_DRAM		0x00000			// Offset to DRAM
#define PRU1_DRAM		0x02000
//#define PRU_SHAREDMEM	0x10000			// Offset to shared memory

//unsigned int *pru0DRAM_32int_ptr;		// Points to the start of PRU 0 usable RAM
unsigned int *pru1DRAM_32int_ptr;		// Points to the start of PRU 1 usable RAM
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

	unsigned int *pru;		// Points to start of PRU memory
//	unsigned int *pruDRAM_32int_ptr;
	int	fd;

	fd = open ("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1)
	{
		printf ("*** ERROR: could not open /dev/mem.\n");
		return EXIT_FAILURE;
	}
	pru = mmap (0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);
	if (pru == MAP_FAILED)
	{
		printf ("*** ERROR: could not map memory.\n");
		return EXIT_FAILURE;
	}
	close(fd);

	// Set memory pointers
//	pru0DRAM_32int_ptr =     pru + PRU0_DRAM/4 + 0x200/4;	// Points to 0x200 of PRU0 memory
	pru1DRAM_32int_ptr =     pru + PRU1_DRAM/4 + 0x200/4;	// Points to 0x200 of PRU1 memory
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
//					spID1 = 0xFF;
//					spID2 = 0xFF;
					spID1 = *pruStatusPtr + 1;
					spID2 = *pruStatusPtr + 2;
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

				destID = *(rcvdPacketPtr + 7);						// with msb = 1
				type   = *(rcvdPacketPtr + 9);						// 0x80=Cmd, 0x81=Status, 0x82=Data
				cmdNum = *(rcvdPacketPtr + 15);

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
	running = 0;
	(void) signal(SIGINT, SIG_DFL);		// reset signal handling of SIGINT
}

//____________________
void myDebug(int sig)
{
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
