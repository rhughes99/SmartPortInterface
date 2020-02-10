/*	SmartPort Controller
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

// First image is boot device
//const char *diskImages[] = {"IIGSSystem604/LiveInstall.po", "Large/BigBlank.po"};

//const char *diskImages[] = {"Large/Sys604Copy3.po", "Large/BBBProgram.po"};
//const char *diskImages[] = {"Large/Sys604Copy3.po", "Large/Assembleur.2mg"};
//const char *diskImages[] = {"Large/BBBProgram.po", "Large/Sys604Copy3.po"};
//const char *diskImages[] = {"Large/Sys604Copy3.po", "Large/GOProDOS.2mg"};
//const char *diskImages[] = {"Large/Sys604Copy3.po", "Large/GSUtilities.2mg"};
//const char *diskImages[] = {"Large/Sys604Copy3.po", "Large/BigBlank.po"};

const char *diskImages[] = {"Large/Sys604Copy3.po", "Large/ZipChipUtil.po"};

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

	pruStatusPtr	= pru1DRAM_char_ptr + 0x0100;				// 0x200 + 0x100 =  768
	rcvdPacketPtr	= pru1DRAM_char_ptr + 0x0200;				// 0x200 + 0x200 = 1024
	respPacketPtr	= pru1DRAM_char_ptr + 0x0600;				// 0x200 + 0x600 = 2048
	initResp1Ptr	= pru1DRAM_char_ptr + 0x0A00;				// 0x200 + 0xA00 = 3072
	initResp2Ptr	= pru1DRAM_char_ptr + 0x0C00;				// 0x200 + 0xC00 = 3584


	loadDiskImages(diskImages[0], diskImages[1]);			// load both images
	diskImage1Changed = 0;
	diskImage2Changed = 0;
	
	(void) signal(SIGINT,  myShutdown);						// ^c = graceful shutdown
	(void) signal(SIGTSTP, myDebug);						// ^z


	lastPruStatus = eUNKNOWN;
	spID1 = 0xFF;											// we are not inited yet
	spID2 = 0xFF;
	resetCnt = 0;
	readCnt1 = 0;
	readCnt2 = 0;
	writeCnt1 = 0;
	writeCnt2 = 0;
	loopCnt = 0;											// do something every n times around the loop
	running = 1;
	imageToggle = 0;

	encodeInitReplyPackets();								// put two Init reply packets in PRU ram


	printf("\n--- SmartPortIF running: ");
	do
	{
//RMH		usleep(1000);
		
		if (spID1 == 0xFF)			// Are we inited yet?
		{
			usleep(1);
			spID1 = *(pruStatusPtr + 2);
			spID2 = *(pruStatusPtr + 3);
			if (spID1 != 0xFF)
				printf("IDs 0x%X and 0x%X\n", spID1, spID2);
		}
		else
		{
			usleep(100);
			pruStatus = *pruStatusPtr;		
			switch (pruStatus)
			{
				case eIDLE:
				{
					if (pruStatus != lastPruStatus)
					{
//						printf("Idle\n");
						lastPruStatus = pruStatus;
					}
					break;
				}
				case eRESET:
				{
					if (pruStatus != lastPruStatus)
					{
						printf("\n--- Reset %d: ", resetCnt);
						spID1 = 0xFF;
						spID2 = 0xFF;
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
//						printf("Enabled\n");
						lastPruStatus = pruStatus;
					}
					break;
				}
				case eRCVDPACK:
				{	// PRU has a packet, command or data
//					printf("Received packet\n");

					destID = *(rcvdPacketPtr + 7);							// with msb = 1
					type   = *(rcvdPacketPtr + 9);							// 0x80=Cmd, 0x81=Status, 0x82=Data
					cmdNum = *(rcvdPacketPtr + 15);

					destDevice = destID - spID1;							// theImages[0] or [1]
					if (imageToggle)
					{
						// DEBUG - toggle devices
						if (destDevice == 0)
							destDevice = 1;
						else
							destDevice = 0;
					}

//					printPacket(destID);
					if ((destID == spID1) || (destID == spID2))
					{
						if (type == 0x82)										// data packet
						{
							// blkNum was set previously by WriteBlock command so 
							//  decode to tempBuffer[] and check status
							if (decodeDataPacket() == 0)						// checksum ok
							{
//								printf("[0x%X] CS GOOD\n", destID);
								for (i=0; i<512; i++)
									theImages[destDevice][blkNum][i] = tempBuffer[i];

								if (destDevice == 0)
									diskImage1Changed = 1;
								else
									diskImage2Changed = 1;

								encodeStdStatusReplyPacket(destID, 0x00);		// 0x00 = no error
							}
							else
							{
								printf("*** [0x%X] Bad Write datablock status\n", destID);
								encodeStdStatusReplyPacket(destID, 0x06);		// 0x06 = bus error
								
								debugDataPacket();
							}

							// DEBUG
							if (blkNum > 100)
								printf("\tDP %d\n", blkNum);
						}
						else														// command packet
						{
							checkCmdChecksum();
							switch (cmdNum)
							{
								case eSTATUS:
								case eEXTSTATUS:
								{
									statCode = *(rcvdPacketPtr + 20) & 0x7F;
//									printf("[0x%X] Std Status: %d\n", destID, statCode);

									if (statCode == 0x00)
										encodeStdStatusReplyPacket(destID, 0x00);	// 0x00 = no error

									else if (statCode == 0x03)
										encodeStdDibStatusReplyPacket(destID, 0x00);	// 0x00 = no error

									else
									{
										printf("*** [0x%X] Unsupported statCode: 0x%X\n", destID, statCode);
										encodeStdStatusReplyPacket(destID, 0x21);	// 0x21 = not supported
									}
									break;
								}

								case eREADBLK:
								case eEXTREADBLK:
								{
									if (destID == spID1)
										readCnt1++;
									else
										readCnt2++;
									// Compute block number
									msbs = *(rcvdPacketPtr + 17);
									if (cmdNum == eREADBLK)
									{
										blkNumLow = (*(rcvdPacketPtr + 20) & 0x7F) | ((msbs << 3) & 0x80);
										blkNumMid = (*(rcvdPacketPtr + 21) & 0x7F) | ((msbs << 4) & 0x80);
										blkNumHi  = (*(rcvdPacketPtr + 22) & 0x7F) | ((msbs << 5) & 0x80);
										blkNum = blkNumLow + 256*blkNumMid + 65536*blkNumHi;
//										printf("[0x%X] RB: %d\n", destID, blkNum);
									}
									else
									{
										blkNumLow = (*(rcvdPacketPtr + 19) & 0x7F) | ((msbs << 2) & 0x80);
										blkNumMid = (*(rcvdPacketPtr + 20) & 0x7F) | ((msbs << 3) & 0x80);
										blkNumHi  = (*(rcvdPacketPtr + 21) & 0x7F) | ((msbs << 4) & 0x80);
										blkNum = blkNumLow + 256*blkNumMid + 65536*blkNumHi;
										printf("[0x%X] ExtRB: %d\n", destID, blkNum);
									}

									if (blkNum < NUM_BLOCKS)
										encodeDataPacket(destID, 0x00, destDevice, blkNum);	// 0x00 = no error
									else
									{
										printf("*** [0x%X] Bad Read BlkNum: %d\n", destID, blkNum);
										printPacket(destID);
										encodeStdStatusReplyPacket(destID, 0x06);		// 0x06 = bus error
									}
									break;
								}

								case eWRITEBLK:
								case eEXTWRITEBLK:
								{
									if (destID == spID1)
										writeCnt1++;
									else
										writeCnt2++;
									// Compute block number
									msbs = *(rcvdPacketPtr + 17);
									if (cmdNum == 0x82)
									{
										blkNumLow = (*(rcvdPacketPtr + 20) & 0x7F) | ((msbs << 3) & 0x80);
										blkNumMid = (*(rcvdPacketPtr + 21) & 0x7F) | ((msbs << 4) & 0x80);
										blkNumHi  = (*(rcvdPacketPtr + 22) & 0x7F) | ((msbs << 5) & 0x80);
										blkNum = blkNumLow + 256*blkNumMid + 65536*blkNumHi;
//										printf("[0x%X] WB: %d\n", destID, blkNum);
									}
									else
									{
										blkNumLow = (*(rcvdPacketPtr + 19) & 0x7F) | ((msbs << 2) & 0x80);
										blkNumMid = (*(rcvdPacketPtr + 20) & 0x7F) | ((msbs << 3) & 0x80);
										blkNumHi  = (*(rcvdPacketPtr + 21) & 0x7F) | ((msbs << 4) & 0x80);
										blkNum = blkNumLow + 256*blkNumMid + 65536*blkNumHi;
										printf("[0x%X] ExtWB: %d\n", destID, blkNum);
									}

									// Check blkNum here and send handshake or error status
									if (blkNum < NUM_BLOCKS)				// a gross check
									{
										encodeHandshakeReplyPacket();
									}
									else
									{
										printf("*** [0x%X] Bad Write BlkNum: %d\n", destID, blkNum);
										encodeStdStatusReplyPacket(destID, 0x06);		// 0x06 = bus error
									}

									// DEBUG
									if (blkNum > 100)
										printf("WB %d\n ", blkNum);
									break;
								}

								case eCONTROL:
								{
									statCode = *(rcvdPacketPtr + 11);
									printf("[0x%X] Control: 0x%X\n", destID, statCode);
									encodeStdStatusReplyPacket(destID, 0x21);
									break;
								}

								default:
								{
									printf("*** [0x%X] Unexpected cmdNum= 0x%X\n", destID, cmdNum);
									encodeStdStatusReplyPacket(destID, 0x21);		// 0x21 = not supported
//									printPacket(destID);
								}
							}
						}
					}
					else
					{
						// This should never happen
						printf("*** destID [0x%X] != spID1 [0x%X] or spID2 [0x%X]\n", destID, spID1, SPID2);
					}

					prussdrv_pru_send_event(ARM_PRU0_INTERRUPT);				// allow PRU to continue
					usleep(1);
					prussdrv_pru_clear_event(PRU_EVTOUT0, ARM_PRU0_INTERRUPT);	// PRU halts after next received packet
					break;
				}
				case eSENDING:
				{
					if (pruStatus != lastPruStatus)
					{
//						printf("Sending...\n");
						lastPruStatus = eSENDING;
					}
					break;
				}
				case eWRITING:
				{
					if (pruStatus != lastPruStatus)
					{
//						printf("Writing...\n");
						lastPruStatus = eWRITING;
					}
					break;
				}
				default:
					printf("*** Unexpected pruStatus: %d\n", pruStatus);
			}
		}

//RMH		loopCnt++;
		if (loopCnt == 100000)
		{
			loopCnt = 0;
			printf("\treadCnt= %d\t%d\twriteCnt= %d\t%d\n", readCnt1, readCnt2, writeCnt1, writeCnt2);
		}
	} while (running);

	// Ask about saving changed image(s)
	if (diskImage1Changed)
	{
		printf("\n - Save %s modifications? Enter name or <CR>:  ", diskImages[0]);
		fgets(saveName, 64, stdin);
		length = strlen(saveName);
		if (length > 5)
		{
			strncpy(imageName, saveName, length-1);
			saveDiskImage(0, imageName);
		}
		else if (length == 2)
		{
			printf("NEED TO IMPLEMENT\n");
		}
	}
	if (diskImage2Changed)
	{
		printf(" - Save %s modifications? Enter name or <CR>: ", diskImages[1]);
		fgets(saveName, 64, stdin);
		length = strlen(saveName);
		if (length > 5)
		{
			strncpy(imageName, saveName, length-1);
			saveDiskImage(1, imageName);
		}
		else if (length == 2)
		{
			printf("NEED TO IMPLEMENT\n");
		}
	}

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
	// ctrl-z
	// Toggle image order
	if (imageToggle)
	{
		imageToggle = 0;
		printf(" --- Resetting images to startup configuration\n");
	}
	else
	{
		imageToggle = 1;
		printf(" --- Flipping images\n");
	}
}

//____________________
void loadDiskImages(const char *image1, const char *image2)
{
	//	Load disk images into theImages
	char imagePath[128];
	unsigned int i, j, totalBlksLoaded;
	FILE *fd;
	size_t pathLen, numBlksRead;

	for (i=0; i<NUM_BLOCKS; i++)							// zero theImages[][][]
	{
		for (j=0; j<512; j++)
		{
			theImages[0][i][j] = 0;
			theImages[1][i][j] = 0;
		}
	}

	// Image 1
	sprintf(imagePath, "/root/DiskImages/%s", image1);		// create image path
	printf("--- Image 1: %s ---\n", image1);
	fd = fopen(imagePath, "rb");
	if (!fd)
	{
		printf("*** Problem opening disk image 1\n");
		return;
	}

	// Determine if we are dealing with a *.2mg or a *.po file
	pathLen = strlen(imagePath);
	if (imagePath[pathLen-1] == 'g')
	{
		fread(theImages[0][0], 64, 1, fd);					// to get past 2mg prefix
//		printf("(Detected a *.2mg file)\n");
	}

	totalBlksLoaded = 0;
	for (i=0; i<NUM_BLOCKS; i++)							// read to end of file
	{
		numBlksRead = fread(theImages[0][i], 512, 1, fd);	// block size is 512 bytes
		if (numBlksRead != 1)
			break;

		totalBlksLoaded++;
	}
	fclose(fd);
//	printf("(Total blocks loaded= %d)\n", totalBlksLoaded);

	// Image 2
	sprintf(imagePath, "/root/DiskImages/%s", image2);		// create image path
	printf("--- Image 2: %s ---\n", image2);
	fd = fopen(imagePath, "rb");
	if (!fd)
	{
		printf("*** Problem opening disk image 2\n");
		return;
	}

	// Determine if we are dealing with a *.2mg or a *.po file
	pathLen = strlen(imagePath);
	if (imagePath[pathLen-1] == 'g')
	{
		fread(theImages[1][0], 64, 1, fd);					// to get past 2mg prefix
//		printf("(Detected a *.2mg file)\n");
	}

	totalBlksLoaded = 0;
	for (i=0; i<NUM_BLOCKS; i++)							// read to end of file
	{
		numBlksRead = fread(theImages[1][i], 512, 1, fd);	// block size is 512 bytes
		if (numBlksRead != 1)
			break;
		
		totalBlksLoaded++;
	}
	fclose(fd);
//	printf("(Total blocks loaded= %d)\n", totalBlksLoaded);
}

//____________________
void saveDiskImage(unsigned char image, const char *fileName)
{
	// Always save in .po format to /Saved directory
	char imagePath[128];
	unsigned int i, totalBlksSaved;
	FILE *fd;

	sprintf(imagePath, "/root/DiskImages/Saved/%s", fileName);	// create image path

	printf(" --- Saving: %s ---\n", fileName);
	fd = fopen(imagePath, "wb");
	if (!fd)
	{
		printf("*** Problem opening file for save\n");
		return;
	}

	totalBlksSaved = 0;
	for (i=0; i<NUM_BLOCKS; i++)
	{
		fwrite(theImages[image][i], 512, 1, fd);
//		fwrite(theImages[image][i], 1, 512, fd);
		totalBlksSaved++;
	}
	fclose(fd);
//	printf("(Total blocks saved= %d)\n", totalBlksSaved);
}

//____________________
void encodeStdStatusReplyPacket(unsigned char srcID, unsigned char dataStat)
{
	// Reply to init and standard status commands with Statcode = 0x00
	// Assumes srcID has MSB set
	// spID1 is big HD, spID2 is 800k HD
	unsigned char checksum = 0;
	unsigned int i;

	*(respPacketPtr     ) = 0xFF;				// sync bytes
	*(respPacketPtr +  1) = 0x3F;
	*(respPacketPtr +  2) = 0xCF;
	*(respPacketPtr +  3) = 0xF3;
	*(respPacketPtr +  4) = 0xFC;
	*(respPacketPtr +  5) = 0xFF;
	
	*(respPacketPtr +  6) = 0xC3;				// packet begin
	*(respPacketPtr +  7) = 0x80;				// destination
	*(respPacketPtr +  8) = srcID;				// source
	*(respPacketPtr +  9) = 0x81;				// packet Type: 1 = status
	*(respPacketPtr + 10) = 0x80;				// aux type: 0 = standard packet
	*(respPacketPtr + 11) = dataStat | 0x80;	// data status
	*(respPacketPtr + 12) = 0x84;				// odd byte count: 4
	*(respPacketPtr + 13) = 0x80;				// groups-of-7 count: 0

	for (i=7; i<14; i++)
		checksum ^= *(respPacketPtr+i);
	
	if (srcID == spID1)
	{											// 32 MB  - 0x010000 (or 0x00FFFF ???)
		*(respPacketPtr + 14) = 0xC0;			// odd MSBs: 100 0000
		*(respPacketPtr + 15) = 0xF0;			// device status: 1111 1000, read/write
		checksum ^= 0xF0;
		*(respPacketPtr + 16) = 0x80;			// block size low byte: 0x00
		*(respPacketPtr + 17) = 0x80;			// block size mid byte: 0x00
		*(respPacketPtr + 18) = 0x81;			// block size high byte: 0x01
		checksum ^= 0x01;
	}
	else
	{											// 800kB  - 0x000640
		*(respPacketPtr + 14) = 0xC0;			// odd MSBs: 100 0000
		*(respPacketPtr + 15) = 0xF0;			// device status: 1111 0000, read/write
		checksum ^= 0xF0;
		*(respPacketPtr + 16) = 0xC0;			// block size low byte: 0x40
		checksum ^= 0x40;
		*(respPacketPtr + 17) = 0x86;			// block size mid byte: 0x06
		checksum ^= 0x06;
		*(respPacketPtr + 18) = 0x80;			// block size high byte: 0x00
	}

	*(respPacketPtr + 19) =  checksum	    | 0xAA;	// 1 C6 1 C4 1 C2 1 C0
	*(respPacketPtr + 20) = (checksum >> 1) | 0xAA;	// 1 C7 1 C5 1 C3 1 C1
	*(respPacketPtr + 21) = 0xC8;					// PEND
	*(respPacketPtr + 22) = 0x00;					// end of packet marker in memory
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

	*(initResp1Ptr + 21) = 0xC8;					// PEND
	*(initResp1Ptr + 22) = 0x00;					// end of packet marker in memory

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

	*(initResp2Ptr + 21) = 0xC8;					// PEND
	*(initResp2Ptr + 22) = 0x00;					// end of packet marker in memory
}

//____________________
void encodeStdDibStatusReplyPacket(unsigned char srcID, unsigned char dataStat)
{
	// Reply to standard status commands with Statcode = 0x03
	// Assumes srcID has MSB set
	// spID1 is big HD, spID2 is 800k HD
	unsigned char checksum = 0;
	unsigned int i;

	*(respPacketPtr     ) = 0xFF;				// sync bytes
	*(respPacketPtr +  1) = 0x3F;
	*(respPacketPtr +  2) = 0xCF;
	*(respPacketPtr +  3) = 0xF3;
	*(respPacketPtr +  4) = 0xFC;
	*(respPacketPtr +  5) = 0xFF;

	*(respPacketPtr +  6) = 0xC3;				// packet begin
	*(respPacketPtr +  7) = 0x80;				// destination
	*(respPacketPtr +  8) = srcID;				// source
	*(respPacketPtr +  9) = 0x81;				// packet Type: 1 = status
	*(respPacketPtr + 10) = 0x80;				// aux type: 0 = standard packet
	*(respPacketPtr + 11) = dataStat | 0x80;	// data status
	*(respPacketPtr + 12) = 0x84;				// odd byte count: 4
	*(respPacketPtr + 13) = 0x83;				// groups-of-7 count: 3

	for (i=7; i<14; i++)
		checksum ^= *(respPacketPtr+i);

	if (srcID == spID1)
	{
													// 32 MB  - 0x010000 (or 0x00FFFF ???)
		*(respPacketPtr + 14) = 0xC0;				// odd MSBs: 100 0000
		*(respPacketPtr + 15) = 0xF0;				// device status: 1110 1000, read/write
		checksum ^= 0xF0;
		*(respPacketPtr + 16) = 0x80;				// block size low byte: 0x00
		*(respPacketPtr + 17) = 0x80;				// block size mid byte: 0x00
		*(respPacketPtr + 18) = 0x81;				// block size high byte: 0x01
		checksum ^= 0x01;

		*(respPacketPtr + 19) = 0x80;				// GRP1 MSBs
		*(respPacketPtr + 20) = 0x8A;				// ID string length, 10 chars
		checksum ^= 0x0A;
		*(respPacketPtr + 21) = 'B' | 0x80;			// ID string, 16 chars total
		checksum ^= 'B';
		*(respPacketPtr + 22) = 'e' | 0x80;
		checksum ^= 'e';
		*(respPacketPtr + 23) = 'a' | 0x80;
		checksum ^= 'a';
		*(respPacketPtr + 24) = 'g' | 0x80;
		checksum ^= 'g';
		*(respPacketPtr + 25) = 'l' | 0x80;
		checksum ^= 'l';
		*(respPacketPtr + 26) = 'e' | 0x80;
		checksum ^= 'e';

		*(respPacketPtr + 27) = 0x80;				// GRP2 MSBs
		*(respPacketPtr + 28) = 'B' | 0x80;
		checksum ^= 'B';
		*(respPacketPtr + 29) = 'o' | 0x80;
		checksum ^= 'o';
		*(respPacketPtr + 30) = 'n' | 0x80;
		checksum ^= 'n';
		*(respPacketPtr + 31) = 'e' | 0x80;
		checksum ^= 'e';
		*(respPacketPtr + 32) = ' ' | 0x80;
		checksum ^= 0x20;
		*(respPacketPtr + 33) = ' ' | 0x80;
		checksum ^= 0x20;
		*(respPacketPtr + 34) = ' ' | 0x80;
		checksum ^= 0x20;

		// Pretending to be a non-removable hard disk
		*(respPacketPtr + 35) = 0x80;				// GRP3 MSBs: 000 0000
		*(respPacketPtr + 36) = ' ' | 0x80;
		checksum ^= 0x20;
		*(respPacketPtr + 37) = ' ' | 0x80;
		checksum ^= 0x20;
		*(respPacketPtr + 38) = ' ' | 0x80;
		checksum ^= 0x20;

		*(respPacketPtr + 39) = 0x82;				// device type: 0x02 = Hard disk
		checksum ^= 0x02;
		*(respPacketPtr + 40) = 0xA0;				// device subtype: 0x20 = not removable
		checksum ^= 0x20;

		*(respPacketPtr + 41) = 0x82;				// firmware version, 2 bytes
		checksum ^= 0x02;
		*(respPacketPtr + 42) = 0x80;
	}
	else
	{
													// 800kB  - 0x000640
		*(respPacketPtr + 14) = 0xC0;				// odd MSBs: 100 0000
		*(respPacketPtr + 15) = 0xF0;				// device status: 1111 0000, read/write
		checksum ^= 0xF0;
		*(respPacketPtr + 16) = 0xC0;				// block size low byte: 0x40
		checksum ^=0x40;
		*(respPacketPtr + 17) = 0x86;				// block size mid byte: 0x06
		checksum ^=0x06;
		*(respPacketPtr + 18) = 0x80;				// block size high byte: 0x00

		*(respPacketPtr + 19) = 0x80;				// GRP1 MSBs
		*(respPacketPtr + 20) = 0x83;				// ID string length, 3 chars
		checksum ^= 0x03;
		*(respPacketPtr + 21) = 'B' | 0x80;			// ID string, 16 chars total
		checksum ^= 'B';
		*(respPacketPtr + 22) = 'B' | 0x80;
		checksum ^= 'B';
		*(respPacketPtr + 23) = 'B' | 0x80;
		checksum ^= 'B';
		*(respPacketPtr + 24) = ' ' | 0x80;
		checksum ^= 0x20;
		*(respPacketPtr + 25) = ' ' | 0x80;
		checksum ^= 0x20;
		*(respPacketPtr + 26) = ' ' | 0x80;
		checksum ^= 0x20;

		*(respPacketPtr + 27) = 0x80;				// GRP2 MSBs
		*(respPacketPtr + 28) = ' ' | 0x80;
		checksum ^= 0x20;
		*(respPacketPtr + 29) = ' ' | 0x80;
		checksum ^= 0x20;
		*(respPacketPtr + 30) = ' ' | 0x80;
		checksum ^= 0x20;
		*(respPacketPtr + 31) = ' ' | 0x80;
		checksum ^= 0x20;
		*(respPacketPtr + 32) = ' ' | 0x80;
		checksum ^= 0x20;
		*(respPacketPtr + 33) = ' ' | 0x80;
		checksum ^= 0x20;
		*(respPacketPtr + 34) = ' ' | 0x80;
		checksum ^= 0x20;

		// Pretending to be an 800k RAM disk
		*(respPacketPtr + 35) = 0x80;				// GRP3 MSBs: 000 0000
		*(respPacketPtr + 36) = ' ' | 0x80;
		checksum ^= 0x20;
		*(respPacketPtr + 37) = ' ' | 0x80;
		checksum ^= 0x20;
		*(respPacketPtr + 38) = ' ' | 0x80;
		checksum ^= 0x20;

		*(respPacketPtr + 39) = 0x82;				// device type: 0x02 = Hard disk
		checksum ^= 0x02;
		*(respPacketPtr + 40) = 0xA0;				// device subtype: 0x20 = not removable
		checksum ^= 0x20;

		*(respPacketPtr + 41) = 0x80;				// firmware version, 2 bytes
		checksum ^= 0x00;
		*(respPacketPtr + 42) = 0x80;
	}	
	
	*(respPacketPtr + 43) =  checksum       | 0xAA;	// 1 C6 1 C4 1 C2 1 C0
	*(respPacketPtr + 44) = (checksum >> 1) | 0xAA;	// 1 C7 1 C5 1 C3 1 C1
	*(respPacketPtr + 45) = 0xC8;					// PEND
	*(respPacketPtr + 46) = 0x00;					// End of packet marker in memory
}

//____________________
void encodeHandshakeReplyPacket(void)
{
	// Creates zero-length message; used when all we want to do is handshake with A2
	*(respPacketPtr+6) = 0x00;	
}

//____________________
void encodeDataPacket(unsigned char srcID, unsigned char dataStat, unsigned char device, unsigned int block)
{
	// Creates 512 byte (1 block) data packet for reply to read block command
	// Assumes srcID has MSB set
	unsigned int i, groupByte, groupCount;
	unsigned char checksum = 0, groupMsb;

	*(respPacketPtr     ) = 0xFF;				// sync bytes
	*(respPacketPtr +  1) = 0x3F;
	*(respPacketPtr +  2) = 0xCF;
	*(respPacketPtr +  3) = 0xF3;
	*(respPacketPtr +  4) = 0xFC;
	*(respPacketPtr +  5) = 0xFF;

	*(respPacketPtr +  6) = 0xC3;				// packet begin
	*(respPacketPtr +  7) = 0x80;				// destination
	*(respPacketPtr +  8) = srcID;				// source
	*(respPacketPtr +  9) = 0x82;				// type: 2 = data
	*(respPacketPtr + 10) = 0x80;				// aux type: 0 = standard packet
	*(respPacketPtr + 11) = dataStat | 0x80;	// data status
	*(respPacketPtr + 12) = 0x81;				// odd byte count: 1
	*(respPacketPtr + 13) = 0xC9;				// groups-of-7 count: 73 (for 512-byte packet)

	// Total number of packet data bytes for one block is 584
	// Odd byte
	*(respPacketPtr + 14) = ((theImages[device][block][0] >> 1) & 0x40) | 0x80;
	*(respPacketPtr + 15) =   theImages[device][block][0]			    | 0x80;

	// Groups of 7
	for (groupCount=0; groupCount<73; groupCount++)
	{
		groupMsb = 0;
		for (groupByte=0; groupByte<7; groupByte++)
			groupMsb = groupMsb | ((theImages[device][block][1+(groupCount*7)+groupByte] >> (groupByte+1)) & (0x80 >> (groupByte+1)));

		*(respPacketPtr+16+(groupCount*8)) = groupMsb | 0x80;	// set msb to one

		// Now add group data bytes bits 6-0
		for (groupByte=0; groupByte<7; groupByte++)
			*(respPacketPtr+17+(groupCount*8) + groupByte) = theImages[device][block][1+(groupCount*7) + groupByte] | 0x80;
	}

	// Checksum
	for (i=0; i<512; i++)								// xor data bytes
		checksum = checksum ^ theImages[device][block][i];

	for (i=7; i<14; i++)
		checksum = checksum ^ *(respPacketPtr+i);		// xor packet header bytes

    *(respPacketPtr + 600) =  checksum		 | 0xAA;	// 1 c6 1 c4 1 c2 1 c0
    *(respPacketPtr + 601) = (checksum >> 1) | 0xAA;	// 1 c7 1 c5 1 c3 1 c1
	*(respPacketPtr + 602) = 0xC8;						// PEND
	*(respPacketPtr + 603) = 0x00;						// end of packet marker in memory
}

//____________________
char decodeDataPacket(void)
{
	// Decode 512-byte data packet (1 block) from A2 into tempBuffer[]
	// Returns 0 if checksum good, 6 otherwise
	unsigned int i, groupByte, groupCount;
	unsigned char checksum = 0, bit0to6, bit7, oddbits, evenbits;

	// Add oddbyte, 1 in a 512 data packet
	tempBuffer[0] = ((*(rcvdPacketPtr+14) << 1) & 0x80) | (*(rcvdPacketPtr+15) & 0x7F);

	// 73 grps of 7 in a 512 byte packet
	for (groupCount=0; groupCount<73; groupCount++)
	{
		for (groupByte=0; groupByte<7; groupByte++)
		{
			bit7    = (*(rcvdPacketPtr+16+8*groupCount) << (groupByte+1)) & 0x80;
			bit0to6 = (*(rcvdPacketPtr+17+8*groupCount + groupByte))      & 0x7F;
			tempBuffer[1 + 7*groupCount + groupByte] = (bit7 | bit0to6);
		}
	}

	// Verify checksum
	for (i=0; i<512; i++)							// xor all data bytes
		checksum ^= tempBuffer[i];

	for (i=7; i<14; i++)
        checksum ^= *(rcvdPacketPtr+i);				// xor packet header bytes

	evenbits =  *(rcvdPacketPtr+600) & 0x55;
	oddbits  = (*(rcvdPacketPtr+601) & 0x55) << 1;
	if (checksum == (oddbits | evenbits))
		return 0;									// checksum good
	else
		return 6;									// SmartPort bus error
}

//____________________
char checkCmdChecksum(void)
{
	// Returns 0 if checksum ok
	unsigned char msbs, packetCS, checksum = 0;

	checksum ^=  *(rcvdPacketPtr +  7);
	checksum ^=  *(rcvdPacketPtr +  8);
	checksum ^=  *(rcvdPacketPtr +  9);
	checksum ^=  *(rcvdPacketPtr + 10);
	checksum ^=  *(rcvdPacketPtr + 11);
	checksum ^=  *(rcvdPacketPtr + 12);
	checksum ^=  *(rcvdPacketPtr + 13);

	msbs      =  *(rcvdPacketPtr + 14);
	checksum ^= (*(rcvdPacketPtr + 15) & 0x7F) | ((msbs << 1) & 0x80);
	checksum ^= (*(rcvdPacketPtr + 16) & 0x7F) | ((msbs << 2) & 0x80);

	msbs      = *(rcvdPacketPtr+ 17);
	checksum ^= (*(rcvdPacketPtr + 18) & 0x7F) | ((msbs << 1) & 0x80);
	checksum ^= (*(rcvdPacketPtr + 19) & 0x7F) | ((msbs << 2) & 0x80);
	checksum ^= (*(rcvdPacketPtr + 20) & 0x7F) | ((msbs << 3) & 0x80);
	checksum ^= (*(rcvdPacketPtr + 21) & 0x7F) | ((msbs << 4) & 0x80);
	checksum ^= (*(rcvdPacketPtr + 22) & 0x7F) | ((msbs << 5) & 0x80);
	checksum ^= (*(rcvdPacketPtr + 23) & 0x7F) | ((msbs << 6) & 0x80);
	checksum ^= (*(rcvdPacketPtr + 24) & 0x7F) | ((msbs << 7) & 0x80);

	packetCS = *(rcvdPacketPtr+25) & ((*(rcvdPacketPtr+26) << 1) | 0x01);

//	printf("checksum: computed= 0x%X\tpacket= 0x%X\n\n", checksum, packetCS);
	if (checksum == packetCS)
	{
//		printf("GOOD checksum\n");
		return 0;
	}
	else
	{
		printf("*** BAD checksum\n");
		return 1;
	}
}

//____________________
void printPacket(unsigned char id)
{
	printf("================\n");
	printf("0x%X [0xFF]\n",   *rcvdPacketPtr);
	printf("0x%X [0x3F]\n",   *(rcvdPacketPtr+1));
	printf("0x%X [0xCF]\n",   *(rcvdPacketPtr+2));
	printf("0x%X [0xF3]\n",   *(rcvdPacketPtr+3));
	printf("0x%X [0xFC]\n",   *(rcvdPacketPtr+4));
	printf("0x%X [0xFF]\n\n", *(rcvdPacketPtr+5));

	// PBEGIN
	printf("0x%X [0xC3]\n\n", *(rcvdPacketPtr+6));

	// Dest - Counts
	printf("0x%X [0x%X]\n",   *(rcvdPacketPtr+ 7), id);
	printf("0x%X [0x80]\n",   *(rcvdPacketPtr+ 8));
	printf("0x%X [0x80]\n",   *(rcvdPacketPtr+ 9));
	printf("0x%X [0x80]\n",   *(rcvdPacketPtr+10));
	printf("0x%X [0x80]\n",   *(rcvdPacketPtr+11));
	printf("0x%X [0x82]\n",   *(rcvdPacketPtr+12));
	printf("0x%X [0x81]\n\n", *(rcvdPacketPtr+13));

	// Odd bytes
	printf("0x%X\n",   *(rcvdPacketPtr+14));
	printf("0x%X\n",   *(rcvdPacketPtr+15));
	printf("0x%X\n\n", *(rcvdPacketPtr+16));

	// Group-of-7
	printf("0x%X\n",   *(rcvdPacketPtr+17));
	printf("0x%X\n",   *(rcvdPacketPtr+18));
	printf("0x%X\n",   *(rcvdPacketPtr+19));
	printf("0x%X\n",   *(rcvdPacketPtr+20));
	printf("0x%X\n",   *(rcvdPacketPtr+21));
	printf("0x%X\n",   *(rcvdPacketPtr+22));
	printf("0x%X\n",   *(rcvdPacketPtr+23));
	printf("0x%X\n\n", *(rcvdPacketPtr+24));

	// checksums
	printf("0x%X\n",   *(rcvdPacketPtr+25));
	printf("0x%X\n\n", *(rcvdPacketPtr+26));

	// PEND
	printf("0x%X [0xC8]\n", *(rcvdPacketPtr+27));

	printf("================\n");

	// Checksum check
	checkCmdChecksum();
}

//____________________
void debugDataPacket(void)
{
	// Search for first (known) bad data value

	unsigned int i;
	unsigned char data;

	printf("--- debugDataPacket ---\n");
	for (i=6; i<603; i++)
	{
		data = *(rcvdPacketPtr + i);
		if (data < 0x80)
		{
			printf("%d: 0x%X\n", i, data);
			break;
		}
	}
	if (i < 603)
		printf("0x%X 0x%X [%d]-0x%X 0x%X\n", *(rcvdPacketPtr+i-2), *(rcvdPacketPtr+i-1), i,
											 *(rcvdPacketPtr+i), *(rcvdPacketPtr+i+1));
	else
		printf(" (No bad bytes detected)\n\n");
}