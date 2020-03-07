--- SmartPortInterfaceSharedMem Notes

How to run:
0) Before applying power
	SD card not inserted
	Enable toggle switch in center position
	A2 power off

1) Apply power to system
	Turn on power strip

2) source setup.sh

3) Set enable toggle switch

4) Insert SD card and mount
	mount -v /dev/mmcblk0p1 /root/DiskImages, or
	mount -t /dev/mmcblk0p1 /root/DiskImages

5) make

6) gcc SmartPortController.c -o Controller
   gcc SmartPortControllerTest.c -o Controller

7) ./Controller

8) Turn on A2	



(Ins and Outs relative to BBB)
Inputs:
WDATA	P8_45
P0/REQ	P8_46
P1		P8_43
P2		P8_44
P3		P8_41

Outputs:
OUTEN	P8_42	// 0=LS367 enabled, RDAT active
RDAT	P8_39
ACK		P8_40	// 1=ready to send/receive
LED		P8_27
TEST	P8_29
