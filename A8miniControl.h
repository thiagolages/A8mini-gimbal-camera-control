#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdbool.h>

#define RECV_BUUF_SIZE 64
#define SERVER_PORT 37260           // Gimbal Camera (Server) Port
#define SERVER_IP "192.168.144.25"  // Gimbal Camera (Server) IP Addresses

#define MIN_CMD_SIZE 10
#define MAX_CMD_SIZE 13

/*************************************************/
#define NUM_COMMANDS                27 // update this if more commands are added !
#define MAX_SIZE_CMD_DESCRIPTION    40
/*************************************************/

int setup();
void cleanup();
void printSDKFormat();
void printCommandDescription(int);
void printCommand(unsigned char[], short);
int cmdstrcpy(unsigned char[], unsigned char[]); // custom strcpy()
uint16_t CRC16_cal(uint8_t*, uint32_t, uint16_t);
int findCmdIdx(int *, const unsigned char *, size_t , const unsigned char *, size_t);

bool debug;

/* reference from drone.py (temporary !)
self.GIMBAL_AUTO_CENTER             = 0
self.GIMBAL_PITCH_POSITIVE          = 1
self.GIMBAL_PITCH_NEGATIVE          = 2
self.GIMBAL_YAW_POSITIVE            = 3
self.GIMBAL_YAW_NEGATIVE            = 4
self.GIMBAL_STOP                    = 5
*/

unsigned char commandDescriptions[NUM_COMMANDS][MAX_SIZE_CMD_DESCRIPTION] = {
    "Auto Centering\0",
    "Rotate Up\0",
    "Rotate Down\0",
    "Rotate Right\0",
    "Rotate Left\0",
    "Stop Rotation\0",
    "Zoom +1\0",
    "Zoom -1\0",
    "Absolute Zoom (4.5x)\0",
    "Acquire the Max Zoom Value\0",
    "Manual Focus +1\0",
    "Manual Focus -1\0",
    "Take Pictures\0",
    "Record Video\0",
    "Rotate 100 100\0",
    "Gimbal Status Information\0",
    "Auto Focus\0",
    "Acquire Hardware ID\0",
    "Acquire Firmware Version\0",
    "Lock Mode\0",
    "Follow Mode\0",
    "FPV Mode\0",
    "Acquire Attitude Data\0",
    "Set Video Output as HDMI\0",
    "Set Video Output as CVBS\0",
    "Turn Off both CVBS and HDMI Output\0",
    "Read Range from Laser Rangefinder\0",
};

// fill in the hex values of the commands
unsigned char **cmds;

short cmds_sizes[] = {
    11,// 0  - Auto Centering command size
    12,// 1  - Rotate Up
    12,// 2  - Rotate Down
    12,// 3  - Rotate Right
    12,// 4  - Rotate Left
    12,// 5  - Stop Rotation
    11,// 6  - Zoom +1 command size
    11,// 7  - Zoom -1 command size
    12,// 8  - Absolute Zoom (4.5x) command size
    10,// 9  - Acquire the Max Zoom Value command size
    11,// 10 - Manual Focus +1 command size
    11,// 11 - Manual Focus -1 command size
    11,// 12 - Take Pictures command size
    11,// 13 - Record Video command size
    12,// 14 - Rotate 100 100 command size
    10,// 15 - Gimbal Status Information command size
    11,// 16 - Auto Focus command size
    10,// 17 - Acquire Hardware ID command size
    10,// 18 - Acquire Firmware Version command size
    11,// 19 - Lock Mode command size
    11,// 20 - Follow Mode command size
    11,// 21 - FPV Mode command size
    10,// 22 - Acquire Attitude Data command size
    11,// 23 - Set Video Output as HDMI (Only available on A8 mini, restart to take effect) command size
    11,// 24 - Set Video Output as CVBS (Only available on A8 mini, restart to take effect) command size
    11,// 25 - Turn Off both CVBS and HDMI Output (Only available on A8 mini, restart to take effect) command size
    10 // 26 - Read Range from Laser Rangefinder(Low byte in the front, high byte in the back, available on ZT30) command size
};

const uint16_t crc16_tab[256]= {0x0,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0xa50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0xc60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0xe70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0xa1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x2b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x8e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0xaf1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0xcc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0xed1,0x1ef0
};
