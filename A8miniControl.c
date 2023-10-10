#include "A8miniControl.h"

int main(int argc, char *argv[]){

    int sockfd;
    int ret, recv_len;
    struct sockaddr_in send_addr, recv_addr;
    unsigned char recv_buff[RECV_BUUF_SIZE] = {0};
    int cmd_len = 0, cmd_idx = -1;

    if (argc <2){
        do{
            printf("Insert the command index (0-21): ");
            scanf("%d",&cmd_idx);
        }while (cmd_idx < 0 || cmd_idx > NUM_COMMANDS-1);
        debug = true; // inserting option while running by hand, enable debug mode
    }else{
        cmd_idx = atoi(argv[1]);
        debug = false; // inserting option from cmd line, don't enable debug mode
        if (cmd_idx < 0 || cmd_idx > 21){
           printf("Command index should be between 0 and 21.\n");
           return -1;
        }
    }

    if (debug){
        printSDKFormat();
    }

    if (setup() < 0){return -1;}

    cmd_len = cmds_sizes[cmd_idx];

    unsigned char send_buff[cmd_len];

    memcpy(send_buff, cmds[cmd_idx], cmd_len); // copy final command to buffer
    
    if (debug){
        printCommand(send_buff, cmd_len);
        printCommandDescription(cmd_idx);
    }
    
    /* Create UDP Socket

    AF_INET: ipv4 addresses
    SOCK_DGRAM: UDP protocol
    0: automatically choose the default protocol of the relevant type

    */
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket");
        exit(1);
    }
    /* Set IP addresses and port number of gimbal camera

    sin_family: ipv4 addresses
    sin_addr.s_addr: IP addresses of gimbal camera
    sin_port: port of gimbal camera
    */
    memset(&send_addr, 0, sizeof(send_addr));
    send_addr.sin_family = AF_INET;
    send_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    send_addr.sin_port = htons(SERVER_PORT);
    
    /* Send frame data

    sockfd: descriptor of socket
    send_buff: head address in RAM of the sending data
    sizeof(send_buff): length of sending data
    0: sending mark, usually it is 0
    (struct sockaddr *)&send_addr: structure pointer of the receiving data addresses

    (including IP addresses and port)

    addr_len: structure size of the receiving data addresses

    */
    // printf("Send HEX data\n");
    socklen_t addr_len = sizeof(struct sockaddr_in);
            
    if (sendto(sockfd, send_buff, sizeof(send_buff), 0, (struct sockaddr *)&send_addr, addr_len) < 0)
    {
        perror("sendto");
        exit(1);
    }

    /* Receive the responding data from gimbal camera

    sockfd: descriptor of “sockfd” socket
    recv_buff: head address in RAM of the responding data
    RECV_BUUF_SIZE: size of the buffer, which is the length of the max data to

    receive

    0: receiving mark, usually it is 0
    (struct sockaddr *)&recv_addr: the target structure will be filled with addresses (including

    IP addresses and port) from the data sender
    &addr_len: the target storage position, the structure size of
    “src_addr” and “addrlen” should be filled before calling, the actual size of the sender will be filled after calling
    */
    recv_len = recvfrom(sockfd, recv_buff, RECV_BUUF_SIZE, 0, (struct sockaddr *)&recv_addr,
                        &addr_len);
    if (recv_len < 0)
    {
        perror("recvfrom");
        exit(1);
    }
    
    if (debug){
        // print the received data in hexadecimal
        printf("Received HEX data: ");
        for (int i = 0; i < recv_len; i++)
        {
            printf("%02x ", recv_buff[i]);
        }
        printf("\n");
    }

    // close socket
    close(sockfd);

    //cleanup routine
    cleanup();

    return 0;
}

void printCommand(unsigned char cmd[], short len){
    printf("Command: ");
    for (int i = 0; i < len;i++){
        printf("%x ",cmd[i]);
    }
    printf("\n");
}

// custom strcpy(), since we want to copy at least MIN_CMD_SIZE, even though we may find a 0x00 in the way
// (which is the same as the null terminating character used to stop the copy process)
// returns final string total size
int cmdstrcpy(unsigned char* dst, unsigned char* src){
    int len = 0;

    while (len < MIN_CMD_SIZE || (len < MAX_CMD_SIZE && src[len] != '\0')){
        dst[len++] = src[len];
    }
    return len; // subtract 1
}


void printCommandDescription(int cmd_idx){
    unsigned char *cmd = commandDescriptions[cmd_idx];
    
    printf("Command: ");
    while(*cmd != '\0'){
        printf("%c",*(cmd++));
    }
    printf("\n");
}

void printSDKFormat(){

    printf("|-------------------------------------------------------------------------------------------------------------------------------|\n");
    printf("|                                                     SDK COMMANDS FORMAT                                                       |\n"); 
    // Print the divider row
    printf("|-----------------|-----------------|-------------------|-----------------------------------------------------------------------|\n");

    // Print the content rows
    for (int i = 0; i < 8; i++) {
        if (i == 0) {
            // Row 1
            printf("| %-10s | %-12s | %-16s | %-50s |\n", "\tField", "\tIndex", "\tBytes", "\t\t\tDescription");
        } else if (i == 1) {
            // Row 2
            printf("| %-15s | %-12s | %-16s | %-64s |\n", "STX", "\t0", "\t2" ,"\t0x6655: starting mark Low byte in the front");
        } else if (i == 2) {
            // Row 3
            printf("| %-15s | %-12s | %-16s | %-64s |\n", "CTRL", "\t2", "\t1" ,"\t0: need_ack or 1: (this is an) ack_pack ");
        } else if (i == 3) {
            // Row 4
            printf("| %-15s | %-12s | %-16s | %-64s |\n", "Data_len", "\t3", "\t2" ,"\tDate field byte length Low byte in the front");
        } else if (i == 4) {
            // Row 5
            printf("| %-15s | %-12s | %-16s | %-64s |\n", "SEQ", "\t5", "\t2" ,"\tFrame sequence (0 ~ 65535) Low byte in the front");
        } else if (i == 5) {
            // Row 6
            printf("| %-15s | %-12s | %-16s | %-64s |\n", "CMD_ID", "\t7", "\t1" ,"\tCommand ID");
        } else if (i == 6) {
            // Row 7
            printf("| %-15s | %-12s | %-16s | %-64s |\n", "DATA", "\t8", "\tData_len" ,"\tData");
        } else if (i == 7) {
            // Row 8
            printf("| %-15s | %-12s | %-16s | %-64s |\n", "CRC16", "\t-", "\t2" ,"\tCRC16 check to the complete data package. Low byte in the front");
        }

        // Print the divider row after each content row
        printf("|-----------------|-----------------|-------------------|-----------------------------------------------------------------------|\n");
    }
}

int setup(){

    cmds = (uint8_t **)calloc(NUM_COMMANDS, sizeof(uint8_t *));

    if (cmds == NULL) {
        perror("Memory allocation failed");
        return -1;
    }

    for (int i = 0; i < NUM_COMMANDS; i++){
        cmds[i] = (uint8_t *)calloc(cmds_sizes[i], sizeof(uint8_t));
    }
    
    memcpy(cmds[0 ] , (uint8_t []) {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x08,0x01,0xd1,0x12},       cmds_sizes[0 ]);  // 0  - Auto Centering
    memcpy(cmds[1 ] , (uint8_t []) {0x55,0x66,0x01,0x02,0x00,0x00,0x00,0x07,0x00,0x2D,0x3e,0xd1},  cmds_sizes[1 ]);  // 1  - Rotate Up
    memcpy(cmds[2 ] , (uint8_t []) {0x55,0x66,0x01,0x02,0x00,0x00,0x00,0x07,0x00,-0x2D,0xef,0xdf}, cmds_sizes[2 ]);  // 2  - Rotate Down
    memcpy(cmds[3 ] , (uint8_t []) {0x55,0x66,0x01,0x02,0x00,0x00,0x00,0x07,-0x2D,0x00,0x85,0x64}, cmds_sizes[3 ]);  // 3  - Rotate Right
    memcpy(cmds[4 ] , (uint8_t []) {0x55,0x66,0x01,0x02,0x00,0x00,0x00,0x07,0x2D,0x00,0x4b,0x54},  cmds_sizes[4 ]);  // 4  - Rotate Left
    memcpy(cmds[5 ] , (uint8_t []) {0x55,0x66,0x01,0x02,0x00,0x00,0x00,0x07,0x00,0x00,0xf1,0x24},  cmds_sizes[5 ]);  // 5  - Stop rotation
    memcpy(cmds[6 ] , (uint8_t []) {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x05,0x01,0x8d,0x64},       cmds_sizes[6 ]);  // 6  - Zoom +1
    memcpy(cmds[7 ] , (uint8_t []) {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x05,0xFF,0x5c,0x6a},       cmds_sizes[7 ]);  // 7  - Zoom -1
    memcpy(cmds[8 ] , (uint8_t []) {0x55,0x66,0x01,0x02,0x00,0x01,0x00,0x0F,0x04,0x05,0x60,0xBB},  cmds_sizes[8 ]);  // 8  - 4.5x
    memcpy(cmds[9 ] , (uint8_t []) {0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x16,0xB2,0xA6},            cmds_sizes[9 ]);  // 9  - Acquire the Max Zoom Value
    memcpy(cmds[10] , (uint8_t []) {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x06,0x01,0xde,0x31},       cmds_sizes[10]);  // 10 - Manual Focus +1
    memcpy(cmds[11] , (uint8_t []) {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x06,0xff,0x0f,0x3f},       cmds_sizes[11]);  // 11 - Manual Focus -1
    memcpy(cmds[12] , (uint8_t []) {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x00,0x34,0xce},       cmds_sizes[12]);  // 12 - Take Pictures
    memcpy(cmds[13] , (uint8_t []) {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x02,0x76,0xee},       cmds_sizes[13]);  // 13 - Record Video
    memcpy(cmds[14] , (uint8_t []) {0x55,0x66,0x01,0x02,0x00,0x00,0x00,0x07,0x64,0x64,0x3d,0xcf},  cmds_sizes[14]);  // 14 - Rotate 100 100
    memcpy(cmds[15] , (uint8_t []) {0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x0a,0x0f,0x75},            cmds_sizes[15]);  // 15 - Gimbal Status Information
    memcpy(cmds[16] , (uint8_t []) {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x04,0x01,0xbc,0x57},       cmds_sizes[16]);  // 16 - Auto Focus
    memcpy(cmds[17] , (uint8_t []) {0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x02,0x07,0xf4},            cmds_sizes[17]);  // 17 - Acquire Hardware ID
    memcpy(cmds[18] , (uint8_t []) {0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x01,0x64,0xc4},            cmds_sizes[18]);  // 18 - Acquire Firmware Version
    memcpy(cmds[19] , (uint8_t []) {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x03,0x57,0xfe},       cmds_sizes[19]);  // 19 - Lock Mode
    memcpy(cmds[20] , (uint8_t []) {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x04,0xb0,0x8e},       cmds_sizes[20]);  // 20 - Follow Mode
    memcpy(cmds[21] , (uint8_t []) {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x05,0x91,0x9e},       cmds_sizes[21]);  // 21 - FPV Mode
    memcpy(cmds[22] , (uint8_t []) {0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x0d,0xe8,0x05},            cmds_sizes[22]);  // 22 - Acquire Attitude Data
    memcpy(cmds[23] , (uint8_t []) {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x06,0xf2,0xae},       cmds_sizes[23]);  // 23 - Set Video Output as HDMI (Only available on A8 mini, restart to take effect)
    memcpy(cmds[24] , (uint8_t []) {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x07,0xd3,0xbe},       cmds_sizes[24]);  // 24 - Set Video Output as CVBS (Only available on A8 mini, restart to take effect)
    memcpy(cmds[25] , (uint8_t []) {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x08,0x3c,0x4f},       cmds_sizes[25]);  // 25 -  Turn Off both CVBS and HDMI Output (Only available on A8 mini, restart to take effect)
    memcpy(cmds[26] , (uint8_t []) {0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x15,0xD1,0x96},            cmds_sizes[26]);  // 26 - Read Range from Laser Rangefinder(Low byte in the front, high byte in the back, available on ZT30)

    if(debug){
        printf("Done setting up.\n");
    }

    return 0;
}

void cleanup(){
    for (int i = 0; i < NUM_COMMANDS; i++){
        free(cmds[i]);
    }
    free(cmds);
}

/***********************************************************
CRC16 Coding & Decoding G(X) = X^16+X^12+X^5+1
***********************************************************/
uint16_t CRC16_cal(uint8_t *ptr, uint32_t len, uint16_t crc_init){
    uint16_t crc, oldcrc16;
    uint8_t temp;
    crc = crc_init;
    while (len--!=0){
        temp=(crc>>8)&0xff;
        oldcrc16=crc16_tab[*ptr^temp];
        crc=(crc<<8)^oldcrc16;
        ptr++;
    }
    //crc=~crc; //??
    return(crc);
}

/* 3.3.3 SIYI Gimbal Camera SDK Communication Interface
TTL Serial Port
- Baud rate: 115200
- Data position: 8 digits. Stop position: 1 digit. No check.
UDP
- IP: 192.168.144.25
- Port Number: 37260
TCP
- IP: 192.168.144.25
- Port Number: 37260
- Heartbeat Package Data: 55 66 01 01 00 00 00 00 00 59 8B
*/

/* 3.3.4 SIYI Gimbal Camera SDK Communication Code Examples
{0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x05,0x01,0x8d,0x64};       // Zoom 1
{0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x05,0xFF,0x5c,0x6a};       // Zoom -1
{0x55,0x66,0x01,0x02,0x00,0x01,0x00,0x0F,0x04,0x05,0x60,0xBB};  // Absolute Zoom (4.5X)
{0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x16,0xB2,0xA6};            // Acquire the Max Zoom Value
{0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x06,0x01,0xde,0x31};       // Manual Focus 1
{0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x06,0xff,0x0f,0x3f};       // Manual Focus -1
{0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x00,0x34,0xce};       // Take Pictures
{0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x02,0x76,0xee};       // Record Video
{0x55,0x66,0x01,0x02,0x00,0x00,0x00,0x07,0x64,0x64,0x3d,0xcf};  // Rotate 100 100
{0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x08,0x01,0xd1,0x12};       // Auto Centering
{0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x0a,0x0f,0x75};            // Gimbal Status Information
{0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x04,0x01,0xbc,0x57};       // Auto Focus
{0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x02,0x07,0xf4};            // Acquire Hardware ID
{0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x01,0x64,0xc4};            // Acquire Firmware Version
{0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x03,0x57,0xfe};       // Lock Mode
{0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x04,0xb0,0x8e};       // Follow Mode
{0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x05,0x91,0x9e};       // FPV Mode
{0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x0d,0xe8,0x05};            // Acquire Attitude Data
{0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x06,0xf2,0xae};       // Set Video Output as HDMI (Only available on A8 mini, restart to take effect)
{0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x07,0xd3,0xbe};       // Set Video Output as CVBS (Only available on A8 mini, restart to take effect)
{0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x0c,0x08,0x3c,0x4f};       // Turn Off both CVBS and HDMI Output (Only available on A8 mini, restart to take effect)
{0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x15,0xD1,0x96};            // Read Range from Laser Rangefinder (Low byte in the front, high byte in the back, available on ZT30)
*/