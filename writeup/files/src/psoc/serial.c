/*-----------------------------------------------------------------*/
/* serial.c                                                        */
/*                                                                 */
/* BC Cho (bccho@) and TJ Smith (tjs8@)                            */
/*-----------------------------------------------------------------*/
#include <serial.h>
#include <device.h>

/*-----------------------------------------------------------------*/

#define BUF_RX_SIZE 100
static char bufRx[BUF_RX_SIZE];
static int bufRxPos = 0;
static int needReset = 0;

/*-----------------------------------------------------------------*/

static void resetRx();

/*-----------------------------------------------------------------*/

/* Initializes serial communications */
void Serial_init() {
    resetRx();
}


/* Handles incoming serial data 
   Returns: null if transmission is not complete, the transmitted
            string otherwise */
char *Serial_handleData() {
    // reset the buffer if needed (last call completed a message)
    if (needReset) {
        resetRx();
        needReset = 0;
    }
    
    char recByte;
    while ((recByte = UART_GetChar()) > 0) {
        // if NL, CR, or EOT, indicate end of message and echo back special char
        if (recByte == 0xA || recByte == 0xD || recByte == 0x4) {
            UART_PutChar('$');
            UART_PutChar(recByte);
            needReset = 1;
            //while (UART_GetChar() > 0) {}
            UART_ClearRxBuffer();
            return bufRx;
        }
        // if we have reached max buffer size, indicate overflow error
        if (bufRxPos == BUF_RX_SIZE) {
            UART_PutString("! Buffer Overflow !");
            needReset = 1;
            return bufRx;
        // for normal bytes, put byte in buffer and echo the byte
        } 
        else {
            bufRx[bufRxPos] = recByte;
            bufRxPos++;
            UART_PutChar(recByte);
        }
    }
    return NULL;
}



/*-----------------------------------------------------------------*/


static void resetRx() {
    // Clear mesage buffer and reset position
    int i;
    for (i = 0; i < bufRxPos; i++) bufRx[i] = 0;
    bufRxPos = 0;
}

/* [] END OF FILE */

