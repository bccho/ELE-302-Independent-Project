/*-----------------------------------------------------------------*/
/* serial.h                                                        */
/*                                                                 */
/* BC Cho (bccho@) and TJ Smith (tjs8@)                            */
/*-----------------------------------------------------------------*/
#ifndef SERIAL_H
#define SERIAL_H

/*-----------------------------------------------------------------*/

/* Initializes serial communications */
void Serial_init();

/* Handles incoming serial data 
   Returns: null if transmission is not complete, the transmitted
            string otherwise */
char *Serial_handleData();



#endif