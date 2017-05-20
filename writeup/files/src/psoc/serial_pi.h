/*-----------------------------------------------------------------*/
/* serial_pi.h                                                        */
/*                                                                 */
/* BC Cho (bccho@) and TJ Smith (tjs8@)                            */
/*-----------------------------------------------------------------*/
#ifndef SERIAL_PI_H
#define SERIAL_PI_H

/*-----------------------------------------------------------------*/

/* Initializes serial communications */
void Serial_Pi_init();

/* Handles incoming serial data 
   Returns: null if transmission is not complete, the transmitted
            string otherwise */
char *Serial_Pi_handleData();



#endif