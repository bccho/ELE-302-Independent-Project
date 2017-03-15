/* ========================================
 *
 * Copyright TJ and BC, 2017
 * All Rights Reserved
 * UNPUBLISHED, UNLICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF TJ and BC
 *
 * ========================================
*/
#include <device.h>
#include <stdio.h>
#include <math.h>

// Function prototypes
void setThrottle(int thr);
double getSpeed();
int controlThrottle(double speed);
void handleSerialData();
void parseMessage();
void initRx();
void initSpeedTracking();
void initDerivativeTracking();
void initIntegralTracking();

char buildVersion[] = "Build 1.0.0";

char strbuffer[100];

#define BUF_RX_SIZE 100
#define BUF_WHEEL_TICK_SIZE 3
#define BUF_SPEED_SIZE 10

const int MAX_THROTTLE = 999;
const int CLK_FREQ = 100000;
const int PID_TIMER_PERIOD = 10000;
const int TIMER_PERIOD = 65536;
const double WHEEL_DIAM = 2.41; // in inches

double targetSpeed = 4.0; // in feet per second
int throttleSS = 100;
double errorIntegral = 0.0;

double bufSpeed[BUF_SPEED_SIZE];
int bufSpeedPos = 0;
int bufSpeedLastPos = 0;
double totalDistance = 0;
double targetDistance = 40;
double targetTime = 10; // do not set directly
double elapsedTime = 0;
int dynamicTargetSpeedUpdate = 0;

// PID coefficients
double coeffP = 50.0;
double coeffI = 30.0;
double coeffD = 10.0;

int throttlePIDon = 0;
int distanceTimeoutOn = 0;

int buttonInterrupted = 0;
int rxInterrupted = 0;
int wheelTickInterrupted = 0;
int tickTimeoutInterrupted = 0;
int throttleUpdateInterrupted = 0;

int throttle;

char bufRx[BUF_RX_SIZE];
int bufRxPos = 0;
char recByte;
int msgReceived = 0;

int bufWheelTick[BUF_WHEEL_TICK_SIZE];
int bufWheelTickPos = 0;
int bufWheelTickCurSize = 0;
int oldTickCount = 0;






/*
 * setThrottle()
 *
 * Takes an integer between 0 and MAX_THROTTLE and
 * sets the PWM output to that value
 *
 * Inputs: throttle value (int)
 * Returns: None
 */
void setThrottle(int thr) {
    if (thr < 0)
        thr = 0;
    if (thr > MAX_THROTTLE)
        thr = MAX_THROTTLE;
    
    throttle = thr;
    
    PWM_WriteCompare(throttle);
    
    LCD_Position(1,0);
    char lcdbuffer[4];
    sprintf(lcdbuffer, "%3i", throttle);
    LCD_PrintString(lcdbuffer);
}




/*
 * getSpeed()
 *
 * Calculates average speed (in ft/sec) over the last BUF_WHEEL_TICK_SIZE ticks.
 *
 * Inputs: none
 * Returns: speed (double)
 */
double getSpeed() {
    double cumSum = 0;
    int i;
    if (bufWheelTickCurSize == 0) return 0;
    for (i = 0; i < BUF_WHEEL_TICK_SIZE; i++) {
        cumSum += (double) bufWheelTick[i];
    }
    double averageCount = cumSum / bufWheelTickCurSize;
    double ticksPerSec = CLK_FREQ / averageCount;
    double speed = ticksPerSec * ((M_PI * WHEEL_DIAM) / 5) * (1.0/12.0);
    
    return speed;
}




/*
 * controlThrottle()
 *
 * Updates the error values, and uses them to update
 * the set point throttle for PWM control
 *
 * Inputs: speed (double)
 * Returns: newThrottle (int)
 */
int controlThrottle(double speed) {
    // Dynamically update target speed if enabled
    double dynamicTargetSpeed = targetSpeed;
    double distanceLeft = 0;
    double timeLeft = 0;
    const double MAX_SPEED = targetSpeed * 2;
    if (dynamicTargetSpeedUpdate) {
        distanceLeft = targetDistance - totalDistance;
        timeLeft = targetTime - elapsedTime;
        if (distanceLeft > 0) {
            if (timeLeft <= 0) {
                dynamicTargetSpeed = MAX_SPEED;
            } else {
                dynamicTargetSpeed = distanceLeft / timeLeft;
                if (dynamicTargetSpeed > MAX_SPEED) dynamicTargetSpeed = targetSpeed * 2; // limit dyn target speed
            }
        }
    }

    // Calculate error
    double error = dynamicTargetSpeed - speed;
    bufSpeed[bufSpeedPos] = error;
    // Calculate derivative of error
    double errorDerivative = (bufSpeed[bufSpeedPos] - bufSpeed[bufSpeedLastPos]) / (double) BUF_SPEED_SIZE;
    bufSpeedPos = (bufSpeedPos + 1) % BUF_SPEED_SIZE;
    if (bufSpeedPos == bufSpeedLastPos) { // this gets activated when bufSpeedPos has made one loop around the ring buffer
        bufSpeedLastPos = (bufSpeedLastPos + 1) % BUF_SPEED_SIZE;
    }
    // Calculate integral of error
    errorIntegral += error;
    
    sprintf(strbuffer, "%3i, %6.2f, %6.2f, %6.2f, %8.5f, %6.2f | %6.2f, %6.2f, %6.2f\n", throttle, speed, error, errorIntegral, errorDerivative, totalDistance, dynamicTargetSpeed, distanceLeft, timeLeft);
    UART_1_PutString(strbuffer);
    
    return throttleSS + (int) (error * coeffP + errorIntegral * coeffI - errorDerivative * coeffD);
}




/*
 * handleSerialData()
 *
 * Reads incoming bytes of serial data, stores them in bufRx,
 * and raises the msgReceived flag when the transmission is complete
 *
 * Inputs: none
 * Returns: none
 */
void handleSerialData() {
    // if the last received message hasn't been parsed yet, display error
    if (msgReceived) {
        UART_1_PutString("! Buffer Overwrite !");
    } else {
        recByte = UART_1_GetChar();
        // if NL, CR, or EOT, indicate end of message and echo back special char
        if (recByte == 0xA || recByte == 0xD || recByte == 0x4) {
            msgReceived = 1;
            UART_1_PutChar('$');
            UART_1_PutChar(recByte);
        } else if (recByte > 0) {
            // if we have reached man buffer size, indicate overflow error
            if (bufRxPos == BUF_RX_SIZE) {
                msgReceived = 1;
                UART_1_PutString("! Buffer Overflow !");
            // for normal bytes, put byte in buffer and echo the byte
            } else {
                bufRx[bufRxPos] = recByte;
                bufRxPos++;
                UART_1_PutChar(recByte);
            }
        // byte should never be zero, so print error char if it is
        } else {
            UART_1_PutChar('!');
        }
    }
}


/* set EPT */
void enablePIDwithTimeout() {
    if (!throttlePIDon) { // enable PID if not on already
        initDerivativeTracking();
        initIntegralTracking();
        throttlePIDon = 1;
    }
    totalDistance = 0;
    distanceTimeoutOn = 1;
    elapsedTime = 0;
    UART_1_PutString("PID control on w/ distance timeout\n");
}

void killAll() {
    setThrottle(0);
    throttlePIDon = 0;
    UART_1_PutString("PID and throttle killed\n");
}

/*
 * parseMessage()
 *
 * Parse serial input string and act on it
 *
 * Inputs: none
 * Returns: none
 */
void parseMessage() {
    char bufMsgPrefix[4];
    if (sscanf(bufRx, "%3[A-Za-z]", bufMsgPrefix) == 1) {
        int prefixLen = strlen(bufMsgPrefix);
        
        // Messages of the form "CTx" (x integer) set the duty cycle to x out of MAX_THROTTLE
        if (strcmp(bufMsgPrefix, "CT") == 0) {
            int value = 0;
            if (sscanf(bufRx + prefixLen, "%i", &value) > 0) {
                if (!throttlePIDon) setThrottle(value);
                else UART_1_PutString("  Cannot change throttle when PID on\n");
            }
            
            sprintf(strbuffer, "Throttle = %i\n", throttle);
            UART_1_PutString(strbuffer);
        }
        
        // Messages beginning with "G" return the speed of the car and PID controls
        else if (strcmp(bufMsgPrefix, "G") == 0) {
            sprintf(strbuffer, "Throttle: %d\nSpeed: %f ft/s\nP = %.2f\tI = %.2f\tD = %.2f\n", throttle, getSpeed(), coeffP, coeffI, coeffD);
            UART_1_PutString(strbuffer);
        }
        
        // Messages beginning with "TP" toggle PID control
        else if (strcmp(bufMsgPrefix, "TP") == 0) {
            throttlePIDon = throttlePIDon ? 0 : 1;
            if (throttlePIDon) {
                initDerivativeTracking();
                initIntegralTracking();
                totalDistance = 0;
                elapsedTime = 0;
                UART_1_PutString("PID control on\n");
            } else {
                UART_1_PutString("PID control off\n");
            }
        }
        
        // Messages of the form "EPT" enables PID control w/ distance timeout
        else if (strcmp(bufMsgPrefix, "EPT") == 0) {
            enablePIDwithTimeout();
        }
        
        // Messages beginning with "TDS" toggle whether we dynamically update target speed to reach distance
        else if (strcmp(bufMsgPrefix, "TDS") == 0) {
            dynamicTargetSpeedUpdate = dynamicTargetSpeedUpdate ? 0 : 1;
            if (!throttlePIDon) {
                if (dynamicTargetSpeedUpdate) {
                    UART_1_PutString("Dynamic target speed on\n");
                } else {
                    UART_1_PutString("Dynamic target speed off\n");
                }
            } else UART_1_PutString("  Cannot toggle dynamic target speed when PID on\n");
        }
        
        // Messages of the form "CPx" (x integer) change the proportional term to x
        else if (strcmp(bufMsgPrefix, "CP") == 0) {
            int value = 0;
            if (sscanf(bufRx + prefixLen, "%i", &value) > 0) coeffP = value;

            sprintf(strbuffer, "P = %.2f\n", coeffP);
            UART_1_PutString(strbuffer);
        }
        
        // Messages of the form "CIx" (x float) change the integral term to x
        else if (strcmp(bufMsgPrefix, "CI") == 0) {
            double value = 0;
            if (sscanf(bufRx + prefixLen, "%lf", &value) > 0) coeffI = value;

            sprintf(strbuffer, "I = %.2f\n", coeffI);
            UART_1_PutString(strbuffer);
        }
        
        // Messages of the form "CDx" (x integer) change the derivative term to x
        else if (strcmp(bufMsgPrefix, "CD") == 0) {
            int value = 0;
            if (sscanf(bufRx + prefixLen, "%i", &value) > 0) coeffD = value;

            sprintf(strbuffer, "D = %.2f\n", coeffD);
            UART_1_PutString(strbuffer);
        }
        
        // Messages of the form "CSSx" (x integer) change steady state throttle to x
         else if (strcmp(bufMsgPrefix, "CSS") == 0) {
            int value = 0;
            if (sscanf(bufRx + prefixLen, "%i", &value) > 0) throttleSS = value;
            
            sprintf(strbuffer, "SS Throttle = %i\n", throttleSS);
            UART_1_PutString(strbuffer);
        }
        
        // Messages of the form "CTSx" (x float) change target speed to x ft/s
        else if (strcmp(bufMsgPrefix, "CTS") == 0) {
            double value = 0;
            if (sscanf(bufRx + prefixLen, "%lf", &value) > 0) {
                if (!throttlePIDon) {
                    targetSpeed = value;
                    targetTime = targetDistance / targetSpeed;
                } else UART_1_PutString("  Cannot change target speed when PID on\n");
            }
            
            sprintf(strbuffer, "Target Speed = %f ft/s\n", targetSpeed);
            UART_1_PutString(strbuffer);
        }
        
        // Messages of the form "CTDx" (x float) change target distance to x ft
        else if (strcmp(bufMsgPrefix, "CTD") == 0) {
            double value = 0;
            if (sscanf(bufRx + prefixLen, "%lf", &value) > 0) {
                if (!throttlePIDon) {
                    targetDistance = value;
                    targetTime = targetDistance / targetSpeed;
                } else UART_1_PutString("  Cannot change target distance when PID on\n");
            }
            
            sprintf(strbuffer, "Target distance = %.2f ft\n", targetDistance);
            UART_1_PutString(strbuffer);
        }
        
        // Messages starting with "A" will abort
        else if (strcmp(bufMsgPrefix, "A") == 0) {
            killAll();
        }
    }
}





/*
 * initRx()
 *
 * Reset transmission receiving variables
 *
 * Inputs: none
 * Returns: none
 */
void initRx() {
    // Clear mesage buffer and reset position
    int i;
    for (i = 0; i < bufRxPos; i++) bufRx[i] = 0;
    bufRxPos = 0;
}





/*
 * initSpeedTracking()
 *
 * Resets tick buffer
 *
 * Inputs: none
 * Returns: none
 */
void initSpeedTracking() {
    // Clear wheel tick buffer
    bufWheelTickPos = 0;
    bufWheelTickCurSize = 0;
    int i;
    for (i = 0; i < BUF_WHEEL_TICK_SIZE; i++) bufWheelTick[i] = 0;
}





/*
 * initDerivativeTracking()
 *
 * Resets speed error buffer
 *
 * Inputs: none
 * Returns: none
 */
void initDerivativeTracking() {
    // Clear speed buffer
    bufSpeedPos = 0;
    bufSpeedLastPos = 0;
    int i;
    for (i = 0; i < BUF_SPEED_SIZE; i++) bufSpeed[i] = 0.0;
}





/*
 * initIntegralTracking()
 *
 * Resets integral term
 *
 * Inputs: none
 * Returns: none
 */
void initIntegralTracking() {
    // Reset integral term
    errorIntegral = 0.0;
}




/* Button interrupt */
CY_ISR(button_inter) {
    buttonInterrupted = 1;
}

/* Recieved data interrupt */
CY_ISR(rx_inter) {
    rxInterrupted = 1;
}

/* Wheel tick interrupt */
CY_ISR(wheel_inter) {
    wheelTickInterrupted = 1;
}

/* Wheel tick timeout interrupt */
CY_ISR(timeout_inter) {
    tickTimeoutInterrupted = 1;
}

/* Throttle update timeout interrupt */
CY_ISR(throttle_inter) {
    throttleUpdateInterrupted = 1;
}

int main() {
    /* Initialization */
    CyGlobalIntEnable; // Enable global interrupts
    
    LCD_Start();
    
    PWM_Start();
    
    inter_button_Start();
    inter_button_SetVector(button_inter);
    
    UART_1_Start();
    rx_rcvd_Start();
    rx_rcvd_SetVector(rx_inter);
    
    Timer_1_Start();
    wheel_tick_Start();
    wheel_tick_SetVector(wheel_inter);
    
    Counter_1_Start();
    ShiftReg_1_Start();
    tick_timeout_Start();
    tick_timeout_SetVector(timeout_inter);
    
    Counter_2_Start();
    throttle_update_Start();
    throttle_update_SetVector(throttle_inter);
    
    // Set initial throttle to 0
    throttle = 0;
    PWM_WriteCompare(throttle);
    
    // LCD initial text
    LCD_Position(0, 0);
    LCD_PrintString(buildVersion);
    
    // Init Rx
    initRx();
    
    // Init speed tracking buffers
    initSpeedTracking();
    initDerivativeTracking();
    
    for(;;)
    {
        // Handle push button interrupt
        if (buttonInterrupted) {
            if (throttlePIDon) {
                killAll();
            } else {
                enablePIDwithTimeout();
            }
            buttonInterrupted = 0;
        }
        
        // Handle incoming serial data
        if (rxInterrupted) {
            handleSerialData();
            rxInterrupted = 0;
        }
        
        // Parse received strings
        if (msgReceived) {
            parseMessage();
            // Reset rx buffer
            initRx();
            msgReceived = 0;
        }
        
        // Handle wheel ticks
        if (wheelTickInterrupted) {
            int tickCount = Timer_1_ReadCapture();
            
            // Calculate tick delta
            int tickDelta = oldTickCount - tickCount;
            tickDelta = tickDelta < 0 ? TIMER_PERIOD + tickDelta : tickDelta;
            bufWheelTick[bufWheelTickPos] = tickDelta;
            
            // Increment buffer position, and update size and old tick count
            bufWheelTickPos = (bufWheelTickPos + 1) % BUF_WHEEL_TICK_SIZE;
            if (bufWheelTickCurSize < BUF_WHEEL_TICK_SIZE) bufWheelTickCurSize++;
            oldTickCount = tickCount;
            totalDistance += (1.0/5.0) * (M_PI * WHEEL_DIAM) / 12.0;
            wheelTickInterrupted = 0;
        }
        
        // Handle wheel tick timeout
        if (tickTimeoutInterrupted) {
            UART_1_PutString("Wheel timeout\n");
            // Reset speed and derivative tracking
            initSpeedTracking();
            initDerivativeTracking();
            tickTimeoutInterrupted = 0;
        }
            
        // Handle distance timeout
        if (distanceTimeoutOn && totalDistance >= targetDistance) {
            killAll();
            distanceTimeoutOn = 0;
            
            sprintf(strbuffer, "Target distance reached in %6.2f s\n", elapsedTime);
            UART_1_PutString(strbuffer);
        }
        
        // Perform PID update
        if (throttlePIDon && throttleUpdateInterrupted) {
            elapsedTime += (double) PID_TIMER_PERIOD / (double) CLK_FREQ;
            setThrottle(controlThrottle(getSpeed()));
            throttleUpdateInterrupted = 0;
        }   
    }
    
    return 0;
}

/* [] END OF FILE */
