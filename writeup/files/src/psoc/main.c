/*-----------------------------------------------------------------*/
/* main.c                                                          */
/*                                                                 */
/* BC Cho (bccho@) and TJ Smith (tjs8@)                            */
/*-----------------------------------------------------------------*/
#include <device.h>
#include <hardware.h>
#include <serial.h>
#include <serial_pi.h>
#include <accel.h>
#include <stdio.h>
#include <math.h>

static char buildVersion[] = "Build 1.0.0";
static char strbuffer[100];


#define X_DIST 60
#define Y_DIST 65
static double alphaAccelCorrect;
static double rAccelCorrect;

static int rxInterrupted = 0;
static int rxPiInterrupted = 0;
static int refreshInterrupted = 0;

static int terminal = 0;

#define ACCEL_READ_PERIOD 0.01
#define PADDLE_MIN_CMP 60//100
#define PADDLE_MAX_CMP 240//200

// Function prototypes
static void parseMessage(char *message);
static void driveMagPhase(double direction, double magnitude, double angleMag, double currentAngle);
static void drive(int dirs[], double mags[]);

static int dirs[4];
static double mags[4];

static double currentAngle;
static double currentPos[2];
static double currentVelocity[2];

static int accelTestVal = 0;

static double heading = 0;
static double magnitude = 0;
static double rotation = 0;
static int enableGyro = 0;

static int goToTarget = 0;               
static double target[2];
static double targetMargin = 100;
static double targetTime = 0;
static double timeElapsed = 0;
static int oldCounterVal = 0;

static int accelerationLimitEnabled = 1;
static double accelLimit = 0.03;

static double P = 0.01;
static double flickTime = 0.1;
static int rotOn;
static int rotDone;

/*
 * parseMessage()
 *
 * Parse serial input string and act on it
 *
 * Inputs: message
 * Returns: none
 */
static void parseMessage(char *message) {
    if (message == NULL) return;
    
    char msgPrefix[5];
    if (sscanf(message, "%4[A-Za-z]", msgPrefix) == 1) {
        int prefixLen = strlen(msgPrefix);
        
        // Messages of the form "CTx.y" (x integer, y float) set motor x to throttle y
        if (strcmp(msgPrefix, "CT") == 0) {
            double value = 0;
            if (sscanf(message + prefixLen, "%lf", &value) > 0) {
                int motorNum = floor(fabs(value));
                double throttleMag = fabs(value) - motorNum;
                if (throttleMag > 1) throttleMag = 1;
                if (throttleMag < 0) throttleMag = 0;
                int throttleDir = value > 0 ? 1 : 0;
                switch (motorNum) {
                    case 0:
                        PWM_Front_Left_WriteCompare(throttleMag * PWM_Front_Left_ReadPeriod());
                        break;
                    case 1:
                        PWM_Front_Right_WriteCompare(throttleMag * PWM_Front_Right_ReadPeriod());
                        break;
                    case 2:
                        PWM_Back_Left_WriteCompare(throttleMag * PWM_Back_Left_ReadPeriod());
                        break;
                    case 3:
                        PWM_Back_Right_WriteCompare(throttleMag * PWM_Back_Right_ReadPeriod());
                        break;
                }
                int regVal = Control_Reg_Direction_Read();
                int newRegVal = (regVal & ~(1 << motorNum)) | (throttleDir << motorNum);
                Control_Reg_Direction_Write(newRegVal);
                sprintf(strbuffer, "Motor Num: %d, Throttle Mag: %f, Throttle Dir: %d\n", motorNum, throttleMag, throttleDir);
                terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
                sprintf(strbuffer, "Old Reg Val: %d, New Reg Val: %d\n", regVal, newRegVal);
                terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer); 
            }
        }
        
        // Messages of the form "CPx" (x integer) set PWM to period x
        if (strcmp(msgPrefix, "CP") == 0) {
            int value = 0;
            if (sscanf(message + prefixLen, "%d", &value) > 0) {
                if (value > 100000) value = 100000;
                if (value < 0) value = 0;
                PWM_Front_Left_WritePeriod(value);
                PWM_Front_Right_WritePeriod(value);
                PWM_Back_Left_WritePeriod(value);
                PWM_Back_Right_WritePeriod(value);
                
                sprintf(strbuffer, "New Period: %d\n", value);
                terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
            }
        }
        
        // Drive forward
        if (strcmp(msgPrefix, "w") == 0) {
            heading = 0;
            UART_PutString("Driving Forward\n");
        }
        // Drive backward
        if (strcmp(msgPrefix, "s") == 0) {
            heading = M_PI;
            sprintf(strbuffer, "Driving Backward\n");
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        // Drive left
        if (strcmp(msgPrefix, "a") == 0) {
            heading = M_PI/2;  
            sprintf(strbuffer, "Driving Left\n");
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        // Drive right
        if (strcmp(msgPrefix, "d") == 0) {
            heading = -M_PI/2; 
            sprintf(strbuffer, "Driving Right\n");
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        // Stop
        if ((strcmp(msgPrefix, "q") == 0) || (strcmp(msgPrefix, "Q") == 0)) {
            magnitude = 0;
            rotation = 0;
            goToTarget = 0;
            sprintf(strbuffer, "Stopped\n");
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        // Rotate Right
        if (strcmp(msgPrefix, "c") == 0) {
            rotation = -0.5;
            sprintf(strbuffer, "Rotating Right Enabled\n");
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        // Rotate Left
        if (strcmp(msgPrefix, "z") == 0) {
            rotation = 0.5;    
            sprintf(strbuffer, "Rotating Left Enabled\n");
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        // Stop Rotation
        if (strcmp(msgPrefix, "x") == 0) {
            rotation = 0;    
            sprintf(strbuffer, "Rotating Disabled\n");
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        
        // Enable Gyro
        if (strcmp(msgPrefix, "EG") == 0) {
            enableGyro = 1;    
            sprintf(strbuffer, "Gyro Enabled\n");
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        // Disable Gyro
        if (strcmp(msgPrefix, "DG") == 0) {
            enableGyro = 0;    
            sprintf(strbuffer, "Gyro Disabled\n");
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        
        // Arbitrary direction
        if (strcmp(msgPrefix, "p") == 0) {
            driveMagPhase(1, 0.5, -0.3, 0);    
            sprintf(strbuffer, "Arbitrary\n");
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        
        // Change Magnitude
        if (strcmp(msgPrefix, "CM") == 0) {
            double value = 0;
            if (sscanf(message + prefixLen, "%lf", &value) > 0) {
                magnitude = value;
            }
            sprintf(strbuffer, "Changed magnitude to %f\n", magnitude);
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        
        // Change Rotation
        if (strcmp(msgPrefix, "CR") == 0) {
            double value = 0;
            if (sscanf(message + prefixLen, "%lf", &value) > 0) {
                rotation = value;
            }
            sprintf(strbuffer, "Changed rotation to %f\n", rotation);
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        
        // Change Heading
        if (strcmp(msgPrefix, "CH") == 0) {
            double value = 0;
            if (sscanf(message + prefixLen, "%lf", &value) > 0) {
                heading = value;
            }
            sprintf(strbuffer, "Changed heading to %f\n", heading);
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        
        // Accel Test
        if (strcmp(msgPrefix, "AT") == 0) {
            int value = 0;
            if (sscanf(message + prefixLen, "%d", &value) > 0) {
                accelTestVal = value;
            }
            sprintf(strbuffer, "Accel Test\n");
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        
        // Reset
        if (strcmp(msgPrefix, "R") == 0) {
            currentAngle = 0;
            currentPos[0] = 0;
            currentPos[1] = 0;
            currentVelocity[0] = 0;
            currentVelocity[1] = 0;
            sprintf(strbuffer, "Reset Inertial Tracking\n");
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        
        // Go to pos
        if (strcmp(msgPrefix, "GO") == 0) {
            double x,y, t, angle;
            if (sscanf(message + prefixLen, "(%lf, %lf, %lf, %lf)", &x, &y, &t, &angle) > 0) {
                currentAngle = 0;
                currentPos[0] = 0;
                currentPos[1] = 0;
                currentVelocity[0] = 0;
                currentVelocity[1] = 0;
                targetTime = t;
                timeElapsed = 0;
                
                goToTarget = 1;
                
                target[0] = x;
                target[1] = y;
                
                if (angle > 1) angle = 1;
                if (angle < 0) angle = 0;
                PWM_Servo_WriteCompare(PADDLE_MIN_CMP + angle * (PADDLE_MAX_CMP - PADDLE_MIN_CMP));
                
                sprintf(strbuffer, "X: %f, Y: %f\n", x, y);
                terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
            }
        }
        
        // Change Target Margin
        if (strcmp(msgPrefix, "CTM") == 0) {
            double value = 0;
            if (sscanf(message + prefixLen, "%lf", &value) > 0) {
                targetMargin = value;
            }
            sprintf(strbuffer, "Target Margin: %fmm\n", targetMargin);
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        
        // Print vals
        if (strcmp(msgPrefix, "P") == 0) {
            sprintf(strbuffer, "Angle: %8.3f, Px: %8.3f, Py: %8.3f\n", 
                        currentAngle, currentPos[0], currentPos[1]);
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        
        // Enable Accel Limit
        if (strcmp(msgPrefix, "EAL") == 0) {
            accelerationLimitEnabled = 1;    
            sprintf(strbuffer, "Acceleration Limit Enabled\n");
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        // Disable Accel Limit
        if (strcmp(msgPrefix, "DAL") == 0) {
            accelerationLimitEnabled = 0;    
            sprintf(strbuffer, "Acceleration Limit Disabled\n");
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        // Set Accel Limit
        if (strcmp(msgPrefix, "SAL") == 0) {
            double value = 0;
            if (sscanf(message + prefixLen, "%lf", &value) > 0) {
                accelLimit = value;
            }
            sprintf(strbuffer, "Accel Limit: %fmm\n", accelLimit);
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        
        // Set P
        if (strcmp(msgPrefix, "SP") == 0) {
            double value = 0;
            if (sscanf(message + prefixLen, "%lf", &value) > 0) {
                P = value;
            }
            sprintf(strbuffer, "P: %f\n", P);
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
        
        // Set Flick Time
        if (strcmp(msgPrefix, "SFT") == 0) {
            double value = 0;
            if (sscanf(message + prefixLen, "%lf", &value) > 0) {
                flickTime = value;
            }
            sprintf(strbuffer, "Flick Time: %fs\n", flickTime);
            terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
        }
    }
}

// 0 deg is forward, 90 is left. magnitude is from 0 to 1
static void driveMagPhaseOld(double direction, double magnitude) {
    // angle w.r.t the axis formed by the front right and back left wheels
    double angle = direction * (M_PI/180) + (M_PI/4);
    // ratio of front left and back right wheel power to front right and back left wheel power
    double forceRatio = fabs(tan(angle));
    
    // mag1 is front left and back right wheel power
    // mag2 is front right and back left wheel power
    double mag1, mag2;
    if (forceRatio > 1) {
        mag2 = magnitude;
        mag1 = mag2 / forceRatio;
    } 
    else {
        mag1 = magnitude;
        mag2 = mag1 * forceRatio;
    }
    
    int dir1 = (angle > 0 && angle < M_PI/2) || (angle > 3*M_PI/2 && angle < 2*M_PI) ? 1 : 0;
    int dir2 = (angle > 0 && angle < M_PI) ? 1 : 0;
    
    dirs[0] = dir1; dirs[1] = dir2; dirs[2] = dir2; dirs[3] = dir1;
    mags[0] = mag1; mags[1] = mag2; mags[2] = mag2; mags[3] = mag1;
    
    drive(dirs, mags);
}



// direction and current angle are CCW w.r.t global forward (in rads), 
// magnitude is from 0 to 1, angleMag is CCW
static void driveMagPhase(double direction, double magnitude, double angleMag, double currentAngle) {
    // angle of drive CCW w.r.t the axis formed by the front right and back left wheels
    double angle = fmod(direction - currentAngle + (M_PI/4), 2*M_PI);
    angle = angle < 0 ? angle + 2*M_PI : angle;
//    sprintf(strbuffer, "angle: %f\n", angle);
//    UART_PutString(strbuffer);
    
    if (angleMag > 1) angleMag = 1;
    if (angleMag < -1) angleMag = -1;
    if (magnitude > 1) magnitude = 1;
    if (magnitude < -1) magnitude = -1;
    
    // 1 is dominant, 0 is non-dominant, motor 4 is lsb
    uint8 dirDominant = 0;
    if (fabs(tan(angle)) > 1) dirDominant = 0b0101;
    else dirDominant = 0b1010;
    
    // 0 is backward, 1 is forward
    uint8 directionChangeDir = 0;
    if ((angle > 0 && angle < M_PI/2) || (angle > 3*M_PI/2 && angle < 2*M_PI)) directionChangeDir |= 0b1010;
    else directionChangeDir &= 0b0101;
    if (angle > 0 && angle < M_PI) directionChangeDir |= 0b0101;
    else directionChangeDir &= 0b1010;
    
//    sprintf(strbuffer, "directionChangeDir: %x\n", directionChangeDir);
//    UART_PutString(strbuffer);
    
    
    // 0 is backward, 1 is forward
    uint8 angleChangeDir = 0;
    if (angleMag > 0) angleChangeDir = 0b0110;
    else angleChangeDir = 0b1001;
    
//    sprintf(strbuffer, "angleChangeDir: %x\n", angleChangeDir);
//    UART_PutString(strbuffer);
    
    // 1 is dominant, 0 is non-dominant
    uint8 angleDominant = ~(directionChangeDir ^ angleChangeDir);
    uint8 dominant = dirDominant & angleDominant;
    int dominantNum = (dominant & 8) ? 0 : (dominant & 4) ? 1 : (dominant & 2) ? 2 : 3;
    uint8 offDominant = ~dirDominant & angleDominant;
    int offDominantNum = (offDominant & 8) ? 0 : (offDominant & 4) ? 1 : (offDominant & 2) ? 2 : 3;
    
//    sprintf(strbuffer, "angleDominant: %x\n", angleDominant);
//    UART_PutString(strbuffer);
//    sprintf(strbuffer, "dominant: %x\n", dominant);
//    UART_PutString(strbuffer);
//    sprintf(strbuffer, "offDominant: %x\n", offDominant);
//    UART_PutString(strbuffer);
//    
//    sprintf(strbuffer, "Dom: %d, DomP: %d, OffDom: %d, OffDomP: %2d\n", 
//            dominantNum, (dominantNum + 2) % 4, offDominantNum, (offDominantNum + 2) % 4);
//    UART_PutString(strbuffer);
    
    // Calculate mags
    double domMag = (magnitude + fabs(angleMag));
    if (domMag > 1) {
        angleMag /= domMag;
        domMag = 1;
    }
    double domPartnerMag = domMag - 2*fabs(angleMag);
    double offMultiplier = fabs(tan(angle));
    offMultiplier = offMultiplier > 1 ? (1/offMultiplier) : offMultiplier;
    double offDomMag = (domMag - fabs(angleMag)) * offMultiplier + fabs(angleMag);
    double offDomPartnerMag = (domMag - fabs(angleMag)) * offMultiplier - fabs(angleMag);
    
    // Assign mags
    double mags[4];
    mags[dominantNum] = domMag;
    mags[(dominantNum + 2) % 4] = fabs(domPartnerMag);
    mags[offDominantNum] = offDomMag;
    mags[(offDominantNum + 2) % 4] = fabs(offDomPartnerMag);
//    sprintf(strbuffer, "Mags: %4.3f, %4.3f, %4.3f, %4.3f\n", mags[0], mags[1], mags[2], mags[3]);
//    UART_PutString(strbuffer);
    
    // Find dirs
    int dirs[4];
    dirs[dominantNum] = ((dominant & directionChangeDir) != 0) ? 1 : 0;
    dirs[offDominantNum] = ((offDominant & directionChangeDir) != 0) ? 1 : 0;
    dirs[(dominantNum + 2) % 4] = domPartnerMag > 0 ? dirs[dominantNum] : 1 - dirs[dominantNum];
    dirs[(offDominantNum + 2) % 4] = offDomPartnerMag > 0 ? dirs[offDominantNum] : 1 - dirs[offDominantNum];
//    sprintf(strbuffer, "Dirs: %2d, %2d, %2d, %2d\n", dirs[0], dirs[1], dirs[2], dirs[3]);
//    UART_PutString(strbuffer);
    
    drive(dirs, mags);
}
    

static void drive(int dirs[], double mags[]) {
    int i;
    for (i = 0; i < 4; i++) {
        if (mags[i] < 0) mags[i] = 0;
        if (mags[i] > 1) mags[i] = 1;
        if (dirs[i] != 0 && dirs[i] != 1) dirs[i] = 0;
    }
    
    PWM_Front_Left_WriteCompare(mags[0] * PWM_Front_Left_ReadPeriod());
    PWM_Front_Right_WriteCompare(mags[1] * PWM_Front_Right_ReadPeriod());
    PWM_Back_Left_WriteCompare(mags[3] * PWM_Back_Left_ReadPeriod());
    PWM_Back_Right_WriteCompare(mags[2] * PWM_Back_Right_ReadPeriod());
    
    int regVal = dirs[0] | (dirs[1] << 1) | (dirs[3] << 2) | (dirs[2] << 3);
    Control_Reg_Direction_Write(regVal);
}

/* Received data interrupt */
CY_ISR(rx_ISR) {
    rxInterrupted = 1;
}

/* Received data interrupt pi */
CY_ISR(rx_pi_ISR) {
    rxPiInterrupted = 1;
}

/* Refresh timer interrupt */
CY_ISR(refresh_ISR) {
//    if (refreshInterrupted != 0) UART_PutString("M\n");
//    else UART_PutString("H\n");
    static int count;
    count++;
    if (count > 10) {
        refreshInterrupted = 1;
        count = 0;
    }
}


void main()
{
    /* Initialization */
    CyGlobalIntEnable; // Enable global interrupts
    
    // Start LCD
    LCD_Start();
    LCD_PrintString(buildVersion);
    
    // Start UART
    UART_Start();
    rx_rcvd_Start();
    rx_rcvd_SetVector(rx_ISR);
    
    // Start Pi UART
    UART_Pi_Start();
    rx_rcvd_pi_Start();
    rx_rcvd_pi_SetVector(rx_pi_ISR);
    
    // Start refresh interrupt
    Counter_Refresh_Start();
    refresh_interrupt_Start();
    refresh_interrupt_SetVector(refresh_ISR);
    
    // Start Motors
    PWM_Front_Left_Start();
    PWM_Front_Right_Start();
    PWM_Back_Left_Start();
    PWM_Back_Right_Start();
    UART_Start();
    
    // Init serial
    Serial_init();
    Serial_Pi_init();
    
    // Start I2C
    I2C_Start();
    
    // Init accelerometer
    Accel_init();
    
    alphaAccelCorrect = atan(X_DIST/Y_DIST);
    rAccelCorrect = sqrt(X_DIST*X_DIST + Y_DIST*Y_DIST);
    
    PWM_Servo_Start();
    PWM_Servo_WriteCompare((PADDLE_MIN_CMP + PADDLE_MAX_CMP) / 2);
    
    Counter_Time_Start();

    for(;;)
    {
       // Handle incoming serial data
        if (rxInterrupted) {
            char *message = Serial_handleData();
            parseMessage(message);
            terminal = 0;
            rxInterrupted = 0;
        }
        
        // Handle incoming pi serial data
        if (rxPiInterrupted) {
            char *message = Serial_Pi_handleData();
            parseMessage(message);
            terminal = 1;
            rxPiInterrupted = 0;
        }
        
        // Handle refresh timer
        if (refreshInterrupted) {

            Accel_refresh();
            
            double accelVals[3];
            double gyroVals[3];
            double magVals[3];
            Accel_getAccel(accelVals);
            Accel_getGyro(gyroVals);
            Accel_getMag(magVals);
            
            // Integrate angular velocity to get angle
            currentAngle += gyroVals[2] * ACCEL_READ_PERIOD * (M_PI/180);
            
            // Convert acceleration to mm/s^2
            accelVals[0] *= 9.81;
            accelVals[1] *= 9.81;
            
            // Correct raw accel axis for centrifugal effects
            double omega = gyroVals[2] * ACCEL_READ_PERIOD;
            accelVals[0] += rAccelCorrect*cos(alphaAccelCorrect)*omega*omega;
            accelVals[1] += rAccelCorrect*sin(alphaAccelCorrect)*omega*omega;
            
            // Convert to true x and y
            double trueAccel[2];
            trueAccel[0] = -accelVals[0]*sin(currentAngle) - accelVals[1]*cos(currentAngle);
            trueAccel[1] = accelVals[0]*cos(currentAngle) - accelVals[1]*sin(currentAngle);
            
            // Integrate acceleration to get velocity and position
            int i;
            for (i = 0; i < 2; i++) { 
                currentVelocity[i] += trueAccel[i] * ACCEL_READ_PERIOD;
                currentPos[i] += currentVelocity[i] * ACCEL_READ_PERIOD;
            }
            
//            static int count;
//            count++;;
//            if (count > 10) {
//                sprintf(strbuffer, "Norm: %7.3f\n",
//                        magVals[0]*magVals[0] + magVals[1]*magVals[1]);
//                UART_PutString(strbuffer);
//                sprintf(strbuffer, "%7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f\n", timeElapsed, trueAccel[0], trueAccel[1], currentAngle, currentPos[0], currentPos[1]);
//                UART_PutString(strbuffer);
//                sprintf(strbuffer, "Angle: %8.3f, Px: %8.3f, Py: %8.3f\n", 
//                        currentAngle, currentPos[0], currentPos[1]);
//                UART_PutString(strbuffer);
//                sprintf(strbuffer, "x Raw: %8.3f, y Raw: %8.3f, x Real: %8.3f, y Real: %8.3f\n", 
//                        accelVals[0], accelVals[1], trueAccel[0], trueAccel[1]);
//                UART_PutString(strbuffer);
//                sprintf(strbuffer, "                                                            \n");
//                int breakIndex = (int) currentPos[0]/10 + 17;
//                strbuffer[breakIndex] = '0';
//                breakIndex = (int) currentPos[1]/10 + 38;
//                strbuffer[breakIndex] = '0';           
//                
//                count = 0;
//            }
            
            
            
            if (goToTarget) {
                int newCounterVal = Counter_Time_ReadCounter();
                int countsElapsed = oldCounterVal - newCounterVal;
                if (countsElapsed < 0) countsElapsed += 65535;
                timeElapsed += (double) countsElapsed / 100000.0;
                oldCounterVal = newCounterVal;
                
                double xDist = (target[0] - currentPos[0]);
                double yDist = (target[1] - currentPos[1]);
                if (yDist == 0) yDist = 0.0000001;
                heading = atan(-xDist/yDist);
                if (yDist < 0) heading += M_PI;
                double dist = sqrt(xDist*xDist + yDist*yDist);
                magnitude = P*dist;
                if (magnitude > 1) magnitude = 1;
                
                //targetAngle = M_PI/2;
                
                rotation = 0;
                enableGyro = 1;
                
                if (dist < targetMargin) {
                    goToTarget = 0;
                    magnitude = 0;
                    rotation = 0;
                    sprintf(strbuffer, "Target Reached in %fs\n", timeElapsed);
                    terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
                }
                
                if (timeElapsed > 1) {
                    goToTarget = 0;
                    magnitude = 0;
                    rotation = 0;
                    sprintf(strbuffer, "Time Up\n");
                    terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
                }

                if (timeElapsed > targetTime - flickTime && !rotOn && timeElapsed < targetTime) {
                    rotation = -1;
                    sprintf(strbuffer, "Rot on\n");
                    terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
                    rotOn = 1;
                }
                if (timeElapsed > targetTime && rotOn) {
                    rotation = 0;
                    sprintf(strbuffer, "Rot off\n");
                    terminal == 0 ? UART_PutString(strbuffer) : UART_Pi_PutString(strbuffer);
                    rotOn = 0;
                }
            }
            
            if (accelerationLimitEnabled) {
                static double oldMag;
                double setMag = (fabs(magnitude-oldMag) > accelLimit) ? (magnitude - oldMag)/fabs(magnitude-oldMag) * accelLimit + oldMag : magnitude;
                driveMagPhase(heading, setMag, rotation, enableGyro == 1 ? currentAngle : 0);
                oldMag = setMag;
            }
            else {
                driveMagPhase(heading, magnitude, rotation, enableGyro == 1 ? currentAngle : 0);
            }
            
            
//            static int count;
//            count++;
//            if (count > 10) {  //100x per sec
//                count = 0;
//                Accel_getAccel(accelVals);
//                Accel_getGyro(gyroVals);
//                
//                int breakIndex;
//                switch (accelTestVal) {
//                    case 0:
////                        sprintf(strbuffer, "%8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f\n", accelVals[0], accelVals[1], accelVals[2], gyroVals[0], gyroVals[1], gyroVals[2]);
//                        sprintf(strbuffer, "%2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f\n", accelVals[0], accelVals[1], accelVals[2], gyroVals[0], gyroVals[1], gyroVals[2]);
//                        break;
//                    case 1:
//                        sprintf(strbuffer, "                                                            \n");
//                        breakIndex = (int) accelVals[0]/200 + 6;
//                        strbuffer[breakIndex] = '0';
//                        break;
//                    case 2:
//                        sprintf(strbuffer, "                                                            \n");
//                        breakIndex = (int) accelVals[1]/200 + 6;
//                        strbuffer[breakIndex] = '0';
//                        break;
//                    case 3:
//                        sprintf(strbuffer, "                                                            \n");
//                        breakIndex = (int) accelVals[2]/200 + 6;
//                        strbuffer[breakIndex] = '0';
//                        break;
//                    case 4:
//                        sprintf(strbuffer, "                                                            \n");
//                        breakIndex = (int) gyroVals[0]/100 + 6;
//                        strbuffer[breakIndex] = '0';
//                        break;
//                    case 5:
//                        sprintf(strbuffer, "                                                            \n");
//                        breakIndex = (int) gyroVals[1]/100 + 6;
//                        strbuffer[breakIndex] = '0';
//                        break;
//                    case 6:
//                        sprintf(strbuffer, "                                                            \n");
//                        breakIndex = (int) gyroVals[2]/100 + 6;
//                        strbuffer[breakIndex] = '0';
//                        break;
//                }
//                UART_PutString(strbuffer);
//            }
            refreshInterrupted = 0;
        }
    }
}

/* [] END OF FILE */
