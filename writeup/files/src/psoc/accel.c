/***************************************************************************
  This is a library for the LSM9DS0 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM9DS0 Breakouts

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <accel.h>
#include <device.h>
#include <stdio.h>

/*-----------------------------------------------------------------*/

void readAccel(int *accelData);
void readGyro(int *gyroData);
void readMag(int *magData);
static void write8(int type, uint8_t reg, uint8_t value);
static uint8_t read8(int type, uint8_t reg);
static uint8_t readBuffer(int type, uint8_t reg, uint8_t len, uint8_t *buffer);

/*-----------------------------------------------------------------*/

static char strbuffer[100];

static double accelMgLsb;
static double gyroDpsDigit;
static double magMgaussLsb;

static int zeroed = 0;

#define AVG_BUF_LEN 100
#define NUM_ZERO_READINGS 1000

static int initBufPos = 0;
static int initAccelValsBuf[NUM_ZERO_READINGS][3];
static int initGyroValsBuf[NUM_ZERO_READINGS][3];
static int accelOffset[3];
static int gyroOffset[3];
static double zxCorrelationAccel = 0;
static double zyCorrelationAccel = 0;
//static double zxCorrelationGyro = 0;
//static double zyCorrelationGyro = 0;

static int bufPos = 0;
static int accelValsBuf[AVG_BUF_LEN][3];
static int gyroValsBuf[AVG_BUF_LEN][3];
static int accelValAvg[3];
static int gyroValAvg[3];
static int magValAvg[3];


/*-----------------------------------------------------------------*/



/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
 

int Accel_init() {
    uint8_t id = read8(XMTYPE, LSM9DS0_REGISTER_WHO_AM_I_XM);
    if (id != LSM9DS0_XM_ID) return 0;

    id = read8(GYROTYPE, LSM9DS0_REGISTER_WHO_AM_I_G);
    if (id != LSM9DS0_G_ID) return 0;

	// Accelerometer Settings
//    // Enable FIFO
//	write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG0_XM, 0x40);
    // 1600Hz, enable XYZ
	write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG1_XM, 0xA7);
    // 773Hz anti-alias filter, +-2g range
	write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG2_XM, 0x00);
//    // 50Hz anti-alias filter, +-2g range
//	write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG2_XM, 0xC0);
//    // Enable filters
//	write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG7_XM, 0x20);
//    // Enable FIFO stream mode
//	write8(XMTYPE, LSM9DS0_REGISTER_FIFO_CTRL_REG, 0x40);
    
    accelMgLsb = LSM9DS0_ACCEL_MG_LSB_2G;
	
    // Magnetometer Settings
    write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG7_XM, 0b00000000);
    magMgaussLsb = LSM9DS0_MAG_MGAUSS_2GAUSS;
    
    // Gyro Settings
    // 760Hz, 30 cutoff, enable XYZ
	write8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG1_G, 0xCF);
    // Set high pass filter for 7.2Hz
    write8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG2_G, 0x00);
    // Set scale to +-245dps, self-test off, little endian data
    write8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG4_G, 0x00);
//    // Enable FIFO and high-pass filter
//    write8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG5_G, 0x52);
//    // Enable stream mode for FIFO
//    write8(GYROTYPE, LSM9DS0_REGISTER_FIFO_CTRL_REG_G, 0x40);
    
    gyroDpsDigit = LSM9DS0_GYRO_DPS_DIGIT_245DPS;
    
    // Init arrays
    int i,j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < AVG_BUF_LEN; j++) {
            accelValsBuf[j][i] = 0;
            gyroValsBuf[j][i] = 0;
        }
    }
    
    return 1;
}

void Accel_refresh() {
    // Gather values for zeroing routine
    if (zeroed != 1) {   
        readAccel(initAccelValsBuf[initBufPos]);
        readGyro(initGyroValsBuf[initBufPos]);
        initBufPos++;
        
        if (initBufPos >= NUM_ZERO_READINGS) {
            int i, j;
            // Calculate mean of init data
            for (i = 0; i < 3; i++) {
                for (j = 0; j < NUM_ZERO_READINGS; j++) {
                    accelOffset[i] += initAccelValsBuf[j][i];
                    gyroOffset[i] += initGyroValsBuf[j][i];
                }
                accelOffset[i] /= NUM_ZERO_READINGS;
                gyroOffset[i] /= NUM_ZERO_READINGS;
            }
            
//            // Calculate correlation coefficients
//            double zxAccelInner = 0;
//            double zyAccelInner = 0;
////            double zxGyroInner = 0;
////            double zyGyroInner = 0;
//            double zAccelNormSquare = 0;
////            double zGyroNormSquare = 0;
//            for (i = 0; i < NUM_ZERO_READINGS; i++) {
//                zxAccelInner += (initAccelValsBuf[i][2] - accelOffset[2]) * 
//                                (initAccelValsBuf[i][0] - accelOffset[0]);
//                zyAccelInner += (initAccelValsBuf[i][2] - accelOffset[2]) * 
//                                (initAccelValsBuf[i][1] - accelOffset[1]);
////                zxGyroInner += (initGyroValsBuf[i][2] - gyroOffset[2]) * 
////                               (initGyroValsBuf[i][0] - gyroOffset[0]);
////                zyGyroInner += (initGyroValsBuf[i][2] - gyroOffset[2]) * 
////                               (initGyroValsBuf[i][1] - gyroOffset[1]);
//                zAccelNormSquare += (initAccelValsBuf[i][2] - accelOffset[2]) * 
//                                    (initAccelValsBuf[i][2] - accelOffset[2]);
////                zGyroNormSquare += (initGyroValsBuf[i][2] - gyroOffset[2]) * 
////                                   (initGyroValsBuf[i][2] - gyroOffset[2]);
//            }
//            zxCorrelationAccel = zxAccelInner / zAccelNormSquare;
//            zyCorrelationAccel = zyAccelInner / zAccelNormSquare;
////            zxCorrelationGyro = zxGyroInner / zGyroNormSquare;
////            zyCorrelationGyro = zyGyroInner / zGyroNormSquare;

             // Calculate correlation coefficients test
            int zxAccelInner = 0;
            int zyAccelInner = 0;
            int zAccelNormSquare = 0;
            for (i = 0; i < NUM_ZERO_READINGS; i++) {
                zxAccelInner += (initAccelValsBuf[i][2] - accelOffset[2]) * 
                                (initAccelValsBuf[i][0] - accelOffset[0]) / NUM_ZERO_READINGS;
                zyAccelInner += (initAccelValsBuf[i][2] - accelOffset[2]) * 
                                (initAccelValsBuf[i][1] - accelOffset[1]) / NUM_ZERO_READINGS;
                zAccelNormSquare += (initAccelValsBuf[i][2] - accelOffset[2]) * 
                                    (initAccelValsBuf[i][2] - accelOffset[2]) / NUM_ZERO_READINGS;
            }
            double zxCorrelationAccel = (double) zxAccelInner / (double) zAccelNormSquare;
            double zyCorrelationAccel = (double) zyAccelInner / (double) zAccelNormSquare;
            
            zeroed = 1;
            
            // Print offsets and correlation coefs
            sprintf(strbuffer, "Offsets: %8d, %8d, %8d, %8d, %8d, %8d\n", 
                    accelOffset[0], accelOffset[1], accelOffset[2], 
                    gyroOffset[0], gyroOffset[1], gyroOffset[2]);
            UART_PutString(strbuffer);
            
//            sprintf(strbuffer, "Correlation Coefs: %8.3f, %8.3f, %8.3f. %8.3f\n", 
//                    zxCorrelationAccel, zyCorrelationAccel, 
//                    zxCorrelationGyro, zyCorrelationGyro);
            sprintf(strbuffer, "Correlation Coefs: %8.3f, %8.3f\n", 
                    zxCorrelationAccel, zyCorrelationAccel);
            UART_PutString(strbuffer);
        }
    }
    else if (zeroed == 1) {
        int i;
        
        // Subtract oldest vals from average
        for (i = 0; i < 3; i++) {
            accelValAvg[i] -= accelValsBuf[bufPos][i] / AVG_BUF_LEN;
            gyroValAvg[i] -= gyroValsBuf[bufPos][i] / AVG_BUF_LEN;
        }
        
        // Get new vals
        readAccel(accelValsBuf[bufPos]);
        readGyro(gyroValsBuf[bufPos]);
        
        // Subtract means
        for (i = 0; i < 3; i++) {
            accelValsBuf[bufPos][i] -= accelOffset[i];
            gyroValsBuf[bufPos][i] -= gyroOffset[i];
        }
        
        // Decorrelate
        accelValsBuf[bufPos][0] -= accelValsBuf[bufPos][2] * zxCorrelationAccel;
        accelValsBuf[bufPos][1] -= accelValsBuf[bufPos][2] * zyCorrelationAccel;
        
        // Add new vals to average
        for (i = 0; i < 3; i++) {
            accelValAvg[i] += accelValsBuf[bufPos][i] / AVG_BUF_LEN;
            gyroValAvg[i] += gyroValsBuf[bufPos][i] / AVG_BUF_LEN;
        }
        
        // Increment buffer counter
        bufPos = (bufPos + 1) % AVG_BUF_LEN;
        
        // Read magnetomter
        readMag(magValAvg);
        
//        // Print averages
//        sprintf(strbuffer, "%2.1f,%2.1f,%2.1f,%2.1f,%2.1f,%2.1f\n", 
//                accelValAvg[0], accelValAvg[1], accelValAvg[2], 
//                gyroValAvg[0], gyroValAvg[1], gyroValAvg[2]);
//        sprintf(strbuffer, "%2.1f,%2.1f,%2.1f\n", 
//                accelValAvg[0], accelValAvg[1], accelValAvg[2]);
//        UART_PutString(strbuffer);
    }
}    

void Accel_getAccel(double *accelData) {
    accelData[0] = accelValAvg[0] * accelMgLsb;
    accelData[1] = accelValAvg[1] * accelMgLsb;
    accelData[2] = accelValAvg[2] * accelMgLsb;
}

void Accel_getMag(double *magData) {
    magData[0] = magValAvg[0] * magMgaussLsb;
    magData[1] = magValAvg[1] * magMgaussLsb;
    magData[2] = magValAvg[2] * magMgaussLsb;
}
    
    
void Accel_getGyro(double *gyroData) {
    gyroData[0] = gyroValAvg[0] * gyroDpsDigit;
    gyroData[1] = gyroValAvg[1] * gyroDpsDigit;
    gyroData[2] = gyroValAvg[2] * gyroDpsDigit;
}




/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

void readAccel(int *accelData) {
	// Read the accelerometer
	uint8_t buffer[6];
	readBuffer(XMTYPE, 
        I2C_AUTO_INCREMENT | LSM9DS0_REGISTER_OUT_X_L_A, 
		6, buffer);

	uint8_t xlo = buffer[0];
	int16_t xhi = buffer[1];
	uint8_t ylo = buffer[2];
	int16_t yhi = buffer[3];
	uint8_t zlo = buffer[4];
	int16_t zhi = buffer[5];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8; xhi |= xlo;
	yhi <<= 8; yhi |= ylo;
	zhi <<= 8; zhi |= zlo;
	
    accelData[0] = xhi;
    accelData[1] = yhi;
    accelData[2] = zhi;
//    sprintf(strbuffer, "%d, %d, %d\n", accelData[0], accelData[1], accelData[2]);
//    UART_PutString(strbuffer);
}

void readMag(int *magData) {
	// Read the magnetometer
	uint8_t buffer[6];
	readBuffer(XMTYPE, 
        I2C_AUTO_INCREMENT | LSM9DS0_REGISTER_OUT_X_L_M, 
		6, buffer);

	uint8_t xlo = buffer[0];
	int16_t xhi = buffer[1];
	uint8_t ylo = buffer[2];
	int16_t yhi = buffer[3];
	uint8_t zlo = buffer[4];
	int16_t zhi = buffer[5];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8; xhi |= xlo;
	yhi <<= 8; yhi |= ylo;
	zhi <<= 8; zhi |= zlo;
	
    magData[0] = xhi;
    magData[1] = yhi;
    magData[2] = zhi;
}

void readGyro(int *gyroData) {
	// Read gyro
	uint8_t buffer[6];
	readBuffer(GYROTYPE, 
		I2C_AUTO_INCREMENT | LSM9DS0_REGISTER_OUT_X_L_G, 
		6, buffer);

	uint8_t xlo = buffer[0];
	int16_t xhi = buffer[1];
	uint8_t ylo = buffer[2];
	int16_t yhi = buffer[3];
	uint8_t zlo = buffer[4];
	int16_t zhi = buffer[5];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8; xhi |= xlo;
	yhi <<= 8; yhi |= ylo;
	zhi <<= 8; zhi |= zlo;

	gyroData[0] = xhi;
	gyroData[1] = yhi;
	gyroData[2] = zhi;
}


static void write8(int type, uint8_t reg, uint8_t value) {
    uint8_t address;

    if (type == GYROTYPE) {
        address = LSM9DS0_ADDRESS_GYRO;
    } else {
        address = LSM9DS0_ADDRESS_ACCELMAG;
    }
    I2C_MasterSendStart(address, I2C_WRITE);
    I2C_MasterWriteByte(reg);
    I2C_MasterWriteByte(value);
    I2C_MasterSendStop();
}

static uint8_t read8(int type, uint8_t reg) {
    uint8_t value;

    readBuffer(type, reg, 1, &value);

    return value;
}

static uint8_t readBuffer(int type, uint8_t reg, uint8_t len, uint8_t *buffer) {
    uint8_t address;

    if (type == GYROTYPE) {
        address = LSM9DS0_ADDRESS_GYRO;
    } else {
        address = LSM9DS0_ADDRESS_ACCELMAG;
    }
    
    I2C_MasterSendStart(address, I2C_WRITE);
    I2C_MasterWriteByte(reg);
    
    I2C_MasterSendRestart(address, I2C_READ);
    int i;
    for (i = 0; i < len-1; i++) {
        buffer[i] = I2C_MasterReadByte(I2C_SEND_ACK);
    }
    buffer[len-1] = I2C_MasterReadByte(I2C_SEND_NACK);
    I2C_MasterSendStop();

    return len;
}