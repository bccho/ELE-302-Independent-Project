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

             // Calculate correlation coefficients
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
    }
}    