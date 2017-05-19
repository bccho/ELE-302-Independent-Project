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