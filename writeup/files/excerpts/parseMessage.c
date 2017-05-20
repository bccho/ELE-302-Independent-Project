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