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