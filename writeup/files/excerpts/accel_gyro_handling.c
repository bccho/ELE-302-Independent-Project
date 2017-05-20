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