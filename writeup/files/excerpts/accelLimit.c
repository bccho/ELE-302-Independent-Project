if (accelerationLimitEnabled) {
    static double oldMag;
    double setMag = (fabs(magnitude-oldMag) > accelLimit) ? (magnitude - oldMag)/fabs(magnitude-oldMag) * accelLimit + oldMag : magnitude;
    driveMagPhase(heading, setMag, rotation, enableGyro == 1 ? currentAngle : 0);
    oldMag = setMag;
}
else {
    driveMagPhase(heading, magnitude, rotation, enableGyro == 1 ? currentAngle : 0);
}