// direction and current angle are CCW w.r.t global forward (in rads), 
// magnitude is from 0 to 1, angleMag is CCW
static void driveMagPhase(double direction, double magnitude, double angleMag, double currentAngle) {
    // angle of drive CCW w.r.t the axis formed by the front right and back left wheels
    double angle = fmod(direction - currentAngle + (M_PI/4), 2*M_PI);
    angle = angle < 0 ? angle + 2*M_PI : angle;
    
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
    
    // 0 is backward, 1 is forward
    uint8 angleChangeDir = 0;
    if (angleMag > 0) angleChangeDir = 0b0110;
    else angleChangeDir = 0b1001;

    // 1 is dominant, 0 is non-dominant
    uint8 angleDominant = ~(directionChangeDir ^ angleChangeDir);
    uint8 dominant = dirDominant & angleDominant;
    int dominantNum = (dominant & 8) ? 0 : (dominant & 4) ? 1 : (dominant & 2) ? 2 : 3;
    uint8 offDominant = ~dirDominant & angleDominant;
    int offDominantNum = (offDominant & 8) ? 0 : (offDominant & 4) ? 1 : (offDominant & 2) ? 2 : 3;    
    
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
    
    // Find dirs
    int dirs[4];
    dirs[dominantNum] = ((dominant & directionChangeDir) != 0) ? 1 : 0;
    dirs[offDominantNum] = ((offDominant & directionChangeDir) != 0) ? 1 : 0;
    dirs[(dominantNum + 2) % 4] = domPartnerMag > 0 ? dirs[dominantNum] : 1 - dirs[dominantNum];
    dirs[(offDominantNum + 2) % 4] = offDomPartnerMag > 0 ? dirs[offDominantNum] : 1 - dirs[offDominantNum];
    
    drive(dirs, mags);
}