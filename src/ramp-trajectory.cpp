
static const float CHORALE_RPM = 40.0f;
static const float TREMOLO_RPM = 400.0f;
static const float RISE_TIME_SEC = 4.0f; // 0 -> TREMOLO
static const float FALL_TIME_SEC = 4.0f; // TREMOLO -> 0

static const float RISE_SLOPE_RPM_PER_SEC = (TREMOLO_RPM - 0.0f) / RISE_TIME_SEC;
static const float FALL_SLOPE_RPM_PER_SEC = (TREMOLO_RPM - 0.0f) / FALL_TIME_SEC;



void rampTrajectoryCommand(SpeedCommand cmd){
    //set speed goal
    //set starting speed
    //wake task to start ramping
}



void rampTask(){

    //for(;;)
    //wait for unblock
    //while(currentRPM != jj)
}

