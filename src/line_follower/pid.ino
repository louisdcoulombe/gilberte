
void resetPID()
{
    gPIDEnabled = false;
    gErrorSum = 0;
    gLastError = 0;
}

bool TransversaLineLogic()
{
    bool isTransversal = OnTransversalLine();

    // Calculate time between two lines
    long last_line_time = millis() - gLastTransversalTime;
    gLastTransversalTime = millis();

    // Gap between home line, change state and continue
    if (!isTransversal && gHomeTransversalState == HOME_FIRST_LINE)
    {
        Serial.println("Found gap");
        gHomeTransversalState = HOME_GAP_TIME;
        return true;
    }

    // Every other states needs to be on the line
    if (!isTransversal)
        return true;


    // On go next, stop at the first line we see
    if (gTransversalState == STATE_GO_NEXT)
    {
        Serial.println("Found next, stopping");
        robotStop();
        gLastTransversalTime = 0;
        gHomeTransversalState = HOME_NONE;
        return false;
    }


    // Gap expired, reset state
    if (gHomeTransversalState == HOME_FIRST_LINE && last_line_time > HOME_TIME_MAX)
    {
        gHomeTransversalState = HOME_NONE; 
        Serial.println("Home exipred");
        return true;
    }

    // First time we see the line, change state and continue
    if (gHomeTransversalState == HOME_NONE)
    {
        Serial.println("Found first line");
        gHomeTransversalState = HOME_FIRST_LINE;
        return true;
    }

    // Second home line, stop there
    if (gHomeTransversalState == HOME_GAP_TIME && last_line_time <= HOME_TIME_MAX)
    {
        Serial.println("Found Home, stopping");
        gHomeTransversalState = HOME_NONE;
        robotStop();
        return false;
    }

    return true;
}

bool OnTransversalLine()
{
    for(int i = 0; i < NUM_SENSORS; ++i)
    {
        if (gSensorValues[i] < gSensorThreshold)
            return false;
    }
    return true;
}
 

void updatePid()
{
    if (!gPIDEnabled)
        return;

    long dt = millis() - gLastTime;
    if (dt < 10)
        return;

    gLastTime = millis();  

    int position = gQtrrc.readLine(gSensorValues);
    //Serial.print("Position: "); Serial.println(position);

    // Check for line, on false we stop
    if (!TransversaLineLogic())
        return;

    // PID
    float error = 3500 - position;
    
    float command;
    if (gDirection == FORWARD)
    {
        command = FWD_KP * error;
        gErrorSum += FWD_KI * error;
        command += gErrorSum;
        command += FWD_KD * (error - gLastError);
        gLastError = error;
    }
    else
    {
        command = BWD_KP * error;
        gErrorSum += BWD_KI * error;
        command += gErrorSum;
        command += BWD_KD * (error - gLastError);
        gLastError = error;
    }

    command = constrain(command, -255, 255);
    //Serial.print("command: "); Serial.println(command);

    int left_speed = constrain(gSpeed - command, -255, 255);
    int right_speed = constrain(gSpeed + command, -255, 255);

    int left_dir, right_dir;
    if (gDirection == FORWARD)
    {
        left_dir = left_speed > 0 ? LEFT_FORWARD : LEFT_BACKWARD;
        right_dir = right_speed > 0 ? RIGHT_FORWARD : RIGHT_BACKWARD;
    }
    else
    {
        left_dir = left_speed > 0 ? LEFT_BACKWARD : LEFT_FORWARD;
        right_dir = right_speed > 0 ? RIGHT_BACKWARD : RIGHT_FORWARD;
    }
    
    setDirection(left_dir, right_dir);
    setSpeed(abs(left_speed), abs(right_speed));
    //Serial.print(left_speed); Serial.print(" "); Serial.println(right_speed);
}


