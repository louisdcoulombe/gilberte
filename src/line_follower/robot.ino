
inline void setDirection(int left_dir, int right_dir)
{
    digitalWrite(pinLeftMotorDir, left_dir);
    digitalWrite(pinRightMotorDir, right_dir);
}

inline void setSpeed(int left_speed, int right_speed)
{
    analogWrite(pinLeftMotorPwm, left_speed);
    analogWrite(pinRightMotorPwm, right_speed);
}

inline void robotStop()
{
    setSpeed(0, 0);
    setDirection(LEFT_FORWARD, RIGHT_BACKWARD);
    resetPID();
    delay(500);
}

inline void robotForward(int speed)
{
    gPIDEnabled = true;
    gDirection = FORWARD; 
    setDirection(LEFT_FORWARD, RIGHT_FORWARD);
    setSpeed(speed, speed);
    delay(500);
}

inline void robotReverse(int speed)
{
    gPIDEnabled = true;
    gDirection = BACKWARD;
    setDirection(LEFT_FORWARD, RIGHT_BACKWARD);
    setSpeed(speed, speed);
}

void robotBackOnLine()
{
    
    setDirection(LEFT_BACKWARD, RIGHT_BACKWARD);
    setSpeed(25, 25);
    Serial.println("Backing...");
    do
    {
        gQtrrc.readLine(gSensorValues);
    } while (!OnTransversalLine());

    robotStop();    
}

void robotRotate(int speed)
{
    setDirection(LEFT_FORWARD, RIGHT_BACKWARD);
    setSpeed(155, 155);

    // Wait before we don't see the line 
    delay(1000);

    // Rotate until we see the line again
    while(1)
    {
        int position = gQtrrc.readLine(gSensorValues);
        Serial.println(position);
        if (position > 3000 && position < 4000)
            break;
    }

    // Close enough to stop rotating
    robotStop();
}
