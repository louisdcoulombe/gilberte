
void setDirection(int left_dir, int right_dir)
{
    digitalWrite(pinLeftMotorDir, left_dir);
    digitalWrite(pinRightMotorDir, right_dir);
}

void setSpeed(int left_speed, int right_speed)
{
    analogWrite(pinLeftMotorPwm, left_speed);
    analogWrite(pinRightMotorPwm, right_speed);
}

void robotStop()
{
    setSpeed(0, 0);
    resetPID();
}

void robotForward(int speed)
{
    gPIDEnabled = true;
    gDirection = FORWARD; 
    setDirection(LEFT_FORWARD, RIGHT_FORWARD);
    setSpeed(speed, speed);
}

void robotReverse(int speed)
{
    gPIDEnabled = true;
    gDirection = BACKWARD;
    setDirection(LEFT_BACKWARD, RIGHT_BACKWARD);
    setSpeed(speed, speed);
}

void robotRotate(int speed)
{
    gPIDEnabled = true;
    setDirection(LEFT_FORWARD, RIGHT_BACKWARD);
    setSpeed(speed, speed);

    // Wait before we don't see the line 
    delay(500);

    // Rotate until we see the line again
    while(1)
    {
        int position = gQtrrc.readLine(gSensorValues);
        if (position > 3000 || position < 4000)
            break;
    }

    // Close enough to stop rotating
    robotStop();
}
