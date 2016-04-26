#include <QTRSensors.h>

//////////////////////////////////
// Pin definition
//////////////////////////////////
#define pinLeftMotorPwm  3
#define pinLeftMotorDir  2
#define pinRightMotorPwm  11
#define pinRightMotorDir  12

//////////////////////////////////
// Sensor definition
//////////////////////////////////

// number of sensors used
#define NUM_SENSORS   8     
// waits for 2500 microseconds for sensor outputs to go low
#define TIMEOUT       2500
// emitter is controlled by digital pin 2
#define EMITTER_PIN   13 
// The middle of the sensor
#define SENSOR_MIDDLE 3500
// sensors 0 through 7 are connected to digital pins 3 through 10
QTRSensorsRC gQtrrc((unsigned char[]) {
        A5, 9, 8, 10, A0, A1, A2, A4
        },
        NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int gSensorValues[NUM_SENSORS];
unsigned int gSensorThreshold = 0;

//////////////////////////////////
// PID definition
//////////////////////////////////
#define FORWARD 1
#define BACKWARD 0
#define LEFT_FORWARD   LOW
#define LEFT_BACKWARD  HIGH
#define RIGHT_FORWARD  HIGH
#define RIGHT_BACKWARD LOW

#define LEFT  1
#define RIGHT 0

bool gPIDEnabled = false;
long gLastTime = 0;
float gErrorSum = 0;
float gLastError = 0;
float FWD_KP = 0.125;
float FWD_KI = 0.0;
float FWD_KD = 0.18;

float BWD_KP = 0.001;
float BWD_KI = 0.0;
float BWD_KD = 0.01;
int gSpeed = 100;

int gLineLostTime = 0;
int gDirection = FORWARD;

//////////////////////////////////
// Transversal lines
//////////////////////////////////
#define STATE_GO_HOME 0
#define STATE_GO_NEXT 1
byte gTransversalState = STATE_GO_NEXT;

long gLastTransversalTime = 0;

#define HOME_NONE 0
#define HOME_FIRST_LINE 1
#define HOME_GAP_TIME 2
#define HOME_SECOND_LINE 3
#define HOME_TIME_MAX 1000
byte gHomeTransversalState = HOME_NONE;
//////////////////////////////////
// Setup
//////////////////////////////////
void waitForInput()
{
    while(Serial.available() <= 0);
    while(Serial.available() > 0)
        Serial.read();
}

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

void calibrate_sensor()
{
    Serial.println("Put robot on the line"); 
    waitForInput();
    for(int i = 0; i < 20; ++i)
    {
        gQtrrc.calibrate();
        delay(20);
    }

    Serial.println("Put robot off the line"); 
    waitForInput();
    for(int i = 0; i < 20; ++i)
    {
        gQtrrc.calibrate();
        delay(20);
    }

    int min = 0;
    int max = 65535;
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        if (gQtrrc.calibratedMinimumOn[i] > min)
            min = gQtrrc.calibratedMinimumOn[i];
        if (gQtrrc.calibratedMaximumOn[i] < max)
            max = gQtrrc.calibratedMaximumOn[i];
    }

    gSensorThreshold = min + (max - min)/2;
    Serial.print("Threshold: "); Serial.println(gSensorThreshold);  
}

void setup()
{
    Serial.begin(115200);

    // Définir la direction des pins
    pinMode(pinLeftMotorPwm, OUTPUT);
    pinMode(pinLeftMotorDir, OUTPUT);
    pinMode(pinRightMotorPwm, OUTPUT);
    pinMode(pinRightMotorDir, OUTPUT);

    // Calibrate IR sensor
    calibrate_sensor();
}

//////////////////////////////////
// Main loop
//////////////////////////////////

bool OnTransversalLine()
{
    for(int i = 0; i < NUM_SENSORS; ++i)
    {
        if (gSensorValues[i] < gSensorThreshold)
            return false;
    }
    return true;
}

void resetPID()
{
    gPIDEnabled = false;
    gErrorSum = 0;
    gLastError = 0;
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
    //gPIDEnabled = true;
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

void loop()
{
    updatePid();

    if (Serial.available() > 0)
    {
        switch(Serial.read())
        {
            case 's':
                robotStop();
                break;
            case 'g':
                gSpeed = Serial.parseInt();
                gTransversalState = STATE_GO_HOME;
                Serial.println(gSpeed);
                robotForward(gSpeed);
                break;
            case 'n':
                gSpeed = Serial.parseInt();
                gTransversalState = STATE_GO_NEXT;
                Serial.println(gSpeed);
                robotForward(gSpeed);
                break;
            case 'r':
                gSpeed = Serial.parseInt();
                gTransversalState = STATE_GO_HOME;
                Serial.println(gSpeed);
                robotReverse(gSpeed);
                break;
            case 'p':
                gSpeed = Serial.parseInt();
                gTransversalState = STATE_GO_NEXT;
                Serial.println(gSpeed);
                robotReverse(gSpeed);
                break;
            // Configuration
            case 'o':
                FWD_KP = Serial.parseFloat();
                Serial.print("FWD_KP: "); Serial.println(FWD_KP);
                break;
            case 'i':
                FWD_KI = Serial.parseFloat();
                Serial.print("FWD_KI: "); Serial.println(FWD_KI);
                break;
            case 'd':
                FWD_KD = Serial.parseFloat();
                Serial.print("FWD_KD: "); Serial.println(FWD_KD);
                break;

        }
    }
}
