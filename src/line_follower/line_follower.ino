#include <QTRSensors.h>

//////////////////////////////////
// Pin definition
//////////////////////////////////
#define pinLeftMotorPwm  3
#define pinLeftMotorDir  2
#define pinRightMotorPwm  11
#define pinRightMotorDir  12

//////////////////////////////////
// Constant definition
//////////////////////////////////
// number of sensors used
#define NUM_SENSORS   8     
// waits for 2500 microseconds for sensor outputs to go low
#define TIMEOUT       2500  
// emitter is controlled by digital pin 2
#define EMITTER_PIN   13 
// The middle of the sensor
#define SENSOR_MIDDLE 3500


#define LEFT_FORWARD   LOW
#define LEFT_BACKWARD  HIGH
#define RIGHT_FORWARD  HIGH
#define RIGHT_BACKWARD LOW

#define LEFT  1
#define RIGHT 0

//////////////////////////////////
// Sensor definition
//////////////////////////////////

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
bool gPIDEnabled = false;
long gLastTime = 0;
float gErrorSum = 0;
float gLastError = 0;
float KP = 0.125;
float KI = 0.4;
float KD = 0.18;

int gSpeed = 100;

int gLineLostTime = 0;

byte gRightDirection = RIGHT_FORWARD;
byte gLeftDirection = LEFT_FORWARD;

//////////////////////////////////
// Setup
//////////////////////////////////
void waitForInput(char* msg)
{
    Serial.println("Put robot completely on the line");
    while(Serial.available() <= 0);
    while(Serial.available() > 0)
        Serial.read();
}

void setup()
{
    // Définir la direction des pins
    pinMode(pinLeftMotorPwm, OUTPUT);
    pinMode(pinLeftMotorDir, OUTPUT);

    pinMode(pinRightMotorPwm, OUTPUT);
    pinMode(pinRightMotorDir, OUTPUT);

    // Définir les états par défaut
    digitalWrite(pinLeftMotorDir, LEFT_FORWARD);
    digitalWrite(pinRightMotorDir, RIGHT_FORWARD);

    Serial.begin(115200);

    waitForInput("Put robot completely on the line");
    for(int i = 0; i < 20; ++i)
    {
        gQtrrc.calibrate();
        delay(20);
    }

    waitForInput("Put robot completely off the line");
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

void robotStop()
{
    analogWrite(pinLeftMotorPwm, 0);
    analogWrite(pinRightMotorPwm, 0);
    gPIDEnabled = false;
    gErrorSum = 0;
    gLastError = 0;
}

void robotGo(int speed)
{
    gPIDEnabled = true;
    gRightDirection = RIGHT_FORWARD;
    gLeftDirection = LEFT_FORWARD;
    analogWrite(pinLeftMotorPwm, speed);
    digitalWrite(pinLeftMotorDir, gLeftDirection);
    analogWrite(pinRightMotorPwm, speed);
    digitalWrite(pinRightMotorDir, gRightDirection);
}

void robotRotate(int speed)
{
    gPIDEnabled = true;
    analogWrite(pinLeftMotorPwm, speed);
    digitalWrite(pinLeftMotorDir, LEFT_FORWARD);
    analogWrite(pinRightMotorPwm, speed);
    digitalWrite(pinRightMotorDir, RIGHT_BACKWARD);
    
    while(1)
    {
        int position = gQtrrc.readLine(gSensorValues);
        if (position > 3000 || position < 4000)
            break;
    }
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
    Serial.print("Position: "); Serial.println(position);

    if (OnTransversalLine())
    {
        Serial.println("OnTransversalLine");
        robotStop();
        return;
    }

    // PID
    float error = 3500 - position;
    float command = KP * error;
    gErrorSum += KI * error;
    command += KD * (error - gLastError);
    gLastError = error;

    command = constrain(command, -255, 255);
    Serial.print("command: "); Serial.println(command);

    int left_speed = gSpeed - command; 
    int right_speed = gSpeed + command;

    digitalWrite(pinLeftMotorDir, left_speed > 0 ? 
                                 LEFT_FORWARD : LEFT_BACKWARD);
    analogWrite(pinLeftMotorPwm, abs(left_speed));

    digitalWrite(pinRightMotorDir, right_speed > 0 ? 
                                RIGHT_FORWARD : RIGHT_BACKWARD);
    analogWrite(pinRightMotorPwm, abs(right_speed));

    Serial.print(left_speed); Serial.print(" "); Serial.println(right_speed);
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
                Serial.println(gSpeed);
                robotGo(gSpeed);
                break;
            case 'r':
                gSpeed = Serial.parseInt();
                Serial.println(gSpeed);
                robotRotate(gSpeed);
                break;
            case 'p':
                KP = Serial.parseFloat();
                break;
            case 'i':
                KI = Serial.parseFloat();
                break;
            case 'd':
                KD = Serial.parseFloat();
                break;
                
        }
    }
}
