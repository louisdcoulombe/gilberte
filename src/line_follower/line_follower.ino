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
    Serial.begin(9600);

    // Define pin direction
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
void loop()
{
    // Follow the line
    updatePid();

    // Check for command input
    if (Serial.available() > 0)
    {
        switch(Serial.read())
        {
            // Stop the robot
            case 's':
                robotStop();
                break;
            // Go home
            case 'g':
                gSpeed = Serial.parseInt();
                gTransversalState = STATE_GO_HOME;
                Serial.println(gSpeed);
                robotForward(gSpeed);
                break;
            // Go to next line
            case 'n':
                gSpeed = Serial.parseInt();
                gTransversalState = STATE_GO_NEXT;
                Serial.println(gSpeed);
                robotForward(gSpeed);
                break;
            // Reverse to home
            case 'r':
                gSpeed = Serial.parseInt();
                gTransversalState = STATE_GO_HOME;
                Serial.println(gSpeed);
                robotReverse(gSpeed);
                break;
            // Go to previous line
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
