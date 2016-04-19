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
int gLastError = 0;
float KP = 0.5;
float KD = 2;

int gSpeed = 50;

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
}

void robotGo(int speed)
{
  gPIDEnabled = true;
  analogWrite(pinLeftMotorPwm, speed);
  analogWrite(pinRightMotorPwm, speed);
}

void updatePid()
{
  if (!gPIDEnabled)
    return;

  int position = gQtrrc.readLine(gSensorValues);

  if (OnTransversalLine())
  {
    Serial.println("OnTransversalLine");
    robotStop();
  }

  // Check the error on the sensor and the side
  int error = SENSOR_MIDDLE - position;
  Serial.print("Error: "); Serial.println(error);

  // Calculate the needed correction
  float correction = KP * error + KD * (error - gLastError);
  int pwm_correction = (int)map(correction, -SENSOR_MIDDLE, +SENSOR_MIDDLE, -255, 255);
  gLastError = error;
  Serial.print("pwm_correction: "); Serial.println(pwm_correction);

  int left_speed = gSpeed;
  int right_speed = gSpeed;

  if (pwm_correction > 0)
  {
    // turn left
    left_speed += pwm_correction;
    right_speed -= pwm_correction;
  }
  if (pwm_correction < 0)
  {
    // turn right
    left_speed -= pwm_correction;
    right_speed += pwm_correction;
  }

  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);

  digitalWrite(pinLeftMotorDir, left_speed >= 0 ? LEFT_FORWARD : LEFT_BACKWARD);
  analogWrite(pinLeftMotorPwm, abs(left_speed) + 40);
  digitalWrite(pinRightMotorDir, right_speed >= 0 ? RIGHT_FORWARD : RIGHT_BACKWARD);
  analogWrite(pinRightMotorPwm, abs(right_speed) + 40);

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
    }
  }
}