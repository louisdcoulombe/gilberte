
#include "lineFollowstop.h"
#include <QTRSensors.h>

#include <IR.h>
#include <SONAR.h>

#include <EEPROM.h>
#include <R2WD.h>

#include <fuzzy_table.h>
#include <PID_Beta6.h>

#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

#include <TimerOne.h>

/*********************************************/

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  A5, 9, 8, 10, A0, A1, A2, A4
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

irqISR(irq1, isr1);
MotorWheel wheel1(3, 2, 4, 5, &irq1, REDUCTION_RATIO, int(TANKWHEELDIA*PI));
irqISR(irq2, isr2);
MotorWheel wheel2(11, 12, 6, 7, &irq2, REDUCTION_RATIO, int(TANKWHEELDIA*PI));

R2WD _2WD(&wheel1, &wheel2, TANKWHEELSPAN);
unsigned int speedMMPS = 200;

// Proportional Control loop vars
float error = 0;
float PV = 0 ; // Process Variable value calculated to adjust speeds and keep on line
float kp = 0.7;  // This is the Proportional value. Tune this value to affect follow_line performance
unsigned int* valuePtr;
unsigned int mlspeed, mrspeed;
unsigned int position;
unsigned long wheelPidTime;
unsigned long linePidTime;

robotState myState;
robotState myNextState;
char charRecv;
void istTimer1()
{
  //Serial.print("TISR");
  _2WD.PIDRegulate();
}

void setup() {
  int i;
  error = NOERROR;
  pinMode(A3, INPUT);
  Serial.begin(9600);
  //TCCR0B=TCCR0B&0xf8|0x01;    // warning!! it will change millis()
  //TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
  TCCR2B = TCCR2B & 0xf8 | 0x01; // Pin3,Pin11 PWM 31250Hz
  speedMMPS = 100;
  _2WD.switchMotors();   // Swap motor robot to swap front and rear
  _2WD.PIDEnable(1, 0.02, 0.15, WHEELREFRESH);
  Serial.println("Placing Bot");

  Timer1.initialize(WHEELREFRESH * 1000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt( istTimer1 ); // attach the service routine here
  
  // set the robot on the line, turn on the power,
  // then move your hand away before it begins moving.
  //delay(2000);
  Serial.println("Calibration will Start");
  // calibrate line sensor; determines min/max range of sensed values for the current course
  // read calibrated sensor values and obtain a measure of the line position from 0 to 7000
  _2WD.setCarRotateLeftAngle(CALIBRATIONSPEED, ANGLETORUNCAL);
  Serial.println("Calibration:first left");
  
  _2WD.setCarSlow2Stop(200);

  _2WD.setCarRotateRight(CALIBRATIONSPEED);
  for (i = 0; i < CALLOOPREAD; i++)
  {
    _2WD.delayMS(20);
    qtrrc.calibrate();

  }
  _2WD.setCarSlow2Stop(100);
  _2WD.setCarRotateLeft(CALIBRATIONSPEED);
  for (i = 0; i < CALLOOPREAD; i++)
  {
    _2WD.delayMS(20);
    qtrrc.calibrate();

  }
  _2WD.setCarRotateRightAngle(CALIBRATIONSPEED, ANGLETORUNCAL);
  _2WD.setCarSlow2Stop(100);
  _2WD.delayMS(2000);
  //_2WD.PIDRegulate();
  //line_position = qtrrc.readLine(sensorValues);

  // read the value of only a single sensor to see the line.
  // when the value is greater than 200 the sensor sees the line.
  if (qtrrc.calibratedMinimumOn[3] < 200) {
    error = ERRORCALIBRATION;
  }

  // delay as indicator setup and calibration is complete

  Serial.print("Calibration Done, Error Value:");
  //_2WD.PIDRegulate();
  Serial.println(error, HEX);
  Serial.flush();
  //_2WD.setCarAdvance(100);
  myState = STATEFOLLOW;

  wheelPidTime = millis();
  linePidTime = wheelPidTime;

}


void loop() {
  unsigned long now;

  while (Serial.available() > 0) {
    charRecv = Serial.read();
  }
  now = millis();

  if ( now - linePidTime >= LINEREFRESH)
  {
    linePidTime = millis();
    position = qtrrc.readLine(sensorValues);
    myState = myNextState;

    switch (charRecv) {
      //Force stop...
      case 's':
        myNextState = STATEIDLE;
        break;
      //Force 180deg turn...
      case 't':
        _2WD.setCarRotateLeftAngle(CALIBRATIONSPEED, PI/2);
        _2WD.setCarStop();
        myNextState = STATEFINDLINEONTURN;
        break;
      //go home no stop on target
      case 'h':
        myNextState = STATEFOLLOWNOSTOP;
        break;
      //going next target
      case 'n':
        //slightly advance robot
        _2WD.setCarAdvance(0);
        _2WD.setCarSpeedMMPS(CALIBRATIONSPEED, 1000);
        myNextState = STATEFOLLOW;
        break;
      default:
      break;
      
    }

    charRecv = 0;

    switch (myState) {
      case STATEIDLE:
        _2WD.setCarStop();
        break;

      case STATEFOLLOW:
        follow_line(position);
        if (sensorValues[0] >= QTRTHRESHOLD && sensorValues[NUM_SENSORS - 1] >= QTRTHRESHOLD) {
          myNextState = STATETARGETFOUND;
        }
        break;

      case STATETARGETFOUND:
        //_2WD.setCarSlow2Stop(250);
        _2WD.setCarStop();
        //if we pass sensor go back
        if (sensorValues[0] < QTRTHRESHOLD || sensorValues[NUM_SENSORS - 1] < QTRTHRESHOLD)
          myNextState = STATETARGETPASSED;
        else
          myNextState = STATEONTARGET;
        break;

      case STATETARGETPASSED:
        _2WD.setCarBackoff(CALIBRATIONSPEED);
        if (sensorValues[0] >= QTRTHRESHOLD && sensorValues[NUM_SENSORS - 1] >= QTRTHRESHOLD) {
          myNextState = STATEONTARGET;
        }
        break;

      case STATEONTARGET:
        //we need to return to before start of the line
        _2WD.setCarBackoff(CALIBRATIONSPEED);
        if (sensorValues[0] < QTRTHRESHOLD && sensorValues[NUM_SENSORS - 1] < QTRTHRESHOLD)
        {
          //wait to be sure we pass the line;
          _2WD.delayMS(500);
          myNextState = STATETARGETPOS;
        }
        break;

      case STATETARGETPOS:
        _2WD.setCarAdvance(CALIBRATIONSPEED / 2);
        if (sensorValues[0] >= QTRTHRESHOLD && sensorValues[NUM_SENSORS - 1] >= QTRTHRESHOLD) {
          myNextState = STATEIDLE;
        }
        break;

      case STATEFOLLOWNOSTOP:
        follow_line(position);
        break;
      
      case STATEFINDLINEONTURN:
        _2WD.setCarRotateLeft(CALIBRATIONSPEED);
        if (sensorValues[3] >= QTRTHRESHOLD) {
          myNextState = STATEIDLE;
        }
        break;
      default:
        _2WD.setCarStop();
        //si aucune condition n'est vraie,  alors on exécute cette partie
        // il n'est pas obligatoire d'avoir la partie "default:"
    }

  }

}
void follow_line(unsigned int line_position) //follow the line
{
  const int MAXLINE = 7000;
  const int MINLINE = 0;
  unsigned int PIDTURNCORRECTIONSPEED = 200;
  unsigned int PIDMAXCORRSPEED = 300;
  unsigned int PIDTARGETSPEED = 300;
  float PIDANGLECORR = PI / 4;
  int delta;
  // 0 is far Right sensor while 5 (7000 return) is far Left sensor
  //mlspeed = _2WD.wheelLeftGetSpeedMMPS();
  //mrspeed = _2WD.wheelRightGetSpeedMMPS();
  switch (line_position)
  {

    // Line has moved off the left edge of sensor
    // This will make it turn fast to the left
    case MAXLINE:
      //Serial.println("Hit MAXLINE");
      _2WD.setCarRotateLeft(PIDTURNCORRECTIONSPEED);
      //_2WD.delayMS(500);

      break;

    // Line had moved off the right edge of sensor
    // This will make it turn fast to the right
    case MINLINE:
      //Serial.println("Hit MINLINE");
      _2WD.setCarRotateRight(PIDTURNCORRECTIONSPEED);
      //_2WD.delayMS(500);

      break;
    // The line is still within the sensors.
    // This will calculate adjusting speed to keep the line in center.
    default:

      error = ((float)line_position - (MAXLINE / 2)) / (MAXLINE / 2); // 3500 is center measure of 7000 far left and 0 on far right

      _2WD.setCarAdvance(PIDTARGETSPEED);
      // This sets the motor speed based on a proportional only formula.
      // kp is the floating-point proportional constant you need to tune.
      // Maybe start with a kp value around 1.0, tuned in declared Proportional Control loop vars at the top of this code.
      // Note that it's very important you get your signs right, or else the
      // control loop will be unstable.

      // calculate the new Process Variable
      // this is the value that will be used to alter the speeds
      PV = kp * error;
      delta = PV * PIDMAXCORRSPEED;

      if ( PV <= 0 )
      {
        mlspeed = PIDTARGETSPEED - delta;
        mrspeed = PIDTARGETSPEED + delta;
      }
      else
      {
        mlspeed = PIDTARGETSPEED - delta;
        mrspeed = PIDTARGETSPEED + delta;
      }

      // if error negative
      _2WD.wheelLeftSetSpeedMMPS(mlspeed, DIR_ADVANCE);
      _2WD.wheelRightSetSpeedMMPS(mrspeed, DIR_BACKOFF);
      break;
  }

} // end follow_line



