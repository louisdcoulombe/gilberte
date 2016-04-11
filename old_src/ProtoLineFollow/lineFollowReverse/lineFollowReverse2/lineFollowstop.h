//robotState
typedef enum  {
  STATEFOLLOW,   //follow line
  STATETARGETFOUND,  //Target is Found need to stop
  STATETARGETPASSED, //Target Passed, need to backup
  STATEONTARGET,
  STATETARGETPOS,
  STATERIGHTON,
  STATEFOLLOWNOSTOP
}robotState;

#define NOERROR 0
#define ERRORCALIBRATION 0xEC
#define CALLOOPREAD 30
#define CALIBRATIONSPEED 125
#define ANGLETORUNCAL  PI/6

#define LINEREFRESH 25
#define WHEELREFRESH 2

//Internal wheelspan : 200mm
//External wheelspan : 280mm
//WhellDiameter : 106mm
//GearRatio : 1:64
#define TANKWHEELDIA 106
#define TANKWHEELSPAN 450

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   13     // emitter is controlled by digital pin 2
#define QTRTHRESHOLD  800
