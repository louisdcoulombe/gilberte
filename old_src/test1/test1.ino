#include <IR.h>
#include <SONAR.h>

 #include <EEPROM.h>
#include <R2WD.h>

#include <fuzzy_table.h>
#include <PID_Beta6.h>

#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
/*********************************************/

//Internal wheelspan : 200mm
//External wheelspan : 280mm
//WhellDiameter : 105mm
//GearRatio : 1:64
#define TANKWHEELDIA 105
#define TANKWHEELSPAN 460

irqISR(irq1,isr1);
MotorWheel wheel1(3,2,4,5,&irq1,REDUCTION_RATIO,int(106*PI));

irqISR(irq2,isr2);
MotorWheel wheel2(11,12,6,7,&irq2,REDUCTION_RATIO,int(106*PI));

R2WD _2WD(&wheel1,&wheel2,TANKWHEELSPAN);
unsigned int speedMMPS=300;
char FirstPass;
void setup() {
    Serial.begin(19200);
    //TCCR0B=TCCR0B&0xf8|0x01;    // warning!! it will change millis()
    TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
    TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz
    _2WD.PIDEnable(1,0.02,0.5,2);
    FirstPass = 1;
  
  }

void loop() {
  _2WD.setCarAdvance(600);
  _2WD.delayMS(5000);
  _2WD.wheelLeftSetSpeedMMPS(150,DIR_ADVANCE);
  _2WD.wheelRightSetSpeedMMPS(250,DIR_BACKOFF);
  _2WD.delayMS(5000);
  _2WD.setCarStop();
  _2WD.delayMS(1000);
  //_2WD.PIDRegulate();
   
}
