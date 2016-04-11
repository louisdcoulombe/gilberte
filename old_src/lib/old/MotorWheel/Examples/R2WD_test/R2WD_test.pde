//#define _NAMIKI_MOTOR  //For Namiki 22CL-3501PG80:1

#include <fuzzy_table.h>
#include <PID_Beta6.h>

#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

#include <MotorWheel.h>
#include <R2WD.h>
#include <Omni3WD.h>
#include <Omni4WD.h>


/*

	Wheel1 ||-----|| Wheel2	

 */

irqISR(irq1,isr1);
MotorWheel wheel1(9,8,4,5,&irq1,REDUCTION_RATIO,int(144*PI));

irqISR(irq2,isr2);
MotorWheel wheel2(10,11,6,7,&irq2,REDUCTION_RATIO,int(144*PI));

R2WD _2WD(&wheel1,&wheel2,WHEELSPAN);

void setup() {
	//TCCR0B=TCCR0B&0xf8|0x01;    // warning!! it will change millis()
	TCCR1B=TCCR1B&0xf8|0x01;      // Pin9,Pin10 PWM 31250Hz
	//TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz

	_2WD.PIDEnable(0.26,0.01,0,10);
}

void loop() {
	_2WD.demoActions(100,5000);
  
	/*
		_2WD.setCarAdvance(0);
		_2WD.setCarSpeedMMPS(100,500);
		_2WD.delayMS(5000);
		_2WD.setCarSlow2Stop(500);

		_2WD.setCarBackoff(0);
		_2WD.setCarSpeedMMPS(100,500);
		_2WD.delayMS(5000);
		_2WD.setCarSlow2Stop(500);
	 */  
}

