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

	Wheel3	//		\\ Wheel2


				==
			  Wheel1

 */

irqISR(irq1,isr1);
MotorWheel wheel1(9,8,6,7,&irq1);

irqISR(irq2,isr2);
MotorWheel wheel2(10,11,12,13,&irq2);

irqISR(irq3,isr3);
MotorWheel wheel3(3,2,4,5,&irq3);
//MotorWheel wheel3(5,4,2,3,&irq3);

Omni3WD Omni(&wheel1,&wheel2,&wheel3);

void setup() {
	TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
	TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz

	Omni.PIDEnable(0.26,0.02,0,5);
}

void loop() {
	Omni.demoActions(100,5000,1000,false);

	/*
	Omni.setCarLeft(0);
	Omni.setCarSpeedMMPS(300,1000);
	Omni.delayMS(10000,true);
	Omni.setCarSlow2Stop(1000);
	Omni.setCarRight(0);
	Omni.setCarSpeedMMPS(100,1000);
	Omni.delayMS(10000,true);
	Omni.setCarSlow2Stop(1000);    
     */
    /*
	Omni.setCarLeft(100);
	for(int i=0;i<1000;++i) {
		Omni.PIDRegulate();
		delay(10);
	}
	Omni.setCarRight(100);
	for(int i=0;i<1000;++i) {
		Omni.PIDRegulate();
		delay(10);
	}
    */
}

