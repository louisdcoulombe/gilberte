//#define _NAMIKI_MOTOR  //For Namiki 22CL-3501PG80:1

#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

#include <fuzzy_table.h>
#include <PID_Beta6.h>

#include <MotorWheel.h>

 
irqISR(irq1,isr1);
MotorWheel wheel1(9,8,6,7,&irq1);	// Motor PWM:Pin9, DIR:Pin8, 
									// Encoder A:Pin6, B:Pin7

void setup() {    
	TCCR1B=TCCR1B&0xf8|0x01;	// Pin9,Pin10 PWM 31250Hz, Silent PWM

	//wheel1.PIDEnable(0.31,0.01,0,5);
	wheel1.PIDEnable(KC,TAUI,TAUD,SAMPLETIME);

	wheel1.setSpeedMMPS(100,DIR_ADVANCE);

	Serial.begin(19200);
}

void loop() {
	wheel1.PIDRegulate();
	if(millis()%500==0) {
		Serial.print("speedRPM> ");
		Serial.println(wheel1.getSpeedRPM(),DEC);
		Serial.print("MMPS  --> ");
		Serial.println(wheel1.getSpeedMMPS(),DEC);

        //wheel1.debugger();
	}
}

