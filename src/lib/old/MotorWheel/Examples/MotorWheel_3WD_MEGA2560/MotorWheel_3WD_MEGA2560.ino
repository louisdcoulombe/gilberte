//#define _NAMIKI_MOTOR  //For Namiki 22CL-3501PG80:1

#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <fuzzy_table.h>
#include <PID_Beta6.h>

#include <MotorWheel.h>
#include <Omni3WD.h>
#include <Omni4WD.h>
#include <R2WD.h>

#include <SONAR.h>


irqISR(irq1,isr1);
MotorWheel wheel1(5,4,2,3,&irq1);	// Motor PWM:Pin5, DIR:Pin4, Encoder A:Pin2, B:Pin3

irqISR(irq2,isr2);
MotorWheel wheel2(6,7,10,11,&irq2);    // Motor PWM:Pin6, DIR:Pin7, Encoder A:Pin10, B:Pin11

irqISR(irq3,isr3);
MotorWheel wheel3(9,8,12,13,&irq3);    // Motor PWM:Pin9, DIR:Pin8, Encoder A:Pin12, B:Pin13

SONAR s11=SONAR(0x11);

void setup() {    
	//TCCR1B=TCCR1B&0xf8|0x01;	// Atmega168/328, Pin9,Pin10 PWM 31250Hz, Silent PWM
        TCCR3B=TCCR3B&0xf8|0x01;        // Atmega1280/2560, Pin5-->PWM, Timer3 ChannelA
        TCCR4B=TCCR4B&0xf8|0x01;        // Atmega1280/2560, Pin6-->PWM, Timer4 ChannelA
        TCCR2B=TCCR2B&0xf8|0x01;        // Atmega1280/2560, Pin9-->PWM, Timer2 ChannelB

	wheel1.setSpeedMMPS(50,DIR_ADVANCE);
	wheel1.PIDEnable(KC,TAUI,TAUD,10);
        wheel2.setSpeedMMPS(50,DIR_ADVANCE);
        wheel2.PIDEnable(KC,TAUI,TAUD,10);
        wheel3.setSpeedMMPS(50,DIR_ADVANCE);
        wheel3.PIDEnable(KC,TAUI,TAUD,10);
        
	//Serial.begin(19200);
        //SONAR::init(54);   // Digital Pin54 (Analog0) as RS485 Control Pin
}

void loop() {
	wheel1.PIDRegulate();
        wheel2.PIDRegulate();
        wheel3.PIDRegulate();
	if(millis()%500==0) {
		Serial.print("Wheel1 speedRPM> ");
		Serial.println(wheel1.getSpeedRPM(),DEC);
                Serial.print("Wheel2 speedRPM> ");
		Serial.println(wheel2.getSpeedRPM(),DEC);
                Serial.print("Wheel3 speedRPM> ");
		Serial.println(wheel3.getSpeedRPM(),DEC);             
        //wheel1.debugger();
	}
        if(millis()%50==0) {
                s11.getDist();
                s11.showDat();
                s11.trigger();
        }
}

