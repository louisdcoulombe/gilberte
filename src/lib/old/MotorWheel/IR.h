
#ifndef IR_H
#define IR_H

#if defined (__AVR_ATmega8__) || defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__) 
#define ANALOGMAX 5
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
#define ANALOGMAX 15
#endif

#ifndef PIN_UNDEFINED
#define PIN_UNDEFINED 255
#endif

class IR {
public:
	IR(unsigned char analog);
	unsigned int getDist();
	unsigned int getLastDist() const;

	unsigned char setPin(unsigned char analog);
	unsigned char getPin() const;

private:
	IR();
	unsigned char _analogPin;	// analog pin only
	unsigned int _lastDist;		// cm
};

IR::IR(unsigned char analog) {
	setPin(analog);
}
unsigned int IR::getDist() {
	if(getPin()==PIN_UNDEFINED) return 0xffff;
	lastDist=6762/(analog(_analogPin)-9)-4;
	if(lastDist<10 || lastDist>80) lastDist=0xffff;		// out of range
	return getLastDist();
}
unsigned int IR::getLastDist() const {
	return _lastDist;
}
unsigned char IR::setPin(unsigned char analog) {
	// assert();
	_analogPin=(analog<ANALOGMAX?analog:PIN_UNDEFINED);
	return getPin();
}
unsigned char IR::getPin() const {
	return _analogPin;
}

#endif



