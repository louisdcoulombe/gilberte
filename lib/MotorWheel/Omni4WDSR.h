#ifndef Omni4WDSR_H
#define Omni4WDSR_H

#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

#include <fuzzy_table.h>
#include <PID_Beta6.h>

#include <MotorWheel.h>
#include <Omni4WD.h>
#include <Omni4WDAction.h>

#include <SONAR.h>
#include <IR.h>


/*				  headerSONAR
 *				  headerIR

            \                    /
   wheel1   \                    /   wheel4
   Left     \                    /   Right
    
    leftSONAR	  TOP VIEW		rightSONAR
    leftIR						rightIR
    
            /                    \
   wheel2   /                    \   wheel3
   Right    /                    \   Left
				  backSONAR
				  headerIR
 */


/*
 *	class SONAR
 *	class IR
 *
 *	class PID
 *	class Motor
 *	class GearedMotor
 *	class MotorWheel
 *	class Omni4WD
 *	class Omni4WDSR
 *
 *	class Omni4WDAction
 *
 *	class COMM
 *	class SlaveOmni4WD
 *
 */


#ifndef Baudrate
#define Baudrate 19200
#endif

//#ifndef DEBUG
//#define DEBUG
//#endif

#ifdef DEBUG
#define debug() { \
	Serial.begin(Baudrate); \
	Serial.println(__func__);}
#else
#define debug() {}
#endif


class Omni4WDSR: public Omni4WD {
public:
	Omni4WDSR(MotorWheel* wheelUL, MotorWheel* wheelLL,
		MotorWheel* wheelLR, MotorWheel* wheelUR,unsigned int wheelspan=WHEELSPAN);
	unsigned char init();
	unsigned char initOmni4WD();

	unsigned char attachSONAR(SONAR* left,SONAR* back,SONAR* right,SONAR* head);
	unsigned char attachIR(IR* left,IR* back,IR* right,IR* head);

	unsigned char getMode() const;
	unsigned char setMode(unsigned char mode);

	unsigned char getSensorMode() const;		// sensors update mode, auto, manual or disable
	unsigned char setSensorMode(unsigned char mode);

	int getDefaultSpeed() const;
	int setDefaultSpeed(int speed);

	unsigned char getSafeDist() const;
	unsigned char setSafeDist(unsigned char dist);
	
	enum {MODE_UNKNOWN,MODE_AUTO,MODE_INTERACTIVE,MODE_SEQUENTIAL,MODE_STOP,MODE_SLEEP};
	enum {POS_LEFT,POS_BACK,POS_RIGHT,POS_HEAD};
	//enum {SONAR_LEFT,SONAR_BACK,SONAR_RIGHT,SONAR_HEAD};
	//enum {IR_LEFT,IR_BACK,IR_RIGHT,IR_HEAD};
	enum {SENSOR_DISABLE,SENSOR_AUTO,SENSOR_MANUAL};
	enum {PRIO_1ST,PRIO_2ND,PRIO_3RD,PRIO_4TH,PRIO_5TH,PRIO_6TH,PRIO_7TH,PRIO_8TH,PRIOS=PRIO_8TH};

	IR* getLeftIR();
	IR* getBackIR();
	IR* getRightIR();
	IR* getHeadIR();
	IR* getIR(unsigned char ir);				// get IR sensor by ID
	unsigned char getIRDist(IR* ir);			// get IR distance by object
	unsigned char getIRDist(unsigned char ir);	// get IR distance by ID
	unsigned char getIRDistAll();				// update all ID distance
	unsigned char initIR();
	unsigned char setIRDistAll();

	SONAR* getLeftSONAR();
	SONAR* getBackSONAR();
	SONAR* getRightSONAR();
	SONAR* getHeadSONAR();
	SONAR* getSONAR(unsigned char sr);	// get SONAR sensor by ID
	unsigned int getSONARDist(SONAR* sr, bool trig=true);			// get SONAR distance by object
	unsigned int getSONARDist(unsigned char sr,bool trig=true);	// get SONAR distance by ID
	unsigned char getSONARDistAll();
	unsigned char initSONAR();
	unsigned char setSONARDistAll();
	unsigned char trigSONAR(SONAR* sr);				// trigger SONAR by object
	unsigned char trigSONAR(unsigned char sr);		// trigger SONAR by ID
	unsigned char trigSONARAll();
	unsigned char getSONARTemp(SONAR* sr);
	unsigned char getSONARTemp(unsigned char sr=POS_HEAD);

	unsigned char updateSensorAll();

	unsigned char initSafeDistBitmap();
	unsigned char updateSafeDistBitmap();
	unsigned int initSafeActionsBitmap();
	unsigned int updateSafeActionsBitmap();

	unsigned long initActionsPrio();
	unsigned long getActionsPrio() const;
	unsigned long setActionsPrio(unsigned long prioMap);

	unsigned char selectAction();
	bool checkExecutable(Omni4WDAction* action);

	Omni4WDAction* execAction();
	Omni4WDAction* getActions();
	Omni4WDAction* addAction(Omni4WDAction* action);
	Omni4WDAction* addAction(unsigned char carStat,int speed=0,unsigned int duration=0,unsigned int uptime=0);

	bool routine();
	bool debugger() const;
	void del(bool isAll = false);
	bool actIsNull();
	
	void showAction();
private:
	Omni4WDSR();	// disable default constructor
	Omni4WDAction* _actions;

	SONAR *_leftSONAR, *_backSONAR, *_rightSONAR, *_headSONAR;
	IR *_leftIR, *_backIR, *_rightIR, *_headIR;

	int _defaultSpeed;
	unsigned char _safeDist;

	unsigned char _mode;		// auto, interactive, sequential
	unsigned char _sensorMode;	// sensors update mode, auto, manual or disable

	unsigned int SONARDist[4];
	unsigned char IRDist[4];
	bool safeDistBitmap[4];
	//unsigned char safeDistBitmap=0;	// 4 bits
	bool safeActionsBitmap[ACTIONTYPES];
	//unsigned int safeActionsBitmap=0;	// 11 bits
	unsigned char actionsPrio[ACTIONTYPES];

};
/*
	enum {STAT_UNKNOWN,
			STAT_STOP,
			STAT_ADVANCE,
			STAT_BACKOFF,
			STAT_LEFT,
			STAT_RIGHT,
			STAT_ROTATELEFT,
			STAT_ROTATERIGHT,
			STAT_UPPERLEFT,
			STAT_LOWERLEFT,
			STAT_LOWERRIGHT,
			STAT_UPPERRIGHT,
			ACTIONTYPES=STAT_UPPERRIGHT,	// 201209
	};
 */
#endif





