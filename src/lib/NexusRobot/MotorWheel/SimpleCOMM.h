#ifndef _SIMPLECOMM_H
#define _SIMPLECOMM_H
#include<Omni4WDSR.h>
#include <EEPROM.h>
class SimpleCOMM
{
private:
	unsigned char _buff[2];
	unsigned char _cmd;
	
	unsigned char recvFlag;
	unsigned char datSize;
public:
	Omni4WDSR* _omni;
	SimpleCOMM(Omni4WDSR* pOmni,bool isHead = true);
	unsigned char  replyMsg(unsigned char* pFlag);
	unsigned char recvMsg(unsigned char desiredSize,unsigned char ms);
	void clearBuf();
	void showDat();
	unsigned char getRecvStat();
	unsigned char sonarsUpdate();
	void softReset();
	enum{ADDR = 1,BRID = 2};
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
			RECV_SONAR,
			ACTIONTYPES=STAT_UPPERRIGHT,	// 201209
	};
	enum{RECE_UNKNOW = 0,RECE_BEGIN,RECE_SECONDTIME,RECE_SUCCEEDED};
	enum{TRIGGER_SONAR = 0,QUEST_DAT,GET_SONAR_DIST};
	enum{_COMM = 0,_SIMPLE_COMM = 1};
};
#endif