#include <SimpleCOMM.h>
#define DEBUG
	SimpleCOMM::SimpleCOMM(Omni4WDSR* pOmni,bool isHead)
	{
		this->_omni = pOmni;
		this->datSize = 0;
		recvFlag = RECE_UNKNOW;
	}
	unsigned char SimpleCOMM::recvMsg(unsigned char desiredSize,unsigned char ms)
	{
		unsigned char ibyte;
		unsigned long endtime = millis() + ms;
		if(recvFlag == RECE_BEGIN) recvFlag = RECE_SECONDTIME;
		while(datSize < desiredSize && (ibyte =Serial.read()) != 0xff && millis() < endtime)
		{
			if(datSize == 0 && ((ibyte - 0x30) != EEPROM.read(ADDR))) {return recvFlag;}
			if(datSize == 0 && ((ibyte - 0x30) == EEPROM.read(ADDR))) recvFlag = RECE_BEGIN;
			_buff[datSize++] = ibyte;
		}
		if(datSize == 2) {recvFlag = RECE_SUCCEEDED;}
		if(datSize != 2 && recvFlag == RECE_SECONDTIME) clearBuf();
		return recvFlag;
	}
	unsigned char SimpleCOMM::replyMsg(unsigned char* pFlag)
	{
		
		if(recvFlag != RECE_SUCCEEDED) {return NULL;}
		unsigned char motion = STAT_UNKNOWN;
		unsigned int duration = 1000;
		unsigned char speed = 160;
		unsigned int uptime = 500;
		switch(_buff[1]) 
		{
			case '5':
			motion = STAT_STOP;break;
			case '8':
			motion = STAT_ADVANCE;duration = 5000;break;
			case '2':
			motion = STAT_BACKOFF; duration = 5000;break;
			case '6':
			motion = STAT_RIGHT;duration = 5000;break;
			case '4':
			motion = STAT_LEFT;duration = 5000;break;
			case '0':
			motion = STAT_ROTATELEFT;break;
			case '.':
			motion = STAT_ROTATERIGHT;break;
			case '7':
			motion = STAT_UPPERLEFT;duration = 5000;break;
			case '1':
			motion = STAT_LOWERLEFT; duration = 3000;break;
			case '3':
			motion = STAT_LOWERRIGHT;duration = 5000;break;
			case '9':
			motion = STAT_UPPERRIGHT;duration = 5000;break;
			case 't':
			*pFlag = _COMM;Serial.println("turnMode succeed");break;
			case '*': 
			sonarsUpdate();
			clearBuf();
			return NULL; break;
			case 'r':
			softReset();
			default:motion = STAT_STOP; break;
		}
		clearBuf();
		_omni->addAction(motion,speed,duration,uptime);
		return motion;
	}
	void SimpleCOMM::showDat()
	{
		for(int i = 0; i < sizeof(_buff); ++i)
		{
			Serial.print(*(_buff+i),HEX);
			Serial.print(' ');
		}
		Serial.println();
	}
	
	void SimpleCOMM::clearBuf()
	{
		_buff[0] = 0x00;
		_buff[1] = 0x00;
		recvFlag = RECE_UNKNOW;
		datSize = 0;
	}
	
	unsigned char SimpleCOMM::getRecvStat()
	{
		return recvFlag;
	}
	
	static unsigned char sonarCurr = Omni4WDSR::POS_LEFT;
	unsigned char SimpleCOMM::sonarsUpdate() 
	{
		if(sonarCurr==Omni4WDSR::POS_HEAD) sonarCurr=Omni4WDSR::POS_LEFT;
		else ++sonarCurr;      
		_omni->getSONARDist(sonarCurr); 
		if(sonarCurr == Omni4WDSR::POS_HEAD)
		{ 
			_omni->updateSafeDistBitmap();
			_omni->updateSafeActionsBitmap();	
		}
		return sonarCurr;
	}
	
	void SimpleCOMM::softReset() // Restarts program from beginning but does not reset the peripherals and registers
	{
		asm volatile ("  jmp 0");  
	} 
	