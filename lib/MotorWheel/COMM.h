#include<Omni4WDSR.h>
#ifndef _COMM_H
#define _COMM_H
//
// header, addr, lengthOfVal, CmdCode, Val, checksum
// 0x55,0xaa, 0xa1, 0x00, 0x10, NA, 0xb0
//
//

class COMM {
public:
	COMM(Omni4WDSR* pSr);
	Omni4WDSR* _omni;
	unsigned char getAddr() const;//--
	unsigned char setAddr(unsigned char addr=0xa0);//--

	unsigned long getBaudrate() const;//--
	unsigned char setBaudrate(unsigned char brID=BR_19200);//--
	unsigned char init();//--

	unsigned char clearBuf();//--
	unsigned char initMsg();//--
	unsigned char finalizeMsg();//--

	unsigned char checksum() const;//--
	unsigned char getMsgLen() const;//--

	unsigned char replyMsg(unsigned char* pFlag);//--
	unsigned char sendMsg();//--

	unsigned char recvMsg(unsigned char desiredSize,unsigned char ms);//--
	//analyseMsg();

	unsigned char sayHi();//--

	unsigned char sayGetAddr();//--
	unsigned char actSetAddr();//--

	unsigned char sayGetMode();//--
	unsigned char actSetMode();//--
	
	unsigned char sayGetBaudrate();//--
	unsigned char actSetbaudrate();//--

	unsigned char sayGetMotion();//--
	unsigned char actSetMotion();//--
	
	unsigned char sayGetPID();//--
	unsigned char actSetPID();//--
	
	unsigned char sayGetSensorMode();//--
	unsigned char actSetSensorMode();//--

	unsigned char actTrigIR();//--
	unsigned char actTrigSONAR();//--
	
	unsigned char resetRom();//--
	unsigned char getCmd();
	
	unsigned char isAGoodPacket();
	unsigned char sonarsUpdate();
	
	unsigned char actGetCommMode();
	
	unsigned char actReset();
	unsigned char actGetReset();
	
	void showDat();
	enum {
		CMD_SAYHI=0x10,
		CMD_GETADDR,CMD_SETADDR,
		CMD_GETMODE,CMD_SETMODE,
		CMD_GETBAUDRATE,CMD_SETBAUDRATE,
		CMD_GETMOTION,CMD_SETMOTION,
		CMD_GETPID,CMD_SETPID,
		CMD_GETSENSORMODE,CMD_SETSENSORMODE,
		CMD_TRIGIR,CMD_TRIGSONAR,
		CMD_COMMMODE ,CMD_RESET
	};

	enum {BR_2400=1,BR_4800=2,BR_9600=4,BR_19200=8,BR_38400=16,BR_76800=32,BR_115200=48};

	enum {MSG_HEAD,MSG_ADDR=2,MSG_LEN,MSG_CMD,MSG_VAL,MSG_CHECKSUM};
	
	enum{ADDR = 1,BRID ,MODE ,SENSORMODE};
	
	enum{RECE_UNKNOW = 0,RECE_BEGIN,RECE_SECONDTIME,RECE_SUCCEEDED};
	
	enum{_COMM = 0,_SIMPLE_COMM = 1};
private:
	unsigned char receFlag;
	unsigned char datSize;
	unsigned char _buff[20];
};
#endif








