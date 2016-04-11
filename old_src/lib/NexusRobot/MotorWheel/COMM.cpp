#include<COMM.h>
#include <EEPROM.h>
//
// header, addr, lengthOfVal, CmdCode, Val, checksum
// 0x55,0xaa, 0xa1, 0x00, 0x10, NA, 0xb0
//
//

COMM::COMM(Omni4WDSR* pSr)
{
	this->_omni = pSr;
	clearBuf();
	datSize = 0;
	receFlag = RECE_UNKNOW;
}

unsigned char COMM::resetRom()
{
	EEPROM.write(ADDR,1);
	EEPROM.write(BRID,BR_19200);
	EEPROM.write(MODE,Omni4WDSR::MODE_SEQUENTIAL);
	EEPROM.write(SENSORMODE,Omni4WDSR::SENSOR_MANUAL);
}
unsigned char COMM::getAddr() const {
	//return _addr;
	return EEPROM.read(ADDR);
}
unsigned char COMM::setAddr(unsigned char addr) {
	EEPROM.write(ADDR,addr);
	//return _addr=addr;
	return EEPROM.read(ADDR);
}

unsigned long COMM::getBaudrate() const {
	//return _brID;
	return EEPROM.read(BRID);
}
unsigned char COMM::setBaudrate(unsigned char brID) {
	if(BR_2400<=brID && brID<=BR_115200) 
	{
		EEPROM.write(BRID,brID);
	}
	else EEPROM.write(BRID,BR_19200);
	return EEPROM.read(BRID);;
}
unsigned char COMM::init() {

	Serial.begin(getBaudrate()*2400);
	_omni->setMode(EEPROM.read(MODE));
	_omni->setSensorMode(Omni4WDSR::SENSOR_MANUAL);
	pinMode(13,OUTPUT);//set tx
	digitalWrite(13,HIGH);
	return getBaudrate();
}

unsigned char COMM::initMsg() {
	clearBuf();
	*(_buff+MSG_HEAD)=0x55;
	*(_buff+MSG_HEAD+1)=0xaa;
	*(_buff+MSG_ADDR)=getAddr();
	return MSG_ADDR+1;
}
unsigned char COMM::finalizeMsg() {
	*(_buff+getMsgLen() - 1)=checksum();	// CHECKSUM
	return getMsgLen();
}
unsigned char COMM::checksum() const {
	unsigned char checksum=0;
	for(unsigned char len=getMsgLen() - 1,i=0;i<len;++i) {
		checksum+=*(_buff+i);
	}
	return checksum;
}

// Head:2Byte + Addr:1Byte + Len:1Byte + Cmd:1Byte + Val:LenByte + Checksum:1Byte
unsigned char COMM::getMsgLen() const {
	return 6+_buff[MSG_LEN];
}

unsigned char COMM::sendMsg() {
	for(unsigned char i=0,len=getMsgLen();i<len;++i) Serial.write(_buff[i]);
	return getMsgLen();
}

unsigned char COMM::sayHi() {
	initMsg();
	*(_buff+MSG_LEN)=0;
	*(_buff+MSG_CMD)=CMD_SAYHI;
	finalizeMsg();
	return sendMsg();
}
unsigned char COMM::sayGetAddr() {
	initMsg();
	*(_buff+MSG_LEN)=1;
	*(_buff+MSG_CMD)=CMD_GETADDR;
	*(_buff+MSG_VAL)=getAddr();
	finalizeMsg();
	return sendMsg();
}

unsigned char COMM::sayGetBaudrate() {
	initMsg();
	*(_buff+MSG_LEN)=1;
	*(_buff+MSG_CMD)=CMD_GETBAUDRATE;
	*(_buff+MSG_VAL)=getBaudrate();
	finalizeMsg();
	return sendMsg();
}
// PID_Stat, P_integer,P_decimal*100, I_integer,I_decimal*100, D_integar,D_decimal*100
unsigned char COMM::sayGetPID() {
	initMsg();
	*(_buff+MSG_LEN)=7;
	*(_buff+MSG_CMD)=CMD_GETPID;
	*(_buff+MSG_VAL)=_omni->PIDGetStatus();
	*(_buff+MSG_VAL+1)=floor(_omni->PIDGetP_Param());
	*(_buff+MSG_VAL+2)=(_omni->PIDGetP_Param()-floor(_omni->PIDGetP_Param()))*100;
	*(_buff+MSG_VAL+3)=floor(_omni->PIDGetI_Param());
	*(_buff+MSG_VAL+4)=(_omni->PIDGetI_Param()-floor(_omni->PIDGetI_Param()))*100;
	*(_buff+MSG_VAL+5)=floor(_omni->PIDGetD_Param());
	*(_buff+MSG_VAL+6)=(_omni->PIDGetD_Param()-floor(_omni->PIDGetD_Param()))*100;
	finalizeMsg();
	return sendMsg();
}


// ID, distance
unsigned char COMM::actTrigIR() {
}
// ID, distance
static unsigned char sonarCurr = Omni4WDSR::POS_LEFT;
unsigned char COMM::actTrigSONAR() {
	initMsg();
	*(_buff+MSG_LEN)=2;
	*(_buff+MSG_CMD)=CMD_TRIGSONAR;
	*(_buff+MSG_VAL)=sonarCurr;
	*(_buff+MSG_VAL+1)=sonarsUpdate();
	finalizeMsg();
	return 0;//sendMsg();
}


	unsigned char COMM::sonarsUpdate() 
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
static unsigned long recvMs = 0;
unsigned char COMM::recvMsg(unsigned char desiredSize,unsigned char ms) {
	unsigned char datSize = 0;
	for(int j=0;datSize<desiredSize && j<5000;++j) {
		unsigned char ibyte=Serial.read();
		if(ibyte!=0xff) _buff[datSize++]=ibyte;
	}
	return datSize;
}

unsigned char COMM::clearBuf() {
	for(unsigned char i=0;i<sizeof(_buff);++i) *(_buff+i)=0;
	datSize = 0;
	receFlag = RECE_UNKNOW;
	return sizeof(_buff);
}

unsigned char COMM::replyMsg(unsigned char* pFlag) {
	if(!isAGoodPacket()) {return 0xff;}
	unsigned char cmd = getCmd();
	switch(cmd) {
		case CMD_SAYHI:
			sayHi();
			break;
		case CMD_SETADDR:
			actSetAddr(); break;
		case CMD_GETADDR:
			sayGetAddr(); break;
		case CMD_SETMODE:
			actSetMode(); break;
		case CMD_GETMODE:
			sayGetMode(); break;
		case CMD_SETBAUDRATE:
			actSetbaudrate(); break;
		case CMD_GETBAUDRATE:
			sayGetBaudrate(); break;
		case CMD_SETMOTION:
			actSetMotion(); break;
		case CMD_GETMOTION:
			sayGetMotion(); break;
		case CMD_SETPID:
			//actSetPID();
			break;
		case CMD_GETPID:
			//sayGetPID();
			break;
		case CMD_SETSENSORMODE:
			actSetSensorMode(); break;
		case CMD_GETSENSORMODE:
			sayGetSensorMode(); break;
		case CMD_TRIGIR:
			break;
		case CMD_TRIGSONAR:
			actTrigSONAR(); break;
		case CMD_COMMMODE:
			*pFlag = _SIMPLE_COMM;actGetCommMode();break;
		case CMD_RESET:
			actReset();break;
		default:
			break;
	}
	clearBuf();
	return getMsgLen();
}

unsigned char COMM::actReset()
{
	resetRom();
	init();
	actGetReset();
}
unsigned char COMM::actGetCommMode()
{
	initMsg();
	*(_buff+MSG_LEN)=0;
	*(_buff+MSG_CMD)=CMD_COMMMODE;
	finalizeMsg();
	return sendMsg();
}

unsigned char COMM::actGetReset()
{
	initMsg();
	*(_buff+MSG_LEN)=0;
	*(_buff+MSG_CMD)=CMD_RESET;
	finalizeMsg();
	return sendMsg();
}
unsigned char COMM::actSetAddr() 
{
	
	if(*(_buff+MSG_CMD)!=CMD_SETADDR) return 0;
	setAddr(_buff[MSG_VAL]);
	return sayGetAddr();
}

unsigned char COMM::actSetMode()
{	
	if(*(_buff+MSG_CMD) != CMD_SETMODE) return 0;
	_omni->setMode(*(_buff + MSG_VAL));
	EEPROM.write(MODE,*(_buff + MSG_VAL));
	return sayGetMode();
}

unsigned char COMM::sayGetMode() 
{
	initMsg();
	*(_buff+MSG_LEN)=1;
	*(_buff+MSG_CMD)=CMD_GETMODE;
	*(_buff+MSG_VAL)=_omni->getMode();
	finalizeMsg();
	return sendMsg();
}

unsigned char COMM::actSetbaudrate()
{
	if(*(_buff + MSG_CMD) != CMD_SETBAUDRATE) return 0;
	setBaudrate(*(_buff+MSG_VAL));
	sayGetBaudrate();
}
unsigned char COMM::actSetMotion()
{
	if(*(_buff + MSG_CMD) != CMD_SETMOTION) return 0;
	unsigned char motion = *(_buff + MSG_VAL);
	int speed = *(_buff + MSG_VAL + 1)*256 + *(_buff + MSG_VAL + 2);
	int duration = *(_buff + MSG_VAL + 3)*256 + *(_buff + MSG_VAL + 4);
	int uptime = *(_buff + MSG_VAL + 5)*10;
	_omni->addAction(motion,speed,duration,uptime);
	return sayGetMotion();
}

unsigned char COMM::sayGetMotion() {
	initMsg();
	*(_buff+MSG_LEN)=3;
	*(_buff+MSG_CMD)=CMD_GETMOTION;
	*(_buff+MSG_VAL)=_omni->getCarStat();
	*(_buff+MSG_VAL+1)=_omni->getCarSpeedMMPS()>>8;
	*(_buff+MSG_VAL+2)=_omni->getCarSpeedMMPS()&0xff;
	finalizeMsg();
	return sendMsg();
}
unsigned char COMM::actSetPID()
{
	if(*(_buff + MSG_CMD) != CMD_SETPID) return 0;
	unsigned char status = *(_buff + MSG_VAL);
	float kc = float(*(_buff + MSG_VAL + 1)) + float(*(_buff + MSG_VAL + 2))/(float)100;
	float taui = float(*(_buff + MSG_VAL + 3)) + float(*(_buff + MSG_VAL + 4))/(float)100;
	float taud = float(*(_buff + MSG_VAL + 5)) + float(*(_buff + MSG_VAL + 6))/(float)100;
	_omni->PIDEnable(kc,taui,taud,10);
	return 1;
}

unsigned char COMM::actSetSensorMode()
{
	if(*(_buff + MSG_CMD) != CMD_SETSENSORMODE) return 0;
	EEPROM.write(SENSORMODE,*(_buff + MSG_VAL));
	_omni->setSensorMode(*(_buff + MSG_VAL));	
	return sayGetSensorMode();
}

unsigned char COMM::sayGetSensorMode() {
	initMsg();
	*(_buff+MSG_LEN)=1;
	*(_buff+MSG_CMD)=CMD_GETSENSORMODE;
	*(_buff+MSG_VAL)=_omni->getSensorMode();
	finalizeMsg();
	return sendMsg();
}
unsigned char COMM::getCmd()
{
	return _buff[MSG_CMD];
}

void COMM::showDat()
{
	for(int i = 0; i < sizeof(_buff); ++i)
	{
		Serial.print(_buff[i],HEX);
		Serial.print(' ');
	}
	Serial.println();
}

unsigned char COMM::isAGoodPacket()
{
	if(_buff[MSG_HEAD] != 0x55) return 0;
	if(_buff[MSG_HEAD+1] != 0xaa) return 0;
	if(_buff[MSG_CMD] != CMD_RESET && _buff[MSG_ADDR] != getAddr()) {return 0;}
	if(_buff[getMsgLen()-1] != checksum()) {return 0;}
	return 1;
}


















