
#include<Omni4WDSR.h>


Omni4WDSR::Omni4WDSR(MotorWheel* wheelUL, MotorWheel* wheelLL,
			MotorWheel* wheelLR, MotorWheel* wheelUR,unsigned int wheelspan) 
				:Omni4WD(wheelUL,wheelLL,wheelLR,wheelUR,wheelspan) {
	//init();
}
unsigned char Omni4WDSR::init() {
	debug();

	attachIR(NULL,NULL,NULL,NULL);
	attachSONAR(NULL,NULL,NULL,NULL);

	setSafeDist(30);	// 30cm
	initIR();
	initSONAR();
  
	initSafeDistBitmap();
	initSafeActionsBitmap();
	initActionsPrio();
 	
	addAction(new Omni4WDAction(this));
	//Omni4WDAction::setOmni4WD(this);
	setMode(MODE_STOP);
	setSensorMode(SENSOR_DISABLE);
 
	return initOmni4WD();
}
bool Omni4WDSR::debugger() const {

	return true;
}

unsigned char Omni4WDSR::initOmni4WD() {
	debug();
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__)
	TCCR1B=TCCR1B&0xf8|0x01;
	TCCR2B=TCCR2B&0xf8|0x01;
#elif defined(__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
	TCCR2B=TCCR2B&0xf8|0x01;
	TCCR3B=TCCR3B&0xf8|0x01;
	TCCR4B=TCCR4B&0xf8|0x01;
#endif

	Omni4WD::PIDEnable(KC,TAUI,TAUD,SAMPLETIME);
	return Omni4WD::setCarStop();
}

unsigned char Omni4WDSR::attachSONAR(SONAR* left, SONAR* back, SONAR* right, SONAR* head) {
	debug();
	unsigned char count;
	if((_leftSONAR=left)!=NULL) ++count;
	if((_backSONAR=back)!=NULL) ++count;
	if((_rightSONAR=right)!=NULL) ++count;
	if((_headSONAR=head)!=NULL) ++count;
	return count;
}
unsigned char Omni4WDSR::attachIR(IR* left, IR* back, IR* right, IR* head) {
	debug();
	unsigned char count;
	if((_leftIR=left)!=NULL) ++count;
	if((_backIR=back)!=NULL) ++count;
	if((_rightIR=right)!=NULL) ++count;
	if((_headIR=head)!=NULL) ++count;
	return count;
}
unsigned char Omni4WDSR::getMode() const {
	return _mode;
}
unsigned char Omni4WDSR::setMode(unsigned char mode) {
	debug();
	if(getMode()==mode) return mode;
	if(MODE_UNKNOWN<=mode && mode<=MODE_SLEEP) 
	{
		_mode=mode;
	}
	else return getMode();

	switch(getMode()) {
		case MODE_AUTO:
			getActions()->delAll();
			break;
		case MODE_INTERACTIVE:
			getActions()->haltAll();
			break;
		case MODE_SEQUENTIAL:
			getActions()->activeAll();
			break;
		case MODE_SLEEP:
			getActions()->delAll();
			this->PIDDisable();
			break;
		case MODE_STOP:
		default:
			getActions()->haltAll();
			break;
	}
	addAction(Omni4WD::STAT_STOP)->exec();	// stop the chassis if mode changed
	return getMode();
}

unsigned char Omni4WDSR::getSensorMode() const {
	return _sensorMode;
}
unsigned char Omni4WDSR::setSensorMode(unsigned char mode) {
	debug();
	if(SENSOR_MANUAL >= mode && mode >= SENSOR_DISABLE) _sensorMode=mode;
	return getSensorMode();
}

int Omni4WDSR::getDefaultSpeed() const {
	return _defaultSpeed;
}
int Omni4WDSR::setDefaultSpeed(int speed) {
	if(abs(speed)<=MAXSPEED) _defaultSpeed=speed;
	return getDefaultSpeed();
}
unsigned char Omni4WDSR::getSafeDist() const {
	return _safeDist;
}
unsigned char Omni4WDSR::setSafeDist(unsigned char dist) {
	debug();
	_safeDist=dist;
}

IR* Omni4WDSR::getIR(unsigned char ir) {
	if(ir==POS_LEFT) return getLeftIR();
	if(ir==POS_BACK) return getBackIR();
	if(ir==POS_RIGHT) return getRightIR();
	if(ir==POS_HEAD) return getHeadIR();
	return NULL;
}
IR* Omni4WDSR::getLeftIR() {
	return _leftIR;
}
IR* Omni4WDSR::getBackIR() {
	return _backIR;
}
IR* Omni4WDSR::getRightIR() {
	return _rightIR;
}
IR* Omni4WDSR::getHeadIR() {
	return _headIR;
}
unsigned char Omni4WDSR::getIRDist(unsigned char ir) {
	if(getSensorMode()==SENSOR_DISABLE) return 0;
	if(POS_LEFT<=ir && ir<=POS_HEAD) {
		if(getIR(ir)) return IRDist[ir]=getIR(ir)->getDist();
	}
	return 0;
}
unsigned char Omni4WDSR::getIRDist(IR* ir) {
	for(unsigned char pos=POS_LEFT;pos<=POS_HEAD;++pos) {
		if(ir==getIR(pos)) return getIRDist(pos);
	}
	return 0;
}
unsigned char Omni4WDSR::getIRDistAll() {
	debug();
	unsigned char count=0;
	for(unsigned char pos=POS_LEFT;pos<=POS_HEAD;++pos) {
		if(getIRDist(pos)) ++count;
	}
	return count;
}
unsigned char Omni4WDSR::initIR() {
	debug();
	unsigned char count=0;
	for(unsigned char pos=POS_LEFT;pos<=POS_HEAD;++pos) {
		if(getIR(pos)) ++count;
		IRDist[pos]=0;
	}
	return count;
}

SONAR* Omni4WDSR::getLeftSONAR() {
	return _leftSONAR;
}
SONAR* Omni4WDSR::getBackSONAR() {
	return _backSONAR;
}
SONAR* Omni4WDSR::getRightSONAR() {
	return _rightSONAR;
}
SONAR* Omni4WDSR::getHeadSONAR() {
	return _headSONAR;
} 
SONAR* Omni4WDSR::getSONAR(unsigned char sr) {
	if(sr==POS_LEFT) return getLeftSONAR();
	if(sr==POS_BACK) return getBackSONAR();
	if(sr==POS_RIGHT) return getRightSONAR();
	if(sr==POS_HEAD) return getHeadSONAR();
	return NULL;
}
unsigned int Omni4WDSR::getSONARDist(unsigned char sr,bool trig) {
	if(getSensorMode()==SENSOR_DISABLE) {Serial.println("sensor disable");return 0;}
	if(POS_LEFT<=sr && sr<=POS_HEAD) {
		if(getSONAR(sr)) {
			delay(2);
			SONARDist[sr]=getSONAR(sr)->getDist();	// get last distance and trigger
			getSONAR(sr)->showDat();
			if(trig) getSONAR(sr)->trigger();
			return SONARDist[sr];
		}
		//else{Serial.println("sonar not found");}
	}
	return 0;
}
unsigned int Omni4WDSR::getSONARDist(SONAR* sr,bool trig) {
	if(sr==NULL) return 0;
	for(unsigned char pos=POS_LEFT;pos<=POS_HEAD;++pos) {
		if(sr==getSONAR(pos)) return getSONARDist(pos,trig);
	}
	return 0;
}
unsigned char Omni4WDSR::getSONARDistAll() {
	debug();
	unsigned char count=0;
	for(unsigned char pos=POS_LEFT;pos<=POS_HEAD;++pos) {
		if(getSONARDist(pos)) ++count;
	}
	return count;
}
unsigned char Omni4WDSR::initSONAR() {
	debug();
	unsigned char count=0;
	for(unsigned char pos=POS_LEFT;pos<=POS_HEAD;++pos) {
		if(getSONAR(pos)) ++count;
		SONARDist[pos]=0;
	}
	return count;
}
unsigned char Omni4WDSR::trigSONAR(unsigned char sr) {
	if(getSensorMode()==SENSOR_DISABLE) return 0;
	if(POS_LEFT<=sr && sr<=POS_HEAD) {
		if(getSONAR(sr)) return getSONAR(sr)->trigger();
	}
	return 0;
}
unsigned char Omni4WDSR::trigSONAR(SONAR* sr) {
	if(sr==NULL) return 0;
	for(unsigned char pos=POS_LEFT;pos<=POS_HEAD;++pos) {
		if(sr==getSONAR(pos)) return trigSONAR(pos);
	}
	return 0;
}
unsigned char Omni4WDSR::trigSONARAll() {
	debug();
	unsigned char count=0;
	for(unsigned char pos=POS_LEFT;pos<=POS_HEAD;++pos) {
		if(trigSONAR(pos)) ++count;
	}
	return count;
}
unsigned char Omni4WDSR::getSONARTemp(unsigned char sr) {
	if(getSensorMode()==SENSOR_DISABLE) return 0;
	for(unsigned char pos=POS_LEFT;pos<=POS_HEAD;++pos) {
		if(getSONAR(sr)) return getSONAR(sr)->getTemp();
	}
	return 0;
}
unsigned char Omni4WDSR::getSONARTemp(SONAR* sr) {
	if(sr==NULL) return 0;
	for(unsigned char pos=POS_LEFT;pos<=POS_HEAD;++pos) {
		if(sr==getSONAR(pos)) return getSONARTemp(pos);
	}
	return 0;
}
unsigned char Omni4WDSR::updateSensorAll() {
	debug();
	if(getSensorMode()==SENSOR_DISABLE) return 0;
	unsigned char count=0;
	for(unsigned char pos=POS_LEFT;pos<=POS_HEAD;++pos) {
		if(getSONAR(pos) && getSONARDist(pos)) ++count;
		if(getIR(pos) && getIRDist(pos)) ++count;
	}
	return count;
}

unsigned char Omni4WDSR::initSafeDistBitmap() {
	debug();
	for(unsigned char pos=POS_LEFT;pos<=POS_HEAD;++pos) {
		//safeDistBitmap[pos]=false;
		safeDistBitmap[pos]=true;
		
	}
	return 0;
}
unsigned char Omni4WDSR::updateSafeDistBitmap() {
	debug();
	unsigned char map=0;
	for(unsigned char pos=POS_LEFT;pos<=POS_HEAD;++pos) {
		safeDistBitmap[pos]=(/*IRDist[pos]>getSafeDist() && */SONARDist[pos]>getSafeDist());
		map|=(safeDistBitmap[pos]<<pos);
	}
	return map;
}
unsigned int Omni4WDSR::initSafeActionsBitmap() {
	debug();
	for(unsigned char pos=Omni4WD::STAT_UNKNOWN;pos<=Omni4WD::ACTIONTYPES;++pos) {
		//safeActionsBitmap[pos]=false;
		safeActionsBitmap[pos]=true;
	}
	return 0;
}
unsigned int Omni4WDSR::updateSafeActionsBitmap() {
	debug();
	unsigned int map=0;
	safeActionsBitmap[Omni4WD::STAT_UNKNOWN]=false;
	safeActionsBitmap[Omni4WD::STAT_STOP]=true;
	safeActionsBitmap[Omni4WD::STAT_ADVANCE]=safeDistBitmap[POS_HEAD];
	safeActionsBitmap[Omni4WD::STAT_BACKOFF]=safeDistBitmap[POS_BACK];
	safeActionsBitmap[Omni4WD::STAT_LEFT]=safeDistBitmap[POS_LEFT];
	safeActionsBitmap[Omni4WD::STAT_RIGHT]=safeDistBitmap[POS_RIGHT];
	safeActionsBitmap[Omni4WD::STAT_ROTATELEFT]=true;
	safeActionsBitmap[Omni4WD::STAT_ROTATERIGHT]=true;
	safeActionsBitmap[Omni4WD::STAT_UPPERLEFT]=(safeDistBitmap[POS_LEFT] && safeDistBitmap[POS_HEAD]);
	safeActionsBitmap[Omni4WD::STAT_LOWERLEFT]=(safeDistBitmap[POS_LEFT] && safeDistBitmap[POS_BACK]);
	safeActionsBitmap[Omni4WD::STAT_LOWERRIGHT]=(safeDistBitmap[POS_BACK] && safeDistBitmap[POS_RIGHT]);
	safeActionsBitmap[Omni4WD::STAT_UPPERRIGHT]=(safeDistBitmap[POS_RIGHT] && safeDistBitmap[POS_HEAD]);

	for(unsigned char pos=Omni4WD::STAT_UNKNOWN;pos<=Omni4WD::ACTIONTYPES;++pos) {
		map|=(safeActionsBitmap[pos]<<pos);
	}
	return map;
}
unsigned long Omni4WDSR::initActionsPrio() {
	debug();
	for(unsigned char pos=Omni4WD::STAT_UNKNOWN;pos<=Omni4WD::ACTIONTYPES;++pos) {
		actionsPrio[pos]=Omni4WD::STAT_STOP;
	}
	return actionsPrio[0];
}
unsigned long Omni4WDSR::getActionsPrio() const {
	debug();
	unsigned long prioMap=0;
	for(unsigned char pos=PRIO_1ST;pos<=PRIOS;++pos)
		prioMap|=(actionsPrio[pos]<<pos);
	return prioMap;
}
unsigned long Omni4WDSR::setActionsPrio(unsigned long prioMap) {
	debug();
	for(unsigned char pos=PRIO_1ST;pos<=PRIOS;++pos) {
		unsigned char stat=(prioMap>>pos&0xff);
		if(Omni4WD::STAT_UNKNOWN<stat && stat<=Omni4WD::ACTIONTYPES)
			actionsPrio[pos]=(prioMap>>pos&0xff);
		else actionsPrio[pos]=Omni4WD::STAT_STOP;
	}
	return getActionsPrio();
}

unsigned char Omni4WDSR::selectAction() {
	debug();
	if(getSensorMode()==SENSOR_DISABLE)
		return actionsPrio[PRIO_1ST];

	updateSensorAll();
	updateSafeDistBitmap();
	updateSafeActionsBitmap();

	for(unsigned char pos=PRIO_1ST;pos<=PRIOS;++pos) {
		if(safeActionsBitmap[actionsPrio[pos]])
			return safeActionsBitmap[actionsPrio[pos]];
	}
	return Omni4WD::STAT_STOP;
}
bool Omni4WDSR::checkExecutable(Omni4WDAction* action) {
	debug();
	if(action==NULL || (action->getStat()!=Omni4WDAction::STAT_QUEUING && action->getStat()!=Omni4WDAction::STAT_ACTING)) {return false;}
	if(getSensorMode()==SENSOR_DISABLE) {return true;}
	return safeActionsBitmap[action->getCarStat()];
}

Omni4WDAction* Omni4WDSR::getActions() {
	return _actions;
}
Omni4WDAction* Omni4WDSR::addAction(Omni4WDAction* action) {
	debug();
	if(action) {
		if(_actions==NULL) {_actions=action;}
		else {_actions->add(action);}
		return action;
	}
	return NULL;
}
Omni4WDAction* Omni4WDSR::addAction(unsigned char carStat,int speed,unsigned int duration,unsigned int uptime) {
	debug();
	Omni4WDAction* action=_actions->findNReuse(carStat,speed,duration,uptime);
	if(action==NULL)
		action=_actions->add(new Omni4WDAction(carStat,speed,duration,uptime));
	return action;
}
Omni4WDAction* Omni4WDSR::execAction() {
	debug();
	Omni4WDAction* action = getActions()->findActing();//find the active action
	if(!action)
		action=getActions()->find1stPrio();//if not exist active action,find next action
	if(action)
	{
		//elay(1500);
		//Serial.println(checkExecutable(action));
		if(!checkExecutable(action))//check the safe distance map,if distance from obstacle less than the safe distance,action will be killed and the robot will stop
		{
			action->Kill(true);
		}
		else if(Omni4WDAction* pNextAct = action->getNextAction())// if exist next action,it means have new action to deal,so current action will be killed 
																//and next action start to move
		{	
			if(pNextAct->getCarStat() != action->getCarStat())
				action->Kill(true);
			else
				action->Kill(false);
			pNextAct->Start();
		}
		else if(checkExecutable(action) && action->getStat() == Omni4WDAction::STAT_QUEUING)//if action's state is STAT_QUEUING.It will start to move
		{
			action->Start();
		}
		else if(checkExecutable(action) && action->getStat()== Omni4WDAction::STAT_ACTING)//if action's state is STAT_ACTING.It will continue to move 
																						//during your define time
		{
			action->exec();
		}
		else
		{
			action->Kill(true);
		}
	}
	else 
	{
		action->setStat(Omni4WDAction::STAT_FAILED);
	}
	return action;
}
bool Omni4WDSR::routine() {
	debug();
	//
	// 1. called by WatchDog timer
	// 2. PIDRegulate()
	// 3. check Serial Cmd and Reply or Exec
	// 4. clear wdtCount when cmd received
	// 5. emergency stop when wdtCount>=200 ( 15ms * 200 = 3000ms )
	//
	//
	switch(getMode()) {
		case MODE_AUTO:
			addAction(selectAction(),getDefaultSpeed())->exec();
			break;
		case MODE_INTERACTIVE:
			break;
		case MODE_SEQUENTIAL:
			if(getActions()->findActing()==NULL) execAction();
			break;
		case MODE_SLEEP:
			getActions()->delAll();
			this->PIDDisable();
			break;
		case MODE_STOP:
		default:
			getActions()->haltAll();
			addAction(Omni4WD::STAT_STOP)->exec();
			break;
	}
	return true;
}


void Omni4WDSR::del(bool isAll)
{
	getActions()->delAction(isAll);
}
bool Omni4WDSR::actIsNull()
{
	return getActions()->isNull();
}

void Omni4WDSR::showAction()
{
	for(int i = 0; i<sizeof(safeActionsBitmap);++i)
		Serial.print(safeActionsBitmap[i]);
	Serial.println();
}




















