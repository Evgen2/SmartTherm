/* slightly changed  from v 1.13 + 5 July 2022
   https://github.com/ihormelnyk/opentherm_library 

*/
/*
OpenTherm.cpp - OpenTherm Communication Library For Arduino, ESP8266
Copyright 2018, Ihor Melnyk
*/
#include "Smart_Config.h"
#include "OpenTherm.h"

OpenTherm::OpenTherm(int inPin, int outPin, bool isSlave):
	status(OpenThermStatus::NOT_INITIALIZED),
	inPin(inPin),
	outPin(outPin),
	isSlave(isSlave),
	response(0),
	responseStatus(OpenThermResponseStatus::NONE),
	responseTimestamp(0),
	handleInterruptCallback(NULL),
	processResponseCallback(NULL)
{
	status = NOT_INITIALIZED;
	Lastresponse = 0;
	LastRequestId = 0;
	Immergas_fix = false;
}

//#define PULLUP            0x04
//#define INPUT_PULLUP      0x05
//#define PULLDOWN          0x08
//#define INPUT_PULLDOWN    0x09

void OpenTherm::begin(void(*handleInterruptCallback)(void), void(*processResponseCallback)(unsigned long, OpenThermResponseStatus))
{	init_OTids();
	if(isSlave)
//		pinMode(inPin, INPUT);
//		pinMode(inPin, INPUT_PULLDOWN);
		pinMode(inPin, INPUT_PULLUP);
	else
		pinMode(inPin, INPUT);
	pinMode(outPin, OUTPUT);
	if (handleInterruptCallback != NULL) {
		this->handleInterruptCallback = handleInterruptCallback;
		attachInterrupt(digitalPinToInterrupt(inPin), handleInterruptCallback, CHANGE);
	}
	activateBoiler(0);
	status = OpenThermStatus::READY;
	this->processResponseCallback = processResponseCallback;
}

/*
void OpenTherm::begin(void(*handleInterruptCallback)(void))
{
	begin(handleInterruptCallback, NULL);
}
*/
bool  IRAM_ATTR OpenTherm::isReady()
{
	return status == OpenThermStatus::READY;
}

int IRAM_ATTR OpenTherm::readState() {
	return digitalRead(inPin);
}

void OpenTherm::setActiveState() {
	digitalWrite(outPin, LOW);
}

void OpenTherm::setIdleState() {
	digitalWrite(outPin, HIGH);
}

void OpenTherm::activateBoiler(int wait) {
	setIdleState();
	if(wait)
		delay(1000);
}

void OpenTherm::sendBit(bool high) {
	if (high) setActiveState(); else setIdleState();
	delayMicroseconds(500);
	if (high) setIdleState(); else setActiveState();
	delayMicroseconds(500);
}

bool OpenTherm::sendRequestAync(unsigned long request)
{
	//Serial.println("Request: " + String(request, HEX));
	noInterrupts();
	const bool ready = isReady();
	interrupts();

	if (!ready)
	  return false;

	status = OpenThermStatus::REQUEST_SENDING;
	response = 0;
	responseStatus = OpenThermResponseStatus::NONE;

	sendBit(HIGH); //start bit
	for (int i = 31; i >= 0; i--) {
		sendBit(bitRead(request, i));
	}
	sendBit(HIGH); //stop bit
	setIdleState();

	status = OpenThermStatus::RESPONSE_WAITING;
	responseTimestamp = micros();
	return true;
}

unsigned long OpenTherm::sendRequest(unsigned long request)
{
response =0;
	if (!sendRequestAync(request)) return 0;
	while (!isReady()) {
		process();
		yield();
	}
	Lastresponse = response;
	return response;
}

bool OpenTherm::sendResponse(unsigned long request)
{
	status = OpenThermStatus::REQUEST_SENDING;
	response = 0;
	responseStatus = OpenThermResponseStatus::NONE;

	sendBit(HIGH); //start bit
	for (int i = 31; i >= 0; i--) {
		sendBit(bitRead(request, i));
	}
	sendBit(HIGH); //stop bit
	setIdleState();
	status = OpenThermStatus::READY;
	return true;
}

unsigned long OpenTherm::getLastResponse()
{
	return response;
}

OpenThermResponseStatus OpenTherm::getLastResponseStatus()
{
	return responseStatus;
}

#if OT_MASTER_DEBUG || OT2_SLAVE_DEBUG 
otst otst_d[128];
int Notst = 0;
int Flag_otst = 0;
#endif

void IRAM_ATTR OpenTherm::handleInterrupt()
{
#if OT_MASTER_DEBUG || OT2_SLAVE_DEBUG 
 #if OT_MASTER_DEBUG
	if(isSlave == 0)
 #elif OT2_SLAVE_DEBUG 
	if(isSlave)
 #endif
	{	if(Flag_otst == 0)
		{	
			otst_d[Notst].status =  (unsigned int)status;
			otst_d[Notst].state =  readState();
			otst_d[Notst].t =  micros();
			otst_d[Notst].ind = responseBitIndex; 
			otst_d[Notst].resp = response;
			if(Notst < 128 && responseBitIndex < 32 ) Notst++;
			else 
				Flag_otst = 1;
		}
 	}
#endif

	if (isReady())
	{
		if (isSlave && readState() == HIGH) {
		   status = OpenThermStatus::RESPONSE_WAITING;
		}
		else {
			return;
		}
	}

	unsigned long newTs = micros();
	if (status == OpenThermStatus::RESPONSE_WAITING) {
		if (readState() == HIGH) {
			status = OpenThermStatus::RESPONSE_START_BIT;
			responseTimestamp = newTs;
		}
		else {
			status = OpenThermStatus::RESPONSE_INVALID;
			responseTimestamp = newTs;
		}
	}
	else if (status == OpenThermStatus::RESPONSE_START_BIT) {
		if ((newTs - responseTimestamp < 750) && readState() == LOW) {
			status = OpenThermStatus::RESPONSE_RECEIVING;
			responseTimestamp = newTs;
			responseBitIndex = 0;
		}
		else {
			status = OpenThermStatus::RESPONSE_INVALID;
			responseTimestamp = newTs;
		}
	}
	else if (status == OpenThermStatus::RESPONSE_RECEIVING) {
		if ((newTs - responseTimestamp) > 750) {
			if (responseBitIndex < 32) {
				response = (response << 1) | !readState();
				responseTimestamp = newTs;
				responseBitIndex++;
			}
			else { //stop bit
				status = OpenThermStatus::RESPONSE_READY;
				responseTimestamp = newTs;
			}
		}
	}
}

void OpenTherm::process()
{
	noInterrupts();
	OpenThermStatus st = status;
	unsigned long ts = responseTimestamp;
	interrupts();

	if (st == OpenThermStatus::READY) return;
	unsigned long newTs = micros();
	if (st != OpenThermStatus::NOT_INITIALIZED && st != OpenThermStatus::DELAY && (newTs - ts) > 1000000) {
		status = OpenThermStatus::READY;
		responseStatus = OpenThermResponseStatus::TIMEOUT;
		if (processResponseCallback != NULL) {
			processResponseCallback(response, responseStatus);
		}
	}
	else if (st == OpenThermStatus::RESPONSE_INVALID) {
		status = OpenThermStatus::DELAY;
		responseStatus = OpenThermResponseStatus::INVALID;
		if (processResponseCallback != NULL) {
			processResponseCallback(response, responseStatus);
		}
	}
	else if (st == OpenThermStatus::RESPONSE_READY) {
		status = OpenThermStatus::DELAY;
		responseStatus = (isSlave ? isValidRequest(response) : isValidResponse(response)) ? OpenThermResponseStatus::SUCCESS : OpenThermResponseStatus::INVALID;
		if (processResponseCallback != NULL) {
			processResponseCallback(response, responseStatus);
		}
	}
	else if (st == OpenThermStatus::DELAY) {
		if ((newTs - ts) > 100000) {
			status = OpenThermStatus::READY;
		}
	}
}

bool OpenTherm::parity(unsigned long frame) //odd parity
{
	byte p = 0;
	while (frame > 0)
	{
		if (frame & 1) p++;
		frame = frame >> 1;
	}
	return (p & 1);
}

OpenThermMessageType OpenTherm::getMessageType(unsigned long message)
{
	OpenThermMessageType msg_type = static_cast<OpenThermMessageType>((message >> 28) & 7);
	return msg_type;
}

OpenThermMessageID OpenTherm::getDataID(unsigned long frame)
{
	return (OpenThermMessageID)((frame >> 16) & 0xFF);
}

unsigned long OpenTherm::buildRequest(OpenThermMessageType type, OpenThermMessageID id, unsigned int data)
{
	unsigned long request = data;
	if (type == OpenThermMessageType::WRITE_DATA) {
		request |= 1ul << 28;
	}
	request |= ((unsigned long)id) << 16;
	if (parity(request)) request |= (1ul << 31);
	LastRequestId = id;
	return request;
}

unsigned long OpenTherm::buildResponse(OpenThermMessageType type, OpenThermMessageID id, unsigned int data)
{
	unsigned long response = data;
	response |= ((unsigned long)type) << 28;
	response |= ((unsigned long)id) << 16;
	if (parity(response)) response |= (1ul << 31);
	return response;
}

bool OpenTherm::isValidResponse(unsigned long response)
{
	if (parity(response)) return false;
	byte msgType = (response << 1) >> 29;
	return msgType == READ_ACK || msgType == WRITE_ACK;
}

bool OpenTherm::isValidRequest(unsigned long request)
{
	if (parity(request)) return false;
	byte msgType = (request << 1) >> 29;
	return msgType == READ_DATA || msgType == WRITE_DATA;
}

void OpenTherm::end() {
	if (this->handleInterruptCallback != NULL) {
		detachInterrupt(digitalPinToInterrupt(inPin));
	}
}

const char *OpenTherm::statusToString(OpenThermResponseStatus status)
{
	switch (status)
	{
	case NONE:
		return "NONE";
	case SUCCESS:
		return "SUCCESS";
	case INVALID:
		return "INVALID";
	case TIMEOUT:
		return "TIMEOUT";
	default:
		return "UNKNOWN";
	}
}

const char *OpenTherm::messageTypeToString(OpenThermMessageType message_type)
{
	switch (message_type)
	{
	case READ_DATA:
		return "READ_DATA";
	case WRITE_DATA:
		return "WRITE_DATA";
	case INVALID_DATA:
		return "INVALID_DATA";
	case RESERVED:
		return "RESERVED";
	case READ_ACK:
		return "READ_ACK";
	case WRITE_ACK:
		return "WRITE_ACK";
	case DATA_INVALID:
		return "DATA_INVALID";
	case UNKNOWN_DATA_ID:
		return "UNKNOWN_DATA_ID";
	default:
		return "UNKNOWN";
	}
}

//building requests
unsigned long OpenTherm::buildSetBoilerStatusRequest(bool enableCentralHeating, bool enableHotWater, bool enableCooling, bool enableOutsideTemperatureCompensation, bool enableCentralHeating2, bool enableWinterMode) {
	unsigned int data = enableCentralHeating | (enableHotWater << 1) | (enableCooling << 2) | (enableOutsideTemperatureCompensation << 3) | (enableCentralHeating2 << 4) | (enableWinterMode << 5); 
	data <<= 8;
	if(Immergas_fix) data |= 0xca;
	return buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Status, data);
}

unsigned long OpenTherm::buildSetBoilerTemperatureRequest(float temperature) {
	unsigned int data = temperatureToData(temperature);
	return buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::TSet, data);
}

unsigned long OpenTherm::buildSetBoilerCH2TemperatureRequest(float temperature) {
	unsigned int data = temperatureToData(temperature);
	return buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::TsetCH2, data);
}

unsigned long OpenTherm::buildSetDHWSetpointTemperatureRequest(float temperature) {
	unsigned int data = temperatureToData(temperature);
	return buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::TdhwSet, data);
}

unsigned long OpenTherm::buildGetBoilerTemperatureRequest() {
	return buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tboiler, 0);
}

unsigned long OpenTherm::buildGetBoilerCH2TemperatureRequest() {
	return buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::TflowCH2, 0);
}


//parsing responses
bool OpenTherm::isFault(unsigned long _response) {
	return _response & 0x1;
}

bool OpenTherm::isCentralHeatingActive(unsigned long _response) {
	return _response & 0x2;
}

bool OpenTherm::isHotWaterActive(unsigned long _response) {
	return _response & 0x4;
}

bool OpenTherm::isFlameOn(unsigned long _response) {
	return _response & 0x8;
}

bool OpenTherm::isCoolingActive(unsigned long _response) {
	return _response & 0x10;
}

bool OpenTherm::isDiagnostic(unsigned long _response) {
	return _response & 0x40;
}

uint16_t OpenTherm::getUInt(const unsigned long _response) const {
	const uint16_t u88 = _response & 0xffff;
	return u88;
}

float OpenTherm::getFloat(const unsigned long _response) const {
	const uint16_t u88 = getUInt(_response);
	const float f = (u88 & 0x8000) ? -(0x10000L - u88) / 256.0f : u88 / 256.0f;
	return f;
}

unsigned int OpenTherm::temperatureToData(float temperature) {
	if (temperature < 0) temperature = 0;
	if (temperature > 100) temperature = 100;
	unsigned int data = (unsigned int)(temperature * 256);
	return data;
}

//basic requests
#if 0    

unsigned long OpenTherm::setBoilerStatus(bool enableCentralHeating, bool enableHotWater, bool enableCooling, bool enableOutsideTemperatureCompensation, bool enableCentralHeating2) {
	return sendRequest(buildSetBoilerStatusRequest(enableCentralHeating, enableHotWater, enableCooling, enableOutsideTemperatureCompensation, enableCentralHeating2));
}

bool OpenTherm::setBoilerTemperature(float temperature) {
	unsigned long response_tmp = sendRequest(buildSetBoilerTemperatureRequest(temperature));
	return isValidResponse(response_tmp);
}

float OpenTherm::getBoilerTemperature() {
	unsigned long response_tmp = sendRequest(buildGetBoilerTemperatureRequest());
	return isValidResponse(response_tmp) ? getFloat(response_tmp) : 0;
}

float OpenTherm::getReturnTemperature() {
    unsigned long response_tmp = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Tret, 0));
    return isValidResponse(response_tmp) ? getFloat(response_tmp) : 0;
}

bool OpenTherm::setDHWSetpoint(float temperature) {
    unsigned int data = temperatureToData(temperature);
    unsigned long response_tmp = sendRequest(buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::TdhwSet, data));
    return isValidResponse(response_tmp);
}

float OpenTherm::getDHWTemperature() {
    unsigned long response_tmp = sendRequest(buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tdhw, 0));
    return isValidResponse(response_tmp) ? getFloat(response_tmp) : 0;
}

float OpenTherm::getModulation() {
    unsigned long response_tmp = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::RelModLevel, 0));
    return isValidResponse(response_tmp) ? getFloat(response_tmp) : 0;
}

float OpenTherm::getPressure() {
    unsigned long response_tmp = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::CHPressure, 0));
    return isValidResponse(response_tmp) ? getFloat(response_tmp) : 0;
}

unsigned char OpenTherm::getFault() {
    return ((sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::ASFflags, 0)) >> 8) & 0xff);
}

#endif //0

#if defined(ARDUINO_ARCH_ESP8266)
	#define OTD(x) 
#elif defined(ARDUINO_ARCH_ESP32)
	#define OTD(x) x
#endif

OpenThermID OT_ids[N_OT_NIDS] =
{
 {	Status,					OtRW,	FLAG8,	FLAG8,	1,	0,	0,	OTD("Master and Slave Status flags") },
 { 	TSet,					OtW,	F88,	OTNONE,	0,	0,	0,	OTD("Control setpoint ie CH water temperature setpoint (°C)")},
 {	MConfigMMemberIDcode,	OtW,	FLAG8,	OTU8,	0,	0,	0,	OTD("Master Configuration Flags / Master MemberID Code") }, 
 {	SConfigSMemberIDcode,	OtR,	FLAG8,	OTU8,	0,	0,	0,	OTD("Slave Configuration Flags / Slave MemberID Code") },
 {	RemoteRequest,			OtW,	OTU8,	OTU8,	0,	0,	0,	OTD("Remote Command")},
 {	ASFflags,				OtR,	FLAG8,	OTU8,	0,	0,	0,	OTD("OEM-fault-code Application-specific fault flags and OEM fault code")},
 {	RBPflags,				OtR,	FLAG8,	FLAG8,	0,	0,	0,	OTD("Remote boiler parameter transfer-enable & read/write flags")},
 {	CoolingControl,			OtW,	F88,	OTNONE,	0,	0,	0,	OTD("Cooling control signal (%)")},
 { 	TsetCH2,				OtW,	F88,	OTNONE,	0,	0,	0,	OTD("Control setpoint for 2e CH circuit (°C)")},
 { 	TrOverride,				OtR,	F88,	OTNONE,	0,	0,	0,	OTD("Remote override room setpoint")},

 { 	TSP,					OtR,	OTU8,	OTU8,	0,	0,	0,	OTD("Number of Transparent-Slave-Parameters supported by slave")},
 { 	TSPindexTSPvalue,		OtRW,	OTU8,	OTU8,	0,	0,	0,	OTD("Index number / Value of referred-to transparent slave parameter")},
 { 	FHBsize,				OtR,	OTU8,	OTU8,	0,	0,	0,	OTD("Size of Fault-History-Buffer supported by slave")},
 { 	FHBindexFHBvalue,		OtR,	OTU8,	OTU8,	0,	0,	0,	OTD("Index number / Value of referred-to fault-history buffer entry")},
 { 	MaxRelModLevelSetting,	OtW,	F88,	OTNONE,	0,	0,	0,	OTD("Maximum relative modulation level setting (%)")},
 { 	MaxCapacityMinModLevel,	OtR,	OTU8,	OTU8,	0,	0,	0,	OTD("Maximum boiler capacity (kW) / Minimum boiler modulation level(%)")},
 {	TrSet,					OtW,	F88,	OTNONE,	0,	0,	0,	OTD("Room Setpoint (°C)")},
 { 	RelModLevel,			OtR,	F88,	OTNONE,	0,	0,	0,	OTD("Relative Modulation Level (%)")},
 { 	CHPressure,				OtR,	F88,	OTNONE,	0,	0,	0,	OTD("Water pressure in CH circuit  (bar)")},
 { 	DHWFlowRate,			OtR,	F88,	OTNONE,	0,	0,	0,	OTD("Water flow rate in DHW circuit. (litres/minute)")},

 {	DayTime,				OtRW,	OTSP, 	OTU8, 	0,	0,	0,	OTD("Day of Week and Time of Day")},
 {	Date,					OtRW, 	OTU8,	OTU8,	0,	0,	0,	OTD("Calendar date")},
 {	Year, 					OtRW,	OTU16,	OTNONE,	0,	0,	0,	OTD("Calendar year")},
 {	TrSetCH2,				OtW,	F88,	OTNONE,	0,	0,	0,	OTD("Room Setpoint for 2nd CH circuit (°C)")},
 {	Tr, 					OtW,	F88,	OTNONE,	0,	0,	0,	OTD("Room temperature (°C)")},
 {	Tboiler, 				OtR,	F88,	OTNONE,	0,	0,	0,	OTD("Boiler flow water temperature (°C)")},
 {	Tdhw, 					OtR,	F88,	OTNONE,	0,	0,	0,	OTD("DHW temperature (°C)")},
 {	Toutside, 				OtR,	F88,	OTNONE,	0,	0,	0,	OTD("Outside temperature (°C)")},
 {	Tret, 					OtR,	F88,	OTNONE,	0,	0,	0,	OTD("Return water temperature (°C)")},
 {	Tstorage, 				OtR,	F88,	OTNONE,	0,	0,	0,	OTD("Solar storage temperature (°C)")},

 {	Tcollector,				OtR,	F88,	OTNONE,	0,	0,	0,	OTD("Solar collector temperature (°C)")}, //s16 (p26) or f8.8 (p32) ?? 
 {	TflowCH2,				OtR,	F88,	OTNONE,	0,	0,	0, 	OTD("Flow water temperature CH2 circuit (°C)")},
 {	Tdhw2,					OtR,	F88,	OTNONE,	0,	0,	0, 	OTD("Domestic hot water temperature 2 (°C)")},
 {	Texhaust,				OtR,	OTS16,	OTNONE,	0,	0,	0, 	OTD("Boiler exhaust temperature (°C)")},
 {	TdhwSetUBTdhwSetLB,		OtRW,	OTS8,	OTS8,	0,	0,	0, 	OTD("DHW setpoint upper & lower bounds for adjustment (°C)")}, //48 
 {	MaxTSetUBMaxTSetLB, 	OtRW,	OTS8,	OTS8,	0,	0,	0, 	OTD("Max CH water setpoint upper & lower bounds for adjustment (°C)")},
 {	HcratioUBHcratioLB,  	OtRW,	OTS8,	OTS8,	0,	0,	0, 	OTD("OTC heat curve ratio upper & lower bounds for adjustment")},
 {	TdhwSet,				OtRW,	F88,	OTNONE,	0,	0,	0, 	OTD("DHW setpoint (°C) (Remote parameter 1)")}, // 56
 {	MaxTSet,				OtRW,	F88,	OTNONE,	0,	0,	0, 	OTD("Max CH water setpoint (°C) (Remote parameters 2)")},
 {	Hcratio,				OtRW,	F88,	OTNONE,	0,	0,	0, 	OTD("f8.8  OTC heat curve ratio (°C) (Remote parameter 3)")},

 {	RemoteOverrideFunction,	OtR,	FLAG8,	OTNONE,	0,	0,	0, OTD("Function of manual and program changes in master and remote room setpoint")}, // = 100
 {	OEMDiagnosticCode,		OtR,	OTU16,	OTNONE,	0,	0,	0, OTD("OEM-specific diagnostic/service code")}, // = 115
 {	SuccessfulBurnerStarts,	OtRW,	OTU16,	OTNONE,	0,	0,	0, OTD("Number of succesful starts burner")}, 
 {	CHPumpStarts,			OtRW,	OTU16,	OTNONE,	0,	0,	0, OTD("Number of starts CH pump")},
 {	DHWPumpValveStarts, 	OtRW,	OTU16,	OTNONE,	0,	0,	0, OTD("Number of starts DHW pump/valve")},
 
 {	DHWBurnerStarts,		OtRW,	OTU16,	OTNONE,	0,	0,	0, OTD("Number of starts burner during DHW mode")},
 {	BurnerOperationHours,	OtRW,	OTU16,	OTNONE,	0,	0,	0, OTD("Number of hours that burner is in operation (i.e. flame on)")},
 {	CHPumpOperationHours, 	OtRW,	OTU16,	OTNONE,	0,	0,	0, OTD("Number of hours that CH pump has been running")},
 {	DHWPumpValveOperationHours,
						 	OtRW,	OTU16,	OTNONE,	0,	0,	0,	OTD("Number of hours that DHW pump has been running or DHW valve has been opened")},
 {	DHWBurnerOperationHours, 
						 	OtRW,	OTU16,	OTNONE,	0,	0,	0,	OTD("Number of hours that burner is in operation during DHW mode")},

 {	OpenThermVersionMaster,	OtW,	F88,	OTNONE,	0,	0,	0,	OTD("The implemented version of the OpenTherm Protocol Specification in the master")},
 {	OpenThermVersionSlave,	OtR,	F88,	OTNONE,	0,	0,	0,	OTD("The implemented version of the OpenTherm Protocol Specification in the slave")},
 {	MasterVersion,			OtW, 	OTU8,	OTU8,	0,	0,	0,	OTD("Master product version number and type")},
 {	SlaveVersion,			OtR, 	OTU8,	OTU8,	0,	0,	0,	OTD("Slave product version number and type")}
 
};

byte id_to_index[128];

void OpenTherm::init_OTids(void)
{  int i;
   i = 0;
   for(i=0; i<N_OT_NIDS; i++)
   {	OT_ids[i].used = 2;
   		OT_ids[i].count = 0;
   		OT_ids[i].countOk = 0;

		id_to_index[(OT_ids[i].id)] = i;
   }
   OT_ids[0].used = 1;

}


int OpenTherm::OTid_used(OpenThermMessageID id)
{	int ind;
//	int ind, rc = 0 ;
	ind = id_to_index[id];

//	if(OT_ids[ind].used) rc = 1;
//	return rc;
	return OT_ids[ind].used;
}

void OpenTherm::SetUsed_OTid(int id, int used)
{ 	int ind;
	if(id < 0 || id > 127)
		return;
	ind = id_to_index[id];
	if(ind == -1)
		return;
	OT_ids[ind].used = used;	
}

int OpenTherm::update_OTid(int id, int sts)
{ 	int ind;
	if(id < 0 || id > 127)
		return 1;
	ind = id_to_index[id];
	if(ind == -1)
		return 2;
	if(OT_ids[ind].used == 2)
	{
		OT_ids[ind].count++;
		if(sts )
			OT_ids[ind].countOk++;
//Serial.printf("OT_ids[%d].count %d  %d\n", ind, OT_ids[ind].count, OT_ids[ind].countOk );
		if(OT_ids[ind].count > 8)
		{
    		if(OT_ids[ind].countOk > 4)
				OT_ids[ind].used  = 1;
			else
				OT_ids[ind].used = 0;
		}
	}

	return 0;
}

int OpenTherm::Get_OTid_count(OpenThermMessageID id, int &count, int &countok)
{	int ind;
	ind = id_to_index[id];
	if(ind == -1)
	{	countok = count = 0;
		return 0;
	}
	count = OT_ids[ind].count;
	countok= OT_ids[ind].countOk;
	return OT_ids[ind].used;
}

// use https://github.com/Jeroen88/EasyOpenTherm/blob/main/src/EasyOpenTherm.h
OpenThermVendor OTvendorList[] =
{	1,  "Baxi Fourtech/Luna 3",
    2,  "AWB/Brink/Viessmann", 
	4,  "Baxi Slim", //    4:    "ATAG/Baxi/Brötje/ELCO/GEMINOX",
    5,  "Itho Daalderop",
    6,  "IDEAL",
    8,  "Buderus/Bosch/Hoval",
	9,  "Ferrolli",
    11,  "Remeha/De Dietrich",
	13, "Lamborghini", // sb f24
	16, "Unical",
	24, "Vaillant/Bulex",
	27, "Baxi Eco4s/Luna Duo-Tec P67=0", //Baxi Nuvola B40
	29, "Itho Daalderop",
	33, "Viessmann",
	41, "Italtherm/Radiant",
	56,	"Baxi Luna Duo-Tec P67=2",
	125, "Kotitonttu", // Toivo Т24 OK with OpenTherm
	131, "Nefit",
    148, "Navien",
    173, "Intergas",
    247, "Baxi Ampera",
    248, "Zota" // Lux-X, mk-s plus
};

const char * GetOTVendorName(int id)
{	int i, n;
	n = sizeof(OTvendorList)/sizeof(OpenThermVendor);
	for(i=0;i<n;i++)
	{	if(id == OTvendorList[i].id)
		{	return OTvendorList[i].name;
		}
	}
	return NULL;
}