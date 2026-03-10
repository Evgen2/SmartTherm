/*
OpenTherm.h - OpenTherm Library for the ESP8266 & ESP32/Arduino platform

based on v 1.13 + 5 July 2022

https://github.com/ihormelnyk/OpenTherm
http://ihormelnyk.com/pages/OpenTherm
Licensed under MIT license
Copyright 2018, Ihor Melnyk

Frame Structure:
P MGS-TYPE SPARE DATA-ID  DATA-VALUE
0 000      0000  00000000 00000000 00000000
*/

#ifndef OpenTherm_h
#define OpenTherm_h

#include <stdint.h>
#include <Arduino.h>



enum OpenThermResponseStatus {
	NONE,
	SUCCESS,
	INVALID,
	TIMEOUT
};

#if defined(ARDUINO_ARCH_ESP32) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
enum OpenThermMessageType {
	/*  Master to Slave */
	READ_DATA       = 0b000,
	READ            = READ_DATA, // for backwared compatibility
	WRITE_DATA      = 0b001,
	WRITE           = WRITE_DATA, // for backwared compatibility
	INVALID_DATA    = 0b010,
	RESERVED        = 0b011,
	/* Slave to Master */
	READ_ACK        = 0b100,
	WRITE_ACK       = 0b101,
	DATA_INVALID    = 0b110,
	UNKNOWN_DATA_ID = 0b111
};

#else
enum OpenThermMessageType {
	/*  Master to Slave */
	READ_DATA       = B000,
	READ            = READ_DATA, // for backwared compatibility
	WRITE_DATA      = B001,
	WRITE           = WRITE_DATA, // for backwared compatibility
	INVALID_DATA    = B010,
	RESERVED        = B011,
	/* Slave to Master */
	READ_ACK        = B100,
	WRITE_ACK       = B101,
	DATA_INVALID    = B110,
	UNKNOWN_DATA_ID = B111
};
#endif

typedef OpenThermMessageType OpenThermRequestType; // for backwared compatibility

enum  OpenThermMessageID {
/*
В частности по регистру ID0:

ID0:HB0: Master status: CH enable
ID0:HB1: Master status: DHW enable
ID0:HB2: Master status: Cooling enable
ID0:HB3: Master status: OTC active  (OTC = Outside Temperature Compensation)
ID0:HB4: Master status: CH2 enable
ID0:HB5: Master status: Summer/winter mode
ID0:HB6: Master status: DHW blocking
ID0:LB0: Slave Status: Fault indication
ID0:LB1: Slave Status: CH mode
ID0:LB2: Slave Status: DHW mode
ID0:LB3: Slave Status: Flame status
ID0:LB4: Slave Status: Cooling status
ID0:LB5: Slave Status: CH2 mode
ID0:LB6: Slave Status: Diagnostic/service indication
ID0:LB7: Slave Status: Electricity production (???)
*/	
    Status                                       = 0, // flag8/flag8  Master and Slave Status flags.
    TSet                                         = 1, // f8.8    Control Setpoint i.e.CH water temperature Setpoint(°C)
    MConfigMMemberIDcode                         = 2, // flag8/u8  Master Configuration Flags / Master MemberID Code
    SConfigSMemberIDcode                         = 3, // flag8/u8  Slave Configuration Flags / Slave MemberID Code
    RemoteRequest                                = 4, // u8/u8     Remote Request
    ASFflags                                     = 5, // flag8/u8  Application - specific fault flags and OEM fault code
    RBPflags                                     = 6, // flag8/flag8   Remote boiler parameter transfer - enable & read / write flags
    CoolingControl                               = 7, // f8.8    Cooling control signal(%)
    TsetCH2                                      = 8, // f8.8    Control Setpoint for 2e CH circuit(°C)
    TrOverride                                   = 9, // f8.8    Remote override room Setpoint
    TSP                                         = 10, // u8/u8     Number of Transparent - Slave - Parameters supported by slave
    TSPindexTSPvalue                            = 11, // u8/u8     Index number / Value of referred - to transparent slave parameter.
    FHBsize                                     = 12, // u8/u8     Size of Fault - History - Buffer supported by slave
    FHBindexFHBvalue                            = 13, // u8/u8     Index number / Value of referred - to fault - history buffer entry.
    MaxRelModLevelSetting                       = 14, // f8.8    Maximum relative modulation level setting(%)
    MaxCapacityMinModLevel                      = 15, // u8/u8     Maximum boiler capacity(kW) / Minimum boiler modulation level(%)
    TrSet                                       = 16, // f8.8    Room Setpoint(°C)
    RelModLevel                                 = 17, // f8.8    Relative Modulation Level(%)
    CHPressure                                  = 18, // f8.8    Water pressure in CH circuit(bar)
    DHWFlowRate                                 = 19, // f8.8    Water flow rate in DHW circuit. (litres / minute)
    DayTime                                     = 20, // special/u8    Day of Week and Time of Day
    Date                                        = 21, // u8/u8     Calendar date
    Year                                        = 22, // u16     Calendar year
    TrSetCH2                                    = 23, // f8.8    Room Setpoint for 2nd CH circuit(°C)
    Tr                                          = 24, // f8.8    Room temperature(°C)
    Tboiler                                     = 25, // f8.8    Boiler flow water temperature(°C)
    Tdhw                                        = 26, // f8.8    DHW temperature(°C)
    Toutside                                    = 27, // f8.8    Outside temperature(°C)
    Tret                                        = 28, // f8.8    Return water temperature(°C)
    Tstorage                                    = 29, // f8.8    Solar storage temperature(°C)
    Tcollector                                  = 30, // f8.8    Solar collector temperature(°C)
    TflowCH2                                    = 31, // f8.8    Flow water temperature CH2 circuit(°C)
    Tdhw2                                       = 32, // f8.8    Domestic hot water temperature 2 (°C)
    Texhaust                                    = 33, // s16     Boiler exhaust temperature(°C)
    TboilerHeatExchanger                        = 34, // f8.8    Boiler heat exchanger temperature(°C)
    BoilerFanSpeedSetpointAndActual             = 35, // u8/u8     Boiler fan speed Setpoint and actual value
    FlameCurrent                                = 36, // f8.8    Electrical current through burner flame[μA]
    TrCH2                                       = 37, // f8.8    Room temperature for 2nd CH circuit(°C)
    RelativeHumidity                            = 38, // f8.8    Actual relative humidity as a percentage
    TrOverride2                                 = 39, // f8.8    Remote Override Room Setpoint 2

    TdhwSetUBTdhwSetLB                          = 48, // s8/s8     DHW Setpoint upper & lower bounds for adjustment(°C)
    MaxTSetUBMaxTSetLB                          = 49, // s8/s8     Max CH water Setpoint upper & lower bounds for adjustment(°C)
    HcratioUBHcratioLB				= 50, // s8 / s8  OTC heat curve ratio upper & lower bounds for adjustment

    TdhwSet                                     = 56, // f8.8    DHW Setpoint(°C) (Remote parameter 1)
    MaxTSet                                     = 57, // f8.8    Max CH water Setpoint(°C) (Remote parameters 2)
    Hcratio					= 58, // f8.8	OTC heat curve ratio (°C)  (Remote parameter 3)

    StatusVentilationHeatRecovery               = 70, // flag8/flag8   Master and Slave Status flags ventilation / heat - recovery
    Vset                                        = 71, // -/u8  Relative ventilation position (0-100%). 0% is the minimum set ventilation and 100% is the maximum set ventilation. 
    ASFflagsOEMfaultCodeVentilationHeatRecovery = 72, // flag8/u8  Application-specific fault flags and OEM fault code ventilation / heat-recovery 
    OEMDiagnosticCodeVentilationHeatRecovery    = 73, // u16     An OEM-specific diagnostic/service code for ventilation / heat-recovery system 
    SConfigSMemberIDCodeVentilationHeatRecovery = 74, // flag8/u8  Slave Configuration Flags / Slave MemberID Code ventilation / heat-recovery 
    OpenThermVersionVentilationHeatRecovery     = 75, // f8.8    The implemented version of the OpenTherm Protocol Specification in the ventilation / heat-recovery system. 
    VentilationHeatRecoveryVersion              = 76, // u8/u8     Ventilation / heat-recovery product version number and type 
    RelVentLevel                                = 77, // -/u8  Relative ventilation (0-100%) 
    RHexhaust                                   = 78, // -/u8  Relative humidity exhaust air (0-100%) 
    CO2exhaust                                  = 79, // u16     CO2 level exhaust air (0-2000 ppm) 
    Tsi                                         = 80, // f8.8    Supply inlet temperature (°C) 
    Tso                                         = 81, // f8.8    Supply outlet temperature (°C) 
    Tei                                         = 82, // f8.8    Exhaust inlet temperature (°C) 
    Teo                                         = 83, // f8.8    Exhaust outlet temperature (°C) 
    RPMexhaust                                  = 84, // u16     Exhaust fan speed in rpm 
    RPMsupply                                   = 85, // u16     Supply fan speed in rpm 
    RBPflagsVentilationHeatRecovery             = 86, // flag8/flag8   Remote ventilation / heat-recovery parameter transfer-enable & read/write flags 
    NominalVentilationValue                     = 87, // u8/-  Nominal relative value for ventilation (0-100 %) 
    TSPventilationHeatRecovery                  = 88, // u8/u8     Number of Transparent-Slave-Parameters supported by TSP’s ventilation / heat-recovery 
    TSPindexTSPvalueVentilationHeatRecovery     = 89, // u8/u8     Index number / Value of referred-to transparent TSP’s ventilation / heat-recovery parameter. 
    FHBsizeVentilationHeatRecovery              = 90, // u8/u8     Size of Fault-History-Buffer supported by ventilation / heat-recovery 
    FHBindexFHBvalueVentilationHeatRecovery     = 91, // u8/u8     Index number / Value of referred-to fault-history buffer entry ventilation / heat-recovery 

    Brand                                       = 93, // u8/u8     Index number of the character in the text string ASCII character referenced by the above index number 
    BrandVersion                                = 94, // u8/u8     Index number of the character in the text string ASCII character referenced by the above index number 
    BrandSerialNumber                           = 95, // u8/u8     Index number of the character in the text string ASCII character referenced by the above index number 
    CoolingOperationHours                       = 96, // u16     Number of hours that the slave is in Cooling Mode. Reset by zero is optional for slave 
    PowerCycles                                 = 97, // u16     Number of Power Cycles of a slave (wake-up after Reset), Reset by zero is optional for slave 
    RFsensorStatusInformation                   = 98, // special/special   For a specific RF sensor the RF strength and battery level is written 
    RemoteOverrideOperatingModeHeatingDHW       = 99, // special/special   Operating Mode HC1, HC2/ Operating Mode DHW 
    RemoteOverrideFunction                     = 100, // flag8/-   Function of manual and program changes in master and remote room Setpoint 
    StatusSolarStorage                         = 101, // flag8/flag8   Master and Slave Status flags Solar Storage 
    ASFflagsOEMfaultCodeSolarStorage           = 102, // flag8/u8  Application-specific fault flags and OEM fault code Solar Storage 
    SConfigSMemberIDcodeSolarStorage           = 103, // flag8/u8  Slave Configuration Flags / Slave MemberID Code Solar Storage 
    SolarStorageVersion                        = 104, // u8/u8     Solar Storage product version number and type
    TSPSolarStorage                            = 105, // u8/u8     Number of Transparent - Slave - Parameters supported by TSP’s Solar Storage
    TSPindexTSPvalueSolarStorage               = 106, // u8/u8     Index number / Value of referred - to transparent TSP’s Solar Storage parameter.
    FHBsizeSolarStorage                        = 107, // u8/u8     Size of Fault - History - Buffer supported by Solar Storage
    FHBindexFHBvalueSolarStorage               = 108, // u8/u8     Index number / Value of referred - to fault - history buffer entry Solar Storage
    ElectricityProducerStarts                  = 109, // U16     Number of start of the electricity producer.
    ElectricityProducerHours                   = 110, // U16     Number of hours the electricity produces is in operation
    ElectricityProduction                      = 111, // U16     Current electricity production in Watt.
    CumulativElectricityProduction             = 112, // U16     Cumulative electricity production in KWh.
    UnsuccessfulBurnerStarts                   = 113, // u16     Number of un - successful burner starts
    FlameSignalTooLowNumber                    = 114, // u16     Number of times flame signal was too low
    OEMDiagnosticCode                          = 115, // u16     OEM - specific diagnostic / service code

    SuccessfulBurnerStarts                     = 116, // u16     Number of succesful starts burner
    CHPumpStarts                               = 117, // u16     Number of starts CH pump
    DHWPumpValveStarts                         = 118, // u16     Number of starts DHW pump / valve
    DHWBurnerStarts                            = 119, // u16     Number of starts burner during DHW mode
    BurnerOperationHours                       = 120, // u16     Number of hours that burner is in operation(i.e.flame on)
    CHPumpOperationHours                       = 121, // u16     Number of hours that CH pump has been running
    DHWPumpValveOperationHours                 = 122, // u16     Number of hours that DHW pump has been running or DHW valve has been opened
    DHWBurnerOperationHours                    = 123, // u16     Number of hours that burner is in operation during DHW mode
    OpenThermVersionMaster                     = 124, // f8.8    The implemented version of the OpenTherm Protocol Specification in the master.
    OpenThermVersionSlave                      = 125, // f8.8    The implemented version of the OpenTherm Protocol Specification in the slave.
    MasterVersion                              = 126, // u8/u8   Master product version number and type
    SlaveVersion                               = 127, // u8/u8   Slave product version number and type
};

enum OpenThermStatus {
	NOT_INITIALIZED,
	READY,
	DELAY,
	REQUEST_SENDING,
	RESPONSE_WAITING,
	RESPONSE_START_BIT,
	RESPONSE_RECEIVING,
	RESPONSE_READY,
	RESPONSE_INVALID
};

class OpenTherm
{
public:

	OpenTherm(int inPin = 4, int outPin = 5, bool isSlave = false);
	int LastRequestId;
	volatile OpenThermStatus status;
	unsigned long Lastresponse;
	bool Immergas_fix; 
// (1) https://arduino.ru/forum/programmirovanie/termostat-opentherm-na-esp8266?page=15#comment-649392
// (2) https://github.com/Laxilef/OTGateway/releases v1.4.5
// (3) https://github.com/esp32m/core/blob/master/esp32m/include/esp32m/dev/opentherm.hpp 609 class ImmergasMaster 

//	void begin(void(*handleInterruptCallback)(void)); 
	void begin(void(*handleInterruptCallback)(void), void(*processResponseCallback)(unsigned long, OpenThermResponseStatus));
	
	void init_OTids(void);
	int update_OTid(int id, int sts);
    void SetUsed_OTid(int id, int used);
	int OTid_used(OpenThermMessageID id);
	int Get_OTid_count(OpenThermMessageID id, int &count, int &countok);

	bool isReady();
	unsigned long sendRequest(unsigned long request);
	bool sendResponse(unsigned long request);
	bool sendRequestAync(unsigned long request);
	unsigned long buildRequest(OpenThermMessageType type, OpenThermMessageID id, unsigned int data);
	unsigned long buildResponse(OpenThermMessageType type, OpenThermMessageID id, unsigned int data);
	unsigned long getLastResponse();
	OpenThermResponseStatus getLastResponseStatus();
	const char *statusToString(OpenThermResponseStatus status);
	void handleInterrupt();
	void process();
	void end();

	bool parity(unsigned long frame);
	OpenThermMessageType getMessageType(unsigned long message);
	OpenThermMessageID getDataID(unsigned long frame);
	const char *messageTypeToString(OpenThermMessageType message_type);
	bool isValidRequest(unsigned long request);
	bool isValidResponse(unsigned long response);

	//requests
	unsigned long buildSetBoilerStatusRequest(bool enableCentralHeating, bool enableHotWater = false, bool enableCooling = false, bool enableOutsideTemperatureCompensation = false, bool enableCentralHeating2 = false, bool enableWinterMode = false);
	unsigned long buildSetBoilerTemperatureRequest(float temperature);
	unsigned long buildSetBoilerCH2TemperatureRequest(float temperature);
	unsigned long buildGetBoilerTemperatureRequest();
	unsigned long buildGetBoilerCH2TemperatureRequest();
	unsigned long buildSetDHWSetpointTemperatureRequest(float temperature);

	//responses
	bool isFault(unsigned long response);
	bool isCentralHeatingActive(unsigned long response);
	bool isHotWaterActive(unsigned long response);
	bool isFlameOn(unsigned long response);
	bool isCoolingActive(unsigned long response);
	bool isDiagnostic(unsigned long response);
	uint16_t getUInt(const unsigned long response) const;
	float getFloat(const unsigned long response) const;	
	unsigned int temperatureToData(float temperature);

	//basic requests
#if 0	
	unsigned long setBoilerStatus(bool enableCentralHeating, bool enableHotWater = false, bool enableCooling = false, bool enableOutsideTemperatureCompensation = false, bool enableCentralHeating2 = false);
	bool setBoilerTemperature(float temperature);
	float getBoilerTemperature();
    float getReturnTemperature();
    bool setDHWSetpoint(float temperature);
    float getDHWTemperature();
    float getModulation();
    float getPressure();
    unsigned char getFault();
#endif
private:
	const int inPin;
	const int outPin;
	const bool isSlave;

	volatile unsigned long response;
	volatile OpenThermResponseStatus responseStatus;
	volatile unsigned long responseTimestamp;
	volatile byte responseBitIndex;

	int readState();
	void setActiveState();
	void setIdleState();
	void activateBoiler(int wait);

	void sendBit(bool high);
	void(*handleInterruptCallback)();
	void(*processResponseCallback)(unsigned long, OpenThermResponseStatus);
};


enum OpenThermRWtype {
	ONONE,
	OtR,
	OtW,
	OtRW
};

enum OpenThermMSGpartype {
	OTNONE,
	FLAG8, //flag8
	OTU8,  // u8
	OTS8,  // s8
	OTU16, // u16
	OTS16, // s16
	F88,   // f8.8
	OTSP   // special (DAY TIME only)
};


#define N_OT_NIDS 54

//ESP8266: too small memory
#pragma pack(1)
class OpenThermID
{   
public:
	byte /* OpenThermMessageID */ id;
	byte rw:2; //read 0x01, write 0x02
	byte ptype1:3; //parameter 1 type
	byte ptype2:3; //parameter 2 type
	byte used:2; //0 not used, 1 used, 2 not tested
	byte count:7; //
	byte countOk:7; //
#if defined(ARDUINO_ARCH_ESP8266)
//none
#elif defined(ARDUINO_ARCH_ESP32)
	const char *descript; //description
#endif
};
#pragma pack()

#ifndef ICACHE_RAM_ATTR
#define ICACHE_RAM_ATTR
#endif

#ifndef IRAM_ATTR
#define IRAM_ATTR ICACHE_RAM_ATTR
#endif


struct OpenThermVendor
{	int id;
	const char *name;
};

const char * GetOTVendorName(int id);

#if OT_MASTER_DEBUG || OT2_SLAVE_DEBUG 

struct otst
{	unsigned int status;
	unsigned int state;
	unsigned long t;
    int ind;
    int resp;
};

#endif //OT_MASTER_DEBUG || OT2_SLAVE_DEBUG 
#endif // OpenTherm_h
