////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	AioCommon.h	:	Common header file																				
//	Parts that need to be changed when adding boards		None													
//	Parts that need to be changed when adding functions		I/O control code										
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <linux/ioctl.h>
#include <pthread.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Macro definition																								
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DEVICE_MAX								256		// Maximum number of devices 
#define PROCESS_MAX								16		// Maximum number of processes
#define AICH_MAX								256		// Maximum number of AI channels
#define AOCH_MAX								256		// Maximum number of AO channels
#define	CNTCH_MAX								2		// Maximum number of CNT channels
#define SET_FUNC								0		// Set function flag
#define GET_FUNC								1		// Get function flag
#define	CNTM_MAX_CH								8		// Maximum value of channel
#define	CNTM_MAX_COMP_REG						2		// Maximum value of comparison register
#define	CNTM_MAX_SENCE_NUM						1024	// Size of interrupt queue
#define INVALID_HANDLE_VALUE 					-1


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	I/O control code
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define	CAIO_AIO_IOC_MAGIC						'A'		// Magic No.
#define	CAIO_DIO_IOC_MAGIC						'D'		// Magic No.
#define	CAIO_CNT_IOC_MAGIC						'C'		// Magic No.
#define	CAIO_OTHER_IOC_MAGIC					'O'		// Magic No.

// API-AIO common
#define	IOCTL_AIOINIT							_IOWR(CAIO_AIO_IOC_MAGIC, 0, PAIO_COMMON)
#define	IOCTL_AIOEXIT							_IOWR(CAIO_AIO_IOC_MAGIC, 1, PAIO_COMMON)
#define	IOCTL_AIOCHECKDEVICE					_IOWR(CAIO_AIO_IOC_MAGIC, 2, PAIO_COMMON)
#define	IOCTL_AIORESETDEVICE					_IOWR(CAIO_AIO_IOC_MAGIC, 3, PAIO_COMMON)
#define	IOCTL_AIOIO								_IOWR(CAIO_AIO_IOC_MAGIC, 4, PAIO_COMMON)
#define	IOCTL_AIOCONTROLFILTER					_IOWR(CAIO_AIO_IOC_MAGIC, 5, PAIO_COMMON)
#define	IOCTL_AIOECUSIGNAL						_IOWR(CAIO_AIO_IOC_MAGIC, 6, PAIO_COMMON)
#define	IOCTL_AIOGETADDRESS						_IOWR(CAIO_AIO_IOC_MAGIC, 7, PAIO_COMMON)
#define	IOCTL_AIODEVICEREVISION					_IOWR(CAIO_AIO_IOC_MAGIC, 8, PAIO_COMMON)

// AI function
#define	IOCTL_AIOAIINPUTMETHOD					_IOWR(CAIO_AIO_IOC_MAGIC, 100, PAIO_AI)
#define	IOCTL_AIOAICHANNELS						_IOWR(CAIO_AIO_IOC_MAGIC, 101, PAIO_AI)
#define	IOCTL_AIOAIRANGE						_IOWR(CAIO_AIO_IOC_MAGIC, 102, PAIO_AI)
#define	IOCTL_AIOAIRANGEALL						_IOWR(CAIO_AIO_IOC_MAGIC, 103, PAIO_AI)
#define	IOCTL_AIOAIMEMORYTYPE					_IOWR(CAIO_AIO_IOC_MAGIC, 104, PAIO_AI)
#define	IOCTL_AIOAIREPEATTIMES			       	_IOWR(CAIO_AIO_IOC_MAGIC, 105, PAIO_AI)
#define	IOCTL_AIOAICLOCKTYPE					_IOWR(CAIO_AIO_IOC_MAGIC, 106, PAIO_AI)
#define	IOCTL_AIOAISAMPLINGCLOCK				_IOWR(CAIO_AIO_IOC_MAGIC, 107, PAIO_AI)
#define	IOCTL_AIOAISTARTTRIGGER					_IOWR(CAIO_AIO_IOC_MAGIC, 108, PAIO_AI)
#define	IOCTL_AIOAISTARTLEVEL					_IOWR(CAIO_AIO_IOC_MAGIC, 109, PAIO_AI)
#define	IOCTL_AIOAISTOPTRIGGER					_IOWR(CAIO_AIO_IOC_MAGIC, 110, PAIO_AI)
#define	IOCTL_AIOAISTOPTIMES					_IOWR(CAIO_AIO_IOC_MAGIC, 111, PAIO_AI)
#define	IOCTL_AIOAISTOPLEVEL					_IOWR(CAIO_AIO_IOC_MAGIC, 112, PAIO_AI)
#define	IOCTL_AIOAISTOPDELAYTIMES				_IOWR(CAIO_AIO_IOC_MAGIC, 113, PAIO_AI)
#define	IOCTL_AIOAIEVENTSAMPLINGTIMES			_IOWR(CAIO_AIO_IOC_MAGIC, 114, PAIO_AI)
#define	IOCTL_AIOSETAICONDITION					_IOWR(CAIO_AIO_IOC_MAGIC, 115, PAIO_AI)
#define	IOCTL_AIOSINGLEAI						_IOWR(CAIO_AIO_IOC_MAGIC, 116, PAIO_AI)
#define	IOCTL_AIOMULTIAI						_IOWR(CAIO_AIO_IOC_MAGIC, 117, PAIO_AI)
#define	IOCTL_AIOSTARTAI						_IOWR(CAIO_AIO_IOC_MAGIC, 118, PAIO_AI)
#define	IOCTL_AIOSTOPAI							_IOWR(CAIO_AIO_IOC_MAGIC, 119, PAIO_AI)
#define	IOCTL_AIOGETAISTATUS					_IOWR(CAIO_AIO_IOC_MAGIC, 120, PAIO_AI)
#define	IOCTL_AIOGETAISAMPLINGCOUNT				_IOWR(CAIO_AIO_IOC_MAGIC, 121, PAIO_AI)
#define	IOCTL_AIOGETAIREPEATCOUNT				_IOWR(CAIO_AIO_IOC_MAGIC, 122, PAIO_AI)
#define	IOCTL_AIOGETAISTOPTRIGGERCOUNT			_IOWR(CAIO_AIO_IOC_MAGIC, 123, PAIO_AI)
#define	IOCTL_AIOGETAISAMPLINGDATA				_IOWR(CAIO_AIO_IOC_MAGIC, 124, PAIO_AI)
#define	IOCTL_AIORESETAISTATUS					_IOWR(CAIO_AIO_IOC_MAGIC, 125, PAIO_AI)
#define	IOCTL_AIORESETAIMEMORY					_IOWR(CAIO_AIO_IOC_MAGIC, 126, PAIO_AI)
#define	IOCTL_AIOAIMAXCHANNELS					_IOWR(CAIO_AIO_IOC_MAGIC, 127, PAIO_AI)
#define	IOCTL_AIOAIRESOLUTION					_IOWR(CAIO_AIO_IOC_MAGIC, 128, PAIO_AI)
#define	IOCTL_AIOAIMEMORYSIZE					_IOWR(CAIO_AIO_IOC_MAGIC, 129, PAIO_AI)
#define	IOCTL_AIOAIRANGEARRAY					_IOWR(CAIO_AIO_IOC_MAGIC, 130, PAIO_AI)
#define	IOCTL_AIOAISEQUENCE						_IOWR(CAIO_AIO_IOC_MAGIC, 131, PAIO_AI)
#define	IOCTL_AIOAICALLBACKPROC					_IOWR(CAIO_AIO_IOC_MAGIC, 132, PAIO_AI)
#define	IOCTL_AIOAITRANSFERMODE					_IOWR(CAIO_AIO_IOC_MAGIC, 133, PAIO_AI)
#define	IOCTL_AIOAITRANSFERDATA					_IOWR(CAIO_AIO_IOC_MAGIC, 134, PAIO_AI)
#define	IOCTL_AIOAIATTACHEDDATA					_IOWR(CAIO_AIO_IOC_MAGIC, 135, PAIO_AI)
#define	IOCTL_AIOGETAISAMPLINGDATASIZE			_IOWR(CAIO_AIO_IOC_MAGIC, 136, PAIO_AI)
#define	IOCTL_AIOAISTARTINRANGE					_IOWR(CAIO_AIO_IOC_MAGIC, 137, PAIO_AI)
#define	IOCTL_AIOAISTARTOUTRANGE				_IOWR(CAIO_AIO_IOC_MAGIC, 138, PAIO_AI)
#define	IOCTL_AIOAISTOPINRANGE					_IOWR(CAIO_AIO_IOC_MAGIC, 139, PAIO_AI)
#define	IOCTL_AIOAISTOPOUTRANGE					_IOWR(CAIO_AIO_IOC_MAGIC, 140, PAIO_AI)
#define	IOCTL_AIOAIEVENTTRANSFERTIMES			_IOWR(CAIO_AIO_IOC_MAGIC, 141, PAIO_AI)
#define	IOCTL_AIOGETAITRANSFERCOUNT				_IOWR(CAIO_AIO_IOC_MAGIC, 142, PAIO_AI)
#define	IOCTL_AIOGETAITRANSFERLAP				_IOWR(CAIO_AIO_IOC_MAGIC, 143, PAIO_AI)
#define	IOCTL_AIOGETAISTOPTRIGGERTRANSFERCOUNT	_IOWR(CAIO_AIO_IOC_MAGIC, 144, PAIO_AI)
#define	IOCTL_AIOAIFREEBMBUFFER					_IOWR(CAIO_AIO_IOC_MAGIC, 147, PAIO_AI)
#define	IOCTL_AIOAISCANCLOCK					_IOWR(CAIO_AIO_IOC_MAGIC, 148, PAIO_AI)
#define	IOCTL_AIOAIMINSAMPLINGCLOCK				_IOWR(CAIO_AIO_IOC_MAGIC, 149, PAIO_AI)
#define	IOCTL_AIOAICLOCKEDGE					_IOWR(CAIO_AIO_IOC_MAGIC, 150, PAIO_AI)
#define	IOCTL_AIOAIDEVICDBUFFERMODE				_IOWR(CAIO_AIO_IOC_MAGIC, 151, PAIO_AI)
#define	IOCTL_AIOAICHANNEL						_IOWR(CAIO_AIO_IOC_MAGIC, 152, PAIO_AI)
#define IOCTL_AIOAITHWAKEUP						_IOWR(CAIO_AIO_IOC_MAGIC, 153, PAIO_AI)

// AO function
#define	IOCTL_AIOAOOUTPUTMODE					_IOWR(CAIO_AIO_IOC_MAGIC, 200, PAIO_AO)
#define	IOCTL_AIOAOCHANNELS						_IOWR(CAIO_AIO_IOC_MAGIC, 201, PAIO_AO)
#define	IOCTL_AIOAORANGE						_IOWR(CAIO_AIO_IOC_MAGIC, 202, PAIO_AO)
#define	IOCTL_AIOAORANGEALL						_IOWR(CAIO_AIO_IOC_MAGIC, 203, PAIO_AO)
#define	IOCTL_AIOAOMEMORYTYPE					_IOWR(CAIO_AIO_IOC_MAGIC, 204, PAIO_AO)
#define	IOCTL_AIOAOREPEATTIMES					_IOWR(CAIO_AIO_IOC_MAGIC, 205, PAIO_AO)
#define	IOCTL_AIOAOCLOCKTYPE					_IOWR(CAIO_AIO_IOC_MAGIC, 206, PAIO_AO)
#define	IOCTL_AIOAOSAMPLINGCLOCK				_IOWR(CAIO_AIO_IOC_MAGIC, 207, PAIO_AO)
#define	IOCTL_AIOAOSTARTTRIGGER					_IOWR(CAIO_AIO_IOC_MAGIC, 208, PAIO_AO)
#define	IOCTL_AIOAOSTOPTRIGGER					_IOWR(CAIO_AIO_IOC_MAGIC, 209, PAIO_AO)
#define	IOCTL_AIOAOSAMPLINGDATA					_IOWR(CAIO_AIO_IOC_MAGIC, 210, PAIO_AO)
#define	IOCTL_AIOAOEVENTSAMPLINGTIMES			_IOWR(CAIO_AIO_IOC_MAGIC, 211, PAIO_AO)
#define	IOCTL_AIOSETAOCONDITION					_IOWR(CAIO_AIO_IOC_MAGIC, 212, PAIO_AO)
#define	IOCTL_AIOSINGLEAO						_IOWR(CAIO_AIO_IOC_MAGIC, 213, PAIO_AO)
#define	IOCTL_AIOMULTIAO						_IOWR(CAIO_AIO_IOC_MAGIC, 214, PAIO_AO)
#define	IOCTL_AIOSTARTAO						_IOWR(CAIO_AIO_IOC_MAGIC, 215, PAIO_AO)
#define	IOCTL_AIOSTOPAO							_IOWR(CAIO_AIO_IOC_MAGIC, 216, PAIO_AO)
#define	IOCTL_AIOGETAOSTATUS					_IOWR(CAIO_AIO_IOC_MAGIC, 218, PAIO_AO)
#define	IOCTL_AIOGETAOSAMPLINGCOUNT				_IOWR(CAIO_AIO_IOC_MAGIC, 219, PAIO_AO)
#define	IOCTL_AIOGETAOREPEATCOUNT				_IOWR(CAIO_AIO_IOC_MAGIC, 220, PAIO_AO)
#define	IOCTL_AIORESETAOSTATUS					_IOWR(CAIO_AIO_IOC_MAGIC, 221, PAIO_AO)
#define	IOCTL_AIORESETAOMEMORY					_IOWR(CAIO_AIO_IOC_MAGIC, 222, PAIO_AO)
#define	IOCTL_AIOAOMAXCHANNELS					_IOWR(CAIO_AIO_IOC_MAGIC, 223, PAIO_AO)
#define	IOCTL_AIOAORESOLUTION					_IOWR(CAIO_AIO_IOC_MAGIC, 224, PAIO_AO)
#define	IOCTL_AIOAORANGEARRAY					_IOWR(CAIO_AIO_IOC_MAGIC, 226, PAIO_AO)
#define	IOCTL_AIOAOCALLBACKPROC					_IOWR(CAIO_AIO_IOC_MAGIC, 227, PAIO_AO)
#define	IOCTL_AIOAOTRANSFERMODE					_IOWR(CAIO_AIO_IOC_MAGIC, 228, PAIO_AO)
#define	IOCTL_AIOAOTRANSFERDATA					_IOWR(CAIO_AIO_IOC_MAGIC, 229, PAIO_AO)
#define	IOCTL_AIOAOATTACHEDDATA					_IOWR(CAIO_AIO_IOC_MAGIC, 230, PAIO_AO)
#define	IOCTL_AIOGETAOSAMPLINGDATASIZE			_IOWR(CAIO_AIO_IOC_MAGIC, 231, PAIO_AO)
#define	IOCTL_AIOAOEVENTTRANSFERTIMES			_IOWR(CAIO_AIO_IOC_MAGIC, 232, PAIO_AO)
#define	IOCTL_AIOGETAOTRANSFERCOUNT				_IOWR(CAIO_AIO_IOC_MAGIC, 233, PAIO_AO)
#define	IOCTL_AIOGETAOTRANSFERLAP				_IOWR(CAIO_AIO_IOC_MAGIC, 234, PAIO_AO)
#define	IOCTL_AIOAOFREEBMBUFFER					_IOWR(CAIO_AIO_IOC_MAGIC, 237, PAIO_AO)
#define	IOCTL_AIOAOMINSAMPLINGCLOCK				_IOWR(CAIO_AIO_IOC_MAGIC, 238, PAIO_AO)
#define	IOCTL_AIOAOCLOCKEDGE					_IOWR(CAIO_AIO_IOC_MAGIC, 239, PAIO_AO)
#define	IOCTL_AIOAODEVICDBUFFERMODE				_IOWR(CAIO_AIO_IOC_MAGIC, 240, PAIO_AO)
#define	IOCTL_AIOAOREPEATMODE					_IOWR(CAIO_AIO_IOC_MAGIC, 241, PAIO_AO)
#define IOCTL_AIOAOTHWAKEUP						_IOWR(CAIO_AIO_IOC_MAGIC, 242, PAIO_AO)

// DIO function
#define	IOCTL_AIODIOBIT							_IOWR(CAIO_DIO_IOC_MAGIC, 100, PAIO_DIO)
#define	IOCTL_AIODIOBYTE						_IOWR(CAIO_DIO_IOC_MAGIC, 101, PAIO_DIO)
#define	IOCTL_AIODIFILTER						_IOWR(CAIO_DIO_IOC_MAGIC, 102, PAIO_DIO)

// CNT function
#define	IOCTL_AIOCNTCOMPARISONMODE				_IOWR(CAIO_CNT_IOC_MAGIC, 100, PAIO_CNT)
#define	IOCTL_AIOCNTPRESETREG					_IOWR(CAIO_CNT_IOC_MAGIC, 101, PAIO_CNT)
#define	IOCTL_AIOCNTCOMPARISONREG				_IOWR(CAIO_CNT_IOC_MAGIC, 102, PAIO_CNT)
#define	IOCTL_AIOCNTINPUTSIGNAL					_IOWR(CAIO_CNT_IOC_MAGIC, 103, PAIO_CNT)
#define	IOCTL_AIOCNTCALLBACKPROC				_IOWR(CAIO_CNT_IOC_MAGIC, 104, PAIO_CNT)
#define	IOCTL_AIOCNTFILTER						_IOWR(CAIO_CNT_IOC_MAGIC, 105, PAIO_CNT)
#define	IOCTL_AIOSTARTCNT						_IOWR(CAIO_CNT_IOC_MAGIC, 106, PAIO_CNT)
#define	IOCTL_AIOSTOPCNT						_IOWR(CAIO_CNT_IOC_MAGIC, 107, PAIO_CNT)
#define	IOCTL_AIOPRESETCNT						_IOWR(CAIO_CNT_IOC_MAGIC, 108, PAIO_CNT)
#define	IOCTL_AIOGETCNTSTATUS					_IOWR(CAIO_CNT_IOC_MAGIC, 109, PAIO_CNT)
#define	IOCTL_AIOGETCNTCOUNT					_IOWR(CAIO_CNT_IOC_MAGIC, 110, PAIO_CNT)
#define	IOCTL_AIORESETCNTSTATUS					_IOWR(CAIO_CNT_IOC_MAGIC, 111, PAIO_CNT)
#define	IOCTL_AIOCNTMAXCHANNELS					_IOWR(CAIO_CNT_IOC_MAGIC, 112, PAIO_CNT)
#define IOCTL_AIOCNTTHWAKEUP					_IOWR(CAIO_CNT_IOC_MAGIC, 113, PAIO_CNT)

// Others
#define	IOCTL_CHECKPROCESS						_IOWR(CAIO_OTHER_IOC_MAGIC, 100, PAIO_CHECKPROCESS)
#define	IOCTL_AIOSETAIEVENT_THREAD				_IOWR(CAIO_OTHER_IOC_MAGIC, 101, PAIO_AI)
#define	IOCTL_AIOGETAIEVENT_THREAD				_IOWR(CAIO_OTHER_IOC_MAGIC, 102, PAIO_AI)
#define	IOCTL_AIOSETAOEVENT_THREAD				_IOWR(CAIO_OTHER_IOC_MAGIC, 103, PAIO_AO)
#define	IOCTL_AIOGETAOEVENT_THREAD				_IOWR(CAIO_OTHER_IOC_MAGIC, 104, PAIO_AO)
#define	IOCTL_AIOSETCNTEVENT_THREAD				_IOWR(CAIO_OTHER_IOC_MAGIC, 105, PAIO_CNT)
#define	IOCTL_AIOGETCNTEVENT_THREAD				_IOWR(CAIO_OTHER_IOC_MAGIC, 106, PAIO_CNT)
#define	IOCTL_RESETPROCESS						_IOWR(CAIO_OTHER_IOC_MAGIC, 109, PAIO_CHECKPROCESS)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Control structure for DeviceIOControl
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Check process (I/ O total 8 byte)
typedef struct _AIO_CHECKPROCESS {
	long			Ret;					// Return code
	unsigned long	ProcessID;				// Process ID
} AIO_CHECKPROCESS, * PAIO_CHECKPROCESS;

// API-AIO common
typedef struct _AIO_COMMON {				// Used in common function
	long			Ret;					// Return code
	unsigned long	ProcessID;				// Process ID
	unsigned long	Data;					// Port I/O buffer
	unsigned long	FilterValue;			// Filter value						AioSet(Get)ControlFilter
	unsigned long	LocalAddress;			// Local base address 
	unsigned long	MasterAddress;			// Master base address
	unsigned short	DeviceID;				// Device ID
	short			Mode;					// Check item (AioChackBoard)
	short			Result;					// Check result (AioChackBoard)
	short			Offset;					// Port I / O data offset address
	short			IODir;					// I/O direction (0 is Out function, 1 is In function)
	short			IOType;					// Number of I/O bytes (1,2,4)
	short			FilterSignal;			// Filter type						AioSet(Get)ControlFilter
	short			EcuDestination;			// ECU destination signal 			AioSet(Get)EcuSignal
	short			EcuSource;				// ECU source signal				AioSet(Get)EcuSignal
	short			Flag;					// Master I/O:1, Local I/O:0
	char			Revision[10];			// Device revision
} AIO_COMMON, * PAIO_COMMON;

// AI function
typedef struct _AIO_AI {					// Used in AI function
	short			AiTransferMode;			// Transfer mode					AioSet(Get)AiTransferMode
	short			AiBufferMode;			// Device buffer mode				AioSet(Get)AiDeviceBufferMode
	long			AiUserBufferNumber;		// Size of user buffer				AioSetAiTransferData
	long			*AiUserBuffer;			// User buffer						AioSetAiTransferData
	short			AiUserBufferRingFlag;	// User buffer is forcedly used in overwrite mode 
	long			AiAttachedData;			// Attached data					AioSetAiAttachedData
	short			AiDataSize;				// 1 sample data size				AioGetAiSamplingDataSize
	long			AiLevel1;				// Level 1							AioSet(Get)AiStartInRange, AioSet(Get)AiStartOutRange
											//									AioSet(Get)AiStopInRange, AioSet(Get)AiStopOutRange
	long			AiLevel2;				// Level 2							AioSet(Get)AiStartInRange, AioSet(Get)AiStartOutRange
											//									AioSet(Get)AiStopInRange, AioSet(Get)AiStopOutRange
	long			AiStateTimes;			// State times						AioSet(Get)AiStartInRange, AioSet(Get)AiStartOutRange
											//									AioSet(Get)AiStopInRange, AioSet(Get)AiStopOutRange
	long			AiTransferTimes;		// Transfer times					AioSet(Get)AiEventTransferTimes
	long			AiTransferCount;		// Current transfer times			AioGetAiTransferCount, AioGetAiStopTriggerTransferCount
	long			AiLap;					// Number of buffer overwrites		AioGetAiTransferLap
	short			AiAdjustType;			// Calibration type
	short			AiAdjustData;			// Calibration data
	unsigned long	AiEventStatusArray[256];// Event status queue array
	unsigned long	AiEventCount;			// Event status queue count 
	short			AiType;					// Device type
	short			AiEndFlag;				// Operation end flag
	long			Ret;					// Return code
	unsigned long	ProcessID;				// Process ID
	long			AiStopTimes;			// Conversion times 				AioSet(Get)AiSamplingTimes
	long			AiSamplingClock;		// Conversion speed u part			AioSet(Get)AiSamplingClock
	long			AiSamplingClockf;		// Conversion speed n part			AioSet(Get)AiSamplingClock
	long			AiScanClock;			// Scan clock u part				AioSet(Get)AiScanClock
	long			AiScanClockf;			// Scan clock n part				AioSet(Get)AiScanClock
	long			AiRepeatTimes;			// Repeat times						AioSet(Get)AiRepeatTimes
	long			AiLevel;				// Level comparison data			AioSet(Get)AiStartLevel, AioSet(Get)AiStopLevel
	long			AiEvent;				// Event factor						AioSet(Get)AiEvent
	long			AiStatus;				// Analog input status 				AioGetAiStatus
	short			Mode;					// Status is ON or OFF				AioGetAiSamplingData(Ex)
	long			AiSamplingTimes;		// Sampling times					AioGetAiSamplingData
	long			AiStopDelayTimes;		// Conversion stop delay times  	AioSet(Get)AiStopDelayTimes
	long			AiSamplingCount;		// Number of scans					AioGetAiSamplingCount, AiEventThread
	long			AiRepeatCount;			// Repeat count						AioGetAiRepeatCount, AiEventThread
	long			AiStopTriggerCount;		// Conversion stop delay times		AioGetAiStopTriggerCount
	long			AiEventSamplingTimes;	// Number of scans for event		AioSet(Get)AiEventSamplingTimes
	long			AiEventStatus;			// Event status						AiEventThread
	pthread_t		hEvent;					// Event handle						AiEventThread
	long			AiMemorySize;			// Memory size						AioSet(Get)AiMemorySize
	long			*pAiData;
	short			AiResolution;			// Resolution						AioGetAiResolution
	short			AiInputMethod;			// Analog input method 				AioSet(Get)AiInputMethod
	short			AiChannels;				// Number of channels				AioSet(Get)AiChannels, AioMultiAi
	short			AiSequence;				// Channel conversion sequence		AioSet(Get)AiChannelSequence
	short			AiChannel;				// Specify the channel				AioSet(Get)AiRange, AioSetAiRangeAll,
											//									AioSet(Get)AiStartLevel, AioSet(Get)AiStopLevel
											//									AioStartAi
	short			AiRange;				// Analog input range				AioSet(Get)AiRange, AioSetAiRangeAll,
	short			AiStartTrigger;			// Conversion start condition 		AioSet(Get)AiStartTrigger
	short			AiStopTrigger;			// Conversion stop condition		AioSet(Get)AiStopTrigger
	short			AiClockType;			// Clock type						AioSet(Get)AiClockType
	short			AiClockEdge;			// Clock timing						AioSet(Get)AiClockEdge
	short			AiMemoryType;			// Memory type						AioSet(Get)AiMemoryType
	short			AiDirection;			// Direction of level comparison 	AioSet(Get)AiStartLevel, AioSet(Get)AiStopLevel
	short			IODir;					// I/O direction					(0 is Set function, 1 is Get function)
	short			AiRangeArray[256];		// All channels range				AioMultiAiEx, AioGetAiSamplingDataEx
	unsigned short	AiData[256];			// Conversion data (USB only)		AioSingleAi, AioMultiAi
	short			Enabled;				// Enable/disable					AioSet(Get)AiChannel
	short			AiTransferTarget;		// Busmaster transfer destination	AioSetAiTransferData
} AIO_AI, * PAIO_AI;

// AO function
typedef struct _AIO_AO {					// Used in AO function
	short			AoTransferMode;			// Transfer mode					AioSet(Get)AoTransferMode
	short			AoBufferMode;			// device buffer mode				AioSet(Get)AoDeviceBufferMode
	long			AoUserBufferNumber;		// Size of user buffer				AioSetAoTransferData
	long			*AoUserBuffer;			// User buffer						AioSetAoTransferData
	long			AoAttachedData;			// Attached data					AioSetAoAttachedData
	short			AoDataSize;				// 1 sample data size				AioGetAoSamplingDataSize
	long			AoTransferTimes;		// Transfer times					AioSet(Get)AoEventTransferTimes
	long			AoTransferCount;		// Current transfer times			AioGetAoTransferCount
	long			AoLap;					// Number of buffer overwrites		AioGetAoTransferLap
	short			AoAdjustType;			// Calibration type
	short			AoAdjustData;			// Calibration data
	unsigned long	AoEventStatusArray[256];// Event status queue array	
	unsigned long	AoEventCount;			// Event status queue count
	short			AoType;					// Device type
	short			AoEndFlag;				// Operation end flag
	long			Ret;					// Return code
	unsigned long	ProcessID;				// Process ID
	long			AoStopTimes;			// Conversion times					AioSet(Get)AoScanTimes
	long			AoSamplingClock;		// Conversion speed u part			AioSet(Get)AoSamplingClock
	long			AoSamplingClockf;		// Conversion speed n part			AioSet(Get)AoSamplingClock
	long			AoRepeatTimes;			// Repeat times						AioSet(Get)AoRptTimes
	long			AoEvent;				// Event factor						AioSet(Get)AoEvent
	long			AoStatus;				// Analog output status	AioGetAoStatus
	long			AoSamplingTimes;		// Number of scans					AioSetAoSamplingData, AioGetAoSamplingTimes
	long			AoSamplingCount;		// Number of scans					AioGetAoSamplingTimes
	long			AoRepeatCount;			// Repeat count						AioSet(Get)AoRptTimes
	long			AoStopTriggerCount;		// Repeat count						AioSet(Get)AoRptTimes
	pthread_t		hEvent;					// Event handle						AoEventThread 
	long			AoEventSamplingTimes;	// Number of scans for event		AioSet(Get)AoEventSamplingTimes
	long			AoEventStatus;			// Event status						AoEventThread
	long			AoMemorySize;			// Memory size						AioSet(Get)AoMemorySize	
	long			*pAoData;
	short			AoResolution;			// Resolution						AioGetAoResolution
	short			AoOutputMode;			// Analog output mode 				AioSet(Get)AoOutputMode
	short			AoChannels;				// Number of channels 				AioSet(Get)AoChannels, AioMultiAo
	short			AoChannel;				// Specify the channel				AioSet(Get)AoRange, AioSetAoRangeAll,
											//									AioStartAo
	short			AoRange;				// Analog output range				AioSet(Get)AoRange, AioSetAoRangeAll,
	short			AoStartTrigger;			// Conversion start condition		AioSet(Get)AoStartTrg
	short			AoStopTrigger;			// Conversion stop condition		AioSet(Get)AoStopTrg
	short			AoClockType;			// Clock type						AioSet(Get)AoClockType
	short			AoClockEdge;			// Clock timing						AioSet(Get)AoClockEdge
	short			AoMemoryType;			// Memory type						AioSet(Get)AoMemoryType
	short			IODir;					// I/O direction					(0 is Set function, 1 is Get function)
	short			AoRangeArray[256];		// All channels range				AioMultiAoEx, AioSetAoSamplingDataEx
	short			AoData[256];			// Conversion data (USB only)		AioSingleAi, AioMultiAi
	short			RepeatMode;				// Auto repeat mode
} AIO_AO, * PAIO_AO;

// DIO function
typedef struct _AIO_DIO {
	long			Ret;					// Return code
	unsigned long	ProcessID;				// Process ID
	long			DioFilterValue;			// Filter setting value				AioSet(Get)DiFilter
	short			DioBit;					// Bit number
	short			DioPort;				// Port number
	short			DiData;					// Digital input data
	short			DoData;					// Digital output data
	short			IODir;					// I/O direction (0 is Output function, 1 is Input function)
	short			FilterSignal;			// Filter type
} AIO_DIO, * PAIO_DIO;

// CNT function
typedef struct _AIO_CNT {
	long			Ret;					// Return code
	unsigned long	ProcessID;				// Process ID
	long			CntStatus;				// Counter status					AioGetCntStatus
	long			CntEvent;				// Event factor						AioSet(Get)CntEvent
	long			CntEventStatus;			// Event status						CntEventThread
	pthread_t		hEvent;					// Event handle						CntEventThread
	long			CntPresetNumber;		// Number of preset data			AioSetCntPresetReg
	long			*CntPresetData;			// Pointer of preset data			AioSetCntPresetReg
	long			CntComparisonNumber;	// Number of comparison count data	AioSetCntComparisonReg
	long			*CntComparisonData;		// Pointer of comparison count data	AioSetCntComparisonReg, AioSetCntComparisonReg
	long			CntPreset;				// Preset data						AioSetCntPreset
	long			CntCount;				// Count value						AioGetCntCount
	long			CntFilterValue;			// Filter setting value				AioSet(Get)CntFilter
	long			CntMode;				// Operation mode					AioSet(Get)CntComparisonMode
	unsigned long	CntEventStatusArray[256];//Event status queue array		
	unsigned long	CntEventCount;			// Event status queue count		
	short			IODir;					// I/O direction (0 is Output function, 1 is Input function)
	short			CntChannels;			// Number of counter channels		AioGetCntMaxChannels
	short			CntChannel;				// counter channel					All functions
	short			CntFlag;				// Repeat operation flag			AioSetCntPresetReg, AioSetCntComparisonReg
	short			CntInputSignal;			// Input signal						AioSetCntInputSignal
	short			CntFilterSignal;		// Filter signal					AioSet(Get)CntFilter
	short			CntEndFlag;				// Operation end flag
} AIO_CNT, * PAIO_CNT;

