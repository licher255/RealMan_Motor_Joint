#ifndef ZLGCAN_H_
#define ZLGCAN_H_

#include <time.h>

#include "canframe.h"
#include "config.h"

#define ZCAN_PCI5121                        1
#define ZCAN_PCI9810                        2
#define ZCAN_USBCAN1                        3
#define ZCAN_USBCAN2                        4
#define ZCAN_PCI9820                        5
#define ZCAN_CAN232                         6
#define ZCAN_PCI5110                        7
#define ZCAN_CANLITE                        8
#define ZCAN_ISA9620                        9
#define ZCAN_ISA5420                        10
#define ZCAN_PC104CAN                       11
#define ZCAN_CANETUDP                       12
#define ZCAN_CANETE                         12
#define ZCAN_DNP9810                        13
#define ZCAN_PCI9840                        14
#define ZCAN_PC104CAN2                      15
#define ZCAN_PCI9820I                       16
#define ZCAN_CANETTCP                       17
#define ZCAN_PCIE_9220                      18
#define ZCAN_PCI5010U                       19
#define ZCAN_USBCAN_E_U                     20
#define ZCAN_USBCAN_2E_U                    21
#define ZCAN_PCI5020U                       22
#define ZCAN_EG20T_CAN                      23
#define ZCAN_PCIE9221                       24
#define ZCAN_WIFICAN_TCP                    25
#define ZCAN_WIFICAN_UDP                    26
#define ZCAN_PCIe9120                       27
#define ZCAN_PCIe9110                       28
#define ZCAN_PCIe9140                       29
#define ZCAN_USBCAN_4E_U                    31
#define ZCAN_CANDTU_200UR                   32
#define ZCAN_CANDTU_MINI                    33
#define ZCAN_USBCAN_8E_U                    34
#define ZCAN_CANREPLAY                      35
#define ZCAN_CANDTU_NET                     36
#define ZCAN_CANDTU_100UR                   37
#define ZCAN_PCIE_CANFD_100U                38
#define ZCAN_PCIE_CANFD_200U                39
#define ZCAN_PCIE_CANFD_400U                40
#define ZCAN_USBCANFD_200U                  41
#define ZCAN_USBCANFD_100U                  42
#define ZCAN_USBCANFD_MINI                  43
#define ZCAN_CANFDCOM_100IE                 44
#define ZCAN_CANSCOPE                       45
#define ZCAN_CLOUD                          46
#define ZCAN_CANDTU_NET_400                 47
#define ZCAN_CANFDNET_TCP                   48
#define ZCAN_CANFDNET_200U_TCP              48
#define ZCAN_CANFDNET_UDP                   49
#define ZCAN_CANFDNET_200U_UDP              49
#define ZCAN_CANFDWIFI_TCP                  50
#define ZCAN_CANFDWIFI_100U_TCP             50
#define ZCAN_CANFDWIFI_UDP                  51
#define ZCAN_CANFDWIFI_100U_UDP             51
#define ZCAN_CANFDNET_400U_TCP              52
#define ZCAN_CANFDNET_400U_UDP              53
#define ZCAN_CANFDBLUE_200U                 54
#define ZCAN_CANFDNET_100U_TCP              55
#define ZCAN_CANFDNET_100U_UDP              56
#define ZCAN_CANFDNET_800U_TCP              57
#define ZCAN_CANFDNET_800U_UDP              58
#define ZCAN_USBCANFD_800U                  59
#define ZCAN_PCIE_CANFD_100U_EX             60
#define ZCAN_PCIE_CANFD_400U_EX             61
#define ZCAN_PCIE_CANFD_200U_MINI           62
#define ZCAN_PCIE_CANFD_200U_M2             63
#define ZCAN_CANFDDTU_400_TCP               64
#define ZCAN_CANFDDTU_400_UDP               65
#define ZCAN_CANFDWIFI_200U_TCP             66
#define ZCAN_CANFDWIFI_200U_UDP             67
#define ZCAN_CANFDDTU_800ER_TCP             68
#define ZCAN_CANFDDTU_800ER_UDP             69
#define ZCAN_CANFDDTU_800EWGR_TCP           70
#define ZCAN_CANFDDTU_800EWGR_UDP           71
#define ZCAN_CANFDDTU_600EWGR_TCP           72
#define ZCAN_CANFDDTU_600EWGR_UDP           73
#define ZCAN_CANFDDTU_CASCADE_TCP           74
#define ZCAN_CANFDDTU_CASCADE_UDP           75
#define ZCAN_USBCANFD_400U                  76
#define ZCAN_CANFDDTU_200U                  77

#define ZCAN_OFFLINE_DEVICE                 98
#define ZCAN_VIRTUAL_DEVICE                 99

#define ZCAN_ERROR_CAN_OVERFLOW             0x0001
#define ZCAN_ERROR_CAN_ERRALARM             0x0002
#define ZCAN_ERROR_CAN_PASSIVE              0x0004
#define ZCAN_ERROR_CAN_LOSE                 0x0008
#define ZCAN_ERROR_CAN_BUSERR               0x0010
#define ZCAN_ERROR_CAN_BUSOFF               0x0020
#define ZCAN_ERROR_CAN_BUFFER_OVERFLOW      0x0040

#define ZCAN_ERROR_DEVICEOPENED             0x0100
#define ZCAN_ERROR_DEVICEOPEN               0x0200
#define ZCAN_ERROR_DEVICENOTOPEN            0x0400
#define ZCAN_ERROR_BUFFEROVERFLOW           0x0800
#define ZCAN_ERROR_DEVICENOTEXIST           0x1000
#define ZCAN_ERROR_LOADKERNELDLL            0x2000
#define ZCAN_ERROR_CMDFAILED                0x4000
#define ZCAN_ERROR_BUFFERCREATE             0x8000

#define ZCAN_ERROR_CANETE_PORTOPENED        0x00010000
#define ZCAN_ERROR_CANETE_INDEXUSED         0x00020000
#define ZCAN_ERROR_REF_TYPE_ID              0x00030001
#define ZCAN_ERROR_CREATE_SOCKET            0x00030002
#define ZCAN_ERROR_OPEN_CONNECT             0x00030003
#define ZCAN_ERROR_NO_STARTUP               0x00030004
#define ZCAN_ERROR_NO_CONNECTED             0x00030005
#define ZCAN_ERROR_SEND_PARTIAL             0x00030006
#define ZCAN_ERROR_SEND_TOO_FAST            0x00030007

typedef UINT ZCAN_RET_STATUS;
#define STATUS_ERR                          0
#define STATUS_OK                           1
#define STATUS_ONLINE                       2
#define STATUS_OFFLINE                      3
#define STATUS_UNSUPPORTED                  4
#define STATUS_BUFFER_TOO_SMALL             5

typedef UINT ZCAN_LAST_ERROR_STATUS;
//#define STATUS_NO_ERR                       0
//#define STATUS_NO_ERR                       1


typedef UINT ZCAN_UDS_DATA_DEF;
#define DEF_CAN_UDS_DATA                    1 // CAN/CANFD UDS data
#define DEF_LIN_UDS_DATA                    2 // LIN UDS data
#define DEF_DOIP_UDS_DATA                   3 // DOIP UDS data (not supported yet)    

#define CMD_DESIP                           0
#define CMD_DESPORT                         1
#define CMD_CHGDESIPANDPORT                 2
#define CMD_SRCPORT                         2
#define CMD_TCP_TYPE                        4
#define TCP_CLIENT                          0
#define TCP_SERVER                          1

#define CMD_CLIENT_COUNT                    5
#define CMD_CLIENT                          6
#define CMD_DISCONN_CLINET                  7
#define CMD_SET_RECONNECT_TIME              8

#define TYPE_CAN                            0
#define TYPE_CANFD                          1
#define TYPE_ALL_DATA                       2


//Dynamic configuration and permanent configuration BEGIN
#define	ZCAN_DYNAMIC_CONFIG_DEVNAME "DYNAMIC_CONFIG_DEVNAME"// Device name (max 32 bytes including \0). CANFDNET-200U default: "CANFDNET-200U", CANFDNET-100MINI default: "CANFDNET-100MINI"
//CAN channel configuration info (format CAN%d, channel range 0-7)
#define	ZCAN_DYNAMIC_CONFIG_CAN_ENABLE "DYNAMIC_CONFIG_CAN%d_ENABLE"// Channel enable: 1=enabled, 0=disabled. CANFDNET series channels are enabled by default.
#define	ZCAN_DYNAMIC_CONFIG_CAN_MODE "DYNAMIC_CONFIG_CAN%d_MODE"// Working mode: 0=normal mode (default), 1=listen-only mode
#define	ZCAN_DYNAMIC_CONFIG_CAN_TXATTEMPTS "DYNAMIC_CONFIG_CAN%d_TXATTEMPTS"// Retry on transmission failure: 0=no retry, 1=retry until success or bus-off. Not applicable for CANFDNET-100/200.
#define	ZCAN_DYNAMIC_CONFIG_CAN_NOMINALBAUD "DYNAMIC_CONFIG_CAN%d_NOMINALBAUD"// CAN arbitration baud rate or CANFD nominal baud rate
#define	ZCAN_DYNAMIC_CONFIG_CAN_DATABAUD "DYNAMIC_CONFIG_CAN%d_DATABAUD"// CANFD data baud rate
#define	ZCAN_DYNAMIC_CONFIG_CAN_USERES "DYNAMIC_CONFIG_CAN%d_USERES"// Terminal resistance: 0=disabled, 1=enabled
#define	ZCAN_DYNAMIC_CONFIG_CAN_SNDCFG_INTERVAL "DYNAMIC_CONFIG_CAN%d_SNDCFG_INTERVAL"// Config send interval: 0~255ms
#define	ZCAN_DYNAMIC_CONFIG_CAN_BUSRATIO_ENABLE "DYNAMIC_CONFIG_CAN%d_BUSRATIO_ENABLE"// Bus usage enable: 1=enabled, 0=disabled. When enabled, bus usage is sent during TCP/UDP connection.
#define	ZCAN_DYNAMIC_CONFIG_CAN_BUSRATIO_PERIOD "DYNAMIC_CONFIG_CAN%d_BUSRATIO_PERIOD"// Bus usage sampling period: 200~2000ms

typedef struct tagZCAN_DYNAMIC_CONFIG_DATA
{
	char    key[64];
	char    value[64];
}ZCAN_DYNAMIC_CONFIG_DATA;

#define CANFD_FILTER_COUNT_MAX 16
#define CANFD_DATA_LEN_MAX 64

typedef UINT DynamicConfigDataType;
#define DYNAMIC_CONFIG_CAN     0  //CAN channel config
#define DYNAMIC_CONFIG_FILTER  1  //Filter config    

union unionCANFDFilterRulePresent
{
	struct {
		unsigned int bChnl : 1;        // Channel enable flag
		unsigned int bFD : 1;          // CANFD flag enable
		unsigned int bEXT : 1;         // Standard/extended frame flag enable
		unsigned int bRTR : 1;         // Data/remote frame flag enable
		unsigned int bLen : 1;         // Data length enable
		unsigned int bID : 1;          // Start ID/end ID enable
		unsigned int bTime : 1;        // Start time/end time enable
		unsigned int bFilterMask : 1;  // Filter data mask enable
		unsigned int bErr : 1;         // Error frame CAN/CANFD flag enable
		unsigned int nReserved : 23;   // Reserved
	}unionValue;
	unsigned int     rawValue;
};
// Filter rule configuration
struct CANFD_FILTER_RULE
{
	unionCANFDFilterRulePresent presentFlag;// Flag for corresponding field enable
	int                         nErr;       // Error frame filter: 0=filter error frames, 1=keep error frames
	int                         nChnl;      // Channel
	int                         nFD;        // CANFD flag: 0=CAN, 1=CANFD
	int                         nExt;       // Extended frame flag: 0=standard frame, 1=extended frame
	int                         nRtr;       // Remote frame flag: 0=data frame, 1=remote frame
	int                         nLen;       // Data length: 0-64
	int                         nBeginID;   // Start ID
	int                         nEndID;     // End ID (start ID <= end ID, can be used in pairs)
	int                         nBeginTime; // Filter start time in seconds: 0-(24*60*60-1)
	int                         nEndTime;   // Filter end time in seconds: 0-(24*60*60-1), used with start time
	int                         nFilterDataLen;
	int                         nMaskDataLen;
	BYTE                        nFilterData[CANFD_DATA_LEN_MAX]; // Filter data content, uint8 array, max 64
	BYTE                        nMaskData[CANFD_DATA_LEN_MAX];   // Data mask, uint8 array, max 64, used with filter data
};
typedef UINT enumCANFDFilterBlackWhiteList;
#define CANFD_FILTER_BLACK_LIST  0        // datacount
#define CANFD_FILTER_WHITE_LIST  1        // datacount

struct CANFD_FILTER_CFG
{
	int                             bEnable;
	enumCANFDFilterBlackWhiteList   enBlackWhiteList;
	CANFD_FILTER_RULE				vecFilters[CANFD_FILTER_COUNT_MAX];
};
//Currently only filter is available, data collection will be supported in future versions
typedef struct tagZCAN_DYNAMIC_CONFIG
{
	DynamicConfigDataType dynamicConfigDataType;
	UINT                  isPersist;         // Persistent config saved to device: TRUE=persistent, FALSE=dynamic
	union
	{
		CANFD_FILTER_CFG filterCfg;          // Valid when dynamicConfigDataType = DYNAMIC_CONFIG_FILTER
		BYTE			 reserved[10*1024];  // Reserved
	}data;
}ZCAN_DYNAMIC_CONFIG;
//Dynamic configuration and permanent configuration END

typedef void * DEVICE_HANDLE;
typedef void * CHANNEL_HANDLE;

typedef struct tagZCAN_DEVICE_INFO {
    USHORT hw_Version;                      //Hardware version
    USHORT fw_Version;                      //Firmware version
    USHORT dr_Version;                      //Driver version
    USHORT in_Version;                      //Interface version
    USHORT irq_Num;
    BYTE   can_Num;
    UCHAR  str_Serial_Num[20];
    UCHAR  str_hw_Type[40];
    USHORT reserved[4];
}ZCAN_DEVICE_INFO;

typedef struct tagZCAN_CHANNEL_INIT_CONFIG {
    UINT can_type;                          //type: TYPE_CAN or TYPE_CANFD. can_type is read-only, based on product hardware type. CANFD series products have this value as 1, indicating CANFD device.
    union
    {
        struct
        {
            UINT  acc_code;
            UINT  acc_mask;
            UINT  reserved;
            BYTE  filter;
            BYTE  timing0;
            BYTE  timing1;
            BYTE  mode;
        }can;
        struct
        {
            UINT   acc_code;
            UINT   acc_mask;
            UINT   abit_timing;
            UINT   dbit_timing;
            UINT   brp;
            BYTE   filter;
            BYTE   mode;
            USHORT pad;
            UINT   reserved;
        }canfd;
    };
}ZCAN_CHANNEL_INIT_CONFIG;

typedef struct tagZCAN_CHANNEL_ERR_INFO {
    UINT error_code;
    BYTE passive_ErrData[3];
    BYTE arLost_ErrData;
} ZCAN_CHANNEL_ERR_INFO;

typedef struct tagZCAN_CHANNEL_STATUS {
    BYTE errInterrupt;
    BYTE regMode;
    BYTE regStatus;
    BYTE regALCapture;
    BYTE regECCapture;
    BYTE regEWLimit;
    BYTE regRECounter;
    BYTE regTECounter;
    UINT Reserved;
}ZCAN_CHANNEL_STATUS;

typedef struct tagZCAN_Transmit_Data
{
    can_frame   frame;
    UINT        transmit_type;
}ZCAN_Transmit_Data;

typedef struct tagZCAN_Receive_Data
{
    can_frame   frame;
    UINT64      timestamp;                  //us
}ZCAN_Receive_Data;

typedef struct tagZCAN_TransmitFD_Data
{
    canfd_frame frame;
    UINT        transmit_type;
}ZCAN_TransmitFD_Data;

typedef struct tagZCAN_ReceiveFD_Data
{
    canfd_frame frame;
    UINT64      timestamp;                  //us
}ZCAN_ReceiveFD_Data;

typedef struct tagZCAN_AUTO_TRANSMIT_OBJ{
    USHORT enable;
    USHORT index;                           //0...n
    UINT   interval;                        //ms
    ZCAN_Transmit_Data obj;
}ZCAN_AUTO_TRANSMIT_OBJ, *PZCAN_AUTO_TRANSMIT_OBJ;

typedef struct tagZCANFD_AUTO_TRANSMIT_OBJ{
    USHORT enable;
    USHORT index;                           //0...n
    UINT interval;                          //ms
    ZCAN_TransmitFD_Data obj;
}ZCANFD_AUTO_TRANSMIT_OBJ, *PZCANFD_AUTO_TRANSMIT_OBJ;

//Parameters for scheduled transmission objects, currently only supports USBCANFD-X00U series devices
typedef struct tagZCAN_AUTO_TRANSMIT_OBJ_PARAM
{
    USHORT index;                           // Scheduled transmission frame index
    USHORT type;                            // Parameter type: currently only 1 means delayed transmission
    UINT   value;                           // Parameter value
}ZCAN_AUTO_TRANSMIT_OBJ_PARAM, *PZCAN_AUTO_TRANSMIT_OBJ_PARAM;

//for zlg cloud
#define ZCLOUD_MAX_DEVICES                  100
#define ZCLOUD_MAX_CHANNEL                  16

typedef struct tagZCLOUD_CHNINFO
{
    BYTE enable;                            // 0:disable, 1:enable
    BYTE type;                              // 0:CAN, 1:ISO CANFD, 2:Non-ISO CANFD
    BYTE isUpload;
    BYTE isDownload;
} ZCLOUD_CHNINFO;

typedef struct tagZCLOUD_DEVINFO
{
    int devIndex;           
    char type[64];
    char id[64];
    char name[64];
    char owner[64];
    char model[64];
    char fwVer[16];
    char hwVer[16];
    char serial[64];
    int status;                             // 0: online, 1: offline
    BYTE bGpsUpload;
    BYTE channelCnt;
    ZCLOUD_CHNINFO channels[ZCLOUD_MAX_CHANNEL];
}ZCLOUD_DEVINFO;

typedef struct tagZCLOUD_USER_DATA
{
    char username[64];
    char mobile[64];
    char dllVer[16];                        // cloud dll version
    size_t devCnt;
    ZCLOUD_DEVINFO devices[ZCLOUD_MAX_DEVICES];
}ZCLOUD_USER_DATA;

// GPS
typedef struct tagZCLOUD_GPS_FRAME
{
    float latitude;                         // + north latitude, - south latitude
    float longitude;                        // + east longitude, - west longitude           
    float speed;                            // km/h    
    struct __gps_time {
        USHORT    year;
        USHORT    mon;
        USHORT    day;
        USHORT    hour;
        USHORT    min;
        USHORT    sec;
    }tm;
} ZCLOUD_GPS_FRAME;
//for zlg cloud

//TX timestamp
typedef struct tagUSBCANFDTxTimeStamp
{
    UINT* pTxTimeStampBuffer;               //allocated by user, size:nBufferTimeStampCount * 4, unit:100us
    UINT  nBufferTimeStampCount;            //buffer size
}USBCANFDTxTimeStamp;

typedef struct tagTxTimeStamp
{
    UINT64* pTxTimeStampBuffer;             //allocated by user, size:nBufferTimeStampCount * 8, unit:1us
    UINT    nBufferTimeStampCount;          //buffer timestamp count
    int     nWaitTime;                      //Wait Time ms, -1 means wait until data is available
}TxTimeStamp;

// Bus usage
typedef struct tagBusUsage
{
    UINT64  nTimeStampBegin;                //Bus usage start timestamp in us
    UINT64  nTimeStampEnd;                  //Bus usage end timestamp in us
    BYTE    nChnl;                          //Channel
    BYTE    nReserved;                      //Reserved
    USHORT  nBusUsage;                      //Bus usage (%), displayed as bus_usage*100, range 0~10000, e.g., 8050 means 80.50%
    UINT    nFrameCount;                    //Frame count
}BusUsage;

enum eZCANErrorDEF
{
    //Bus error type
    ZCAN_ERR_TYPE_NO_ERR                = 0,        //No error
    ZCAN_ERR_TYPE_BUS_ERR               = 1,        //Bus error
    ZCAN_ERR_TYPE_CONTROLLER_ERR        = 2,        //Controller error
    ZCAN_ERR_TYPE_DEVICE_ERR            = 3,        //Terminal device error

    //Node state
    ZCAN_NODE_STATE_ACTIVE              = 1,        //Bus active
    ZCAN_NODE_STATE_WARNNING            = 2,        //Bus warning
    ZCAN_NODE_STATE_PASSIVE             = 3,        //Bus passive
    ZCAN_NODE_STATE_BUSOFF              = 4,        //Bus off

    //Bus error sub-type, errType = ZCAN_ERR_TYPE_BUS_ERR
    ZCAN_BUS_ERR_NO_ERR                 = 0,        //No error
    ZCAN_BUS_ERR_BIT_ERR                = 1,        //Bit error
    ZCAN_BUS_ERR_ACK_ERR                = 2,        //ACK error
    ZCAN_BUS_ERR_CRC_ERR                = 3,        //CRC error
    ZCAN_BUS_ERR_FORM_ERR               = 4,        //Format error
    ZCAN_BUS_ERR_STUFF_ERR              = 5,        //Stuff error
    ZCAN_BUS_ERR_OVERLOAD_ERR           = 6,        //Overload error
    ZCAN_BUS_ERR_ARBITRATION_LOST       = 7,        //Arbitration lost
    ZCAN_BUS_ERR_NODE_STATE_CHAGE       = 8,        //Bus node state change

    //Controller error, errType = ZCAN_ERR_TYPE_CONTROLLER_ERR
    ZCAN_CONTROLLER_RX_FIFO_OVERFLOW    = 1,        //Controller RX FIFO overflow
    ZCAN_CONTROLLER_DRIVER_RX_BUFFER_OVERFLOW  = 2, //Driver RX buffer overflow
    ZCAN_CONTROLLER_DRIVER_TX_BUFFER_OVERFLOW  = 3, //Driver TX buffer overflow
    ZCAN_CONTROLLER_INTERNAL_ERROR      = 4,        //Controller internal error

    //Terminal device error, errType = ZCAN_ERR_TYPE_DEVICE_ERR
    ZCAN_DEVICE_APP_RX_BUFFER_OVERFLOW = 1,         //Device application RX buffer overflow
    ZCAN_DEVICE_APP_TX_BUFFER_OVERFLOW = 2,         //Device application TX buffer overflow
    ZCAN_DEVICE_APP_AUTO_SEND_FAILED   = 3,         //Auto transmit failed
    ZCAN_CONTROLLER_TX_FRAME_INVALID   = 4,         //TX frame invalid
};

enum eZCANDataDEF
{
    //Data type
    ZCAN_DT_ZCAN_CAN_CANFD_DATA     = 1,            // CAN/CANFD data
    ZCAN_DT_ZCAN_ERROR_DATA         = 2,            // Error data
    ZCAN_DT_ZCAN_GPS_DATA           = 3,            // GPS data
    ZCAN_DT_ZCAN_LIN_DATA           = 4,            // LIN data
    ZCAN_DT_ZCAN_BUSUSAGE_DATA      = 5,            // BusUsage data
    ZCAN_DT_ZCAN_LIN_ERROR_DATA     = 6,            // LIN error data

    //TX delay time unit
    ZCAN_TX_DELAY_NO_DELAY          = 0,            // No TX delay
    ZCAN_TX_DELAY_UNIT_MS           = 1,            // TX delay unit: ms
    ZCAN_TX_DELAY_UNIT_100US        = 2,            // TX delay unit: 100us (0.1ms)

};

#pragma pack(push, 1)

// CAN/CANFDdata
typedef struct tagZCANCANFDData
{
    UINT64          timeStamp;                      // timestamp,receive timecountinus(us),datacounttimestamp�hour,count�ݵ�inȡdataflag.unionVal.txDelay
    union
    {
        struct{
            UINT    frameType : 2;                  // framedata, 0:CANframe, 1:CANFDframe
            UINT    txDelay : 2;                    // TX delay time, valid for sending. 0:no delay, 1:delay unitcountinms, 2:delay unitcountin100us. count�öcountзdata�timestamp�hourtimestampcount�timeStamp�ֶ�
            UINT    transmitType : 4;               // transmission type, valid for sending. 0:transmission type, 1:single shot, 2:self reception, 3:dataself reception. countdevice֧transmission type�ͣtransmission typecount�οdata�ʹcount�ֲ�
            UINT    txEchoRequest : 1;              // count�ͻdatacount�, valid for sending. ֧�ַcountͻcountԵcount豸,transmission typetimestamp�incount1,�豸dataͨdata�սӿڽdataͳ�ȥdatacountframedata,count�յcountķdatacount�ʹcounttxEchoedincount�
            UINT    txEchoed : 1;                   // data�ǷcountǻcountԱcount�, valid for sending. 0:datacount�߽countձcount�, 1:devicecount�ͻcountԱcount�.
            UINT    reserved : 22;                  // data
        }unionVal;
        UINT    rawVal;                             // frameflagsrawdata
    }flag;                                          // CAN/CANFDframeflags
    BYTE        extraData[4];                       // transmission type,not used
    canfd_frame frame;                              // can/canfdframeID+data
}ZCANCANFDData;

// Error data
typedef struct tagZCANErrorData
{
    UINT64  timeStamp;                              // Timestamp in us
    BYTE    errType;                                // Error type, refer to eZCANErrorDEF "Bus error type"
    BYTE    errSubType;                             // Error sub-type, refer to eZCANErrorDEF "Bus error sub-type"
    BYTE    nodeState;                              // Node state, refer to eZCANErrorDEF "Node state"
    BYTE    rxErrCount;                             // RX error count
    BYTE    txErrCount;                             // TX error count
    BYTE    errData;                                // Error data, specific meaning depends on error type and sub-type, refer to manual
    BYTE    reserved[2];                            // Reserved
}ZCANErrorData;

// GPS data
typedef struct tagZCANGPSData
{
    struct {
        USHORT  year;                               // Year
        USHORT  mon;                                // Month
        USHORT  day;                                // Day
        USHORT  hour;                               // Hour
        USHORT  min;                                // Minute
        USHORT  sec;                                // Second
        USHORT  milsec;                             // Millisecond
    }           time;                               // UTC time
    union{
        struct{
            USHORT timeValid : 1;                   // Time data valid
            USHORT latlongValid : 1;                // Latitude/longitude data valid
            USHORT altitudeValid : 1;               // Altitude data valid
            USHORT speedValid : 1;                  // Speed data valid
            USHORT courseAngleValid : 1;            // Course angle data valid
            USHORT reserved:13;                     // Reserved
        }unionVal;
        USHORT rawVal;
    }flag;                                          // Flag info
    double latitude;                                // Latitude: positive=north, negative=south
    double longitude;                               // Longitude: positive=east, negative=west
    double altitude;                                // Altitude in meters
    double speed;                                   // Speed in km/h
    double courseAngle;                             // Course angle
} ZCANGPSData;


// LIN data
typedef struct tagZCANLINData
{
    union {
        struct {
            BYTE    ID:6;                           // Frame ID
            BYTE    Parity:2;                       // Frame ID parity
        }unionVal;
        BYTE    rawVal;                             // Protected ID raw value
    }       PID;                                    // Protected ID
    struct
    {
        UINT64  timeStamp;                          // Timestamp in us
        BYTE    dataLen;                            // Data length
        BYTE    dir;                                // Direction: 0=RX, 1=TX
        BYTE    chkSum;                             // Checksum, valid only if device supports checksum data retrieval
        BYTE    reserved[13];                       // Reserved
        BYTE    data[8];                            // Data
    }RxData;                                        // Valid when receiving data
	BYTE reserved[7];                               // Reserved
}ZCANLINData;

// LIN error data
typedef struct tagZCANLINErrData
{
	UINT64  timeStamp;                              // Timestamp in us
	union {
		struct {
			BYTE    ID : 6;                           // Frame ID
			BYTE    Parity : 2;                       // Frame ID parity
		}unionVal;
		BYTE    rawVal;                             // Protected ID raw value
	}       PID;                                    // Protected ID
	BYTE    dataLen;
	BYTE    data[8];
	union
	{
		struct
		{
			USHORT errStage : 4;                     // Error stage
			USHORT errReason : 4;                    // Error reason
			USHORT reserved : 8;                     // Reserved
		};
		USHORT unionErrData;
	}errData;
	BYTE    dir;                                    // Direction
	BYTE    chkSum;                                 // Checksum, valid only if device supports checksum data retrieval
	BYTE    reserved[10];                           // Reserved
}ZCANLINErrData;

// Combined receive data structure supporting CAN/CANFD/LIN/GPS/error and other data types
typedef struct tagZCANDataObj
{
    BYTE        dataType;                           // Data type, refer to eZCANDataDEF "Data type" field
    BYTE        chnl;                               // Channel
    union{
        struct{
            USHORT reserved : 16;                   // Reserved
        }unionVal;
        USHORT rawVal;
    }flag;                                          // Flag info, not used
    BYTE        extraData[4];                       // Extra data, not used
    union
    {
        ZCANCANFDData           zcanCANFDData;      // CAN/CANFD data
        ZCANErrorData           zcanErrData;        // Error data
        ZCANGPSData             zcanGPSData;        // GPS data
        ZCANLINData             zcanLINData;        // LIN data
        ZCANLINErrData          zcanLINErrData;     // LIN error data
        BusUsage                busUsage;           // BusUsage data
        BYTE                    raw[92];            // RAW data
    } data;                                         // Actual data, union type, valid member determined by dataType field
}ZCANDataObj;

//LIN
typedef struct _VCI_LIN_MSG{
	BYTE        chnl;                               // Channel
	BYTE        dataType;                           // Data type: 0=LIN data, 1=LIN error data
	union
	{
		ZCANLINData             zcanLINData;        // LIN data
		ZCANLINErrData          zcanLINErrData;     // LIN error data
		BYTE                    raw[46];            // RAW data
	} data;                                         // Actual data, union type, valid member determined by dataType field
}ZCAN_LIN_MSG, *PZCAN_LIN_MSG;

enum eZLINChkSumMode
{
	DEFAULT = 0,                           // Default (configured at init time)
	CLASSIC_CHKSUM,                        // Classic checksum
	ENHANCE_CHKSUM,                        // Enhanced checksum
	AUTOMATIC,                             // Automatic (device auto-detects checksum mode, valid when using ZCAN_SetLINSubscribe)
};
typedef struct _VCI_LIN_INIT_CONFIG
{
	BYTE    linMode;                       // Master/slave mode: 0=slave, 1=master
	BYTE    chkSumMode;                    // Checksum mode: 1=classic, 2=enhanced, 3=automatic (corresponds to eZLINChkSumMode)
	USHORT  reserved;					   // Reserved
	UINT    linBaud;                       // Baud rate: 1000~20000
}ZCAN_LIN_INIT_CONFIG, *PZCAN_LIN_INIT_CONFIG;

typedef struct _VCI_LIN_PUBLISH_CFG
{
	BYTE    ID;                                     // Protected ID, range 0-63
	BYTE    dataLen;                                // Data length, range 1-8
	BYTE    data[8];
	BYTE    chkSumMode;                             // Checksum mode: 0=default (configured at init), 1=classic, 2=enhanced (corresponds to eZLINChkSumMode)
	BYTE    reserved[5];                            // Reserved
}ZCAN_LIN_PUBLISH_CFG, *PZCAN_LIN_PUBLISH_CFG;

typedef struct _VCI_LIN_SUBSCIBE_CFG
{
	BYTE    ID;                                     // Protected ID, range 0-63
	BYTE    dataLen;                                // Data length: 1-8 or 255 (0xff) means auto-detect length by device
	BYTE    chkSumMode;                             // Checksum mode: 0=default (configured at init), 1=classic, 2=enhanced, 3=automatic (corresponds to eZLINChkSumMode)
	BYTE    reserved[5];                            // Reserved
}ZCAN_LIN_SUBSCIBE_CFG, *PZCAN_LIN_SUBSCIBE_CFG;

//end LIN


// UDS transport protocol version
typedef BYTE ZCAN_UDS_TRANS_VER;
#define ZCAN_UDS_TRANS_VER_0        0       // ISO15765-2 (2004 version)
#define ZCAN_UDS_TRANS_VER_1        1       // ISO15765-2 (2016 version)

// Frame type
typedef BYTE ZCAN_UDS_FRAME_TYPE;
#define ZCAN_UDS_FRAME_CAN          0       // CAN frame
#define ZCAN_UDS_FRAME_CANFD        1       // CANFD frame
#define ZCAN_UDS_FRAME_CANFD_BRS    2       // CANFD BRS frame

// CAN UDS request parameters
typedef struct _ZCAN_UDS_REQUEST
{
    UINT req_id;                            // Request ID, range 0~65535, unique identifier for this request
    BYTE channel;                           // Device channel number: 0~255
    ZCAN_UDS_FRAME_TYPE frame_type;         // Frame type
    BYTE reserved0[2];                      // Reserved
    UINT src_addr;                          // Source address
    UINT dst_addr;                          // Destination address
    BYTE suppress_response;                 // 1: suppress response
    BYTE sid;                               // Service ID
    BYTE reserved1[6];                      // Reserved
    struct {
        UINT timeout;                       // Response timeout in ms, PC timeout, minimum 200ms
        UINT enhanced_timeout;              // Enhanced timeout when receiving 0x78 response pending, PC timeout, minimum 200ms
        BYTE check_any_negative_response:1; // Whether to treat unexpected negative response as response when received
        BYTE wait_if_suppress_response:1;   // Whether to wait for response when suppress response is set, wait time is response timeout
        BYTE flag:6;                        // Reserved
        BYTE reserved0[7];                  // Reserved
    } session_param;                        // Session parameters
    struct {
        ZCAN_UDS_TRANS_VER version;         // Transport protocol version: VERSION_0, VERSION_1
        BYTE max_data_len;                  // Max data length per frame: CAN=8, CANFD=64
        BYTE local_st_min;                  // Local separation time, minimum delay between frames: 0x00-0x7F(0ms~127ms), 0xF1-0xF9(100us~900us)
        BYTE block_size;                    // Block size for flow control
        BYTE fill_byte;                     // Fill byte for invalid data
        BYTE ext_frame;                     // 0=standard frame, 1=extended frame
        BYTE is_modify_ecu_st_min;          // Whether to override ECU's STmin, force using remote_st_min
        BYTE remote_st_min;                 // Remote separation time, valid when is_modify_ecu_st_min=1: 0x00-0x7F(0ms~127ms), 0xF1-0xF9(100us~900us)
        UINT fc_timeout;                    // Flow control timeout in ms, wait time for FC frame after sending FF frame
        BYTE reserved0[4];                  // Reserved
    } trans_param;                          // Transport parameters
    BYTE *data;                             // Request data (excluding SID)
    UINT data_len;                          // Request data length
    UINT reserved2;                         // Reserved
} ZCAN_UDS_REQUEST;

// LIN UDS request parameters
typedef struct _ZLIN_UDS_REQUEST
{
	UINT req_id;                            // Request ID, range 0~65535, unique identifier for this request
	BYTE channel;                           // Device channel number: 0~255
	BYTE suppress_response;                 // 1: suppress response, 0: need response
	BYTE sid;                               // Service ID
	BYTE Nad;                               // Node address
	BYTE reserved1[8];                      // Reserved
	struct {
		UINT p2_timeout;                    // Response timeout in ms, PC timeout, minimum 200ms
		UINT enhanced_timeout;              // Enhanced timeout when receiving 0x78 response pending, PC timeout, minimum 200ms
		BYTE check_any_negative_response : 1; // Whether to treat unexpected negative response as response when received
		BYTE wait_if_suppress_response : 1;   // Whether to wait for response when suppress response is set, wait time is response timeout
		BYTE flag : 6;                        // Reserved
		BYTE reserved0[7];                  // Reserved
	} session_param;                        // Session parameters
	struct {
		BYTE fill_byte;                     // Fill byte for invalid data
		BYTE st_min;                        // Separation time from slave node, minimum delay between frames sent by slave node
		BYTE reserved0[6];                  // Reserved
	} trans_param;                          // Transport parameters
	BYTE *data;                             // Request data (excluding SID)
	UINT data_len;                          // Request data length
	UINT reserved2;                         // Reserved
} ZLIN_UDS_REQUEST;

// UDS error code
typedef BYTE ZCAN_UDS_ERROR;
#define ZCAN_UDS_ERROR_OK                   0    // No error
#define ZCAN_UDS_ERROR_TIMEOUT              1    // Response timeout
#define ZCAN_UDS_ERROR_TRANSPORT            2    // Transport error
#define ZCAN_UDS_ERROR_CANCEL               3    // Request cancelled
#define ZCAN_UDS_ERROR_SUPPRESS_RESPONSE    4    // Suppress response
#define ZCAN_UDS_ERROR_BUSY                 5    // Busy
#define ZCAN_UDS_ERROR_REQ_PARAM            6    // Request parameter error
#define ZCAN_UDS_ERROR_OTHTER               100  // Other error

typedef BYTE ZCAN_UDS_RESPONSE_TYPE;
#define ZCAN_UDS_RT_NEGATIVE 0              // Negative response
#define ZCAN_UDS_RT_POSITIVE 1              // Positive response

// UDS response structure
typedef struct _ZCAN_UDS_RESPONSE
{
    ZCAN_UDS_ERROR status;                  // Response status
    BYTE reserved[6];                       // Reserved
    ZCAN_UDS_RESPONSE_TYPE type;            // Response type
    union {
        struct {
            BYTE sid;                       // Response service ID
            UINT data_len;                  // Data length (excluding SID), data stored in interface buffer dataBuf
        } positive;
        struct {
            BYTE  neg_code;                 // Fixed to 0x7F
            BYTE  sid;                      // Request service ID
            BYTE  error_code;               // Error code
        } negative;
        BYTE raw[8]; 
    };
} ZCAN_UDS_RESPONSE;

// UDS control command
typedef UINT ZCAN_UDS_CTRL_CODE;
#define ZCAN_UDS_CTRL_STOP_REQ 0            // Stop UDS request

// UDS control request
typedef struct _ZCAN_UDS_CTRL_REQ
{
    UINT reqID;                              // Request ID, specifies which request to operate
	ZCAN_UDS_CTRL_CODE cmd;                  // Control command
    BYTE reserved[8];                        // Reserved
} ZCAN_UDS_CTRL_REQ;

// UDS control result
typedef UINT ZCAN_UDS_CTRL_RESULT;
#define ZCAN_UDS_CTRL_RESULT_OK  0          // Success
#define ZCAN_UDS_CTRL_RESULT_ERR 1          // Failed

// UDS control response
typedef struct _ZCAN_UDS_CTRL_RESP
{
    ZCAN_UDS_CTRL_RESULT result;            // Control result
    BYTE reserved[8];                       // Reserved
} ZCAN_UDS_CTRL_RESP;

// CAN/CANFD UDS data
typedef struct tagZCANCANFDUdsData
{
	const ZCAN_UDS_REQUEST* req;			// Request info
	BYTE reserved[28];
}ZCANCANFDUdsData;

// LIN UDS data
typedef struct tagZCANLINUdsData
{
	const ZLIN_UDS_REQUEST* req;			// Request info
	BYTE reserved[28];
}ZCANLINUdsData;

// UDS data structure supporting CAN/LIN UDS and other request types
typedef struct tagZCANUdsRequestDataObj
{
	ZCAN_UDS_DATA_DEF    dataType;              // Data type
	union
	{
		ZCANCANFDUdsData zcanCANFDUdsData;      // CAN/CANFD UDS data
		ZCANLINUdsData   zcanLINUdsData;        // LIN UDS data
		BYTE             raw[63];               // RAW data
	} data;                                     // Actual data, union type, valid member determined by dataType field
	BYTE                 reserved[32];          // Reserved
}ZCANUdsRequestDataObj;

#pragma pack(pop)

#ifdef __cplusplus 
#define DEF(a) = a
#else 
#define DEF(a)
#endif 

#ifdef WIN32
#define FUNC_CALL __stdcall
#else
#define FUNC_CALL // __attribute__((stdcall))
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#define INVALID_DEVICE_HANDLE 0
DEVICE_HANDLE FUNC_CALL ZCAN_OpenDevice(UINT device_type, UINT device_index, UINT reserved);
UINT FUNC_CALL ZCAN_CloseDevice(DEVICE_HANDLE device_handle);
UINT FUNC_CALL ZCAN_GetDeviceInf(DEVICE_HANDLE device_handle, ZCAN_DEVICE_INFO* pInfo);

UINT FUNC_CALL ZCAN_IsDeviceOnLine(DEVICE_HANDLE device_handle);

#define INVALID_CHANNEL_HANDLE 0
CHANNEL_HANDLE FUNC_CALL ZCAN_InitCAN(DEVICE_HANDLE device_handle, UINT can_index, ZCAN_CHANNEL_INIT_CONFIG* pInitConfig);
UINT FUNC_CALL ZCAN_StartCAN(CHANNEL_HANDLE channel_handle);
UINT FUNC_CALL ZCAN_ResetCAN(CHANNEL_HANDLE channel_handle);
UINT FUNC_CALL ZCAN_ClearBuffer(CHANNEL_HANDLE channel_handle);
UINT FUNC_CALL ZCAN_ReadChannelErrInfo(CHANNEL_HANDLE channel_handle, ZCAN_CHANNEL_ERR_INFO* pErrInfo);
UINT FUNC_CALL ZCAN_ReadChannelStatus(CHANNEL_HANDLE channel_handle, ZCAN_CHANNEL_STATUS* pCANStatus);
UINT FUNC_CALL ZCAN_GetReceiveNum(CHANNEL_HANDLE channel_handle, BYTE type);//type:TYPE_CAN, TYPE_CANFD, TYPE_ALL_DATA
UINT FUNC_CALL ZCAN_Transmit(CHANNEL_HANDLE channel_handle, ZCAN_Transmit_Data* pTransmit, UINT len);
UINT FUNC_CALL ZCAN_Receive(CHANNEL_HANDLE channel_handle, ZCAN_Receive_Data* pReceive, UINT len, int wait_time DEF(-1));
UINT FUNC_CALL ZCAN_TransmitFD(CHANNEL_HANDLE channel_handle, ZCAN_TransmitFD_Data* pTransmit, UINT len);
UINT FUNC_CALL ZCAN_ReceiveFD(CHANNEL_HANDLE channel_handle, ZCAN_ReceiveFD_Data* pReceive, UINT len, int wait_time DEF(-1));

UINT FUNC_CALL ZCAN_TransmitData(DEVICE_HANDLE device_handle, ZCANDataObj* pTransmit, UINT len);
UINT FUNC_CALL ZCAN_ReceiveData(DEVICE_HANDLE device_handle, ZCANDataObj* pReceive, UINT len, int wait_time DEF(-1));
UINT FUNC_CALL ZCAN_SetValue(DEVICE_HANDLE device_handle, const char* path, const void* value);
const void* FUNC_CALL ZCAN_GetValue(DEVICE_HANDLE device_handle, const char* path);

IProperty* FUNC_CALL GetIProperty(DEVICE_HANDLE device_handle);
UINT FUNC_CALL ReleaseIProperty(IProperty * pIProperty);

void FUNC_CALL ZCLOUD_SetServerInfo(const char* httpSvr, unsigned short httpPort, const char* authSvr, unsigned short authPort);
// return 0:success, 1:failure, 2:https error, 3:user login info error, 4:mqtt connection error, 5:no device
UINT FUNC_CALL ZCLOUD_ConnectServer(const char* username, const char* password);
// return 0:not connected, 1:connected
UINT FUNC_CALL ZCLOUD_IsConnected();
// return 0:success, 1:failure
UINT FUNC_CALL ZCLOUD_DisconnectServer();
const ZCLOUD_USER_DATA* FUNC_CALL ZCLOUD_GetUserData(int update DEF(0));
UINT FUNC_CALL ZCLOUD_ReceiveGPS(DEVICE_HANDLE device_handle, ZCLOUD_GPS_FRAME* pReceive, UINT len, int wait_time DEF(-1));

CHANNEL_HANDLE FUNC_CALL ZCAN_InitLIN(DEVICE_HANDLE device_handle, UINT lin_index, PZCAN_LIN_INIT_CONFIG pLINInitConfig);
UINT FUNC_CALL ZCAN_StartLIN(CHANNEL_HANDLE channel_handle);
UINT FUNC_CALL ZCAN_ResetLIN(CHANNEL_HANDLE channel_handle);
UINT FUNC_CALL ZCAN_TransmitLIN(CHANNEL_HANDLE channel_handle, PZCAN_LIN_MSG pSend, UINT Len);
UINT FUNC_CALL ZCAN_GetLINReceiveNum(CHANNEL_HANDLE channel_handle);
UINT FUNC_CALL ZCAN_ReceiveLIN(CHANNEL_HANDLE channel_handle, PZCAN_LIN_MSG pReceive, UINT Len,int WaitTime);

UINT FUNC_CALL ZCAN_SetLINSubscribe(CHANNEL_HANDLE channel_handle, PZCAN_LIN_SUBSCIBE_CFG pSend, UINT nSubscribeCount);
UINT FUNC_CALL ZCAN_SetLINPublish(CHANNEL_HANDLE channel_handle, PZCAN_LIN_PUBLISH_CFG pSend, UINT nPublishCount);

/**
 * @brief UDS request
 * @param[in] device_handle Device handle
 * @param[in] req Request info
 * @param[out] resp Response info, can be nullptr, meaning no need to wait for response
 * @param[out] dataBuf Response data buffer, stores positive response data (excluding SID), actual length is resp.positive.data_len
 * @param[in] dataBufSize Response data buffer total size, if smaller than response data length, returns STATUS_BUFFER_TOO_SMALL
 * @return Execution result status
 */
ZCAN_RET_STATUS FUNC_CALL ZCAN_UDS_Request(DEVICE_HANDLE device_handle, const ZCAN_UDS_REQUEST* req, ZCAN_UDS_RESPONSE* resp, BYTE* dataBuf, UINT dataBufSize);


/**
 * @brief UDS control, e.g., stop currently executing UDS request
 * @param[in] device_handle Device handle
 * @param[in] ctrl Control request info
 * @param[out] resp Response info, can be nullptr, meaning no need to wait for response
 * @return Execution result status
 */
ZCAN_RET_STATUS FUNC_CALL ZCAN_UDS_Control(DEVICE_HANDLE device_handle, const ZCAN_UDS_CTRL_REQ *ctrl, ZCAN_UDS_CTRL_RESP* resp);

/**
* @brief UDS request (extended)
* @param[in] device_handle Device handle
* @param[in] requestData Request info
* @param[out] resp Response info, can be nullptr, meaning no need to wait for response
* @param[out] dataBuf Response data buffer, stores positive response data (excluding SID), actual length is resp.positive.data_len
* @param[in] dataBufSize Response data buffer total size, if smaller than response data length, returns STATUS_BUFFER_TOO_SMALL
*/
ZCAN_RET_STATUS FUNC_CALL ZCAN_UDS_RequestEX(DEVICE_HANDLE device_handle, const ZCANUdsRequestDataObj* requestData, ZCAN_UDS_RESPONSE* resp, BYTE* dataBuf, UINT dataBufSize);


/**
* @brief UDS control, e.g., stop currently executing UDS request (extended)
* @param[in] device_handle Device handle
* @param[in] dataType Data type
* @param[in] ctrl Control request info
* @param[out] resp Response info, can be nullptr, meaning no need to wait for response
* @return Execution result status
*/
ZCAN_RET_STATUS FUNC_CALL ZCAN_UDS_ControlEX(DEVICE_HANDLE device_handle, ZCAN_UDS_DATA_DEF dataType, const ZCAN_UDS_CTRL_REQ *ctrl, ZCAN_UDS_CTRL_RESP* resp);

/*LIN Slave*/
UINT FUNC_CALL ZCAN_SetLINSlaveMsg(CHANNEL_HANDLE channel_handle, PZCAN_LIN_MSG pSend, UINT nMsgCount);
UINT FUNC_CALL ZCAN_ClearLINSlaveMsg(CHANNEL_HANDLE channel_handle, BYTE* pLINID, UINT nIDCount);

#ifdef __cplusplus
}
#endif

#endif //ZLGCAN_H_
