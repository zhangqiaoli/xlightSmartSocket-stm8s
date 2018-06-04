#ifndef __GLOBAL_H
#define __GLOBAL_H

#include <stm8s.h> //Required for the stdint typedefs
#include "stdio.h"
#include "string.h"
#include "stm8s_conf.h"

// Common Data Type
#define UC                        uint8_t
#define US                        uint16_t
#define UL                        uint32_t
#define SHORT                     int16_t
#define LONG                      int32_t

// Node type
#define NODE_TYP_GW               'g'
#define NODE_TYP_LAMP             'l'
#define NODE_TYP_REMOTE           'r'
#define NODE_TYP_SYSTEM           's'
#define NODE_TYP_AC               'a'
#define NODE_TYP_THIRDPARTY       't'

// NodeID Convention
#define NODEID_GATEWAY          0
#define NODEID_MAINDEVICE       1
#define NODEID_MIN_DEVCIE       8
#define NODEID_MAX_DEVCIE       63
#define NODEID_MIN_REMOTE       64
#define NODEID_MAX_REMOTE       127
#define NODEID_PROJECTOR        128
#define NODEID_KEYSIMULATOR     129
#define NODEID_SUPERSENSOR      130
#define NODEID_SMARTPHONE       139
#define NODEID_MIN_GROUP        192
#define NODEID_MAX_GROUP        223
#define NODEID_RF_SCANNER       250
#define NODEID_DUMMY            255
#define BASESERVICE_ADDRESS     0xFE
#define BROADCAST_ADDRESS       0xFF


#define UNIQUE_ID_LEN           8

// I_GET_NONCE sub-type
enum {
    SCANNER_PROBE = 0,
    SCANNER_SETUP_RF,           // by NodeID & SubID
    SCANNER_SETUPDEV_RF,        // by UniqueID
    
    SCANNER_GETCONFIG = 8,      // by NodeID & SubID
    SCANNER_SETCONFIG,
    SCANNER_GETDEV_CONFIG,      // by UniqueID
    SCANNER_SETDEV_CONFIG,
    
    SCANNER_TEST_NODE = 16,     // by NodeID & SubID
    SCANNER_TEST_DEVICE,        // by UniqueID
};

// Keep alive message interval, around 6 seconds
#define RTE_TM_KEEP_ALIVE               500    // about 5s (500 * 10ms)
#define MAX_RF_FAILED_TIME              8      // Reset RF module when reach max failed times of sending
#define MAX_RF_RESET_TIME               3      // Reset Node when reach max times of RF module consecutive reset

// Xlight Application Identification
#define XLA_VERSION               0x08
#define XLA_ORGANIZATION          "xlight.ca"               // Default value. Read from EEPROM

#define XLA_MIN_VER_REQUIREMENT   0x08
typedef struct
{
  // Static & status parameters
  UC version                  :8;           // Data version, other than 0xFF
  UC present                  :1;           // 0 - not present; 1 - present
  UC state                    :1;           // SuperSensor On/Off
  UC swTimes                  :4;           // On/Off times
  UC reserved0                :2;
  US aircondCode              :16;
  UC aircondStatus[14];

  // Configurable parameters
  UC nodeID;                                // Node ID for this device
  UC subID;                                 // SubID
  UC NetworkID[6];
  UC rfChannel;                             // RF Channel: [0..127]
  UC rfPowerLevel             :2;           // RF Power Level 0..3
  UC rfDataRate               :2;           // RF Data Rate [0..2], 0 for 1Mbps, or 1 for 2Mbps, 2 for 250kbs
  UC rptTimes                 :2;           // Sending message max repeat times [0..3]
  UC reserved1                :2;
  UC type;                                  // Type of SuperSensor
  US token;
  UC reserved2                :8;
  US senMap                   :16;          // Sensor Map
} Config_t;

extern Config_t gConfig;
extern bool gIsChanged;
extern bool gNeedSaveBackup;
extern bool gIsStatusChanged;
extern bool gResetRF;
extern bool gResetNode;
extern uint8_t _uniqueID[UNIQUE_ID_LEN];


bool isIdentityEqual(const UC *pId1, const UC *pId2, UC nLen);
void GotNodeID();
void GotPresented();
bool SendMyMessage();

void tmrProcess();
void idleProcess();

void ResetNodeToRegister();

#define IS_MINE_SUBID(nSID)             ((nSID) == 0 || ((nSID) & gConfig.subID))
#define XLA_PRODUCT_NODEID              72
#define NODEID_MIN_AC                   72
#define NODEID_MAX_AC                   79
#define IS_AC_NODEID(nID)          (nID >= NODEID_MIN_AC && nID <= NODEID_MAX_AC)
#define XLA_PRODUCT_Type            0xC0

//#define TEST

#ifdef TEST
#define     PB5_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_5)
#define     PB4_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_4)
#define     PB3_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_3)
#define     PB2_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_2)
#define     PB1_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_1)
#define     PB0_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_0)
#define     PD1_Low                GPIO_WriteLow(GPIOD , GPIO_PIN_1)
#define     PD2_Low                GPIO_WriteLow(GPIOD , GPIO_PIN_2)
#define     PD7_Low                GPIO_WriteLow(GPIOD , GPIO_PIN_7)
#define     PC1_Low                GPIO_WriteLow(GPIOC , GPIO_PIN_1)
#define     PB5_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_5)
#define     PB4_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_4)
#define     PB3_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_3)
#define     PB2_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_2)
#define     PB1_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_1)
#define     PB0_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_0)
#define     PD1_High                GPIO_WriteHigh(GPIOD , GPIO_PIN_1)
#define     PD2_High                GPIO_WriteHigh(GPIOD , GPIO_PIN_2)
#define     PD7_High                GPIO_WriteHigh(GPIOD , GPIO_PIN_7)
#define     PC1_High                GPIO_WriteHigh(GPIOC , GPIO_PIN_1)
#endif

#endif /* __GLOBAL_H */
