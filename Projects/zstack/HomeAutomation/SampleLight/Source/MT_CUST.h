#pragma once

#include "hal_types.h"

#define    MT_CUST_TEST                                      0x00
#define    MT_CUST_HEARTBEAT                           0x01
#define    MT_CUST_READ_ATTR                           0x02
#define    MT_CUST_WRITE_ATTR                         0x03
#define    MT_CUST_READ_ATTR_RSP                   0x04
#define    MT_CUST_WRITE_ATTR_RSP                 0x05
#define    MT_CUST_COMMAND                              0x06
#define    MT_CUST_COMMAND_RSP                      0x07

#define    MT_CUST_COMMAND_SUBSYSTEM  14
#define    MT_CUST_COMMAND_TYPE_MASK  0xE0

#define    MT_CUST_COMMAND_TYPE_ASK     0x80
#define    MT_CUST_COMMAND_TYPE_ANS     0xa0
#define    MT_CUST_COMMAND_TYPE_RPT      0xc0




uint8 MT_CustCommandProcessing(uint8 *pBuf);
void MT_BuildCustCommandResponse(uint8 cmdType, uint8 cmdId, uint8 dataLen, uint8 *pData);

// Send out Report Heartbeat
void MT_CustSendRptHeartBeat(uint16 addr, uint16 devType, uint8* chipId);
//void MT_CustSendAns(uint8 cmdId,uint8 *pData, uint8 dataLen);

void MT_CustSendReadAns(uint16 addr, uint16 clusterID, uint16 attrID, uint16 
value);

void MT_CustSendRead32Ans(uint16 addr, uint16 clusterID, uint16 attrID, uint32 
value);
void MT_CustSendWriteAns(uint16 addr, uint16 clusterID, uint16 attrID, uint16 
value);

void MT_CustSendCmdAns(uint16 addr, uint16 clusterID, uint16 cmdID, uint16 
value);

void MT_CustSendReadType1Ans(uint16 addr, uint16 clusterID, uint16 attrID, 
    uint16 heartPeriod, uint16 ph,uint16 soilTemp, uint16 soilHumi, uint16 
    alarm, uint16 onOff, uint16 deviceType , uint16 sensorEnable);

void MT_CustSendReadType2Ans(uint16 addr, uint16 clusterID, uint16 attrID, 
    uint16 heartPeriod, uint16 airTemp,uint16 airHumi, uint32 airLight, uint16 
    alarm, uint16 onOff, uint16 deviceType);
