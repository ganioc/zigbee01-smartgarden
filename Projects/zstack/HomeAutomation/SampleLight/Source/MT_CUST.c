#include "MT_CUST.h"
#include "MT_RPC.h"

#include "OSAL.h"
#include "MT.h"

#include "peripheral.h"
#include "cust_func.h"

static void MT_CustTestCmd(uint8 *pBuf);
static void MT_CustReadAttrCmd(uint8 * pBuf);
static void MT_CustWriteAttrCmd(uint8 * pBuf);
static void MT_CustCommandCmd(uint8 * pBuf);
void MT_CustSendAns(uint8 cmdId,uint8 *pData, uint8 dataLen);



uint8 MT_CustCommandProcessing(uint8 *pBuf)
{
    uint8 status = MT_RPC_SUCCESS;

    //debug_str("CCCC");

    switch(pBuf[MT_RPC_POS_CMD1])
    {
        case MT_CUST_TEST:
            MT_CustTestCmd(pBuf);
            break;
        case MT_CUST_READ_ATTR:
            MT_CustReadAttrCmd(pBuf);
            break;
        case MT_CUST_WRITE_ATTR:
            MT_CustWriteAttrCmd(pBuf);
            break;
        case MT_CUST_COMMAND:
            MT_CustCommandCmd( pBuf);
        default:
            status = MT_RPC_ERR_COMMAND_ID;
            break;
    }

    return status;
}

void MT_CustSendRead32Ans(uint16 addr, uint16 clusterID, uint16 attrID, uint32 value){
    uint8 data[10];
    uint8 len = 10;

    data[0] = (addr >> 8) & 0xff;
    data[1] = addr & 0xff;
    data[2] = (clusterID >>8)&0xff;
    data[3] = clusterID&0xff;
    data[4] = (attrID >> 8)&0xff;
    data[5] = attrID & 0xff;
    data[6] = (value >>24)&0x00ff;
    data[7] =  (value >>16)&0x00ff;;
    data[8] = (value >> 8)&0x00ff;
    data[9] = value & 0x00ff;

    MT_CustSendAns(MT_CUST_READ_ATTR_RSP, data, len);

}
void MT_CustSendReadAns(uint16 addr, uint16 clusterID, uint16 attrID, uint16 value){
    uint8 data[8];
    uint8 len = 8;

    data[0] = (addr >> 8) & 0xff;
    data[1] = addr & 0xff;
    data[2] = (clusterID >>8)&0xff;
    data[3] = clusterID&0xff;
    data[4] = (attrID >> 8)&0xff;
    data[5] = attrID & 0xff;
    data[6] = (value >> 8)&0xff;
    data[7] = value & 0xff;

    MT_CustSendAns(MT_CUST_READ_ATTR_RSP, data, len);

}
void MT_CustSendReadType1Ans(uint16 addr, uint16 clusterID, uint16 attrID, 
    uint16 heartPeriod, uint16 ph,uint16 soilTemp, uint16 soilHumi, uint16 
    alarm, uint16 onOff, uint16 deviceType, uint16 sensorEnable){

    uint8 data[22];
    uint8 len = 22;

    data[0] = (addr >> 8) & 0xff;
    data[1] = addr & 0xff;
    data[2] = (clusterID >>8)&0xff;
    data[3] = clusterID&0xff;
    data[4] = (attrID >> 8)&0xff;
    data[5] = attrID & 0xff;
    data[6] = (heartPeriod>> 8)&0xff;
    data[7] = heartPeriod& 0xff;
    data[8] = (ph>> 8)&0xff;
    data[9] = ph& 0xff;
    data[10] = (soilTemp>> 8)&0xff;
    data[11] = soilTemp& 0xff;
    data[12] = (soilHumi>> 8)&0xff;
    data[13] = soilHumi& 0xff;
    data[14] = (alarm>> 8)&0xff;
    data[15] = alarm& 0xff;   
    data[16] = (onOff>> 8)&0xff;
    data[17] = onOff& 0xff;   
    data[18] = (deviceType>> 8)&0xff;
    data[19] = deviceType& 0xff;   
    data[20] = (sensorEnable>> 8)&0xff;
    data[21] = sensorEnable& 0xff;   

    MT_CustSendAns(MT_CUST_READ_ATTR_RSP, data, len);

}
void MT_CustSendReadType2Ans(uint16 addr, uint16 clusterID, uint16 attrID, 
    uint16 heartPeriod, uint16 airTemp,uint16 airHumi, uint32 airLight, uint16 
    alarm, uint16 onOff, uint16 deviceType){

    uint8 data[22];
    uint8 len = 22;

    data[0] = (addr >> 8) & 0xff;
    data[1] = addr & 0xff;
    data[2] = (clusterID >>8)&0xff;
    data[3] = clusterID&0xff;
    data[4] = (attrID >> 8)&0xff;
    data[5] = attrID & 0xff;
    data[6] = (heartPeriod>> 8)&0xff;
    data[7] = heartPeriod& 0xff;
    data[8] = (airTemp>> 8)&0xff;
    data[9] = airTemp& 0xff;
    data[10] = (airHumi>> 8)&0xff;
    data[11] = airHumi& 0xff;
    data[12] = (airLight>> 24)&0xff;
    data[13] = (airLight>> 16)& 0xff;
    data[14] = (airLight>> 8)&0xff;
    data[15] = airLight& 0xff;   
    data[16] = (alarm>> 8)&0xff;
    data[17] = alarm& 0xff;   
    data[18] = (onOff>> 8)&0xff;
    data[19] = onOff& 0xff;   
    data[20] = (deviceType>> 8)&0xff;
    data[21] = deviceType& 0xff;    

    MT_CustSendAns(MT_CUST_READ_ATTR_RSP, data, len);

}


void MT_CustSendWriteAns(uint16 addr, uint16 clusterID, uint16 attrID, uint16 
value){
    uint8 data[8];
    uint8 len = 8;

    data[0] = (addr >> 8) & 0xff;
    data[1] = addr & 0xff;
    data[2] = (clusterID >>8)&0xff;
    data[3] = clusterID&0xff;
    data[4] = (attrID >> 8)&0xff;
    data[5] = attrID & 0xff;
    data[6] = (value >> 8)&0xff;
    data[7] = value & 0xff;

    MT_CustSendAns(MT_CUST_WRITE_ATTR_RSP,data, len);    

}
void MT_CustSendCmdAns(uint16 addr, uint16 clusterID, uint16 cmdID, uint16 
value){
    uint8 data[8];
    uint8 len = 8;

    data[0] = (addr >> 8) & 0xff;
    data[1] = addr & 0xff;
    data[2] = (clusterID >>8)&0xff;
    data[3] = clusterID&0xff;
    data[4] = (cmdID >> 8)&0xff;
    data[5] = cmdID & 0xff;
    data[6] = (value >> 8)&0xff;
    data[7] = value & 0xff;

    MT_CustSendAns(MT_CUST_COMMAND_RSP,data, len);        

}
void MT_CustSendAns(uint8 cmdId,uint8 *pData, uint8 dataLen)
{
    uint8 type = MT_CUST_COMMAND_TYPE_ANS + MT_CUST_COMMAND_SUBSYSTEM;

    MT_BuildCustCommandResponse(type, cmdId, dataLen, pData);
}

// Send out report received
void MT_CustSendRpt(uint8 cmdId,uint8 *pData, uint8 dataLen)
{
    uint8 type = MT_CUST_COMMAND_TYPE_RPT + MT_CUST_COMMAND_SUBSYSTEM;

    MT_BuildCustCommandResponse(type, cmdId, dataLen, pData);

}

void MT_CustSendRptHeartBeat(uint16 addr, uint16 devType, uint8* chipId)
{
    uint8 data[12];
    uint8 len = 12;

    data[0] = (addr >> 8) & 0xff;
    data[1] = addr & 0xff;
    data[2] = (devType >>8)&0xff;
    data[3] = devType&0xff;
    data[4] = chipId[0];
    data[5] = chipId[1];
    data[6] = chipId[2];
    data[7] = chipId[3];
    data[8] = chipId[4];
    data[9] = chipId[5];
    data[10] = chipId[6];
    data[11] =   chipId[7];

    MT_CustSendRpt(MT_CUST_HEARTBEAT, data, len);
}



static void MT_CustTestCmd(uint8 *pBuf)
{
    uint8 ch=97;  // 'a'
    uint8 value;
    
    value = pBuf[MT_RPC_POS_DAT0];

    if(value == 1){
        peripheral_reset();

    }else{
    
        MT_CustSendAns(0, &ch, 1);
    }

}

static void MT_CustReadAttrCmd(uint8 * pBuf)
{
    //uint8 len = pBuf[MT_RPC_POS_LEN];
    uint8 srcEP, dstEP;
    uint16 dstAddr, clusterID, attrID ,tmp;
    //zclReadCmd_t *readCmd;

    srcEP = pBuf[MT_RPC_POS_DAT0];

    tmp = pBuf[MT_RPC_POS_DAT0 + 1] ;
    tmp = tmp << 8;
    
    dstAddr = tmp + pBuf[MT_RPC_POS_DAT0 + 2] ;
    
    dstEP    =  pBuf[MT_RPC_POS_DAT0 + 3] ;

    tmp = pBuf[MT_RPC_POS_DAT0 + 4] ;
    tmp = tmp << 8;
    clusterID =tmp  + pBuf[MT_RPC_POS_DAT0 + 5] ;
    //clusterID =  pBuf[MT_RPC_POS_DAT0 + 5] ;

    tmp = pBuf[MT_RPC_POS_DAT0 + 6] ;
    tmp = tmp << 8;
    attrID =   tmp + pBuf[MT_RPC_POS_DAT0 + 7] ;
    //attrID =    pBuf[MT_RPC_POS_DAT0 + 7] ;

    if( attrID == ATTRID_BASIC_SMARTGARDEN_TYPE1_STATUS){
        CustSendReadType1AttrCmd( srcEP, dstAddr, dstEP);

    }else if(attrID == ATTRID_BASIC_SMARTGARDEN_TYPE2_STATUS){
        CustSendReadType2AttrCmd( srcEP, dstAddr, dstEP);
    }else{

        CustSendReadAttrCmd(srcEP, dstAddr, dstEP, clusterID,  attrID);
    }
   //CustSendCmd(srcEP,  dstAddr,  dstEP,  6, 16);CustSendReadAttrCmd
   //CustSendReadAttrCmd(8, zclSample_DstAddr.addr.shortAddr, 8, 0, 0x4007);

}

static void MT_CustWriteAttrCmd(uint8 * pBuf)
{
    //uint8 len = pBuf[MT_RPC_POS_LEN];
    uint8 srcEP, dstEP, dataType;
    uint16 dstAddr, clusterID, attrID, newValue;

    srcEP = pBuf[MT_RPC_POS_DAT0];
    dstAddr = pBuf[MT_RPC_POS_DAT0 + 1] <<8 + pBuf[MT_RPC_POS_DAT0 + 2] ;
    dstEP    =  pBuf[MT_RPC_POS_DAT0 + 3] ;
    clusterID = pBuf[MT_RPC_POS_DAT0 + 4] <<8 + pBuf[MT_RPC_POS_DAT0 + 5] ;
    attrID =   pBuf[MT_RPC_POS_DAT0 + 6] <<8 + pBuf[MT_RPC_POS_DAT0 + 7] ;
    dataType=  pBuf[MT_RPC_POS_DAT0 + 8] ;
    newValue = pBuf[MT_RPC_POS_DAT0 + 9] <<8 + pBuf[MT_RPC_POS_DAT0 + 10] ;

    CustSendWriteAttrCmd(srcEP,  dstAddr,  dstEP,  clusterID,
                         attrID,  dataType,  newValue);

}
static void MT_CustCommandCmd(uint8 * pBuf)
{
    //uint8 len = pBuf[MT_RPC_POS_LEN];
    uint8 srcEP, dstEP;
    uint16 dstAddr, clusterID, cmdID ,tmp;

    //dstAddr = 0;

    srcEP = pBuf[MT_RPC_POS_DAT0];
    
    tmp = pBuf[MT_RPC_POS_DAT0 + 1];

    tmp = tmp<<8;
    
    dstAddr =  tmp+ pBuf[MT_RPC_POS_DAT0 + 2] ;
    
    dstEP    =  pBuf[MT_RPC_POS_DAT0 + 3] ;
    //clusterID = pBuf[MT_RPC_POS_DAT0 + 4] <<8 + pBuf[MT_RPC_POS_DAT0 + 5] ;
    clusterID =  pBuf[MT_RPC_POS_DAT0 + 5] ;
    //cmdID =   pBuf[MT_RPC_POS_DAT0 + 6] <<8 + pBuf[MT_RPC_POS_DAT0 + 7] ;
    cmdID =    pBuf[MT_RPC_POS_DAT0 + 7] ;


    
    //MT_CustSendAns(0, pBuf + MT_RPC_POS_DAT0, 8);

    CustSendCmd(srcEP,  dstAddr,  dstEP,  clusterID, cmdID);
    
}

uint8 *MT_CustAlloc(uint8 len)
{
    uint8 *p;

    /* Allocate a buffer of data length + SOP+CMD+FCS (5 bytes) */
    p = osal_msg_allocate(len + SPI_0DATA_MSG_LEN);

    if(p)
    {
        p++; /* Save space for SOP_VALUE, msg structure */
        return p;
    }
    else
    {
        return NULL;
    }
}

void MT_BuildCustCommandResponse(uint8 cmdType, uint8 cmdId, uint8 dataLen, uint8 *pData)
{
    uint8 *msg_ptr;

    if((msg_ptr = MT_CustAlloc(dataLen)) != NULL)
    {
        msg_ptr[MT_RPC_POS_LEN] = dataLen;
        msg_ptr[MT_RPC_POS_CMD0] = cmdType;
        msg_ptr[MT_RPC_POS_CMD1] = cmdId;
        (void)osal_memcpy(msg_ptr+MT_RPC_POS_DAT0, pData, dataLen);

        MT_TransportSend(msg_ptr);
    }

}

