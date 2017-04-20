/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "MT_SYS.h"
#include "DebugTrace.h"
#include "zcl_general.h"

#include "peripheral.h"
#include "iocc2538.h"
#include "cust_func.h"
#include "MT_RPC.h"

#include "OSAL_Nv.h"
#include "OnBoard.h"

#include "zcl_samplelight.h"

#include "sensor.h"
#include "sensorph.h"
#include "zGlobals.h"
#include "gptimer.h"
#include "hal_led.h"
#include "cust_timer.h"
   
// Added by Yang
#include "tasks_type1.h"
#include "cust_func.h"


/*********************************************************************
 * GLOBAL VARIABLES
 */
byte peripheral_TaskID;
uint8 peripheralSeqNum=0;

extern uint8 onlinesign;
extern uint16  zclSmartGarden_AlarmStatus;
extern uint8 zclSmartGarden_HeartbeatPeriod;
extern uint64 zclSmartGarden_ChipId;
extern uint8 Heartbeat;
extern uint16    zclSmartGarden_DeviceType;
extern uint16 zclSmartGarden_IrrigateOnOff;

extern uint16 zclSmartGarden_Sensor_Enable;
afAddrType_t zclSample_CoorAddr;

#ifdef ZCL_ON_OFF
afAddrType_t zclSample_DstAddr;
#endif


void peripheral_reset(void);
void peripheralCoordinator_ProcessIncomingCommand(peripheralCmd_t *msg_ptr);
void peripheralRouter_ProcessIncomingCommand(peripheralCmd_t *msg_ptr);

void peripheral_Init(byte task_id)
{
    peripheral_TaskID = task_id;

    debug_str("Peripheral_Init()");


    // Set destination address to indirect
    zclSample_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    zclSample_DstAddr.endPoint = 8;
    zclSample_DstAddr.addr.shortAddr = 0;

    zclSample_CoorAddr.addrMode = (afAddrMode_t)Addr16Bit;
    zclSample_CoorAddr.endPoint = 8;
    zclSample_CoorAddr.addr.shortAddr = 0;

}


void peripheral_ProcessIncomingCommand(peripheralCmd_t *msg_ptr)
{

    if(msg_ptr->hdr.event == PERIPH_TEST_EVENT)
    {
        cust_debug_str("srcEp:%d  cmdId :%d, param1:%x, param2:%x",
                       msg_ptr->srcEP,
                       msg_ptr->cmdID,
                       msg_ptr->param1,
                       msg_ptr->param2
                      );

        peripheralCoordinator_ProcessIncomingCommand(msg_ptr);
    }
    else
    {
        debug_str("Test event unrecognized");
    }
}
uint16 peripheral_event_loop(uint8 task_id, uint16 events)
{
    (void)task_id;  // Intentionally unreferenced parameter

    if(events & SYS_EVENT_MSG)
    {

        uint8 *msg_ptr = osal_msg_receive(task_id);

        if(msg_ptr != NULL)
        {
            peripheral_ProcessIncomingCommand((peripheralCmd_t *)msg_ptr);
            osal_msg_deallocate(msg_ptr);
        }
    }

    if(events & PERIPH_RESET_EVENT)
    {
        //SystemReset();
        peripheral_reset();
        sys_recover();
        cust_debug_str("reset");
        SystemReset();

    }
    
    // TYPE1    
    if(events & PERIPH_PH_SENSOR_UPDATE)
    {

        update_soil_ph_sensor();
        osal_start_timerEx(peripheral_TaskID, PERIPH_PH_SENSOR_UPDATE, 1 * 1000);
        // return events & PERIPH_SENSOR_UPDATE;
    }

     if(events & PERIPH_TEMPHUMI_SENSOR_UPDATE)
    {

        update_soil_temphumi_sensor();
        osal_start_timerEx(peripheral_TaskID,
                           PERIPH_TEMPHUMI_SENSOR_UPDATE, 1 * 1000);
    }

    if(events & PERIPH_TYPE_SENSOR_UPDATE)
    {
#ifdef CUST_ALARM
    if(zclSmartGarden_AlarmStatus){
        HalLedBlink (HAL_LED_3, 3, 60, 5000);
        HalLedBlink (HAL_BEEP, 5, 50, 2000);
      }
#endif
   
        runSensorTypeTask();
    }
    




    if(events & PERIPH_HEARTBEAT_REPORT){
      
      
      zclReportCmd_t  *reportCmd;
      reportCmd = osal_mem_alloc(sizeof(zclReportCmd_t ) + 2*sizeof(zclReport_t));
        
        if(reportCmd){

            cust_debug_str("chipid:%d", zclSmartGarden_ChipId);

            reportCmd->numAttr = 2;
            reportCmd->attrList[0].attrID = ATTRID_BASIC_SMARTGARDEN_CHIPID;
            reportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT64;
            reportCmd->attrList[0].attrData = (uint8 *)&zclSmartGarden_ChipId;
            
            reportCmd->attrList[1].attrID = ATTRID_BASIC_SMARTGARDEN_DEVICE_TYPE;
            reportCmd->attrList[1].dataType = ZCL_DATATYPE_UINT16;
            reportCmd->attrList[1].attrData = (uint8 *)&zclSmartGarden_DeviceType;
            
            

            zcl_SendReportCmd(
                8, 
                &zclSample_CoorAddr, 
                ZCL_CLUSTER_ID_GEN_BASIC, 
                reportCmd, 
                ZCL_FRAME_CLIENT_SERVER_DIR,
                0, 
                peripheralSeqNum++);
            
            osal_mem_free(reportCmd);
                
        }

        cust_debug_str("Send heartbeat %d", Heartbeat);
        
        if(Heartbeat > 0)
        {
          Heartbeat --;
        }
        else
        {
          //close the tune;
          zclSmartGarden_IrrigateOnOff = 0;
          if(read_relay0_state()){
             relay0_turn_off();
          }
          if(onlinesign){
            onlinesign = 0;
            HalLedBlink (HAL_LED_3, 0xFF, 62, 8000);
          }
         
        }
        
        // Added by Yang
        start_1shot_cust_timer();
        
    }
    // Discard unknown events
    return 0;
}

byte peripheral_taskId(void)
{
    return peripheral_TaskID;
}

/**
Default pBuf size is: 10 byte
len:1
cmd0: 1
cmd1: 1
srcEP: 1
cmdId: 2
param: 2
param: 2
**/
void  peripheral_TestCmd(uint8 * pBuf)
{
    uint16 app_cmd;
    uint8 srcEp;
    // cmdId;
    uint16 param1;
    uint16 param2;
    peripheralCmd_t *pMsg;

    /* parse header */
    //cmdId = pBuf[MT_RPC_POS_CMD1];
    pBuf += MT_RPC_FRAME_HDR_SZ;

    srcEp = *pBuf++;

    app_cmd = osal_build_uint16(pBuf);
    pBuf = pBuf + sizeof(uint16);

    param1 = osal_build_uint16(pBuf);
    pBuf = pBuf + sizeof(uint16);

    param2 = osal_build_uint16(pBuf);

    pMsg = (peripheralCmd_t *)osal_msg_allocate(sizeof(peripheralCmd_t));

    pMsg->hdr.event = PERIPH_TEST_EVENT;

    pMsg->srcEP = srcEp;
    pMsg->cmdID = app_cmd;
    pMsg->param1 = param1;
    pMsg->param2 = param2;

    osal_msg_send(peripheral_TaskID, (uint8*)pMsg);

}
// void peripheral_reset_default(void)
// {
//     zgWriteStartupOptions(ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_NETWORK_STATE);
//     SystemResetSoft();

// }
void peripheral_reset(void)
{
  
#ifndef ZDO_COORDINATOR    
    NLME_LeaveReq_t leaveReq;

    debug_str("_basicResetCB");
    
    // Set every field to 0
    osal_memset(&leaveReq, 0, sizeof(NLME_LeaveReq_t));

    // This will enable the device to rejoin the network after reset.
    leaveReq.rejoin = TRUE;

    // Set the NV startup option to force a "new" join.
    zgWriteStartupOptions(ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_NETWORK_STATE);

    // Leave the network, and reset afterwards
    if(NLME_LeaveReq(&leaveReq) != ZSuccess)
    {
        // Couldn't send out leave; prepare to reset anyway
        ZDApp_LeaveReset(FALSE);
    }
#else    
    SystemResetSoft();
#endif

}

void CustSendCmd(uint8 srcEP, uint16 dstAddr, uint8 dstEP, uint16 clusterID,uint16 cmdID)
{
    zclSample_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    zclSample_DstAddr.endPoint =          dstEP;
    zclSample_DstAddr.addr.shortAddr = dstAddr;
    zcl_SendCommand(
        (srcEP),
        (&zclSample_DstAddr),
        clusterID,
        cmdID,
        TRUE,
        ZCL_FRAME_CLIENT_SERVER_DIR,
        (true),
        0,
        (peripheralSeqNum++),
        0,
        NULL);

}

void CustSendReadType1AttrCmd(uint8 srcEP, uint16 dstAddr, uint8 dstEP)
{
    zclReadCmd_t *readCmd;

//read light status
    zclSample_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    zclSample_DstAddr.endPoint =          dstEP;
    zclSample_DstAddr.addr.shortAddr = dstAddr;

    readCmd = osal_mem_alloc(sizeof(zclReadCmd_t) + 8*sizeof(uint16));

    if(readCmd != NULL)
    {

        readCmd->numAttr = 8;
        readCmd->attrID[0] = ATTRID_BASIC_SMARTGARDEN_HEARTBEAT_PERIOD;
        readCmd->attrID[1] = ATTRID_BASIC_SMARTGARDEN_PH_VALUE;

        readCmd->attrID[2] = ATTRID_BASIC_SMARTGARDEN_TEMP;

        readCmd->attrID[3] = ATTRID_BASIC_SMARTGARDEN_HUMIDITY;

        readCmd->attrID[4] = ATTRID_BASIC_SMARTGARDEN_ALARM_STATUS;

        readCmd->attrID[5] = ATTRID_BASIC_SMARTGARDEN_IRRIGATE_ONOFF;
        
        readCmd->attrID[6] = ATTRID_BASIC_SMARTGARDEN_DEVICE_TYPE;

        readCmd->attrID[7] = ATTRID_BASIC_SMARTGARDEN_TYPE1_SENSOR_ENABLE;
        

        zcl_SendRead(
            srcEP,
            &zclSample_DstAddr,
            ZCL_CLUSTER_ID_GEN_BASIC,
            readCmd,
            ZCL_FRAME_CLIENT_SERVER_DIR,
            0,
            peripheralSeqNum++);

        
        osal_mem_free(readCmd);

    }


}
void CustSendReadType2AttrCmd(uint8 srcEP, uint16 dstAddr, uint8 dstEP)
{
    zclReadCmd_t *readCmd;

//read light status
    zclSample_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    zclSample_DstAddr.endPoint =          dstEP;
    zclSample_DstAddr.addr.shortAddr = dstAddr;

    readCmd = osal_mem_alloc(sizeof(zclReadCmd_t) + 8 * sizeof(uint16));

    if(readCmd != NULL)
    {

        readCmd->numAttr = 8;
        readCmd->attrID[0] = ATTRID_BASIC_SMARTGARDEN_HEARTBEAT_PERIOD;
        readCmd->attrID[1] = ATTRID_BASIC_SMARTGARDEN_TEMP_INTENSITY;
        readCmd->attrID[2] = ATTRID_BASIC_SMARTGARDEN_HUMI_INTENSITY;
        readCmd->attrID[3] = ATTRID_BASIC_SMARTGARDEN_LIGHT_INTENSITY;
        readCmd->attrID[4] = ATTRID_BASIC_SMARTGARDEN_ALARM_STATUS;
        readCmd->attrID[5] = ATTRID_BASIC_SMARTGARDEN_IRRIGATE_ONOFF;
        
        readCmd->attrID[6] = ATTRID_BASIC_SMARTGARDEN_DEVICE_TYPE;
        
        readCmd->attrID[7] = ATTRID_BASIC_SMARTGARDEN_TYPE1_SENSOR_ENABLE;
        

        zcl_SendRead(
            srcEP,
            &zclSample_DstAddr,
            ZCL_CLUSTER_ID_GEN_BASIC,
            readCmd,
            ZCL_FRAME_CLIENT_SERVER_DIR,
            0,
            peripheralSeqNum++);
        
        osal_mem_free(readCmd);

    }


}
void CustSendReadAttrCmd(uint8 srcEP, uint16 dstAddr, uint8 dstEP, uint16 clusterID,uint16 attrID)
{
    zclReadCmd_t *readCmd;

    zclSample_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    zclSample_DstAddr.endPoint =          dstEP;
    zclSample_DstAddr.addr.shortAddr = dstAddr;

    readCmd = osal_mem_alloc(sizeof(zclReadCmd_t) + sizeof(uint16));

    if(readCmd != NULL)
    {

        readCmd->numAttr = 1;
        readCmd->attrID[0] = attrID;

        zcl_SendRead(
            srcEP,
            &zclSample_DstAddr,
            clusterID,
            readCmd,
            ZCL_FRAME_CLIENT_SERVER_DIR,
            0,
            peripheralSeqNum++);
        osal_mem_free(readCmd);

    }


}

// 16 bit write , single attr id for now
void CustSendWriteAttrCmd(uint8 srcEP, uint16 dstAddr, uint8 dstEP, uint16 clusterID,uint16
                          attrID, uint8 dataType, uint16 newValue)
{
    zclWriteCmd_t *writeCmd;

    uint16 newPeriod = newValue;

    zclSample_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    zclSample_DstAddr.endPoint =          dstEP;
    zclSample_DstAddr.addr.shortAddr = dstAddr;

    writeCmd = osal_mem_alloc(sizeof(zclWriteCmd_t) + sizeof(zclWriteRec_t));

    if(writeCmd != NULL)
    {

        writeCmd->numAttr = 1;
        writeCmd->attrList[0].attrID = attrID;
        writeCmd->attrList[0].dataType = dataType;
        writeCmd->attrList[0].attrData = (uint8 *)&newPeriod;

        zcl_SendWrite(
            srcEP,
            &zclSample_DstAddr,
            clusterID,
            writeCmd,
            ZCL_FRAME_CLIENT_SERVER_DIR,
            0,
            peripheralSeqNum++);

        osal_mem_free(writeCmd);

    }

}

/***
  srcEP=8
  ���Ƶ����
  cmdID=0   toggle
  cmdID=1   on
  cmdID=2   off

  param1 = dstAddr

  param2 = dstEP, 8

  cmdID=


***/
void peripheralCoordinator_ProcessIncomingCommand(peripheralCmd_t *msg_ptr)
{

    ZStatus_t        status;
    zclReadCmd_t *readCmd;
    zclWriteCmd_t *writeCmd;
    zclReportCmd_t  *reportCmd;
    
    //uint16             attrId;

   // debug_str("Test event received by ");

    if(msg_ptr->srcEP != 8)
    {
        debug_str("Unrecognized EP");
        return;
    }
    else{
        debug_str("Recognized EP");
    }
    
    zclSample_DstAddr.addrMode =  (afAddrMode_t)Addr16Bit;
    zclSample_DstAddr.endPoint =          msg_ptr->param2;
    zclSample_DstAddr.addr.shortAddr = msg_ptr->param1;
    
     // toggle it
    if(msg_ptr->cmdID == FUNC_LIGHT_TOGGLE_CMD)
    {

        status = zcl_SendCommand( 
            (8), 
            (&zclSample_DstAddr), 
            ZCL_CLUSTER_ID_GEN_ON_OFF, 
            COMMAND_TOGGLE, 
            TRUE, 
            ZCL_FRAME_CLIENT_SERVER_DIR, 
            (true), 
            0, 
            (0), 
            0, 
            NULL );

        cust_debug_str("status toggle %d", status);

    }
    else if(msg_ptr->cmdID == FUNC_TURN_ON_LIGHT_CMG)
    {

        status = zcl_SendCommand( 
            (8), 
            (&zclSample_DstAddr), 
            ZCL_CLUSTER_ID_GEN_ON_OFF, 
            COMMAND_ON, 
            TRUE, 
            ZCL_FRAME_CLIENT_SERVER_DIR,
            (true), 
            0, 
            (0), 
            0, 
            NULL);

        cust_debug_str("son %d", status);

    }
    else if(msg_ptr->cmdID == FUNC_TURN_OFF_LIGHT_CMD)
    {

        status = zcl_SendCommand( 
            (8), 
            (&zclSample_DstAddr), 
            ZCL_CLUSTER_ID_GEN_ON_OFF, 
            COMMAND_OFF, 
            TRUE, 
            ZCL_FRAME_CLIENT_SERVER_DIR, 
            (true),
            0, 
            (0), 
            0, 
            NULL);

        cust_debug_str("so %d", status);

    }    
    // read attribute
    else if(msg_ptr->cmdID == FUNC_READ_ATTR_CMD)
    {
      
         readCmd = osal_mem_alloc(sizeof(zclReadCmd_t) + sizeof(uint16));

        if(readCmd)
        {
          
            readCmd->numAttr = 1;
            readCmd->attrID[0] = ATTRID_ON_OFF;

            zcl_SendRead(
                8, 
                &zclSample_DstAddr, 
                ZCL_CLUSTER_ID_GEN_ON_OFF, 
                readCmd, 
                ZCL_FRAME_CLIENT_SERVER_DIR,
                0, 
                peripheralSeqNum++);

            osal_mem_free(readCmd);
                
        }
    }
    else if(msg_ptr->cmdID == FUNC_READ_LIGHT_STA_CMD)
    {
        //read light status
        readCmd = osal_mem_alloc(sizeof(zclReadCmd_t) + sizeof(uint16));

        if(readCmd)
        {
          
            readCmd->numAttr = 1;
            readCmd->attrID[0] = ATTRID_ON_OFF;

            status = zcl_SendRead(
                8, 
                &zclSample_DstAddr, 
                ZCL_CLUSTER_ID_GEN_ON_OFF, 
                readCmd, 
                ZCL_FRAME_CLIENT_SERVER_DIR,
                0, 
                peripheralSeqNum++);
            osal_mem_free(readCmd);

            cust_debug_str("read light status: %d", status);

        }
    }
    else if(msg_ptr->cmdID == FUNC_READ_HEATBEAT_PERIOD_CMD)
    {
        // read HEARTBEAT_PERIOD uint16
        readCmd = osal_mem_alloc(sizeof(zclReadCmd_t) + sizeof(uint16));
        
        if(readCmd)
        {
          
            readCmd->numAttr = 1;
            readCmd->attrID[0] = ATTRID_BASIC_SMARTGARDEN_HEARTBEAT_PERIOD;

            status = zcl_SendRead(
                8, 
                &zclSample_DstAddr, 
                ZCL_CLUSTER_ID_GEN_BASIC, 
                readCmd, 
                ZCL_FRAME_CLIENT_SERVER_DIR,
                0, 
                peripheralSeqNum++);
            osal_mem_free(readCmd);

            cust_debug_str("read light status: %d", status);
                
        }
        
    }
    else if(msg_ptr->cmdID == FUNC_SET_HEARTBEAT_PERIOD_30_CMD)
    {
        // set HEARTBEAT_PERIOD uint8 to 30
        uint16 newPeriod = 30;
        writeCmd = osal_mem_alloc(sizeof(zclWriteCmd_t) + sizeof(zclWriteRec_t));
        
        if(writeCmd){
          
            writeCmd->numAttr = 1;
            writeCmd->attrList[0].attrID = ATTRID_BASIC_SMARTGARDEN_HEARTBEAT_PERIOD;
            writeCmd->attrList[0].dataType = ZCL_DATATYPE_UINT16;
            writeCmd->attrList[0].attrData = (uint8*)&newPeriod;

            zcl_SendWrite(
                8, 
                &zclSample_DstAddr, 
                ZCL_CLUSTER_ID_GEN_BASIC, 
                writeCmd, 
                ZCL_FRAME_CLIENT_SERVER_DIR,
                0, 
                peripheralSeqNum++);
            
            osal_mem_free(writeCmd);
                
        }
    }
    else if(msg_ptr->cmdID == FUNC_SET_HEARTBEAT_PERIOD_15_CMD){
        // set HEARTBEAT_PERIOD uint8 to 15
        uint16 newPeriod = 15;
        writeCmd = osal_mem_alloc(sizeof(zclWriteCmd_t) + sizeof(zclWriteRec_t));
        
        if(writeCmd){
          
            writeCmd->numAttr = 1;
            writeCmd->attrList[0].attrID = ATTRID_BASIC_SMARTGARDEN_HEARTBEAT_PERIOD;
            writeCmd->attrList[0].dataType = ZCL_DATATYPE_UINT16;
            writeCmd->attrList[0].attrData = (uint8*)&newPeriod;

            zcl_SendWrite(
                8, 
                &zclSample_DstAddr, 
                ZCL_CLUSTER_ID_GEN_BASIC, 
                writeCmd, 
                ZCL_FRAME_CLIENT_SERVER_DIR,
                0, 
                peripheralSeqNum++);
            
            osal_mem_free(writeCmd);
                
        }
    }
    else if(msg_ptr->cmdID == FUNC_SEND_HEARTBEAT_REPORT_CMD){
        // send heartbeat report
        
        reportCmd = osal_mem_alloc(sizeof(zclReportCmd_t ) + sizeof(zclReport_t));
        
        if(reportCmd)
        {
          
            reportCmd->numAttr = 1;
            reportCmd->attrList[0].attrID = ATTRID_BASIC_SMARTGARDEN_HEARTBEAT_PERIOD;
            reportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT8;
            reportCmd->attrList[0].attrData = &zclSmartGarden_HeartbeatPeriod;

            zcl_SendReportCmd(
                8, 
                &zclSample_DstAddr, 
                ZCL_CLUSTER_ID_GEN_BASIC, 
                reportCmd, 
                ZCL_FRAME_CLIENT_SERVER_DIR,
                0, 
                peripheralSeqNum++);
            
            osal_mem_free(reportCmd);
                
        }
    }
    
    else if(msg_ptr->cmdID == FUNC_READ_SOIL_TEMP_CMD)
    {
      readCmd = osal_mem_alloc(sizeof(zclReadCmd_t) + sizeof(uint16));
      
      if(readCmd)
      {
          
            readCmd->numAttr = 1;
            readCmd->attrID[0] = ATTRID_BASIC_SMARTGARDEN_TEMP;

            zcl_SendRead(
                8, 
                &zclSample_DstAddr, 
                ZCL_CLUSTER_ID_GEN_BASIC, 
                readCmd, 
                ZCL_FRAME_CLIENT_SERVER_DIR,
                0, 
                peripheralSeqNum++);
            osal_mem_free(readCmd);
                
        }
    }
    
    else if(msg_ptr->cmdID == FUNC_READ_SOIL_HUMI_CMD)
    {
       readCmd = osal_mem_alloc(sizeof(zclReadCmd_t) + sizeof(uint16));
      
      if(readCmd)
      {
          
            readCmd->numAttr = 1;
            readCmd->attrID[0] = ATTRID_BASIC_SMARTGARDEN_HUMIDITY;

            zcl_SendRead(
                8, 
                &zclSample_DstAddr, 
                ZCL_CLUSTER_ID_GEN_BASIC, 
                readCmd, 
                ZCL_FRAME_CLIENT_SERVER_DIR,
                0, 
                peripheralSeqNum++);
            osal_mem_free(readCmd);
                
        }
    }
    
    else if(msg_ptr->cmdID == FUNC_READ_SOIL_PH_CMD)
    {
      readCmd = osal_mem_alloc(sizeof(zclReadCmd_t) + sizeof(uint16));
      
      if(readCmd)
      {
          
            readCmd->numAttr = 1;
            readCmd->attrID[0] = ATTRID_BASIC_SMARTGARDEN_PH_VALUE;

            zcl_SendRead(
                8, 
                &zclSample_DstAddr, 
                ZCL_CLUSTER_ID_GEN_BASIC, 
                readCmd, 
                ZCL_FRAME_CLIENT_SERVER_DIR,
                0, 
                peripheralSeqNum++);
            osal_mem_free(readCmd);
                
        }
    }
    else if(msg_ptr->cmdID == FUNC_IDENTIFYING_CMD)
    {

        status = zcl_SendCommand( 
            (8), 
            (&zclSample_DstAddr), 
            ZCL_CLUSTER_ID_GEN_ON_OFF, 
            COMMAND_IDENTIFYING, 
            TRUE, 
            ZCL_FRAME_CLIENT_SERVER_DIR, 
            (true),
            0, 
            (0), 
            0, 
            NULL);

        cust_debug_str("identifying %d", status);


    }
    else if(msg_ptr->cmdID == FUNC_READ_IRRIGATE_CMD)
    {

        CustSendReadAttrCmd(8, zclSample_DstAddr.addr.shortAddr, 8, 0, 0x4007);

    }
    else if(msg_ptr->cmdID == FUNC_READ_ALARM_STATUS_CMD)
    {
        CustSendReadAttrCmd(8, zclSample_DstAddr.addr.shortAddr, 8, 0, 0x4002);

        cust_debug_str("read alarm status");

    }

    else if(msg_ptr->cmdID == FUNC_TURN_ON_IRRIGATE_CMD)
    {
            status = zcl_SendCommand( 
            (8), 
            (&zclSample_DstAddr), 
            ZCL_CLUSTER_ID_GEN_ON_OFF, 
            COMMAND_TURN_ON_IRRIGATE, 
            TRUE, 
            ZCL_FRAME_CLIENT_SERVER_DIR, 
            (true),
            0, 
            (peripheralSeqNum++), 
            0, 
            NULL);

        cust_debug_str("turnon irrigate %d", status);
    }
    else if(msg_ptr->cmdID == FUNC_TURN_OFF_IRRIGATE_CMD)
    {
            status = zcl_SendCommand( 
            (8), 
            (&zclSample_DstAddr), 
            ZCL_CLUSTER_ID_GEN_ON_OFF, 
            COMMAND_TURN_OFF_IRRIGATE, 
            TRUE, 
            ZCL_FRAME_CLIENT_SERVER_DIR, 
            (true),
            0, 
            (peripheralSeqNum++), 
            0, 
            NULL);

        cust_debug_str("turnoff irrigate %d", status);
    }
    else if(msg_ptr->cmdID == FUNC_TURN_ON_PERMITJOINING_CMD)
    {
            status = zcl_SendCommand( 
            (8), 
            (&zclSample_DstAddr), 
            ZCL_CLUSTER_ID_GEN_ON_OFF, 
            COMMAND_TURN_ON_PERMIT_JOINING, 
            TRUE, 
            ZCL_FRAME_CLIENT_SERVER_DIR, 
            (true),
            0, 
            (peripheralSeqNum++), 
            0, 
            NULL);

        cust_debug_str("turnoff irrigate %d", status);
    }
    else if(msg_ptr->cmdID == FUNC_READ_ALL_STATUS_CMD)
    {
        //read light status
        readCmd = osal_mem_alloc(sizeof(zclReadCmd_t) + 2*sizeof(uint16));

        if(readCmd != NULL)
        {

            readCmd->numAttr = 2;
            readCmd->attrID[0] = ATTRID_BASIC_SMARTGARDEN_IRRIGATE_ONOFF;
            readCmd->attrID[1] = ATTRID_BASIC_SMARTGARDEN_HEARTBEAT_PERIOD;


            zcl_SendRead(
                8,
                &zclSample_DstAddr,
                ZCL_CLUSTER_ID_GEN_BASIC,
                readCmd,
                ZCL_FRAME_CLIENT_SERVER_DIR,
                0,
                peripheralSeqNum++);
            osal_mem_free(readCmd);

        }

    }
        else if(msg_ptr->cmdID == FUNC_READ_TYPE1_ALL_STATUS_CMD)
    {
        //read light status
        readCmd = osal_mem_alloc(sizeof(zclReadCmd_t) + 7*sizeof(uint16));

        if(readCmd != NULL)
        {

            readCmd->numAttr = 7;
            readCmd->attrID[0] = ATTRID_BASIC_SMARTGARDEN_HEARTBEAT_PERIOD;
            readCmd->attrID[1] = ATTRID_BASIC_SMARTGARDEN_PH_VALUE;

            readCmd->attrID[2] = ATTRID_BASIC_SMARTGARDEN_TEMP;

            readCmd->attrID[3] = ATTRID_BASIC_SMARTGARDEN_HUMIDITY;

            readCmd->attrID[4] = ATTRID_BASIC_SMARTGARDEN_ALARM_STATUS;

            readCmd->attrID[5] = ATTRID_BASIC_SMARTGARDEN_IRRIGATE_ONOFF;
            
            readCmd->attrID[6] = ATTRID_BASIC_SMARTGARDEN_DEVICE_TYPE;
            

            zcl_SendRead(
                8,
                &zclSample_DstAddr,
                ZCL_CLUSTER_ID_GEN_BASIC,
                readCmd,
                ZCL_FRAME_CLIENT_SERVER_DIR,
                0,
                peripheralSeqNum++);
            osal_mem_free(readCmd);

        }

    }
    else if(msg_ptr->cmdID == FUNC_READ_TYPE2_ALL_STATUS_CMD)
    {
        //read light status
        readCmd = osal_mem_alloc(sizeof(zclReadCmd_t) + 7*sizeof(uint16));

        if(readCmd != NULL)
        {

            readCmd->numAttr = 7;
            readCmd->attrID[0] = ATTRID_BASIC_SMARTGARDEN_HEARTBEAT_PERIOD;
            readCmd->attrID[1] = ATTRID_BASIC_SMARTGARDEN_TEMP_INTENSITY;
            readCmd->attrID[2] = ATTRID_BASIC_SMARTGARDEN_HUMI_INTENSITY;
            readCmd->attrID[3] = ATTRID_BASIC_SMARTGARDEN_LIGHT_INTENSITY;
            readCmd->attrID[4] = ATTRID_BASIC_SMARTGARDEN_ALARM_STATUS;
            readCmd->attrID[5] = ATTRID_BASIC_SMARTGARDEN_IRRIGATE_ONOFF;
            readCmd->attrID[6] = ATTRID_BASIC_SMARTGARDEN_DEVICE_TYPE;


            zcl_SendRead(
                8,
                &zclSample_DstAddr,
                ZCL_CLUSTER_ID_GEN_BASIC,
                readCmd,
                ZCL_FRAME_CLIENT_SERVER_DIR,
                0,
                peripheralSeqNum++);
            osal_mem_free(readCmd);

        }

    }
     else if(msg_ptr->cmdID == FUNC_READ_TYPE1_SPECIFIC_CMD)
    {
        CustSendReadType1AttrCmd( 8, msg_ptr->param1, 8);
     }
    else if(msg_ptr->cmdID == FUNC_READ_TYPE2_SPECIFIC_CMD)
    {
        CustSendReadType2AttrCmd( 8, msg_ptr->param1, 8);
     }
         else if(msg_ptr->cmdID == FUNC_TYPE1_SOIL_ENABLE_CMD)
    {
         CustSendCmd(8,  msg_ptr->param1,  8,  6, 20);
     }
    else if(msg_ptr->cmdID == FUNC_TYPE1_SOIL_DISABLE_CMD)
    {
        CustSendCmd(8,  msg_ptr->param1,  8,  6, 21);
     }
    else if(msg_ptr->cmdID == FUNC_TYPE1_PH_ENABLE_CMD)
    {
        CustSendCmd(8,  msg_ptr->param1,  8,  6, 22);
     }
    else if(msg_ptr->cmdID == FUNC_TYPE1_PH_DISABLE_CMD)
    {
        CustSendCmd(8,  msg_ptr->param1,  8,  6, 23);
     }
    else if(msg_ptr->cmdID == FUNC_IDENTIFY_DUPLICATE_CMD)
    {
        CustSendCmd(8,  msg_ptr->param1,  8,  6, 16);
     }
     else if(msg_ptr->cmdID == FUNC_READ_TYPE1_SOIL_ENABLE_CMD)
    {
        CustSendReadAttrCmd(8, zclSample_DstAddr.addr.shortAddr, 8, 0,  ATTRID_BASIC_SMARTGARDEN_TYPE1_SENSOR_ENABLE);
     }    
     else if(msg_ptr->cmdID == FUNC_READ_TYPE1_PH_ENABLE_CMD)
    {
        CustSendReadAttrCmd(8,zclSample_DstAddr.addr.shortAddr, 8, 0,  ATTRID_BASIC_SMARTGARDEN_TYPE1_PH_ENABLE);
    }
    else
    {
            debug_str("Unknown cmdId");
    }
}


void peripheralRouter_ProcessIncomingCommand(peripheralCmd_t *msg_ptr)
{

    //ZStatus_t status;
    debug_str("Test event received by router");

    if(msg_ptr->srcEP != 8)
    {
        debug_str("Unrecognized EP");
        return;
    }
   
}
