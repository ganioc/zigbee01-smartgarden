#ifndef PERIPHERAL_H
#define PERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif
  
 /*********************************************************************
 * INCLUDES
 */
#include "zcl.h"

#define PERIPH_TEST_EVENT   1
#define PERIPH_RESET_EVENT  2
#define PERIPH_HEARTBEAT_REPORT 4
#define PERIPH_PH_SENSOR_UPDATE 8
#define PERIPH_TEMPHUMI_SENSOR_UPDATE 16
#define PERIPH_AIR_LIGHT_UPDATE 32
#define PERIPH_AIR_TEMPHUMI_UPDATE 64
#define PERIPH_TYPE_SENSOR_UPDATE  128  
//
  
  
#define PERIPH_TEST_CMD1    0
#define PERIPH_TEST_CMD2    1 





   
   
typedef struct
{
  	osal_event_hdr_t hdr;
  	uint8          srcEP;
	uint16         cmdID;
	uint16         param1;
	uint16         param2;
} peripheralCmd_t;

typedef struct
{
  uint8  numAttr;
  uint16 attrID[1];
} peripheral_ReadCmd_t;

void CustSendReadAttrCmd(uint8 srcEP, uint16 dstAddr, uint8 dstEP, uint16 clusterID,uint16 
attrID);

void CustSendCmd(uint8 srcEP, uint16 dstAddr, uint8 dstEP, uint16 clusterID,uint16 
cmdID);
void CustSendWriteAttrCmd(uint8 srcEP, uint16 dstAddr, uint8 dstEP, uint16 clusterID,uint16
                          attrID, uint8 dataType, uint16 newValue);

void CustSendReadType1AttrCmd(uint8 srcEP, uint16 dstAddr, uint8 dstEP);

void CustSendReadType2AttrCmd(uint8 srcEP, uint16 dstAddr, uint8 dstEP);

 /*
  * Initialization for the task
  */
extern void peripheral_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 peripheral_event_loop( byte task_id, UINT16 events );
  
byte peripheral_taskId(void);

void  peripheral_TestCmd(uint8 * pBuf);

void peripheral_reset_default(void);

void peripheral_reset(void);


#ifdef __cplusplus
}
#endif

#endif
