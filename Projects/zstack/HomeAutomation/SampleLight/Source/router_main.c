#include "router_main.h"


#include "cust_func.h"
#include "peripheral.h"
#include "cust_timer.h"
#include "hal_led.h"
#include "sensor_timer.h"

uint32 chipidH;
uint32 chipidL;
extern byte peripheral_TaskID;
extern uint16 zclSmartGarden_HeartbeatPeriod;
extern uint8 blink_sign;
extern uint16 zclSmartGarden_Sensor_Enable;

void router_init(){

  relay_init();
  cust_wdt_init();
  chipidH = *((uint32*)0x00280028);
  chipidL = *((uint32*)0x0028002c);

  HalLedSet(HAL_LED_3, HAL_LED_MODE_ON);
  
#ifdef NV_RESTORE
  osal_nv_read(CUST_NV_SENSOR, 0, sizeof(zclSmartGarden_Sensor_Enable), 
        &zclSmartGarden_Sensor_Enable);
#endif

  cust_bspLedInit();
  cust_uart_init();
  cust_uart_open();  

  
  beep_init();

  sensor_timer2_init();
  sensor_switch_init();

  HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);

  periph_uart_init();
  periph_uart_open();

  init_1shot_cust_timer();
  start_1shot_cust_timer();

  trigger_sensor_timer();  
  
}
