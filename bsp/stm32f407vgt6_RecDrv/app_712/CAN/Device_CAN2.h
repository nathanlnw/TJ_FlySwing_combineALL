#ifndef   _CAN2_DEV
#define    _CAN2_DEV
#include <rtthread.h>
#include <rthw.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>//Êý×Ö×ª»»³É×Ö·û´®
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"
#include "Device_CAN.h"
#include <finsh.h>


   #define      U3_OUT_PWR_ON       GPIO_SetBits(GPIOE,GPIO_Pin_7)       
   #define      U3_OUT_PWR_OFF      GPIO_ResetBits(GPIOE,GPIO_Pin_7)    
extern  struct rt_device  Device_CAN2;

extern void  CAN2_RxHandler(unsigned char rx_data);   
extern void  Device_CAN2_regist(void );    



#endif 
