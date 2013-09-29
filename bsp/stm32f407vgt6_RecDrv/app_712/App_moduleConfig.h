/*      Create By  Nathanli
       APP_712  Project use      
*/

#ifndef     _APP_MODULE_
#define      _APP_MODULE_

#include  <stdlib.h>//数字转换成字符串
#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>

/*   scons  stytle 
#include <./gps/Device_gps.h>
#include <./gsm/Device_gsm.h>  

#include <./U4_485/Device_485.h>

#include <./protocol_808/App_808.h>

#include <./HMI/App_hmi.h>
#include <./U4_485/App_485.h>

#include <./gps/App_gps.h>
#include <./gsm/App_gsm.h>

#include <./protocol_808/Protocol_808.h>
#include <./protocol_808/Device_808.h>
#include <./protocol_808/Sys_config.h>

#include <./rtc/Rtc.h>

#include <./dataflash/Sst25vfxx.h>
#include <./dataflash/Spi_flash.h>
#include <./dataflash/SST25/SST25VF.h>


//#include <./lcd/Menu_Include.h> 
#include <./lcd/lcd.h>
#include <./lcd/Include.h>
#include <./lcd/SLE4442.h>
#include <./lcd/bmp.h>
#include <./lcd/SED1520.h>
#include <./lcd/Fonts.h>    

#include <./printer/Device_printer.h> 
*/

//#include "Device_gps.h"
#include "Device_gsm.h" 
#include "Device_CAN.h"
#include "Device_CAN2.h"
#include "Device_485.h"

#include "App_808.h"

#include "App_hmi.h"
#include "App_485.h"

//#include "App_gps.h"
#include "App_gsm.h"

#include "Protocol_808.h"
#include "Device_808.h"
#include "Sys_config.h"

#include "rtc/Rtc.h"

//#include "Sst25vfxx.h"
//#include "Spi_flash.h"
#include "SST25VF.h"
#include "DF_Function.h"
#include "DF_Oper.h"
#include "Flash_Oper.h" 


#include "Menu_Include.h" 
#include "lcd.h"
//#include "Include.h"
//#include "SLE4442.h"
#include "bmp.h"
#include "SED1520.h"
//#include "Fonts.h"   
//#include "Device_printer.h"
#include "spi_sd.h"
#include "Usbh_conf.h"
#include <dfs_posix.h>
#include <finsh.h>
#include "vuart.h" 
#include "gps.h"  
#include "Device_printer.h" 
#include "mma8451.h" 

#include "SMS.h"
#include "SLE4442.h"
#include "IC_card.h"
#include "CRC.h"  

#include "Vdr.h"
#include "DF_Oper.h"



// size                 

#define true       1
#define false      0




//   1.  Module  Related  
#define  Finsh_UART1       // Finsh Use
#define  GSM_UART           //   GSM 模块相关
#define  PRINTER     //   打印  
#define  _485_UART4     //   摄像头，485 
#define  GPS_UART            //  GPS 模块
#define  APP808
#define  HMI
#define  USBhost_712          //  USB host
#define  DATAFLASH            //


//  2. MsgQ  Setting  
#define   MsgQ_Timeout            3

#ifdef GSM_UART
#define GSM_DEVICE     "uart4"         
#endif 


#ifdef _485_UART4
#define  _485_DEVICE     "uart2"          
#endif 

#ifdef GPS_UART
#define GPS_DEVICE      "uart5"
#endif


//------  Defined    according to   712   rule   ---------
 typedef  struct   _Msg_QTYPE_T
{
    u16   len; 
    u8     *info;
    u8     link_num;	
}MSG_Q_TYPE;   



//-------- Fuction  ----------------------------
extern rt_err_t rt_712Rule_MsgQue_Post(rt_mq_t Dest_mq,u8 *buffer, rt_size_t size);      


#endif
