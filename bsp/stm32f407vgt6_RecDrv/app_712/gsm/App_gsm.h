/*
     APP_GSM.H 
*/

#ifndef   _APP_GSM
#define  _APP_GSM  

#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"

#include  <stdlib.h>
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"


//----------- ModuleStatus----------
#define Status_GPS			0x01    // Bit 0:   1 定位   0 不定位
#define Status_GPRS 		0x02    // Bit 1:   1 连接   0 断开
#define Status_Battery      0x04    // Bit 2:   1 用电池 0 用外电 
#define Status_Pcheck       0x08    // Bit 3:   1 速度脉冲系数校验过   0  未被校验


// ------------  GSM  Device  define ------------
#define                     APN                         0x01
#define                     main_socket           0x02
#define                     aux_socket             0x03
#define                     isp_socket              0x04
#define                     DNSR1                     0x05
#define                     DNSR2                     0x06

#define                     power_on                0x11 
#define                     at_init                     0x12
#define                     dial_gprs                0x13 

#define                     query_online          0x21
#define                     send_gprsdata       0x22



typedef struct  _SOCKET
{
    u8 ip[4];
    u16  port;	

}SOCKET;




#define GSM_RAWINFO_SIZE 9000
ALIGN(RT_ALIGN_SIZE)
extern  uint8_t					GSM_rawinfo[GSM_RAWINFO_SIZE];
extern struct rt_messagequeue	mq_GSM; 


extern u8 	ModuleStatus;   //网络状态 
extern u8       Mocule_Data_Come; // 模块收到数据 



/*
    应用相关  
*/
extern  u8 DataLink_Online;   //  DataLink  在线标志    以前的GPRS_Online
extern u8  DataLink_EndFlag;         // Close_DataLink
extern  u8 DataLink_EndCounter; 
extern u8  PositionInfo_sdFlag;     // 发送定位信息标志
extern u8  Datalink_close;  //挂断后不再登陆
extern u8  Current_UDP_sd;   // 及时上报 标志位



extern u8  COPS_Couter;             // COPS  返回次数

extern u16  CSQ_counter;
extern u16  CSQ_Duration;    //查询CSQ 的定时间隔 
extern u8  CSQ_flag;
extern u8  ModuleSQ;  //GSM 模块信号强度数值
extern u8  ModuleStatus;   //网络状态 

extern u8   Light;
extern u8   Send_DataFlag;  // 发送GSM data Flag  
extern u8	LinkNum;  // 通信链路	0	 LINK 1    1   LINK     
extern u8   Receive_DataFlag;// 接收数据  
extern  u8    GSM_HEX[1024];  
extern  u16   GSM_HEX_len;    
extern 	u8	 Redial_reset_save; // 拨号重启前存储参数

/*
    应用相关函数
*/

extern u8      DataLink_Status(void);
extern void   DialLink_TimeOut_Clear(void);    
extern void   DialLink_TimeOut_Enable(void);      
extern u8     Close_DataLink(void);
extern  void  DataLinkOK_Process(void);    // 数据连接成功后做清除状态处理

extern u8   PositionSD_Enable(void);
extern u8   PositionSD_Disable(void); 
extern u8   PositionSD_Status(void);
extern u8   Stop_Communicate(void);
extern void Gsm_rxAppData_SemRelease(u8* instr, u16 inlen, u8 link_num);


/*
         RT      相关 
*/
extern void _gsm_startup(void);

#endif 

