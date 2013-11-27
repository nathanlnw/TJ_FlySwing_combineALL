#ifndef   SYS_CONFIG
#define  SYS_CONFIG

#include <rthw.h>
#include <rtthread.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

//------------------------ RT_Thread  Config   ID  list  --------------------------------




 #define    ID_CONF_SYS                              0                           // 系统初始化 ID ,用来区别首次初始化


/*   
         Directory  Name    &    Size (SectorNum)                 ---Start 
*/
  //   1.  配置
#define   config                                                "config"
#define   config_size                                      1

#define    jt808                                                 "jt808"
#define    jt808_size                                       2

#define    tired_config                                     "tired_config"
#define    tired_config_size                            1

#define    BD_ext_config                                      "BD_extent"
#define    BD_ext_size                                      1

//  2.  循环存储
#define   cyc_gps                                            "cyc_gps" 
#define   cyc_gps_size                                    900           /* 1 record=128Byte    1 sector=4KB=32 条
                                                                                             1天30s 间隔 2880 条=>  2880/32=90 Sector/Day
                                                                                             至少存储10 天数据, 所以  size=900   Sector
                                                                                        */ 

// 2.  固定序号	 
#define    event_808                                       "event"  
#define    event_size                                      1 

#define    msg_broadcast                               "msg_broadcast"
#define    msg_broadcast_size                      1

#define    phonebook                                      "phonebook"
#define    phonebook_size                             1

#define    Rail_cycle                                        "Rail_cycle"
#define    Rail_cycle_size                               1

#define    Rail_rect                                          "Rail_rect"
#define    Rail_rect_size                                 1

#define    Rail_polygen                                   "Rail_polygen"
#define    Rail_polygen_size                           1

#define    turn_point                                      "turn_point"
#define    turn_point_size                               1

#define    route_line                                        "route_line"
#define    route_line_size                                      1

#define    ask_quesstion                                 "ask_quesstion"
#define    ask_quesstion_size                               1 

#define    text_msg                                         "text_msg"
#define    text_msg_size                                       1 

 // 3.  记录 

#define    spd_warn                                         "spd_warn"
#define    spd_warn_size                                 1

#define    tired_warn                                       "tired_warn"
#define    tired_warn_size                              1

#define    doubt_data                                     "doubt_data"
#define    doubt_data_size                             3

#define    spdpermin                                      "spdpermin"
#define    spdpermin_size                              3

#define    pospermin                                       "pospermin"
#define    pospermin_size                               3

#define    pic_index                                     "pic_index"
#define    pic_index_size                                2

#define    voice_index                                     "voice_index"
#define    voice_index_size                            2


  // 4. 拍照
#define    camera_1                                        "camera_1"
#define    camera_1_size                                 8

#define    camera_2                                        "camera_2"
#define    camera_2_size                                 8

#define    camera_3                                        "camera_3"
#define    camera_3_size                                 8

#define    camera_4                                        "camera_4"
#define    camera_4_size                                 8


// 5. 录音 
#define    voice                                                "voice"
#define    voice_size                                         8
/*   
         Directory  Name    &    Size (SectorNum)                 ---End
*/








//------------------------------------------------------------------------------
#define   SOFTWARE_VER     0x0001

#define  STM32F103_Recoder_16MbitDF            0x00000005        // 行车记录仪ID 标识  16Mit
#define  STM32F103_Recoder_32MbitDF            0x00000007        // 行车记录仪ID 标识  16Mit 
#define  STM32F103_Recoder_32MbitMG323         0x00000008        // 行车记录仪ID 标识  16Mit   
#define  STM32F407_Recoder_32MbitDF            0x00000009      




#define   Max_SystemCounter            86400// 28800 // 86400   //定时重启时间24小时 一天   

//-----------------------  Max  Add    ---------------------------------
#define   Max_CycleNum                  16384
                                                 /*
												   每一条记录为31字节，这里取每条记录32个字节，每page 16条记录
												   1024page=1024x512 Bytes=1024*16=16284Record 
                                                         */
#define   Max_PicNum                    400
                                                 /*
												  每张图片32Page，1Page 索引，31page 图片内容
												  32768page=32768/32=1024 pics 
												  */
#define   Max_RecoderNum                400
                                                 /*
                                                            每条记录256个字节，1page =8 Record
                                                         */
#define   Max_CommonNum                  128 
                                                 /*
                                                             1page=64Records    2048 page =131072 Records 
                                                         */
#define  Max_SPDSperMin                  2800
                                                 /*
                                                             1record=70Bytes   1page=29Records  2048 page=59292 Records   
                                                         */
#define  Max_SPDerSec                    560      
                                                 /*
                                                             1record=70Bytes   1page=29Records  16000 page=464000 Records   
                                                         */
#define  Max_MintPos                     48   
                                                 /*  单位小时内每分钟的位置信息
                                                             1record=485Byte   1page=1Record   2048Page=8192records
                                                          */


/* Function  Select  Define  */
#define        IP2
#define        IP3 

// status
#define  AccON_Over                1                          //   1 : ACC 点火操作完成     0 :  ACC  关火操作完成 
#define  AccOFF_Over               0
#define  LOG_IN                    1                          //   01H:登录，02H：退出，03H：更换驾驶员
#define  LOG_OUT                   2 
#define  LOG_Change                3
#define  Power_Normal              1                          //01H:上电，02H：断电
#define  Power_Cut                 2
#define  SETTING_INFO              0x82            
#define  SETTING_Status            0x84 
#define  SETTING_Time              0xC2 
#define  SETTING_Plus              0xC3
#define  Route_Mum                  16

                                                              /*
																	82H:设置车辆信息，84H：设置状态量
																	C2H:设置记录仪时钟 
																	C3H:设置记录仪速度脉冲系数
															  */





/* System Basic  Define   */

//---------------------  Flash 读写函数的 宏定义  --------------
//#define    FlashWrite         FSMC_NAND_Write_withOffset      // Flash  写入  (uint8_t *pBuffer, NAND_ADDRESS Address, u16 RdOffset,u16 Rdlen)
//#define    FlashRead	         FSMC_NAND_Read_withOffset       // Flash  读取
//#define    FlashErase         FSMC_NAND_EraseBlock_InPage     // Flash  擦除  


#define Init_RecordSize 40



extern ALIGN(RT_ALIGN_SIZE)  SYS_CONF        SysConf_struct;   //  系统配置
extern ALIGN(RT_ALIGN_SIZE)  JT808_CONF      JT808Conf_struct;   //  JT 808   相关配置 
extern  ALIGN(RT_ALIGN_SIZE) JT808_CONF		 JT808_struct_Bak;	  //  JT808 相关模式设置备份 
extern  ALIGN(RT_ALIGN_SIZE) JT808_CONF		 JT808_struct_Bak2;	  //  JT808 相关模式设置备份  
extern ALIGN(RT_ALIGN_SIZE)  TIRED_CONF      TiredConf_struct;    //  疲劳驾驶相关配置 



//----------  Basic  Config---------------------------
extern u8		DeviceNumberID[13];//="800130100001";	 // 车辆DeviceID	---- 河北天地通用
extern u8		SimID_12D[13]; // 
extern u8       HardWareVerion;   //   硬件版本检测 

extern u8       RemoteIP_Dnsr[4]; 
extern u8		RemoteIP_main[4];//
extern u16		RemotePort_main;
extern u8		RemoteIP_aux[4];    
extern u16		RemotePort_aux; 

//      Link2  Related 
extern u8      Remote_Link2_IP[4]; 
extern u16     Remote_Link2_Port;     



extern u8		APN_String[30];	  // APN  字符串   
extern u8		DomainNameStr[50];	// 域名  天地通  
extern u8       DomainNameStr_aux[50];        


extern u32		   Current_SD_Duration;  //GPS 信息报告的时间间隔       
extern u32 		   Current_SD_Distance; // GPS 信息定距上报距离  
extern u8		   Current_State;		 // 上报实时标志位信息	 预防DF 损坏  
extern u32 		   DistanceAccumulate;	 // 定距上报累加器
extern u16         ACC_on_sd_Duration;    //  ACC 开启的时候 上报的时间间隔
extern u16         ACC_off_sd_Duration;    //  ACC 关闭时候上报的时间间隔
extern u8		   TriggerSDsatus;   // 传感器触发上报状态位 

extern u16         StopLongDuration;    //超长停车报警最长时间
extern u16         Stop_counter;               //超长停车的临时计数器


extern u8       EmergentWarn;               // 紧急报警





extern u8     Vechicle_TYPE;                //   车辆类型    1:大型货车  2: 小型货车  3:大型客车  4: 中型客车   5:小型客车
extern u8	  OnFire_Status; 					//      1 : ACC 点火操作完成	   0 :	ACC  关火操作完成 
extern u8     Login_Status;                    //   01H:登录，02H：退出，03H：更换驾驶员
extern u8     Settingchg_Status;              /*
												82H:设置车辆信息，84H：设置状态量
												C2H:设置记录仪时钟 
												C3H:设置记录仪速度脉冲系数
											   */

											   


                                            /*
                                                            4  bytes                     4 bytes
                                                            写地址                      存储在Nand中的地址
                                                           
                                                    */

//extern u16    DaySpdMax;                                //  当天最大速度
//extern u16    DayDistance;                              //  当天行驶距离


//--------------  定距上报  --------------------
extern u32  former_distance_meter;     //   上一次距离    定距回传时候有用
extern u32  current_distance_meter;    //   当前距离    

//---------  SytemCounter ------------------
extern u32  Systerm_Reset_counter; 
extern u8   SYSTEM_Reset_FLAG;        // 系统复位标志位  
extern u8   DistanceWT_Flag;  //  写里程标志位

extern u32      Device_type;    // 硬件类型   STM32103  新A1 
extern u32      Firmware_ver;   // 软件版本 
extern u8	    ISP_resetFlag;		 //远程升级复位标志位 


extern void TEXTMSG_Write_Init(void);
extern void TEXTMsg_Read (void);

//-------------------------------------------------------------------------------------------------------------   	
extern void TIRED_DoorValue_Read(void);
extern void SendMode_ConterProcess(void);
extern void SendMode_Config(void)  ;
extern void JT808_Vehicleinfo_Init(void);
extern void JT808_RealTimeLock_Init(void);
extern void Event_Init(u8  Intype);
extern void Question_Read(void);
extern void Event_Read(void);
extern void PhoneBook_Init(u8  Intype);
extern void PhoneBook_Read(void);    
extern void  RailCycle_Init(void);
extern void  RailCycle_Read(void);
extern void  RailRect_Init(void);
extern void  RailRect_Read(void);
extern void  RailPolygen_Init(void);
extern void  RailPolygen_Read(void);
extern void  RouteLine_Init(void);
extern void  RouteLine_Read(void);  
extern void  ProductAttribute_init(void);






extern void Rails_Routline_Read(void);        
extern void SysConfiguration(void);
extern void SetConfig(void);
extern void DefaultConfig(void);
extern void RstWrite_ACConoff_counter(void);
extern void BD_EXT_Write(void);
extern void MSG_BroadCast_Init(u8  Intype);
extern void  idip(u8 *str);  


//写入文本信息1-8
extern void TEXTMSG_Write(u8 num,u8 new_state,u8 len,u8 *str);


#endif
