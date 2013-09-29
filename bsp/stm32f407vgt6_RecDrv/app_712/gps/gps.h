#ifndef _GPS_H_
#define _GPS_H_


//------ GPS 模块类型 -----------
#define   Module_3017A                0         // 北斗模块 3017A
#define   Module_3020C                1         // 北斗模块 3020C   


#define CTL_GPS_BAUD	0x30
#define CTL_GPS_OUTMODE	0x31	/*gps信息的输出模式*/


#define GPS_OUTMODE_TRIGGER	0x01	/*秒触发输出*/
#define GPS_OUTMODE_GPRMC	0x02
#define GPS_OUTMODE_GPGSV	0x04
#define GPS_OUTMODE_BD		0x08

#define GPS_OUTMODE_ALL		0xFFFFFFFF	/*全数据输出*/


/*
北斗更新的操作状态结果码，分为几个部分

0000 0000-0FFF FFFF  包序号


*/


#define BDUPG_RES_UART_OK		(0x10000000)	/*没有错误升级成功-升级成功*/
#define BDUPG_RES_UART_READY	(0x1000FFFF)	/*串口更新就绪*/


#define BDUPG_RES_USB_OK		(0x20000000)	/*没有错误升级成功-升级成功*/

#define BDUPG_RES_USB_FILE_ERR	(0x2000FFFC)	/*更新文件格式错误*/
#define BDUPG_RES_USB_NOFILE	(0x2000FFFD)	/*更新文件不存在*/
#define BDUPG_RES_USB_WAITUSB	(0x2000FFFD)	/*更新文件不存在*/
#define BDUPG_RES_USB_NOEXIST	(0x2000FFFE)	/*u盘不存在*/
#define BDUPG_RES_USB_READY		(0x2000FFFF)	/*u盘就绪*/


#define BDUPG_RES_USB_MODULE_H	(0x20010000)	/*模块型号高16bit*/
#define BDUPG_RES_USB_MODULE_L	(0x20020000)	/*模块型号低16bit*/
#define BDUPG_RES_USB_FILE_VER	(0x20030000)	/*软件版本*/


#define BDUPG_RES_THREAD	(0xFFFFFFFE)	/*创建升级线程失败-创建更新失败*/
#define BDUPG_RES_RAM		(0xFFFFFFFD)	/*分配RAM失败*/
#define BDUPG_RES_TIMEOUT	(0xFFFFFFFC)	/*超时失败*/




//=============================
typedef struct gps_rmc { 
char utc_year;
char utc_mon;
char utc_day;
char utc_hour;
char utc_min;
char utc_sec;
char status; 
float latitude_value; 
char latitude; 
float longtitude_value; 
char longtitude; 
float speed; 
float azimuth_angle; 
} GPS_RMC; 

typedef  struct  GPS_STATUS
{
   u8  Position_Moule_Status;  // 1: BD   2:  GPS   3: BD+GPS    定位模块的状态
   u8  Antenna_Flag;//显示提示开路 
   u8  Raw_Output;   //  原始数据输出  
}GPSSTATUS;

typedef struct  Gps_Abnormal
{
   u32   current_datacou;  // 当前计数值
   u32   last_datacou;       // 上次计数值
   u16   no_updateTimer; // 没有数据更新定时器
   u16   GPS_Rst_counter;
   u8     Reset_gps;           // GPS timer   
   
   //       add  on  2013  -4-20         
   u16   GPS_circuit_short_couter;  // GPS 短路 次数判断 
   u8    GPS_short_checkFlag;//  GPS 短路判断标志位  常态 0    三次内短路 1    3次以上 2 
   u16   GPS_short_timer;   // short  判断计数器   
   u16   GPS_short_keepTimer;  //  短路持续时间
  
}GPS_ABNORMAL;

typedef struct _POS_ASC
{
  u8 Lat_ASCII[20];
  u8 Longi_ASCII[20];
  u8 AV_ASCII; 
}POS_ASC;

extern  POS_ASC Posit_ASCII;  // 位置信息 ASCII

// -----  GPS   App  Related  -------
#define	GPSRX_SIZE	130

extern  uint8_t	   flag_bd_upgrade_uart ;
extern  GPSSTATUS  GpsStatus;
extern  GPS_ABNORMAL   Gps_Exception;   
extern  u8	 GPS_MODULE_TYPE; //  GPS 定位模块类型 	0X00:3017A	  1:  3017A   其他: 未检测模块类型

//============================  
extern void BD_MODULE_Read(void);
extern void BD_MODULE_Write(u8  in);   




void gps_init( void );
extern void gps_baud( int baud );
extern void  gps_mode(u8 *str) ;
extern void  GpsIo_Init(void);
extern rt_err_t gps_onoff( uint8_t openflag );
extern void  GPS_Abnormal_process(void);  
extern void  GPS_ANTENNA_status(void);      //  天线开短路状态检测
extern void  GPS_short_judge_timer(void);  
void thread_gps_upgrade_uart( void* parameter );
void thread_gps_upgrade_udisk( void* parameter );


#endif

