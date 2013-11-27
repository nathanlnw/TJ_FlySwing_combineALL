/************************************************************
 * Copyright (C), 2008-2012,
 * FileName:		// 文件名
 * Author:			// 作者
 * Date:			// 日期
 * Description:		// 模块描述
 * Version:			// 版本信息
 * Function List:	// 主要函数及其功能
 *     1. -------
 * History:			// 历史修改记录
 *     <author>  <time>   <version >   <desc>
 *     David    96/10/12     1.0     build this moudle
 ***********************************************************/
#include <stdio.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <dfs_posix.h>

#include "stm32f4xx.h"
#include "gps.h"
#include "App_moduleConfig.h"

#include <finsh.h>

#define GPS_PWR_PORT	GPIOD
#define GPS_PWR_PIN		GPIO_Pin_10

#define GPS_GPIO_TX			GPIO_Pin_12 // PC12
#define GPS_GPIO_RX			GPIO_Pin_2  // PD2
#define GPS_GPIO_TxC		GPIOC
#define GPS_GPIO_RxD		GPIOD
#define RCC_APBPeriph_UART5 RCC_APB1Periph_UART5

/*声明一个gps设备*/
static struct rt_device dev_gps;

/*串口接收缓存区定义*/
#define UART5_RX_SIZE 256

#define GPS_RAWINFO_SIZE    4096 
#define  NMEA_SEGlength     128
#define  NMEA_SEGNUM         32


typedef __packed struct
{
	uint16_t	wr;
	uint8_t		body[UART5_RX_SIZE];
}LENGTH_BUF;

typedef __packed struct
{
	u8	 wr_num;
	u8	 len_sigema[NMEA_SEGNUM];
}NMEA_SEG;


static LENGTH_BUF uart5_rxbuf;
static LENGTH_BUF gps_rx;
//static uint8_t	uart5_rxbuf[UART5_RX_SIZE];	/*预留前两个字节，保存长度*/
//static uint16_t uart5_rxbuf_wr = 2;

/*gps原始信息数据区定义*/
static uint8_t					gps_rawinfo[GPS_RAWINFO_SIZE];
static struct rt_messagequeue	mq_gps;

uint8_t							flag_bd_upgrade_uart = 0;

//---------------------
static  NMEA_SEG   NMEA_WR;
static   u8     nmea_rd=0;  
static   u8   nmea_segBuf[GPS_RAWINFO_SIZE];
//---------------------


static rt_uint8_t				*ptr_mem_packet = RT_NULL;

//*****************************************
//CRC16 高字节表
//*****************************************

static unsigned char CRC16TabH[256] =
{
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

//*****************************************
//CRC16 低字节表
//*****************************************
static unsigned char CRC16TabL[256] =
{
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2,
	0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
	0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
	0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
	0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
	0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
	0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6,
	0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
	0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
	0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE,
	0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
	0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA,
	0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
	0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
	0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62,
	0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
	0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE,
	0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
	0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
	0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76,
	0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
	0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
	0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
	0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
	0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A,
	0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86,
	0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

//==========================

/*  RT_thread    应用相关*/
u8	 systemTick_TriggerGPS=1;  //	触发系统时间定时发送GPS标志    默认启动本地定时 
u16  systemTick_trigGPS_counter=0;//触发系统定时下的时间计数器


const char BD_MODE[]={"$CCSIR,1,0*49\r\n"};
const char GPS_MODE[]={"$CCSIR,2,0*4A\r\n"};
const char GPSBD_MODE[]={"$CCSIR,3,0*4B\r\n"};  

GPSSTATUS    GpsStatus;    
GPS_ABNORMAL   Gps_Exception;  // gps  模块异常处理
u8   GPS_MODULE_TYPE=0;        //  GPS 定位模块类型     0X00:3017A    1:  3017A   其他: 未检测模块类型
u32  GNTXT_Counter=0;          //  GPS 文本记录 
u32  RMC_Counter=0;            // RMC  计数器 ，到20  判断TXT ,否则就是3020C
POS_ASC Posit_ASCII;  // 位置信息 ASCII
//==========================
static rt_err_t dev_gps_init( rt_device_t dev );

void  gps_keyIO_init(void) 
{
  	GPIO_InitTypeDef	GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART5, ENABLE );

	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = GPS_PWR_PIN;
	GPIO_Init( GPS_PWR_PORT, &GPIO_InitStructure );
	GPIO_SetBits( GPS_PWR_PORT, GPS_PWR_PIN );    //  上电  ---


	if(Module_3017A!=GPS_MODULE_TYPE)
	{
		//#ifdef HC_595_CONTROL      
		       // 2013-4-20    更改PCB   用PD4 : GPS 天线开路      PB6 : GPS  天线短路
		       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
		       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
		       GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; 	 
		      // 		IN
			//------------------- PD4 -----------------------------
			GPIO_InitStructure.GPIO_Pin	 = GPIO_Pin_4;	  //GPS 天线开路
			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; 
			GPIO_Init(GPIOD, &GPIO_InitStructure);
		      //------------------- PB6 -----------------------------
			GPIO_InitStructure.GPIO_Pin	 = GPIO_Pin_6;	  //GPS 天线短路 
			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; 
			GPIO_Init(GPIOB, &GPIO_InitStructure);   
		//#endif
	}

}


void BD_MODULE_Read(void)
{
   DF_ReadFlash(DF_GPS_MODULE_offset,0,&GPS_MODULE_TYPE,1);  
   if(Module_3017A==GPS_MODULE_TYPE)
     rt_kprintf("\r\n	北斗定位模块: 3017A");
  else
  if(Module_3020C==GPS_MODULE_TYPE)
     rt_kprintf("\r\n	北斗定位模块: 3020C");
 // else
  //	rt_kprintf("\r\n	北斗定位模块: 尚未自动检测");
   
}

void BD_MODULE_Write(u8  in)
{
  RMC_Counter=0;
  GNTXT_Counter=0;   
  if(in)
    GPS_MODULE_TYPE=1;
  else
  	 GPS_MODULE_TYPE=in; 
  DF_WriteFlashSector(DF_GPS_MODULE_offset,0,&GPS_MODULE_TYPE,1);  
  
  if(Module_3017A==GPS_MODULE_TYPE)
     rt_kprintf("\r\n	设置北斗定位模块: 3017A");
  else
  if(Module_3020C==GPS_MODULE_TYPE)
     rt_kprintf("\r\n	设置北斗定位模块: 3020C");
 //  else
  //	rt_kprintf("\r\n	设置北斗定位模块: 尚未自动检测");

      gps_keyIO_init();   // 去掉3020C  硬件开短路检测的管脚初始化
	  APP_IOpinInit();         //  配置 3017A的蜂鸣器
	  printer_port_init();     //  打印控制还原3017A 模块控制
}
FINSH_FUNCTION_EXPORT(BD_MODULE_Write, BD_MODULE_Write[1|0]); 


void BD_MODULE_TXT_Check(void)
{
    
  if(Module_3017A!=GPS_MODULE_TYPE)
  {
      BD_MODULE_Write(Module_3017A);	  
     rt_kprintf("\r\n	检测到TXT 改成模式为: 3017A!\r\n");  
  }
}

void BD_MODULE_3020C_check(void)
{
    
  if(Module_3017A==GPS_MODULE_TYPE)
  {
      BD_MODULE_Write(Module_3020C);  
	  rt_kprintf("\r\n	20s  未检测到TXT  切换到: 3020C!\r\n"); 
  }
}
	

void GPS_Abnormal_init(void)
{
   Gps_Exception.current_datacou=0;
   Gps_Exception.no_updateTimer=0;
   Gps_Exception.last_datacou=0;
   Gps_Exception.GPS_Rst_counter=0;
   Gps_Exception.Reset_gps=0; 
}
void  GPS_Abnormal_process(void)
{
    		  //----------GPS 异常情况处理-----------------------
		   Gps_Exception.no_updateTimer++;
		  if( Gps_Exception.no_updateTimer>=300) 
			{
			   Gps_Exception.no_updateTimer=0;
			  if(Gps_Exception.Reset_gps==1)
				{
				  Gps_Exception.Reset_gps=0;
				  gps_onoff(1); // GPS power on
				  // rt_kprintf("\r\n	gps power on");   
				   Gps_Exception.GPS_Rst_counter++;
				   if(Gps_Exception.GPS_Rst_counter>5)
				   	{
                                        Gps_Exception.GPS_Rst_counter=0; 
				   	}
				}  
				else
				{
					if(Gps_Exception.current_datacou-Gps_Exception.last_datacou<300) 
						{
						        gps_onoff(0); //GPS  power off
							  Gps_Exception.Reset_gps=1;						  
							  //---------断电就置为不定位 -----------------						  
							  UDP_dataPacket_flag=0X03;
							  Car_Status[3]&=~0x02;   //Bit(1)
	                                            ModuleStatus &= ~Status_GPS;
							  GPS_getfirst=0;
						  //-------------------------------------------
						}
					else
						{ 
						   Gps_Exception.last_datacou=0;
						   Gps_Exception.current_datacou=0;          
						}
				}
	    }




}


//$GPRMC,063835.00,A,3909.11361,N,11712.50398,E,0.192,150.85,110305,,,A*64
//$GPRMC,063835.00,A,4000.81000,N,11556.40000,E,0.192,150.85,110305,,,A*64

u8 Process_RMC(u8* packet)  
{
	u8  CommaCount=0,iCount=0,k=0; 
	u8  tmpinfo[15]; // 新版本的北斗模块经度更高了
	                          //$GNRMC,085928.00,A,3920.020977,N,11744.385579,E,0.7,,020113,,,A*67
	                          //$GNRMC,090954.00,A,3920.024800,N,11744.384457,E,0.3,,020113,,,A*65
	                          //           11744.385579    
	u8  hour=0,min=0,sec=0,fDateModify=0;

    //----------------------------------------------
     RMC_Counter++;
	 if(RMC_Counter==20)
	 {
	      if((GNTXT_Counter==0)&&(HardWareVerion==7))// 判断是否有TXT 报文  ,如果一直没有  ，硬件版本全1 下判断
              BD_MODULE_3020C_check();      
	 } 

	
    //----------------- Initial Speed and Direction -----------------    
    GPRMC_Funs.Speed(tmpinfo, INIT, k);	
    GPRMC_Funs.Direction(tmpinfo, INIT, k);  	
    //-------------------------------------------------------------------	
 	while (*packet!=0){ 
		if(*packet==','){
			CommaCount++;
			packet++; 
			if(iCount==0) continue; 
			switch(CommaCount){
				case 2: //时间 
				   //systemTickGPS_Set();    
				   if ( iCount < 6 ) 	  //  格式检查
				  {    
				      StatusReg_GPS_V();  
					  return false;
				  }

				    if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39)&&(tmpinfo[3]>=0x30)&&(tmpinfo[3]<=0x39)&&(tmpinfo[4]>=0x30)&&(tmpinfo[4]<=0x39)&&(tmpinfo[5]>=0x30)&&(tmpinfo[5]<=0x39)) 
				          ;
				   else
					{  StatusReg_GPS_V();   	return false;}

				       
					hour=(tmpinfo[0]-0x30)*10+(tmpinfo[1]-0x30)+8;
					min=(tmpinfo[2]-0x30)*10+(tmpinfo[3]-0x30);
					sec=(tmpinfo[4]-0x30)*10+(tmpinfo[5]-0x30);
					if(hour>23)
					{
						fDateModify=1;
						hour-=24;
						tmpinfo[0]=(hour/10)+'0';
						tmpinfo[1]=(hour%10)+'0';
					}
					//systemTickGPS_Clear();   
					//----------------------------------------------------
					GPRMC_Funs.Time(tmpinfo, hour, min, sec); 
                    //-----------------------------------------------------  
					break;
				case 3://数据有效性
					GPRMC_Funs.Status(tmpinfo);
					break;
				case 4://纬度
					if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39)&&(tmpinfo[3]>=0x30)&&(tmpinfo[3]<=0x39)&&(tmpinfo[5]>=0x30)&&(tmpinfo[5]<=0x39)&&(tmpinfo[6]>=0x30)&&(tmpinfo[6]<=0x39)&&(tmpinfo[7]>=0x30)&&(tmpinfo[7]<=0x39)&&(tmpinfo[8]>=0x30)&&(tmpinfo[8]<=0x39))
						  ;
					else
						 break; 
					GPRMC_Funs.Latitude(tmpinfo);
					break;
				case 5://纬度半球 
					GPRMC_Funs.Latitude_NS(tmpinfo);  
					break;
				case 6://经度 
				     if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39)&&(tmpinfo[3]>=0x30)&&(tmpinfo[3]<=0x39)&&(tmpinfo[4]>=0x30)&&(tmpinfo[4]<=0x39)&&(tmpinfo[6]>=0x30)&&(tmpinfo[6]<=0x39)&&(tmpinfo[7]>=0x30)&&(tmpinfo[7]<=0x39)&&(tmpinfo[8]>=0x30)&&(tmpinfo[8]<=0x39)&&(tmpinfo[9]>=0x30)&&(tmpinfo[9]<=0x39))
								;
					else
						break; 
					GPRMC_Funs.Longitude(tmpinfo);
					break;
				case 7://经度半球 
					GPRMC_Funs.Longitude_WE(tmpinfo);
					break;
				case 8://速率 
						   for ( k = 0; k < iCount; k++ )
							{
									if ( tmpinfo[k] == '.' )
									{
											break;
									}
							} 						
							if(k>=iCount)
							{ 
							    rt_kprintf("\r\n K>=icount \r\n");
								//break;
								k=0;
							}
                            GPRMC_Funs.Speed(tmpinfo, PROCESS,k);
							 
					break;
					
				case 9://方向 				    
						   
						   if ( iCount < 3 )			// 格式检查
							{
							    
								 break; 									
							}
						
							for ( k = 0; k < iCount; k++ )
							{
									if ( tmpinfo[k] == '.' )
									{
											break;
									}
							}
						 if(k>=iCount)
							  break;	
						 GPRMC_Funs.Direction(tmpinfo,PROCESS,k);
					break;		
				case 10://日期 
					 if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39)&&(tmpinfo[3]>=0x30)&&(tmpinfo[3]<=0x39)&&(tmpinfo[4]>=0x30)&&(tmpinfo[4]<=0x39)&&(tmpinfo[5]>=0x30)&&(tmpinfo[5]<=0x39)) 
				          ;
					 else
					 	break;
					  GPRMC_Funs.Date(tmpinfo,fDateModify,hour,min,sec);
					break;
				default:
					break;
			}	
			iCount=0;
		}
		else
		{
			tmpinfo[iCount++]=*packet++;
			if(iCount<15)
			tmpinfo[iCount]=0;
			if(iCount>15)
			{  
			       //  rt_kprintf("\r\n GPS  GPRMC iCounte>12  :  CommaCount=%d  tmpinfo:%15s \r\n",CommaCount,tmpinfo);  
				//return CommaCount;
				break;
			}	
		}
	} 
    //-----------------------------------------------------    
    GPS_Delta_DurPro();   //  用GPS 定时触发  
    //-----------------------------------------------------
	return CommaCount;
} 

//----------------
u8 Process_GSA(u8* packet) 
{

	   return true;
}
//------------------
u8 Process_TXT(u8* packet) 
{
 // $GNTXT,01,01,01,ANTENNA SHORT*7D
//$GNTXT,01,01,01,ANTENNA OK*2B
//$GNTXT,01,01,01,ANTENNA OPEN*3B
	
	u8 CommaCount=0,iCount=0;
	u8  tmpinfo[12];
//	float dop;	
	//float Hight1=0,Hight2=0; 
    GNTXT_Counter++;       //  文本条数计数器
    if((GNTXT_Counter==20)&&(HardWareVerion==7))  // 检查BD_Module     硬件版本全1 时
    {
         BD_MODULE_TXT_Check(); 
    }


	while (*packet!=0){
		if((*packet==',')||(*packet=='*')){  
			CommaCount++;
			packet++;
			if(iCount==0)	 continue;
			switch(CommaCount){
				case 5:
						if(strncmp((char*)tmpinfo,"ANTENNA OPEN",12)==0)//开路检测	1:天线开路
						{
							 if(OutGPS_Flag==0)
							 {
								   if((Warn_Status[3]&0x20)==0)
									{
									   rt_kprintf("\r\n	检测到	天线开路\r\n");
									    GpsStatus.Antenna_Flag=1;
								           PositionSD_Enable();   
									    Current_UDP_sd=1;	   
								   	}   
								   Warn_Status[3]|=0x20;
								   Warn_Status[3]&=~0x40;	
								  
							}  
						}
						if(strncmp((char*)tmpinfo,"ANTENNA SHORT",13)==0)//短路检测  0:天线短路
						{
						   Warn_Status[3]&=~0x20;
						   Warn_Status[3]|=0x40;
						}
						else
						if(strncmp((char*)tmpinfo,"ANTENNA OK",10)==0)	
						{
						  GpsStatus.Antenna_Flag=0;
						  if(Warn_Status[3]&0x20)
							{
							    rt_kprintf("\r\n	检测到	天线恢复正常");
						           PositionSD_Enable();    
							    Current_UDP_sd=1;	   
						  	}	
						   Warn_Status[3]&=~0x20;
						   Warn_Status[3]&=~0x40;	  						   
						}
				     break;
				default:
					break;
			}
			iCount=0;
		}else{
			tmpinfo[iCount++]=*packet++;
			tmpinfo[iCount]=0;
			if(iCount>12)
				return CommaCount;
		}
	}
	return CommaCount;
   
}
//---------------------
u8 Process_GGA(u8* packet)  
{	//检查数据完整性,执行数据转换
	u8 CommaCount=0,iCount=0;
	u8  tmpinfo[12];
	float dop;	
	float Hight1=0,Hight2=0; 
	while (*packet!=0){
		if(*packet==','){
			CommaCount++;
			packet++;
			if(iCount==0)	 
			  { 
			     if(CommaCount==8)
			   	     Satelite_num=0;   
			   	continue;
			   }
			switch(CommaCount){
				case 8:
						        //搜到星的个数$GPGGA,045333,3909.1849,N,11712.3104,E,1,03,4.3,20.9,M,-5.4,M,,*66
						        Satelite_num = ( tmpinfo[0] - 0x30 ) * 10 + ( tmpinfo[1] - 0x30 );
							//	rt_kprintf("\r\n 卫星颗数:%d \r\n",Satelite_num);  
                                                                   //--------- 北斗 卫星颗数  --------------
					                 BD_EXT.FJ_SignalValue=BD_EXT.FJ_SignalValue&0xf0;  // 先清除 GSM 信号位
							   BD_EXT.FJ_SignalValue|=	Satelite_num;  // 低4 位

						        break;             

				case 9:	dop=atof((char *)tmpinfo); 
				        
						HDOP_value=dop;		 //  Hdop 数值	  
				break;
				case 10:// MSL altitude
					Hight1=atof((const char*)tmpinfo);
				break;
				case 12:// Geoid Separation
					Hight2=atof((const char*)tmpinfo);
					GPS_Hight=(u16)(Hight1+Hight2);  	 				
					//printf("\r\n 当前海拔高度为:%f,  %f ,%d m\r\n",Hight1,Hight2,GPS_Hight); 
					break;
				default:
					break;
			}
			iCount=0;
		}else{
			tmpinfo[iCount++]=*packet++;
			tmpinfo[iCount]=0;
			if(iCount>12)
				return CommaCount;
		}
	}
	return CommaCount;


}
//------------------------------------------------------------------
void  GPS_ANTENNA_status(void)     //  天线开短路状态检测 
{
    // 2013-4-20    更改PCB   用PD4 : GPS 天线开路      PB6 : GPS  天线短路
   	if(GPIO_ReadOutputDataBit(GPS_PWR_PORT, GPS_PWR_PIN )) // 在GPS 有电时有效   
		{
			if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4))//开路检测	1:天线开路
			{
				 if(OutGPS_Flag==0)
				 {
					   if((Warn_Status[3]&0x20)==0)
					         rt_kprintf("\r\n	检测到	天线开路"); 
					   Warn_Status[3]|=0x20;
					   Warn_Status[3]&=~0x40; 	 
					   GpsStatus.Antenna_Flag=1;
					   Gps_Exception.GPS_circuit_short_couter=0;
			 	}  
			}
			else if(!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6))//短路检测  0:天线短路 
			{
			         if(( Warn_Status[3]&0x40)==0)   
				     {
                        Gps_Exception.GPS_short_keepTimer++;
						if( Gps_Exception.GPS_short_keepTimer>200)     
					    {
					       Gps_Exception.GPS_short_keepTimer=0;  // clear 
					       rt_kprintf("\r\n	检测到	天线短路");     
						   rt_kprintf("\r\n	发现短路，立即断开GPS电源");      
						   GPIO_ResetBits( GPS_PWR_PORT, GPS_PWR_PIN ); 


						      //------------------------------------------ 
							  Gps_Exception.GPS_circuit_short_couter++;
							  if(Gps_Exception.GPS_circuit_short_couter>=4)  
							   {
									Gps_Exception.GPS_short_checkFlag=2;
									Gps_Exception.GPS_short_timer=0; // clear  
									rt_kprintf("\r\n   短路检测大于3次 ，一直断开GPS 电源\r\n");   
									
									//	断开 GPS 电源后，得启动 本地定时 ，否则人家说丢包.NND
									/*
							  
									   */ 
							   }	
							  else
							   {
									  Gps_Exception.GPS_short_checkFlag=1; 
							   } 
			                   //-----------------------------------------------------
                             
							   // set  flag 	
							   Warn_Status[3]&=~0x20;
							   Warn_Status[3]|=0x40;	 
							   //------------------------------------------
					     }
			         } 		
				   

			}
			else
			{
				  if(Warn_Status[3]&0x20)
				  	      rt_kprintf("\r\n	检测到	天线恢复正常");   
	              Warn_Status[3]&=~0x20;
				  Warn_Status[3]&=~0x40;   
				  GpsStatus.Antenna_Flag=0;
				  Gps_Exception.GPS_circuit_short_couter=0;
			} 
			
		}
}

void  GPS_short_judge_timer(void)
{
       if(Gps_Exception.GPS_short_checkFlag==1)
       {
        Gps_Exception.GPS_short_timer++; 
		if(Gps_Exception.GPS_short_timer>100)
		{   //   关电 后开启
		     Gps_Exception.GPS_short_timer=0;  
		     gps_onoff(1);
		     rt_kprintf("\r\n	 再次开启GPS模块\r\n"); 
			 //---------------期待模块正常-----------
               Warn_Status[3]&=~0x20;
			   Warn_Status[3]&=~0x40; 
			 //----------------
			 Gps_Exception.GPS_short_checkFlag=0; 
		}	 
 
       }



}

//--------------------------------------------------------------------------------
void  GPS_Rx_Process(u8 * Gps_str ,u16  gps_strLen) 
{    
     u8  Gps_instr[160];  
     u8  GPRMC_Enable=0;	 
	 
	                     memset(Gps_instr,0,sizeof(Gps_instr));     
	                     memcpy(Gps_instr,Gps_str,gps_strLen); 

				if(GpsStatus.Raw_Output==1)		  
		                   rt_kprintf((const char*)Gps_instr);        // rt_kprintf((const char*)Gps_str);         

			   //----------------  Mode  Judge    ---------------------			 
				if(strncmp((char*)Gps_instr,"$GNRMC,",7)==0)
				{	
				     GpsStatus.Position_Moule_Status=3;
				     GPRMC_Enable=1;
				     Car_Status[1]&=~0x0C; // clear bit3 bit2 
				     Car_Status[1]|=0x0C; // BD+GPS  mode	1100	 	 
				}
				if(strncmp((char*)Gps_instr,"$BDRMC,",7)==0)
				{	
				       GpsStatus.Position_Moule_Status=1;
				       GPRMC_Enable=1;
					Car_Status[1]&=~0x0C; // clear bit3 bit2 
					Car_Status[1]|=0x08; // BD mode	1000	   
			        }
				if(strncmp((char*)Gps_instr,"$GPRMC,",7)==0)    
				{	
				       GpsStatus.Position_Moule_Status=2;	
				       GPRMC_Enable=1;
				       Car_Status[1]&=~0x0C; // clear bit3 bit2      1100 
					   Car_Status[1]|=0x04; // Gps mode   0100 
		        }  	 	  
				
                //-------------------------------------------------- 
                            //----------- Pick up useful  --------------------------
			      if(GPRMC_Enable==1) 
			      	{
			      	     if(GpsStatus.Raw_Output==2)	
                                        rt_kprintf(" gps_thread tx :                   %s",Gps_instr);      
					Process_RMC(Gps_instr); 	
					Gps_Exception.current_datacou+=gps_strLen;
                                  return;
			      	}	
				if((strncmp((char*)Gps_instr,"$GNTXT,",7)==0)||(strncmp((char*)Gps_instr,"$GPTXT,",7)==0)||(strncmp((char*)Gps_instr,"$BDTXT,",7)==0)) 
				{
					  //rt_kprintf("%s",GPSRx);  
					  Process_TXT(Gps_instr);  
					   return;	 
				}
			     if((strncmp((char*)Gps_instr,"$GPGGA,",7)==0)||(strncmp((char*)Gps_instr,"$GNGGA,",7)==0)||(strncmp((char*)Gps_instr,"$BDGGA,",7)==0))  
			     {   
					   //GNSS_Trans();
					   Process_GGA(Gps_instr);  
					    return;	
		         }
                if((strncmp((char*)Gps_instr,"$GPGSA,",7)==0)||(strncmp((char*)Gps_instr,"$BDGSA,",7)==0)||(strncmp((char*)Gps_instr,"$GNGSA,",7)==0))
		          {   
					//rt_kprintf("%s",GPSRx);
					//GNSS_Trans();
					Process_GSA(Gps_instr);
				      return;	
			      }
}
















//================================================

//*************************************************
//  计算CRC校验和
// unsigned char* src –  待校验数据
// int startpoint –  校验数据偏移
// int len –  校验数据长度
//************************************************
unsigned short  CalcCRC16( unsigned char*  src, int startpoint, int len )
{
	unsigned short	res		= 0;
	unsigned char	crc_h	= 0;
	unsigned char	crc_l	= 0;

	unsigned char	tmp;
	int				i;

	for( i = startpoint; i < ( len + startpoint ); i++ )
	{
		tmp		= crc_h ^ src[i];
		crc_h	= crc_l ^ CRC16TabH[tmp];
		crc_l	= CRC16TabL[tmp];
		//if(len>1000) rt_kprintf("\r\nsrc=%02x tmp=%02x crc_h=%02x crc_l=%02x",src[i],tmp,crc_h,crc_l);
	}
	//res = ( (unsigned short)crc_h ) << 8 + ( unsigned short)crc_l;
	res = ( crc_h << 8 ) | crc_l;

	return res;
}



void GpsIo_Init(void)
{
GPIO_InitTypeDef	GPIO_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART5, ENABLE );

	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = GPS_PWR_PIN;
	GPIO_Init( GPS_PWR_PORT, &GPIO_InitStructure );
	GPIO_ResetBits( GPS_PWR_PORT, GPS_PWR_PIN );

/*uart5 管脚设置*/

	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_12;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init( GPIOD, &GPIO_InitStructure );

	GPIO_PinAFConfig( GPIOC, GPIO_PinSource12, GPIO_AF_UART5 );
	GPIO_PinAFConfig( GPIOD, GPIO_PinSource2, GPIO_AF_UART5 );

/*NVIC 设置*/
	NVIC_InitStructure.NVIC_IRQChannel						= UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	gps_baud( 9600 );
	USART_Cmd( UART5, ENABLE );
	USART_ITConfig( UART5, USART_IT_RXNE, ENABLE );

	GPIO_SetBits( GPIOD, GPIO_Pin_10 );

}
	




/*
   gps接收中断处理，收到\n认为收到一包
   收到一包调用处理函数
 */
	static uint8_t	last_ch = 0;
void UART5_IRQHandler( void )
{
	uint8_t			ch;
	rt_interrupt_enter( );
	if( USART_GetITStatus( UART5, USART_IT_RXNE ) != RESET )
	{
		ch = USART_ReceiveData( UART5 );
		if( ( ch == 0x0a ) && ( last_ch == 0x0d ) ) /*遇到0d 0a 表明结束*/
		{
			uart5_rxbuf.body[uart5_rxbuf.wr++] = ch;
			if( uart5_rxbuf.wr < 124 )
			{
			   //--------- fliter  useful------
			   if((strncmp((char*)uart5_rxbuf.body+3,"RMC,",4)==0)||(strncmp((char*)uart5_rxbuf.body+3,"TXT,",4)==0)||  \
			   	(strncmp((char*)uart5_rxbuf.body+3,"GGA,",4)==0))//||(strncmp((char*)uart5_rxbuf.body+3,"GSA,",4)==0))
			   	{

			      // rt_mq_send( &mq_gps, (void*)&uart5_rxbuf, uart5_rxbuf.wr + 2 ); 
			      //--- replaced---------------------------------
			      if(NMEA_WR.wr_num>=32)
				  	NMEA_WR.wr_num=0;
			      memcpy( nmea_segBuf+NMEA_WR.wr_num*NMEA_SEGlength, uart5_rxbuf.body, uart5_rxbuf.wr);				  
			      NMEA_WR.len_sigema[NMEA_WR.wr_num]=uart5_rxbuf.wr; // length
			      NMEA_WR.wr_num++;			  
				  //----------------------------------------------
			   	}
			}
			uart5_rxbuf.wr = 0;
		}else
		{ 
		      // 1. get  head char
		       if(ch=='$')                                 
			   	      uart5_rxbuf.wr = 0;
		      // 2.  judge  head char	   
		       if(uart5_rxbuf.body[0]!='$')  // add later 
			   	    uart5_rxbuf.wr = 0;
		      // 3.  rx data  	   
			uart5_rxbuf.body[uart5_rxbuf.wr++] = ch;     
			if( uart5_rxbuf.wr == UART5_RX_SIZE )
			{
				uart5_rxbuf.wr = 0;
			}
			uart5_rxbuf.body[uart5_rxbuf.wr] = 0;
		}
		last_ch = ch;
		USART_ClearITPendingBit( UART5, USART_IT_RXNE ); 
	}
	rt_interrupt_leave( );   
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void gps_baud( int baud )
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate				= baud;
	USART_InitStructure.USART_WordLength			= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits				= USART_StopBits_1;
	USART_InitStructure.USART_Parity				= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl	= USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode					= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init( UART5, &USART_InitStructure );
}

//FINSH_FUNCTION_EXPORT( gps_baud, config gsp_baud );

/*初始化*/
static rt_err_t dev_gps_init( rt_device_t dev )
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART5, ENABLE );

	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = GPS_PWR_PIN;
	GPIO_Init( GPS_PWR_PORT, &GPIO_InitStructure );
	GPIO_ResetBits( GPS_PWR_PORT, GPS_PWR_PIN );


	if(Module_3017A!=GPS_MODULE_TYPE)
	{
		//#ifdef HC_595_CONTROL      
		       // 2013-4-20    更改PCB   用PD4 : GPS 天线开路      PB6 : GPS  天线短路
		       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
		       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
		       GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; 	 
		      // 		IN
			//------------------- PD4 -----------------------------
			GPIO_InitStructure.GPIO_Pin	 = GPIO_Pin_4;	  //GPS 天线开路
			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; 
			GPIO_Init(GPIOD, &GPIO_InitStructure);
		      //------------------- PB6 -----------------------------
			GPIO_InitStructure.GPIO_Pin	 = GPIO_Pin_6;	  //GPS 天线短路 
			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; 
			GPIO_Init(GPIOB, &GPIO_InitStructure);   
		//#endif
	}
 
	

/*uart5 管脚设置*/

	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_12;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init( GPIOD, &GPIO_InitStructure );

	GPIO_PinAFConfig( GPIOC, GPIO_PinSource12, GPIO_AF_UART5 );
	GPIO_PinAFConfig( GPIOD, GPIO_PinSource2, GPIO_AF_UART5 );

/*NVIC 设置*/
	NVIC_InitStructure.NVIC_IRQChannel						= UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	gps_baud( 9600 );
	USART_Cmd( UART5, ENABLE );
	USART_ITConfig( UART5, USART_IT_RXNE, ENABLE );

	GPIO_ResetBits( GPIOD, GPIO_Pin_10 );

	return RT_EOK;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
static rt_err_t dev_gps_open( rt_device_t dev, rt_uint16_t oflag )
{
	GPIO_SetBits( GPS_PWR_PORT, GPS_PWR_PIN );
	return RT_EOK;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
static rt_err_t dev_gps_close( rt_device_t dev )
{
	GPIO_ResetBits( GPS_PWR_PORT, GPS_PWR_PIN );
	return RT_EOK;
}

/***********************************************************
* Function:gps_read
* Description:数据模式下读取数据
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
static rt_size_t dev_gps_read( rt_device_t dev, rt_off_t pos, void* buff, rt_size_t count )
{
	return RT_EOK; 
}

/***********************************************************
* Function:		gps_write
* Description:	数据模式下发送数据，要对数据进行封装
* Input:		const void* buff	要发送的原始数据
       rt_size_t count	要发送数据的长度
       rt_off_t pos		使用的socket编号
* Output:
* Return:
* Others:
***********************************************************/

static rt_size_t dev_gps_write( rt_device_t dev, rt_off_t pos, const void* buff, rt_size_t count )
{
	rt_size_t	len = count;
	uint8_t		*p	= (uint8_t*)buff;

	while( len )
	{
		USART_SendData( UART5, *p++ );
		while( USART_GetFlagStatus( UART5, USART_FLAG_TC ) == RESET )
		{
		}
		len--;
	}
	return RT_EOK;
}

/***********************************************************
* Function:		gps_control
* Description:	控制模块
* Input:		rt_uint8_t cmd	命令类型
    void *arg       参数,依据cmd的不同，传递的数据格式不同
* Output:
* Return:
* Others:
***********************************************************/
static rt_err_t dev_gps_control( rt_device_t dev, rt_uint8_t cmd, void *arg )
{
	int i = *(int*)arg;
	switch( cmd )
	{
		case CTL_GPS_OUTMODE:
			break;
		case CTL_GPS_BAUD:
			gps_baud( i );
	}
	return RT_EOK;
}

ALIGN( RT_ALIGN_SIZE )
static char thread_gps_stack[4096]; 
struct rt_thread thread_gps;


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
static void rt_thread_entry_gps( void* parameter )
{
	rt_err_t	res;

      // 1.  init
       GPS_Abnormal_init();

	//2.  main while
	while( 1 )
	{
		//res = rt_mq_recv( &mq_gps, (void*)&gps_rx, 124, 2 ); //等待100ms,实际上就是变长的延时,最长100ms
		//if(res== RT_EOK )                                                     //收到一包数据
         if(NMEA_WR.wr_num!=nmea_rd)
		{
             if(nmea_rd>=32)
				  	nmea_rd=0;  
			 memcpy(gps_rx.body,nmea_segBuf+nmea_rd*NMEA_SEGlength,NMEA_WR.len_sigema[nmea_rd]);
			 gps_rx.wr=NMEA_WR.len_sigema[nmea_rd];  
			 nmea_rd++; 
		     //---------------------------------
			if( flag_bd_upgrade_uart == 0 )
			{
				GPS_Rx_Process( gps_rx.body, gps_rx.wr );  
				GPS_Abnormal_process();//  GPS 模块异常检测 

			}else
			{
				if( gps_rx.body[0] == 0x40 )
				{
					rt_device_write( &dev_vuart, 0, gps_rx.body, gps_rx.wr);
				}
			}
		}
		rt_thread_delay(30);      
		//---------------
		gps_thread_runCounter=0;
	}
}

/*gps设备初始化*/
void gps_init( void )
{
	//rt_sem_init( &sem_gps, "sem_gps", 0, 0 );
	//rt_mq_init( &mq_gps, "mq_gps", &gps_rawinfo[0], 128 - sizeof( void* ), GPS_RAWINFO_SIZE, RT_IPC_FLAG_FIFO );
	rt_thread_init( &thread_gps,
	                "gps",
	                rt_thread_entry_gps,
	                RT_NULL,
	                &thread_gps_stack[0],
	                sizeof( thread_gps_stack ), Prio_GPS, 6 ); 
	rt_thread_startup( &thread_gps );

	dev_gps.type	= RT_Device_Class_Char;
	dev_gps.init	= dev_gps_init;
	dev_gps.open	= dev_gps_open;
	dev_gps.close	= dev_gps_close;
	dev_gps.read	= dev_gps_read;
	dev_gps.write	= dev_gps_write;
	dev_gps.control = dev_gps_control; 

	rt_device_register( &dev_gps, "gps", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE );
	rt_device_init( &dev_gps );
}

/*gps开关*/
rt_err_t gps_onoff( uint8_t openflag )
{
	if( openflag == 0 )
	{
		GPIO_ResetBits( GPIOD, GPIO_Pin_10 );
	} else
	{
		GPIO_SetBits( GPIOD, GPIO_Pin_10 );
	}
	return 0;
}

FINSH_FUNCTION_EXPORT( gps_onoff, gps_onoff([1 | 0] ) );

#if 0
/*线程退出的cleanup函数*/
static void cleanup( struct rt_thread *tid )
{
	if( ptr_mem_packet != RT_NULL )
	{
		rt_free( ptr_mem_packet );
	}
}
#endif

/*升级时的调试输出,当前已无法打印输出,串口被占用*/
static void msg_uart_usb( int res )
{
	//rt_kprintf("bd>%d\r\n",res);
}

/*更新北斗线程，接收调试串口来的数据，透传到gps串口中*/

void thread_gps_upgrade_uart( void* parameter )
{
#define BD_SYNC_40	0
#define BD_SYNC_0D	1
#define BD_SYNC_0A	2

/*定义一个函数指针，用作结果处理	*/

	void			( *msg )( void *p );

	rt_uint8_t		buf[256];
	rt_uint8_t		info[32];
	rt_uint8_t		*p;
	rt_uint16_t		count = 0;
	rt_uint16_t		i;
	rt_size_t		len;
	rt_uint32_t		baud				= 9600;
	rt_uint16_t		packetnum			= 0;
	rt_uint8_t		bd_packet_status	= BD_SYNC_40;   /*北斗升级报文接收状态*/
	rt_uint8_t		last_char			= 0x0;

	rt_tick_t		last_sendtick = 0;                  /*北斗更新时记录收到应答的时刻*/

	msg = parameter;

	ptr_mem_packet = rt_malloc( 1200 );
	if( ptr_mem_packet == RT_NULL )
	{
		msg( "E内存不足" );
		return;
	}
	flag_bd_upgrade_uart = 1;

	dev_vuart.flag &= ~RT_DEVICE_FLAG_STREAM;
	rt_device_control( &dev_vuart, 0x03, &baud );
	p = ptr_mem_packet;

	while( 1 )
	{
		if( ( last_sendtick > 0 ) && ( rt_tick_get( ) - last_sendtick > RT_TICK_PER_SECOND * 12 ) )
		{
			/*升级程序发送数据，收到应答，再次发送数据。超时10s*/
			msg( "E超时错误" );
			goto end_upgrade_uart_memfree;
		}
		while( ( len = rt_device_read( &dev_vuart, 0, buf, 256 ) ) > 0 )
		{
			for( i = 0; i < len; i++ )
			{
				switch( bd_packet_status )
				{
					case BD_SYNC_40:
						if( buf[i] == 0x40 )
						{
							*p++				= 0x40;
							bd_packet_status	= BD_SYNC_0A;
							count				= 1;
						}
						break;
					case BD_SYNC_0A:
						if( ( buf[i] == 0x0a ) && ( last_char == 0x0d ) )
						{
							*p = 0x0a;
							count++;
							dev_gps_write( &dev_gps, 0, ptr_mem_packet, count );
							packetnum++;                                            /*显示传递的包数*/
							sprintf( info, "I发送第%d包", packetnum );
							msg( info );
							last_sendtick = rt_tick_get( );
							if( memcmp( ptr_mem_packet, "\x40\x41\xc0", 3 ) == 0 )  /*修改波特率*/
							{
								baud = ( *( ptr_mem_packet + 4 ) << 24 ) | ( *( ptr_mem_packet + 5 ) << 16 ) | ( *( ptr_mem_packet + 6 ) << 8 ) | *( ptr_mem_packet + 7 );
								gps_baud( baud );
								uart1_baud( baud );
							}
							if( memcmp( ptr_mem_packet, "\x40\x34\xc0", 3 ) == 0 )  /*模块软件复位*/
							{
								msg( "E更新完成" );                                 /*通知lcd显示完成*/
								goto end_upgrade_uart_memfree;
							}
							p					= ptr_mem_packet;
							bd_packet_status	= BD_SYNC_40;
						}else
						{
							*p++ = buf[i];
							count++;
						}
						break;
				}
				last_char = buf[i];
			}
		}
		rt_thread_delay( RT_TICK_PER_SECOND / 50 ); 
	}

end_upgrade_uart_memfree:
	rt_free( ptr_mem_packet );
	ptr_mem_packet = RT_NULL;
//end_upgrade_uart:
	baud = 115200;
	uart1_baud( baud );
	flag_bd_upgrade_uart = 0;  
}

/*更新北斗线程，接收调试串口来的数据，透传到gps串口中*/
void thread_gps_upgrade_udisk( void* parameter )
{
#define READ_PACKET_SIZE 1012

	void		( *msg )( void *p );
	int			fd = -1, size;
	u32   count = 0;
	rt_uint8_t	*pdata;             /*数据*/

	rt_uint8_t	buf[32];
	rt_uint8_t	ch_h, ch_l;
	rt_err_t	res;
	LENGTH_BUF	uart_buf;

	rt_uint32_t file_datalen;       /*升级文件长度*/
	rt_uint8_t	file_matchcode[2];  /*文件匹配码*/

	rt_uint16_t packet_num;

	rt_uint16_t crc=0;

	msg = parameter;

	ptr_mem_packet = rt_malloc( READ_PACKET_SIZE+20 );
	if( ptr_mem_packet == RT_NULL )
	{
		msg( "E内存不足" );
		return;
	}
/*查找U盘*/
	while( 1 )
	{
		if( rt_device_find( "udisk" ) == RT_NULL ) /*没有找到*/
		{
			count++;
			if( count <= 10 )
			{
				msg( "I等待U盘插入" );
				BD_upgrad_contr=1;
			}else
			{
				msg( "EU盘不存在" ); /*指示U盘不存在*/
				BD_upgrad_contr=1;
				goto end_upgrade_usb_0;
			}
			rt_thread_delay( RT_TICK_PER_SECOND );
		}else
		{
			msg( "I查找升级文件" );
			break;
		}
		WatchDog_Feed();
	}

/*查找指定文件BEIDOU.IMG*/
	fd = open( "/udisk/BEIDOU.IMG", O_RDONLY, 0 );
	if( fd >= 0 )
	{
		msg( "I分析文件" );
	}else
	{
		msg( "E升级文件不存在" );
		goto end_upgrade_usb_0;
	}

	size	= read( fd, ptr_mem_packet, 16 );
	pdata	= ptr_mem_packet;
	if( ( *pdata != 0x54 ) || ( *( pdata + 1 ) != 0x44 ) )  
	{
		msg( "E文件头错误" );
		goto end_upgrade_usb_1;
	}

	ch_h	= ( *( pdata + 9 ) & 0xf0 ) >> 4;
	ch_l	= ( *( pdata + 9 ) & 0xf );
	sprintf( buf, "I版本:%d.%d.%d", ch_h, ch_l, *( pdata + 10 ) );
	msg( buf );
/*升级数据长度*/
	file_datalen =0;
	file_datalen	= ( *( pdata + 11 ) ) << 24;
	file_datalen	|= ( *( pdata + 12 ) ) << 16;
	file_datalen	|= ( *( pdata + 13 ) ) << 8;
	file_datalen	|= *( pdata + 14 );
	rt_kprintf("file_datalen=%x",file_datalen);
/*文件匹配码在尾部*/
	count	= 0;
	ch_h	= 0;

	do{
		res		= read( fd, ptr_mem_packet,READ_PACKET_SIZE );
		if(res)	count = res;
		WatchDog_Feed(); 
	}while(res>0);
	//rt_kprintf("res=%02x\r\n",res);
	//if(res==0) res=READ_PACKET_SIZE;
	
	if( ( ptr_mem_packet[count - 1] != 0x54 ) || ( ptr_mem_packet[count - 2] != 0x44 ) )
	{
		msg( "E文件尾错误" );
		goto end_upgrade_usb_1;
	}
	file_matchcode[0]	= ptr_mem_packet[count - 6];
	file_matchcode[1]	= ptr_mem_packet[count - 5];
	rt_kprintf("file datalen=%x matchcode=%02x%02x",file_datalen,file_matchcode[0],file_matchcode[1]);
	close( fd );
	


	msg( "I配置端口" );

	fd = open( "/udisk/BEIDOU.IMG", O_RDONLY, 0 );
	if( fd < 0 )
	{
		msg( "E升级文件不存在" );
		goto end_upgrade_usb_0;
	}

/*开始升级*/
/*停止gps线程*/
	rt_thread_suspend( &thread_gps );

/*进入升级状态*/
	memcpy( buf, "\x40\x30\xC0\x00\x03\x00\x01\x34\x21\x0D\x0A", 11 );
	dev_gps_write( &dev_gps, 0, buf, 11 );
	rt_thread_delay( RT_TICK_PER_SECOND );
/*版本查询*/
	count = 0;
	dev_gps_write( &dev_gps, 0, "\x40\x10\xC0\x00\x10\x00\x01\xC2\x84\x0D\x0A", 11 );
	while( 1 )
	{
	    rt_thread_delay(RT_TICK_PER_SECOND/10);
		res = rt_mq_recv( &mq_gps, (void*)&uart_buf, 124, 50 );
		if( res == RT_EOK )                                             //收到一包数据
		{
		       count=0;  // clear
			rt_kprintf("\r\n版本查询\r\n");
			for( ch_h = 0; ch_h < uart_buf.wr; ch_h++ )
			{
				rt_kprintf( "%02x ", uart_buf.body[ch_h] );
			}
			rt_kprintf( "\r\n" );
			if( ( uart_buf.wr == 15 ) && ( uart_buf.body[4] == 0x02 ) ) /*进入升级状态*/
			{
				ch_h	= ( uart_buf.body[7] & 0xf0 ) >> 4;
				ch_l	= ( uart_buf.body[7] & 0xf );
				sprintf( buf, "I版本:%d.%d.%d", ch_h, ch_l, uart_buf.body[8] );
				msg( buf );
				break;
			}
		}else /*超时*/
		{
		    count++;
		    if(count>100)
		    {
		       count=0;
			msg( "E进入升级错误" );
			goto end_upgrade_usb_2;
		     }	
		}
	}

/*改波特率*/
	dev_gps_write( &dev_gps, 0, "\x40\x41\xC0\x00\x00\x00\xE1\x00\x00\x04\x07\xFC\x0D\x0A", 14 );
	gps_baud( 57600 );
	rt_thread_delay( RT_TICK_PER_SECOND );
/*连接查询*/
	dev_gps_write( &dev_gps, 0, "\x40\x15\xC0\x00\x01\x00\x01\x92\xD4\x0D\x0A", 11 );

    //---------------------------------------------------------------------------------------
	while( 1 )
	{    
	      rt_thread_delay(RT_TICK_PER_SECOND/10);
		res = rt_mq_recv( &mq_gps, (void*)&uart_buf, 124,  5 );
		if( res == RT_EOK )                         //收到一包数据
		{
		       count=0;
			if( ( uart_buf.wr == 11 ) && ( uart_buf.body[4] == 0x15 ) )
			{
				break;
			}
		}else /*超时*/
		{
		       count++;
		    if(count>100)
		    {
		       count=0;
			msg( "E修改波特率错误" );
			goto end_upgrade_usb_2;
		    }	
		} 
		WatchDog_Feed(); 
	}

/*开始烧写,烧写指令*/
	memcpy( buf, "\x40\xF2\xC0\x00\x03\x00\x00\x00\x00\x00\x00\x00\x00\x08\x00\x00\x0D\x0A", 18 );
	buf[6]	= file_datalen >> 24;
	buf[7]	= file_datalen >> 16;
	buf[8]	= file_datalen >> 8;
	buf[9]	= file_datalen & 0xff;
	buf[10] = file_matchcode[0];
	buf[11] = file_matchcode[1];

	crc		= CalcCRC16( buf, 1, 13 );
	buf[14] = ( crc & 0xff00 ) >> 8;
	buf[15] = crc & 0xff;

	dev_gps_write( &dev_gps, 0, buf, 18 );

	ch_l = 1;
	while( ch_l )
	{
	      rt_thread_delay(RT_TICK_PER_SECOND/20);
		res = rt_mq_recv( &mq_gps, (void*)&uart_buf, 124, RT_TICK_PER_SECOND * 10 );
		if( res == RT_EOK )                         //收到一包数据
		{
			for( ch_h = 0; ch_h < uart_buf.wr; ch_h++ )
			{
				rt_kprintf( "%02x ", uart_buf.body[ch_h] );
			}
			rt_kprintf( "\r\n" );

			if( ( uart_buf.wr == 11 ) && ( uart_buf.body[4] == 0xf2 ) )
			{
				ch_l = 0;
			}
		}else /*超时*/
		{
			msg( "E开始升级错误" );
			goto end_upgrade_usb_2;
		}
		WatchDog_Feed();
	}

/*发送数据包*/
	count		= 0;                    /*计算发送字节数*/
	packet_num	= 0;
	read( fd, buf, 15 );                /*跳过文件头*/
	rt_kprintf("fd=%02x\r\n",fd);
	while( 1 )
	{
		rt_thread_delay(RT_TICK_PER_SECOND/2);
		memcpy( ptr_mem_packet, "\x40\xf2\x00\x00\x03\x01", 6 );
		res = read( fd, ptr_mem_packet + 6,READ_PACKET_SIZE );
		if(res<0)
		{
			msg("E读取文件出错");
			goto end_upgrade_usb_2;
		}

		count += res;
		if( res == READ_PACKET_SIZE )               /*判断是否为最后一包*/
		{
			if( count == file_datalen ) /*长度正为1012整数倍*/
			{
				ptr_mem_packet[2]	= 0x80 | ( packet_num >> 8 );
				ptr_mem_packet[3]	= packet_num & 0xff;
				res=res-6;
			}else
			{
				if( count == READ_PACKET_SIZE )                /*第一包*/
				{
					ptr_mem_packet[2]	= 0x40|(packet_num >> 8);
					ptr_mem_packet[3]	= packet_num & 0xff;
				}
				else
				{
					ptr_mem_packet[2]	= packet_num >> 8;
					ptr_mem_packet[3]	= packet_num & 0xff;
				}	
			}
		}else
		{
			ptr_mem_packet[2]	= 0x80 | ( packet_num >> 8 );
			ptr_mem_packet[3]	= packet_num & 0xff;
			/*去掉文件尾部的数据*/
			res=res-6;
		}
		packet_num++;
		ptr_mem_packet[res + 6]		= (res+2) >> 8;
		ptr_mem_packet[res + 7]		= (res+2) & 0xff;
		crc							= CalcCRC16( ptr_mem_packet, 1, res + 7 );
		ptr_mem_packet[res + 8]		= ( crc & 0xff00 ) >> 8;
		ptr_mem_packet[res + 9]		= crc & 0xff;
		ptr_mem_packet[res + 10]	= 0x0d;
		ptr_mem_packet[res + 11]	= 0x0a;

		rt_kprintf("\r\nres=%d,%02x%02x,%04x\r\n",res,ptr_mem_packet[2],ptr_mem_packet[3],crc);
		for(ch_h=0;ch_h<16;ch_h++) rt_kprintf("%02x ",ptr_mem_packet[ch_h]);
		rt_kprintf( "tx tick=%x\r\n",rt_tick_get());
		dev_gps_write( &dev_gps, 0, ptr_mem_packet, res + 12 );

		ch_l = 1;
		while( ch_l )
		{
			res = rt_mq_recv( &mq_gps, (void*)&uart_buf, 124, RT_TICK_PER_SECOND * 12 );
			if( res == RT_EOK ) //收到一包数据
			{
				for( ch_h = 0; ch_h < uart_buf.wr; ch_h++ )
				{
					rt_kprintf( "%02x ", uart_buf.body[ch_h] );
				}
				rt_kprintf( "rx tick=%x\r\n",rt_tick_get());
				if( uart_buf.wr == 11 )
				{
					if( uart_buf.body[4] == 0x02 )
					{
						msg( "E升级完成" );
						dev_gps_write( &dev_gps, 0, "\x40\x34\xC0\x00\x34\x00\x01\x84\x6B\x0D\x0A", 11 );
						goto end_upgrade_usb_2;
					}
					sprintf( buf, "I发送第%d包", packet_num );
					msg( buf );
					ch_l = 0;
				}
				
			}else /*超时*/
			{
				msg( "E升级错误" );
				goto end_upgrade_usb_2;
			}
		}
	}

end_upgrade_usb_2:
	rt_thread_resume( &thread_gps );
end_upgrade_usb_1:
	if( fd >= 0 )
	{
		close( fd );
	}
end_upgrade_usb_0:
	rt_free( ptr_mem_packet );
	ptr_mem_packet = RT_NULL;
}

#if 0
/*gps升级*/
rt_err_t gps_upgrade( char *src )
{
	rt_thread_t tid;
	int			buad = 9600;
	rt_kprintf( "\nNow upgrade from %s\n", src );
	if( strncmp( src, "COM", 3 ) == 0 ) /*串口升级*/
	{
		tid = rt_thread_create( "upgrade", thread_gps_upgrade_uart, (void*)msg_uart_usb, 2048, 5, 5 );
		if( tid != RT_NULL )
		{
			rt_thread_startup( tid );
		}else
		{
			rt_kprintf( "\n Upgrade from uart fail\n" );    
		}
	}else
	{
		tid = rt_thread_create( "upgrade", thread_gps_upgrade_udisk, (void*)msg_uart_usb, 512, 7, 5 );
		if( tid != RT_NULL )
		{
			rt_thread_startup( tid );
		}else
		{
			rt_kprintf( "\n Upgrade from uart fail\n" );
		}
	}
}

//FINSH_FUNCTION_EXPORT( gps_upgrade, upgrade bd_gps );  
#endif

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
rt_size_t gps_write( uint8_t *p, uint8_t len )
{
	return dev_gps_write( &dev_gps, 0, p, len );
}

//FINSH_FUNCTION_EXPORT( gps_write, write to gps );



void  gps_mode(u8 *str) 
{
   if (strlen((const char*)str)==0)
	{
		     rt_kprintf("\r\n    定位模式:    ");  
		      switch(GpsStatus.Position_Moule_Status)
		      	{  
		      	         case 1:           rt_kprintf(" BD\r\n");  
						 	break;
				  case 2:           rt_kprintf(" GPS\r\n");  
				  	              break;
				  case 3:           rt_kprintf(" BD+GPS\r\n");  
				  	              break;
		      	}
	}
	else 
	{
		if(str[0]=='1')
			{
			  dev_gps_write( &dev_gps, 0, BD_MODE, strlen((const char*)BD_MODE));
			   GpsStatus.Position_Moule_Status=1;
			//GPS_PutStr(BD_MODE);
			 rt_kprintf ("\r\n    BD MODE\r\n"); 
			}
		else if(str[0]=='2')
			{
			 dev_gps_write( &dev_gps, 0, GPS_MODE, strlen((const char*)BD_MODE));
			  GpsStatus.Position_Moule_Status=2;
			//GPS_PutStr(GPS_MODE);
			   rt_kprintf("\r\n    GPS MODE\r\n");
			}
		else if(str[0]=='3')
			{
			   dev_gps_write( &dev_gps, 0, GPSBD_MODE, strlen((const char*)BD_MODE));    
			   GpsStatus.Position_Moule_Status=3;
			//GPS_PutStr(GPSBD_MODE);
			rt_kprintf("\r\n    GPS&BD MODE\r\n");
			}
	 }
 }
FINSH_FUNCTION_EXPORT(gps_mode, posit_mode setting);


void  gps_raw(u8* str)
{
      if(str[0]=='1')
      	{
      	    GpsStatus.Raw_Output=1;      
      	}
	else  
	if(str[0]=='2')
      	{
      	    GpsStatus.Raw_Output=2;       
      	}  
      else
     if(str[0]=='0') 
     	{
           GpsStatus.Raw_Output=0;         
     	}
     	 
}

FINSH_FUNCTION_EXPORT(gps_raw, gps raw output);  

/************************************** The End Of File **************************************/

