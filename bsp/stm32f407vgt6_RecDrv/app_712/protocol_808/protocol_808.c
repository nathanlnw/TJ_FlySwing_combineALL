/*
     Protocol_808.C
*/

#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>
#include "math.h"
#include  <stdlib.h>
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"
#include "math.h"
#include "stdarg.h"
#include "string.h"
#include "SMS.h"
#include "Vdr.h"




#define    ROUTE_DIS_Default            0x3F000000


u8 chushilicheng[4];
u8 leijilicheng[4];
u8 Setting08[80]="预留 	    车门  	   雾灯  	   近光灯	   远光灯	   右转灯	   左转灯	   刹车 	    ";





//----   多媒体发送状态 -------
_Media_SD_state Photo_sdState;   //  图片发送状态
_Media_SD_state Sound_sdState;	//声音发送  
_Media_SD_state Video_sdState;	//视频发送


//------ Photo -----
 u32 PicFileSize=0; // 图片文件大小  
 u8  PictureName[40];      



//------  voice -----



//------  video  --------


/*
             杂
*/
//------ phone
u8       CallState=CallState_Idle; // 通话状态

//   ASCII  to   GB    ---- start  
//0-9        10
u8  arr_A3B0[20]={0xA3,0xB0,0xA3,0xB1,0xA3,0xB2,0xA3,0xB3,0xA3,0xB4,0xA3,0xB5,0xA3,0xB6,0xA3,0xB7,0xA3,0xB8,0xA3,0xB9};

//@ A-O      16
u8  arr_A3C0[32]={0xA3,0xC0,0xA3,0xC1,0xA3,0xC2,0xA3,0xC3,0xA3,0xC4,0xA3,0xC5,0xA3,0xC6,0xA3,0xC7,0xA3,0xC8,0xA3,0xC9,0xA3,0xCA,0xA3,0xCB,0xA3,0xCC,0xA3,0xCD,0xA3,0xCE,0xA3,0xCF};

//P-Z         11个
u8  arr_A3D0[22]={0xA3,0xD0,0xA3,0xD1,0xA3,0xD2,0xA3,0xD3,0xA3,0xD4,0xA3,0xD5,0xA3,0xD6,0xA3,0xD7,0xA3,0xD8,0xA3,0xD9,0xA3,0xDA};

//.  a-0       16
u8  arr_A3E0[32]={0xA3,0xE0,0xA3,0xE1,0xA3,0xE2,0xA3,0xE3,0xA3,0xE4,0xA3,0xE5,0xA3,0xE6,0xA3,0xE7,0xA3,0xE8,0xA3,0xE9,0xA3,0xEA,0xA3,0xEB,0xA3,0xEC,0xA3,0xED,0xA3,0xEE,0xA3,0xEF};

//p-z          11
u8  arr_A3F0[22]={0xA3,0xF0,0xA3,0xF1,0xA3,0xF2,0xA3,0xF3,0xA3,0xF4,0xA3,0xF5,0xA3,0xF6,0xA3,0xF7,0xA3,0xF8,0xA3,0xF9,0xA3,0xFA};
//-------  ASCII to GB ------



//----------- 行车记录仪相关  -----------------
Avrg_MintSpeed  Avrgspd_Mint; 
u32         PerMinSpdTotal=0; //记录每分钟速度总数  
u8          avgspd_Mint_Wr=0;       // 填写每分钟平均速度记录下标
u8          avgspd_Sec_Wr=0;       // 填写每秒钟平均速度记录下标
u8          avgWriteOver=0;   // 写溢出标志位
u8          AspdCounter=0;    // 每分钟速度有效报数计数器 
u8          Vehicle_sensor=0; // 车辆传感器状态   0.2s  查询一次                                    
							  /*	   
							  D7  刹车
							  D6  左转灯
							  D5  右转灯
							  D4  喇叭
							  D3  远光灯
							  D2  雨刷
							  D1  预留
							  D0  预留
							  */
u8		  Vehicle_sensor_BAK=0; // 车辆传感器状态	0.2s  查询一次	
                              
DOUBT_TYPE  Sensor_buf[100];// 20s 状态记录
u8          save_sensorCounter=0,sensor_writeOverFlag=0;;


u8  Camera_Number=1;
u8       DispContent=0;   // 发送时是否显示数据内容  
                             /*                          
                                         1 <->  正常显示 
                                         2 <->  显示发送信息的 
                                         3 <->  显示 任务的运行情况  
                                         0<-> 不显示调试输出，只显示协议数据
                                  */ 

u8         TextInforCounter=0;//文本信息条数

u8 		   FCS_GPS_UDP=0;						//UDP 数据异或和
u8         FCS_RX_UDP=0;                       // UDP 数据接收校验

u8          Centre_IP_modify=0;               //  中修改IP了 
u8          IP_change_counter=0;             //   中心修改IP 计数器
u8          Down_Elec_Flag=0;                //   断油断电使能标志位 



//-------------    不同北斗模块设置  ----
//#ifdef HC_595_CONTROL
//---------74CH595  Q5   control Power----
u8   Print_power_Q5_enable=0;   
u8   Buzzer_on_Q7_enable=0; 
//#endif







//------------ 超速报警---------------------  
SPD_EXP speed_Exd;



 GPRMC_PRO GPRMC_Funs=
 {
   Time_pro,
   Status_pro,
   Latitude_pro,
   Lat_NS_pro,
   Longitude_pro,
   Long_WE_pro,
   Speed_pro,
   Direction_pro,
   Date_pro
 };


//--------  GPS prototcol----------------------------------------------------------------------------------
static u32 	fomer_time_seconds, tmp_time_secnonds, delta_time_seconds;
u8	        UDP_dataPacket_flag= 0x03;			   /*V	   0X03 	 ;		   A	  0X02*/
u8          Year_illigel=0; //  年份不合法 
u8	        GPS_getfirst=0; 		 //  首次有经纬度 
u8          HDOP_value=99;         //  Hdop 数值    
u8          Satelite_num=0;   // 卫星颗数
u8 CurrentTime[3];
u8 BakTime[3];
u8 Sdgps_Time[3];  // GPS 发送 时间记录   BCD 方式

//u16  Spd_add_debug=0;

//static u8      UDP_AsciiTx[1800];     
 ALIGN(RT_ALIGN_SIZE)
u8      GPRS_info[900];  
u16     GPRS_infoWr_Tx=0; 

 ALIGN(RT_ALIGN_SIZE)
u8  UDP_HEX_Rx[1024];    // EM310 接收内容hex 

u16 UDP_hexRx_len=0;    // hex 内容 长度
u16 UDP_DecodeHex_Len=0;// UDP接收后808 解码后的数据长度


GPS_RMC GPRMC; // GPMC格式

        /*                         pGpsRmc->status,\
								pGpsRmc->latitude_value,\
								pGpsRmc->latitude,\
								pGpsRmc->longtitude_value,\
								pGpsRmc->longtitude,\
								pGpsRmc->speed,\
								pGpsRmc->azimuth_angle);
								*/


		
//----------808 协议 -------------------------------------------------------------------------------------
u16	   GPS_Hight=0;               //   808协议-> 高程   m    
u16	   Spd_Using=0;			   //   808协议-> 速度   0.1km/h    
u16     GPS_direction=0;           //   808协议-> 方向   度           
u16     Centre_FloatID=0; //  中心消息流水号
u16     Centre_CmdID=0;   //  中心命令ID

u8      Original_info[850]; // 没有转义处理前的原始信息  
u16     Original_info_Wr=0; // 原始信息写地址
//---------- 用GPS校准特征系数相关 ----------------------------
u8      Speed_area=60; // 校验K值范围
u16     Speed_gps=0;  // 通过GPS计算出来的速度 0.1km/h
u16     Speed_jiade=0; //  假的速度
u8      Speed_Rec=0;  // 速度传感器 校验K用的存储器
u16     Speed_cacu=0; // 通过K值计算出来的速度
u16     Spd_adjust_counter=0; // 确保匀速状态计数器
u16     Spd_Deltacheck_counter=0;   // 传感器速度和脉冲速度相差较大判断
u16     Former_DeltaPlus[K_adjust_Duration]; // 前几秒的脉冲数 
u8      Former_gpsSpd[K_adjust_Duration];// 前几秒的速度      
//-----  车台注册定时器  ----------
DevRegst   DEV_regist;  // 注册
DevLOGIN   DEV_Login;   //  鉴权  


//   -------  CAN BD new  --------------
CAN_TRAN     CAN_trans;



 
//------- 文本信息下发 -------
TEXT_INFO      TextInfo;    // 文本信息下发 
//------- 事件 ----
EVENT          EventObj;    // 事件   
EVENT          EventObj_8[8]; // 事件  
//-------文本信息-------
MSG_TEXT       TEXT_Obj;
MSG_TEXT       TEXT_Obj_8[8],TEXT_Obj_8bak[8];

//------ 提问  --------
CENTRE_ASK     ASK_Centre;  // 中心提问
//------  信息点播  ---
MSG_BRODCAST   MSG_BroadCast_Obj;    // 信息点播         
MSG_BRODCAST   MSG_Obj_8[8];  // 信息点播    
//------  电话本  -----
PHONE_BOOK    PhoneBook,Rx_PhoneBOOK;   //  电话本
PHONE_BOOK    PhoneBook_8[8];

//-----  车辆控制 ------
VEHICLE_CONTROL Vech_Control; //  车辆控制     
//-----  电子围栏  -----
POLYGEN_RAIL Rail_Polygen;   // 多边形围栏
RECT_RAIL    Rail_Rectangle; // 矩形围栏
RECT_RAIL    Rail_Rectangle_multi[8]; // 矩形围栏
CIRCLE_RAIL  Rail_Cycle_multi[8];     // 圆形围栏
CIRCLE_RAIL  Rail_Cycle;     // 圆形围栏
//------- 线路设置 -----
POINT        POINT_Obj;      // 路线的拐点
ROUTE        ROUTE_Obj;      // 路线相关 
//-------    行车记录仪  -----
RECODER      Recode_Obj;     // 行车记录仪  
//-------  拍照  ----  
CAMERA        Camera_Obj;     //  中心拍照相关      
//-----   录音  ----
VOICE_RECODE  VoiceRec_Obj;   //  录音功能     
//------ 多媒体  --------
MULTIMEDIA    MediaObj;       // 多媒体信息  
//-------  数据信息透传  -------
DATATRANS     DataTrans;      // 数据信息透传   
//-------  进出围栏状态 --------
INOUT        InOut_Object;    // 进出围栏状态    
//-------- 多媒体检索  ------------
MEDIA_INDEX  MediaIndex;  // 多媒体信息    
//------- 车辆负载状态 ---------------
u8  CarLoadState_Flag=1;//选中车辆状态的标志   1:空车   2:半空   3:重车

//------- 多媒体信息类型---------------
u8  Multimedia_Flag=1;//需要上传的多媒体信息类型   1:视频   2:音频   3:图像
u8  SpxBuf[SpxBuf_Size];  
u16 Spx_Wr=0,Spx_Rd=0;
u8  Duomeiti_sdFlag=0; 

//------- 录音开始或者结束---------------
u8  Recor_Flag=1; //  1:录音开始   2:录音结束


//----------808协议 -------------------------------------------------------------------------------------	
u8		SIM_code[6];							   // 要发送的IMSI	号码
u8		IMSI_CODE[15]="000000000000000";							//SIM 卡的IMSI 号码
u8		Warn_Status[4]		=
{
		0x00, 0x00,0x00,0x00
}; //  报警标志位状态信息
u8  Warn_MaskWord[4]		=
{
		0x00, 0x00,0x00,0x00
};   //  报警屏蔽字    
u8  Text_MaskWord[4]=
	{
			0x00, 0x00,0x00,0x00
	};	 //  文本屏蔽字    
u8  Key_MaskWord[4]=
	{
			0x00, 0x00,0x00,0x00
	};	 //   关键字屏蔽字    
	


u8		Car_Status[4]		=
{
		0x00, 0x0c,0x00,0x00 
}; //  车辆状态信息	
T_GPS_Info_GPRS 	Gps_Gprs,Bak_GPS_gprs;	 
T_GPS_Info_GPRS	Temp_Gps_Gprs; 
u8   A_time[6]; // 定位时刻的时间

u8      ReadPhotoPageTotal=0;
u8      SendPHPacketFlag=0; ////收到中心启动接收下一个block时置位


//-------- 紧急报警 -------- 
u8		warn_flag= 0;		  
u8		f_Exigent_warning= 0;//0;     //脚动 紧急报警装置 (INT0 PD0)
u8		Send_warn_times= 0;     //   设备向中心上报报警次数，最大3 次
u32  	fTimer3s_warncount=0;

// ------  车辆信息单独了 ---------------
VechINFO     Vechicle_Info;     //  车辆信息  
VechINFO     Vechicle_Info_BAK;  //  车辆信息 BAK
VechINFO     Vechicle_info_BAK2; //  车辆信息BAK2     
u8           Login_Menu_Flag=0;       //   登陆界面 标志位


//------  车门开关拍照 -------
DOORCamera   DoorOpen;    //  开关车门拍照

//------- 北斗扩展协议  ------------
BD_EXTEND     BD_EXT;     //  北斗扩展协议  
DETACH_PKG   Detach_PKG; // 分包重传相关
SET_QRY         Setting_Qry; //  终端参数查询
u32     CMD_U32ID=0; 
PRODUCT_ATTRIBUTE   ProductAttribute;// 终端属性
HUMAN_CONFIRM_WARN   HumanConfirmWarn;// 人工确认报警

//-----  ISP    远程下载相关 -------
ISP_BD  BD_ISP; //  BD   升级包    



// ---- 拐点 -----
u16  Inflexion_Current=0;
u16  Inflexion_Bak=0;      
u16  Inflexion_chgcnter=0; //变化计数器
u16  InflexLarge_or_Small=0;     // 判断curent 和 Bak 大小    0 equql  1 large  2 small  
u16  InflexDelta_Accumulate=0;  //  差值累计 

// ----休眠状态  ------------
u8  SleepState=0;  //   0  不休眠ACC on            1  休眠Acc Off 
u8  SleepConfigFlag=0; //  休眠时发送鉴权标志位

//---- 固定文件大小 --- 
u32 mp3_fsize=5616;
u8  mp3_sendstate=0;
u32 wmv_fsize=25964;
u8  wmv_sendstate=0;

//-------------------   公共 ---------------------------------------		
static u8 GPSsaveBuf[40];     // 存储GPS buffer
static u8	ISP_buffer[1024];
static u16 GPSsaveBuf_Wr=0;


POSIT Posit[60];           // 每分钟位置信息存储
u8    PosSaveFlag=0;      // 存储Pos 状态位 

NANDSVFlag   NandsaveFlg;
A_AckFlag    Adata_ACKflag;  // 无线GPRS协议 接收相关 RS232 协议返回状态寄存器
TCP_ACKFlag  SD_ACKflag;     // 无线GPRS协议返回状态标志 
u32  SubCMD_8103H=0;            //  02 H命令 设置记录仪安装参数回复 子命令
u32  SubCMD_FF01H=0;            //  FF02 北斗信息扩展
u32  SubCMD_FF03H=0;     //  FF03  设置扩展终端参数设置1
u8   Fail_Flag=0;


u8  SubCMD_10H=0;            //  10H   设置记录仪定位告警参数
u8  OutGPS_Flag=0;     //  0  默认  1  接外部有源天线
u8   Spd_senor_Null=0;  // 手动传感器速度为0
u32  Centre_DoubtRead=0;     //  中心读取事故疑点数据的读字段 
u32  Centre_DoubtTotal=0;    //  中心读取事故疑点的总字段
u8   Vehicle_RunStatus=0;    //  bit 0: ACC 开 关             1 开  0关
                             //  bit 1: 通过速度传感器感知    1 表示行驶  0 表示停止   
                             //  bit 2: 通过gps速度感知       1 表示行驶  0 表示停止  
u8   Status_TiredwhRst=0;    //  当复位时 疲劳驾驶的状态   0 :停车  1:停车但没触发 2:触发了还没结束                              



u32   SrcFileSize=0,DestFilesize=0,SrcFile_read=0;
u8    SleepCounter=0; 

u16   DebugSpd=0;   //调试用GPS速度  
u8    MMedia2_Flag=0;  // 上传固有音频 和实时视频  的标志位    0 传固有 1 传实时






unsigned short int FileTCB_CRC16=0;
unsigned short int Last_crc=0,crc_fcs=0; 



//---------  中心应答  -----------
u8		 fCentre_ACK=0; 			  // ---------判断中心应答标志位－－
u8		 ACK_timer=0;				   //---------	ACK timer 定时器--------------------- 
u16         ACKFromCenterCounter=0;//十包无应答重新拨号  
u8           Send_Rdy4ok=0;
unsigned char	Rstart_time = 0; 


//---------------  速度脉冲相关--------------
u16  Delta_1s_Plus=0;
u16  Delta_1s_Plus2=0; 
u16  Sec_counter=0;

void  K_AdjustUseGPS(u32  sp_DISP);  // 通过GPS 校准  K 值  (车辆行驶1KM 的脉冲数目) 
void  Delta_Speed_judge(void);   

u16  Protocol_808_Encode(u8 *Dest,u8 *Src, u16 srclen);
void Protocol_808_Decode(void);  // 解析指定buffer :  UDP_HEX_Rx  
void Photo_send_end(void);
#ifdef REC_VOICE_ENABLE
void Sound_send_end(void);
#endif
//void Video_send_end(void);  
unsigned short int  File_CRC_Get(void);  
void Spd_ExpInit(void);
void AvrgSpd_MintProcess(u8 hour,u8 min, u8 sec) ; 
u32   Distance_Point2Line(u32 Cur_Lat, u32  Cur_Longi,u32 P1_Lat,u32 P1_Longi,u32 P2_Lat,u32 P2_Longi);
void  RouteRail_Judge(u8* LatiStr,u8* LongiStr);  

//  A.  Total  

void delay_us(u16 j)
{
	u8 i;
	  while(j--)
	  	{
	  	i=3;
	  	while(i--);
	  	}  
}

void delay_ms(u16 j )
{
  while(j--)
  	{
           DF_delay_us(2000); // 1000
  	} 
}

 u8  Do_SendGPSReport_GPRS(void)   
{  	
  //unsigned short int crc_file=0;  
  u8  packet_type = 0;
  
// 	  u8 i=0;
            if(DEV_Login.Operate_enable==1)   // !=2   
			 	{                  
				  if((1==DEV_Login.Enable_sd)&&(0==DEV_regist.Enable_sd))
					  {
						Stuff_DevLogin_0102H();   //  鉴权   ==2 时鉴权完毕
						DEV_Login.Enable_sd=0;
						//------ 发送鉴权不判断 ------------------
						//DEV_Login.Operate_enable=2;  //  不用判断鉴权了   
						return true; 
					  }
			 	}
            //------------------------ MultiMedia Send--------------------------
		    if((MediaObj.Media_transmittingFlag==2)&&(DEV_Login.Operate_enable==2))	
            {
			    if(1==MediaObj.SD_Data_Flag) 
			 	{
	                  Stuff_MultiMedia_Data_0801H();     
					  MediaObj.SD_Data_Flag=0; 
                  return true;
			 	}
			   return true; // 按照808 协议要求 ，传输多媒体过程中不允许发送别的信息包
            } 
            if(1==DEV_regist.Enable_sd)
               {									
                  Stuff_RegisterPacket_0100H(0);   // 注册
                   JT808Conf_struct.Msg_Float_ID=0;
                  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
				  DEV_Login.Sd_counter=0;
				  DEV_Login.Operate_enable=0;
		          DEV_regist.Enable_sd=0;
		          //  JT808Conf_struct.Regsiter_Status=1; //标注注册，但不存储		  
                return true;
               }
            if(1==DEV_regist.DeRegst_sd)
            	{
            	  Stuff_DeviceDeregister_0101H(); 
                  DEV_regist.DeRegst_sd=0;		     		  
				  return true;
            	}
            if((1==JT808Conf_struct.DURATION.Heart_SDFlag)&&(DataLink_Status())&&(SleepState==0)&&(DEV_Login.Operate_enable==2)) //  心跳
            	{
                  Stuff_DeviceHeartPacket_0002H();
                  JT808Conf_struct.DURATION.Heart_SDFlag=0;
				  JT808Conf_struct.DURATION.TCP_SD_state=1;  //发送完后置 1   
				  return true;  
            	} 			
            if((1==SleepConfigFlag)&&(DataLink_Status())&&(SleepState==1)) //  休眠时心跳
            	{
                 // Stuff_DevLogin_0102H();   //  鉴权   ==2 时鉴权完毕
                 // rt_kprintf("\r\n	 休眠时用鉴权做心跳包 ! \r\n");  
                  SleepConfigFlag=0;  
				  return true;   
            	} 
			
            if(1==SD_ACKflag.f_CurrentPosition_0201H)    // 位置信息查询
            	{
                  Stuff_Current_Data_0201H();
				  SD_ACKflag.f_CurrentPosition_0201H=0;
				  return true;
            	} 
            if(1==SD_ACKflag.f_CurrentEventACK_0301H)   //  事件报告
            	{
                  Stuff_EventACK_0301H();
				  SD_ACKflag.f_CurrentEventACK_0301H=0;
				  return true;                  
            	}
				            
	         if(2==ASK_Centre.ASK_SdFlag)      //  提问应答 
	             	{
	                  Stuff_ASKACK_0302H();
	                  ASK_Centre.ASK_SdFlag=0; 
	                  return true;  
	             	}             	
			             	// 9.  中心拍照命令应答
				if( 1 == SD_ACKflag.f_BD_CentreTakeAck_0805H )                                                                           // 中心拍照命令应答
				{
					Stuff_CentreTakeACK_BD_0805H( );
					SD_ACKflag.f_BD_CentreTakeAck_0805H = 0;
					return true;
				} 	
             if(1==Vech_Control.ACK_SD_Flag)   //  车辆应答控制
             	{
                  Stuff_ControlACK_0500H(); 
				  Vech_Control.ACK_SD_Flag=0;
				  return true;                  
             	}
			 
			 if(SD_ACKflag.f_MsgBroadCast_0303H==1)
			 	{
                   Stuff_MSGACK_0303H(); 
                   SD_ACKflag.f_MsgBroadCast_0303H=0;
                   return true;
			 	}
			 	 if((1==MediaObj.SD_media_Flag)&&(DEV_Login.Operate_enable==2))
			 	{
				   Stuff_MultiMedia_InfoSD_0800H();
				   MediaObj.SD_media_Flag=2;  // original clear  0 ,,HBTDT =2  ,timeout ACK 
                   return true;    
			 	}	
             if(DataTrans.Data_TxLen)
             	{ 
                  Stuff_DataTransTx_0900H();  // 数据透传 做远程下载
                  DataTrans_Init();     //clear 
                  return true;
             	} 		
             if(SD_ACKflag.f_MediaIndexACK_0802H)
             	{
                   Stuff_MultiMedia_IndexAck_0802H();   // 多媒体索引上报 
                   SD_ACKflag.f_MediaIndexACK_0802H=0;
				   return true;
             	}			 
			 if(SD_ACKflag.f_DriverInfoSD_0702H)
			 	{
			 	   Stuff_DriverInfoSD_0702H();  //  驾驶员信息上报
                   SD_ACKflag.f_DriverInfoSD_0702H=0;
                   return true;
			 	}
			 	//  18.数据透传 做远程下载	 
             if(CAN_trans.canid_0705_sdFlag)
             	{ 
                  Stuff_CANDataTrans_BD_0705H();
		          CAN_trans.canid_0705_sdFlag=0; // clear 		   
               //   DataTrans_Init();     //clear 
                  return true;
             	} 	
			  if(SD_ACKflag.f_Worklist_SD_0701H)
			 	{
			 	   Stuff_Worklist_0701H();  //   电子运单 
                   SD_ACKflag.f_Worklist_SD_0701H=0;
                   return true;
			 	}
			  		//  23.   查询终端参数	
	         if(SD_ACKflag.f_SettingPram_0104H)
	        	{
	        	  if(SD_ACKflag.f_SettingPram_0104H==1)
	        	      Stuff_SettingPram_0104H(); 
				  else
				  if(SD_ACKflag.f_SettingPram_0104H==2)
				  	  Sub_stuff_AppointedPram_0106();     
				  
                   SD_ACKflag.f_SettingPram_0104H=0;    
				   return true;
	        	}	
			    //  24 .  终端升级结果上报
              if(SD_ACKflag.f_BD_ISPResualt_0108H)
              	{
                                
				     Stuff_ISP_Resualt_BD_0108H();
                     
                    if(1==SD_ACKflag.f_BD_ISPResualt_0108H)
                    {
                      if( File_CRC_Get()==true)
					       ISP_file_Check();  // update 
                    }
					else
					   BD_ISP.ISP_running=0; // recover normal  	 
					SD_ACKflag.f_BD_ISPResualt_0108H=0;
				     return true;		   
              	}
	        if(SD_ACKflag.f_CentreCMDack_0001H)
	         {
                               Stuff_DevCommmonACK_0001H();        
				   if(SD_ACKflag.f_CentreCMDack_0001H==2)  //  修改IP设置了需要重拨
				   	   Close_DataLink();    //  AT_END    
				   else	   
				   if(SD_ACKflag.f_CentreCMDack_0001H==3) //   远程复位
				   	{
                                       Systerm_Reset_counter=Max_SystemCounter;
					  ISP_resetFlag=2;    //   借助远程下载重启机制复位系统
				   	}
				   else
				   if(SD_ACKflag.f_CentreCMDack_0001H==5) //   关闭数据通信
				   	{
				   	   Close_DataLink();  
                                      Stop_Communicate();      
				   	}				 
                               SD_ACKflag.f_CentreCMDack_0001H=0; 
				   SD_ACKflag.f_CentreCMDack_resualt=0;    
                   
				   
                   return true;    
				}  	 
         
		 //  15.  行车记录仪数据上传
			  //------------ Error	state --------------
				if(Recode_Obj.Error)
				 {
		 
					Stuff_RecoderACK_0700H_Error();
					Recode_Obj.Error=0;
					Recode_Obj.SD_Data_Flag = 0;
					Recode_Obj.CountStep = 0;
					  rt_kprintf("\r\n	记录仪CoutStepFlag=0--3 \r\n");
					return true;
				 } 
             
			 //-----------------------------------------
			 if((1==Recode_Obj.SD_Data_Flag)&&(1==Recode_Obj.CountStep))
			 {
				 //  1. clear	one  packet  flag
				 switch( Recode_Obj.CMD )
				 {
					 /* 			   divide  not	stop	2013-6-20  */
					 //----------------------------------------------------
						case 0x08:
						case 0x09:
						case 0x10:
						case 0x11:
						case 0x12:
						case 0x15:
					  //---------------------------------------------------
					 case 0x00:
					 case 0x01:
					 case 0x02:
					 case 0x03:  
					 case 0x04:
					 case 0x05:
					 case 0x06: 
					 case 0x07:
					 case 0x13:
					 case 0x14:    //	上边是查询的
			 
					  case 0x82: //    中心设置车牌号  
					  case 0x83://	记录仪初次安装时间
					  case 0x84:// 设置信号量配置信息
					  case 0xC2: //设置记录仪时钟
					  case 0xC3: //车辆速度脉冲系数（特征系数）
					  case 0xC4: //   设置初始里程
					 //------------------------
						 Recode_Obj.SD_Data_Flag = 0;
						 Recode_Obj.CountStep= 0;
						   rt_kprintf("\r\n  记录仪CoutStepFlag=0--2 \r\n");
						 break;
					 default:
							   break;
				 }
				 //  2.  stuff	 recorder	infomation
				 //  judge	packet	type
				 if( Recode_Obj.Devide_Flag==1 )
				 {
					 packet_type = Packet_Divide;
				 } else
				 {
					 packet_type = Packet_Normal;
				 }	
			 
				 Stuff_RecoderACK_0700H( packet_type); //	行车记录仪数据上传
				 
				 rt_kprintf( "\r\n 记录仪 CMD_ID =0x%2X \r\n", Recode_Obj.CMD );
				 if( packet_type == Packet_Divide )
				 {
					 rt_kprintf( "\r\n current =%d	Total: %d \r\n", Recode_Obj.Current_pkt_num, Recode_Obj.Total_pkt_num );
				 }
				 //  3. step  by  step	send   from  00H  ---  07H
			 
			 
				 if( Recode_Obj.CountStep==1 )
				 {
					 Recode_Obj.CountStep=2;
					 Recode_Obj.timer = 0;
					 rt_kprintf( "\r\n 记录仪定时发送----CountStep"); 
				 }
			 
				 return true;  
			   } 
			//----------------------------------------------------------------------
			if(Current_State==1)   // 调试时30  实际是10 Current_SD_Duration 
			 {	
				 if(PositionSD_Status()&&(DEV_Login.Operate_enable==2)&&((enable==BD_EXT.Trans_GNSS_Flag)||(DispContent==6))||(Current_UDP_sd&&PositionSD_Status()&&(DEV_Login.Operate_enable==2))||((DF_LOCK==enable)&&PositionSD_Status()))	  //首次定位再发 
				   //  if((PositionSD_Status())&&(DataLink_Status())&&(DEV_Login.Operate_enable==2))	                                                                                                                     // DF  锁定发送当前位置信息  
			      {
			                  PositionSD_Disable();
					          Current_UDP_sd=0; 
	                              //1.   时间超前判断
	                              //if(Time_FastJudge()==false)
								//return false;		
					  // 2. 			  
				 	   Stuff_Current_Data_0200H();  // 上报即时数据  				   
					   //----应答次数 ----		   
					  // ACKFromCenterCounter++; // 只关注应答报数，不关心应答时间 
					   //---------------------------------------------------------------------------------
					   if(DispContent)	
						    rt_kprintf("\r\n 发送 GPS -current !\r\n");     
					   return true;
				    }   
			 }
			 else 
             if((RdCycle_RdytoSD==ReadCycle_status)&&(0==BD_ISP.ISP_running)&&(DataLink_Status())&&(DEV_Login.Operate_enable==2)&&(CameraState.camera_running==0))    // 读取发送--------- 正常GPS 
             {                                       /* 远程下载时不允许上报GPS ，因为有可能发送定位数据时
                                                          正在接收大数据量得下载数据包,导致GSM模块处理不过来，而不是单片机处理不过来*/
                  if (false==Stuff_Normal_Data_0200H())        
				 	return false;
				  fCentre_ACK=1;// 判断应答
				 //-------- change status  Ready  ACK  ------ 
			         ReadCycle_status=RdCycle_SdOver;
				  Send_Rdy4ok=1;  // enable
				  //----应答次数 ----		  
				  ACKFromCenterCounter++; 
				 // if(DispContent)	
					//  rt_kprintf("\r\n 发送 GPS --saved  OK!\r\n");    
				 return true; 
             }			 
 			//-------------------------------------------------------------
			return  false; 
             
}
void strtrim(u8 *s, u8 c)
{
	u8		 *p1, *p2;
	u16  i, j;

	if (s == 0) return;

	// delete the trailing characters
	if (*s == 0) return;
	j = strlen((char const *)s);
	p1 = s + j;
	for (i = 0; i < j; i++)
	{
	   	p1--;
	   	if (*p1 != c) break;
	}
	if (i < j) p1++;
	*p1 = 0;	// null terminate the undesired trailing characters

	// delete the leading characters
  	p1 = s;
	if (*p1 == 0) return;
	for (i = 0; *p1++ == c; i++);
	if (i > 0)
	{
		p2 = s;
	 	p1--;
		for (; *p1 != 0;) *p2++ = *p1++;
		*p2 = 0;
	}
} 

int str2ip(char *buf, u8 *ip)
{	// convert an ip:port string into a binary values
	int	i;
	u16	_ip[4]; 
	

	memset(_ip, 0, sizeof(_ip));

	strtrim((u8*)buf, ' ');
   
	i = sscanf(buf, "%u.%u.%u.%u", (u32*)&_ip[0], (u32*)&_ip[1], (u32*)&_ip[2], (u32*)&_ip[3]);

	*(u8*)(ip + 0) = (u8)_ip[0];
	*(u8*)(ip + 1) = (u8)_ip[1];
	*(u8*)(ip + 2) = (u8)_ip[2];
	*(u8*)(ip + 3) = (u8)_ip[3];

	return i;
}



int IP_Str(char *buf, u32 IP)
{
	T_IP_Addr	ip;
    
	if (!buf) return 0;

	ip.ip32 = IP;
    
	return sprintf(buf, "%u.%u.%u.%u", ip.ip8[0], ip.ip8[1], ip.ip8[2], ip.ip8[3]);
}

u16 AsciiToGb(u8 *dec,u8 InstrLen,u8 *scr)
{
u16 i=0,j=0,m=0;
u16 Info_len=0;


for(i=0,j=0;i<InstrLen;i++,j++) 
	{
	m=scr[i];
	if((m>=0x30)&&(m<=0x39))
		{
		memcpy(&dec[j],&arr_A3B0[(m-'0')*2],2);
		j++;
		}
	else if((m>=0x41)&&(m<=0x4f))
		{
		memcpy(&dec[j],&arr_A3C0[(m-0x41+1)*2],2);
		j++;
		}
	else if((m>=0x50)&&(m<=0x5a))
		{
		memcpy(&dec[j],&arr_A3D0[(m-0x50)*2],2) ;
		j++;
		}
	else if((m>=0x61)&&(m<=0x6f))
		{
		memcpy(&dec[j],&arr_A3E0[(m-0x61+1)*2],2) ;
		j++;
		}
	else if((m>=0x70)&&(m<=0x7a))
		{
		memcpy(&dec[j],&arr_A3F0[(m-0x70)*2],2)  ;
		j++;
		}
	else
		{
		dec[j]=m;
		}	
	}
Info_len=j;
return Info_len;
}


// B.   Protocol
 
  
//==================================================================================================
// 第一部分 :   以下是GPS 解析转换相关函数 
//==================================================================================================

void Time_pro(u8 *tmpinfo, u8 hour, u8 min , u8 sec)
{
  	            //---- record  to memory
				GPRMC.utc_hour=hour;
				GPRMC.utc_min=min;
				GPRMC.utc_sec=sec;
                
				CurrentTime[0] = hour;
				CurrentTime[1] = min;
				CurrentTime[2] = sec; 
                
				//-----------  天地通协议 ------------- 	 
				Temp_Gps_Gprs.Time[0] = hour;
				Temp_Gps_Gprs.Time[1] = ( tmpinfo[2] - 0x30 ) * 10 + tmpinfo[3] - 0x30;
				Temp_Gps_Gprs.Time[2] = ( tmpinfo[4] - 0x30 ) * 10 + tmpinfo[5] - 0x30;  
            
}

void Status_pro(u8 *tmpinfo) 
{
 	                GPRMC.status=tmpinfo[0];
                    Posit_ASCII.AV_ASCII=tmpinfo[0];
					//-------------------------天地通协议-----------------------------
						   if ( tmpinfo[0] == 'V' || tmpinfo[0] == 'v' )
						  {
								UDP_dataPacket_flag=0X03;
								StatusReg_GPS_V();
						  }
						  else if ( tmpinfo[0] == 'A' || tmpinfo[0] == 'a' )
						  {
								UDP_dataPacket_flag=0X02;		
								StatusReg_GPS_A(); 
								  
						  }

					//---------------------------------------------------------
	

}

void Latitude_pro(u8 *tmpinfo)
{
  u32  latitude;
  u32  vdr_lati=0;
  
   GPRMC.latitude_value=atof((char *)tmpinfo);
   /*     Latitude  
          ddmm.mmmm
    */
   memset(Posit_ASCII.Lat_ASCII,0,sizeof(Posit_ASCII.Lat_ASCII));
   memcpy(Posit_ASCII.Lat_ASCII,tmpinfo,strlen((const char*)tmpinfo));  
	//--------	808 协议 --------------------
  if(UDP_dataPacket_flag==0X02)    //精确到百万分之一度
  {  
    
    //------------  dd part   -------- 
	latitude = ( u32 ) ( ( tmpinfo[0] - 0x30 ) * 10 + ( u32 ) ( tmpinfo[1] - 0x30 ) ) * 1000000;
	vdr_lati=latitude;
	//------------  mm  part  -----------
	/*    转换成百万分之一度
	      mm.mmmm   *  1000000/60=mm.mmmm*50000/3=mm.mmmm*10000*5/3
	*/
	latitude = latitude + ( u32 )( (( tmpinfo[2] - 0x30 ) * 100000+ (tmpinfo[3] - 0x30 ) * 10000+(tmpinfo[5] - 0x30 ) * 1000 + ( tmpinfo[6] - 0x30 ) * 100 + ( tmpinfo[7] - 0x30 ) * 10 +( tmpinfo[8] - 0x30 ))*5/3);  

	 if(latitude==0)
	 	{
	 	   GPS_getfirst=0;
	          StatusReg_GPS_V();
	          return;
		}

	Temp_Gps_Gprs.Latitude[0] = ( u8 ) ( latitude >> 24 );
	Temp_Gps_Gprs.Latitude[1] = ( u8 ) ( latitude >> 16 );
	Temp_Gps_Gprs.Latitude[2] = ( u8 ) ( latitude >> 8 );
	Temp_Gps_Gprs.Latitude[3] = ( u8 ) latitude;

    //  以下是行车记录仪的转换
    //                      度                                           分large   
    vdr_lati=vdr_lati*600000+( tmpinfo[2] - 0x30 ) * 100000+ (tmpinfo[3] - 0x30 ) * 10000+(tmpinfo[5] - 0x30 ) * 1000 + ( tmpinfo[6] - 0x30 ) * 100 + ( tmpinfo[7] - 0x30 ) * 10 +( tmpinfo[8] - 0x30 );

	VdrData.Lati[0]= ( u8 ) ( vdr_lati >> 24 );
    VdrData.Lati[1]= ( u8 ) ( vdr_lati >> 16 );
	VdrData.Lati[2]= ( u8 ) ( vdr_lati >> 8 );
	VdrData.Lati[3]= ( u8 ) vdr_lati;
    
	//-------------------------------------------------------------------------------------------
  } 
   //----------------------------------------------
}
 
void Lat_NS_pro(u8 *tmpinfo)
{
   GPRMC.latitude=tmpinfo[0];
}

void Longitude_pro(u8 *tmpinfo)
{
 u32  longtitude;
 u32  Longi=0;
 
 GPRMC.longtitude_value=atof((char *)tmpinfo);  
  /*     Latitude  
          dddmm.mmmm
    */

   memset(Posit_ASCII.Longi_ASCII,0,sizeof(Posit_ASCII.Longi_ASCII)); 
   memcpy(Posit_ASCII.Longi_ASCII,tmpinfo,strlen((const char*)tmpinfo));  
 //--------  808协议  ---------
 if(UDP_dataPacket_flag==0X02)  //精确到百万分之一度
 {
     //------  ddd part -------------------
	 longtitude = ( u32 )( ( tmpinfo[0] - 0x30 ) * 100 + ( tmpinfo[1] - 0x30 ) * 10 + ( tmpinfo[2] - 0x30 ) ) * 1000000;  
     Longi=longtitude;  
	 //------  mm.mmmm --------------------	 
	 /*    转换成百万分之一度
		   mm.mmmm	 *	1000000/60=mm.mmmm*50000/3=mm.mmmm*10000*5/3
	 */
	 longtitude = longtitude + ( u32 ) (( ( tmpinfo[3] - 0x30 ) * 100000 + ( tmpinfo[4] - 0x30 )*10000+(tmpinfo[6] - 0x30 ) * 1000 + ( tmpinfo[7] - 0x30 ) * 100 + ( tmpinfo[8] - 0x30 ) * 10 + ( tmpinfo[9] - 0x30 ))*5/3);
	 if(longtitude==0)
	 	{GPS_getfirst=0; StatusReg_GPS_V();return;} 
	 
	 Temp_Gps_Gprs.Longitude[0] = ( u8 ) ( longtitude >> 24 );
	 Temp_Gps_Gprs.Longitude[1] = ( u8 ) ( longtitude >> 16 );
	 Temp_Gps_Gprs.Longitude[2] = ( u8 ) ( longtitude >> 8 );
	 Temp_Gps_Gprs.Longitude[3] = ( u8 ) longtitude; 


     
	 //  以下是行车记录仪的转换
	 // 					 度 										  分large	
    Longi=Longi*600000+ ( tmpinfo[3] - 0x30 ) * 100000 + ( tmpinfo[4] - 0x30 )*10000+(tmpinfo[6] - 0x30 ) * 1000 + ( tmpinfo[7] - 0x30 ) * 100 + ( tmpinfo[8] - 0x30 ) * 10 + ( tmpinfo[9] - 0x30 );

	VdrData.Longi[0]= ( u8 ) ( Longi >> 24 );
    VdrData.Longi[1]= ( u8 ) ( Longi >> 16 );
	VdrData.Longi[2]= ( u8 ) ( Longi >> 8 );
	VdrData.Longi[3]= ( u8 ) Longi;
    
	 
 }			   
	 
   //---------------------------------------------------  
}

void Long_WE_pro(u8 *tmpinfo)
{

	GPRMC.longtitude=tmpinfo[0]; 
}


void Speed_pro(u8 *tmpinfo,u8 Invalue,u8 Point)
{ 
  u32	  sp=0,sp_DISP=0;
  u32     reg=0;  

  
    //-------------------------------------------------------------------------------------------------------------
	if(Invalue==INIT)
	{	  
	  return;
	}
	else//---------------------------------------------------------------------------------------------------------
	{
	 GPRMC.speed=atof((char *)tmpinfo);
	 //---------------------------------------------------					  
	 if(UDP_dataPacket_flag==0x02 )
	 {
			  //-----808 协议 -------------- 
			 //两个字节单位0.1 km/h  
			 if ( Point == 1 )	//0.0-9.9=>
			 {
												  //++++++++  Nathan Modify on 2008-12-1   ++++++++++
					if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39))
					 {
					   sp = ( tmpinfo[0] - 0x30 ) * 10 + ( tmpinfo[2] - 0x30 );  //扩大10倍
					 }	   
					else
					   return;			  
				 
			 }
			 else if ( Point == 2 )  //10.0-99.9
			 {
											  //++++++++  Nathan Modify on 2008-12-1   ++++++++++
					if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[3]>=0x30)&&(tmpinfo[3]<=0x39)) 
					{
					  sp = ( tmpinfo[0] - 0x30 ) * 100 + ( tmpinfo[1] - 0x30 ) * 10 + tmpinfo[3] - 0x30;
					}
					else
					 return; 									 
	 
			 }
			 else if( Point == 3 ) //100.0-999.9
			 {
												   //++++++++  Nathan Modify on 2008-12-1	++++++++++
					if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39)&&(tmpinfo[4]>=0x30)&&(tmpinfo[4]<=0x39)) 
					 {
					   sp = ( tmpinfo[0] - 0x30 ) * 1000 + ( tmpinfo[1] - 0x30 ) * 100 + ( tmpinfo[2] - 0x30 ) * 10 + tmpinfo[4] - 0x30;
					  }
					else
					  return;  
	 
			 }
		 else		 
			 {
			     if(JT808Conf_struct.Speed_GetType==0)
			          Spd_Using=0;
			 }
	 
		 // --------  sp 当前是0.1 knot------------------	 
		  sp= (u32)(sp * 185.6) ;  //  1 海里=1.856 千米  现在是m/h
								 
		  if(sp>220000)   //时速大于220km/h则剔除
			  return;  
	 
		  sp_DISP=sp/100;   //  sp_Disp 单位是 0.1km/h 
							  
	         //------------------------------ 通过GPS模块数据获取到的速度 --------------------------------
                   Speed_gps=(u16)sp_DISP;



            //Speed_gps=Speed_jiade;//800;  //  假的为了测试   


			 
	         //---------------------------------------------------------------------------
       if(JT808Conf_struct.Speed_GetType)  // 通过速度传感器 获取速度
       	{ 
              K_AdjustUseGPS(Speed_gps);  //  调整K值    
              if(JT808Conf_struct.DF_K_adjustState==0) 
			  {
			     // ---  在未校准前，获得到的速度是通过GPS计算得到的
			     Spd_Using=Speed_gps;      
			       //------- GPS	里程计算-------- 
				 if(sp>=5000)		//	过滤零点漂移  速度大于
				 {
					 reg=sp/3600;  // 除以3600 是m/s 
					 JT808Conf_struct.Distance_m_u32+=reg;
					 if(JT808Conf_struct.Distance_m_u32>0xFFFFFF)
							  JT808Conf_struct.Distance_m_u32=0; 	  //里程最长这么多米	
                     Distance_m_u32=JT808Conf_struct.Distance_m_u32;// add later
					 //----- 定距回传处理---  
                     if(1==JT808Conf_struct.SD_MODE.DIST_TOTALMODE)
                     {
                        DistanceAccumulate+=reg;
                        if(DistanceAccumulate>=Current_SD_Distance)
                        	{
                               DistanceAccumulate=0;
				   PositionSD_Enable();        //发送
				   Current_UDP_sd=1;
                        	}
                     }
					 //------- 定距处理结束 -----
							  
				 }	
				
              }  
			  else
			  	  Spd_Using=Speed_cacu;       
       	}  
	   else
	   	{  // 从GPS 取速度
			     //------- GPS	里程计算-------- 
				 if(sp>=5000)		//	过滤零点漂移  速度大于
				 {
					 JT808Conf_struct.Distance_m_u32+=sp/3600;  // 除以3600 是m/s 
					 if(JT808Conf_struct.Distance_m_u32>0xFFFFFF)
							  JT808Conf_struct.Distance_m_u32=0; 	  //里程最长这么多米	  
                     Distance_m_u32=JT808Conf_struct.Distance_m_u32;// add later  
					 //----- 定距回传处理---  
                     if(1==JT808Conf_struct.SD_MODE.DIST_TOTALMODE)
                     {
                        DistanceAccumulate+=reg;
                        if(DistanceAccumulate>=Current_SD_Distance)
                        	{
                               DistanceAccumulate=0;
					PositionSD_Enable();          //发送
					Current_UDP_sd=1;
                        	}
                     }
					 //------- 定距处理结束 -----		   
				 }			 
					
					Spd_Using=Speed_gps;    // 用GPS数据计算得的速度 单位0.1km/h
					
		            //-----------------------------------------------
					
	   	     }
			 // if(DispContent==2) 
			  //  rt_kprintf("\r\n				  速度: %d Km/h\r\n",GPS_speed/10);     
	 }
	 else if( UDP_dataPacket_flag == 0x03 )
	 { 
       if(0==JT808Conf_struct.Speed_GetType)  
		{
			 //----- GPS 临时速度	km/h  ---------
		     Spd_Using=0;	 
       	} 
        if(JT808Conf_struct.Speed_GetType)  // 通过速度传感器 获取速度
       	{ 
              K_AdjustUseGPS(Speed_gps);  //  调整K值   
              Spd_Using=Speed_cacu;      
       	}      
	   Speed_gps=0;
	   if(DispContent==2)
		   rt_kprintf("\r\n 2 GPS没定位\r\n"); 
	 }  		
	}
   //---------------------------------------------------
   Delta_Speed_judge();//  判断传感器和GPS速度的差值  
	
   //---------------------------------------------------
}


void Direction_pro(u8 *tmpinfo,u8 Invalue,u8 Point)
{
   u32	  sp=0;  
  //------------------------------------------------------------------------------------------------
  if(Invalue==INIT)
  {    
    return;
  }
  else//-------------------------------------------------------------------------------------------
  {
   GPRMC.azimuth_angle=atof((char *)tmpinfo);
	
   
   //--------------808 协议  1 度------------------------- 	 
   if ( UDP_dataPacket_flag== 0x02 )
   {	  
		  
		
		   if ( Point == 1 )   //5.8 
		   {
					  if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39))
				   sp = ( tmpinfo[0] - 0x30 ) ;
				  else 
				   return;
				   
		   }
		   else if ( Point == 2 )  // 14.7
		   {
					  if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[3]>=0x30)&&(tmpinfo[3]<=0x39)) 
				   sp = ( tmpinfo[0] - 0x30 ) * 10 + ( tmpinfo[1] - 0x30 );
				  else
				   return;
		   
		   }
		   else    //357.38
		   if ( Point == 3 )
		   {
			      if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39)&&(tmpinfo[4]>=0x30)&&(tmpinfo[4]<=0x39)) 
				   sp = ( tmpinfo[0] - 0x30 ) * 100 + ( tmpinfo[1] - 0x30 ) * 10 + ( tmpinfo[2] - 0x30 ) ;
				  else
				   return;
   
	       }
		   else
		   	{
               sp=0;   
		   	}
		   GPS_direction=sp;  //  单位 1度

	   //----------  拐点补传相关   ----------
	  // Inflexion_Process();
       
	   
		   
   }
   else if ( UDP_dataPacket_flag == 0x03 )
   {
          GPS_direction=0; 		  
 
   }


    return;
  }
 


}

void Date_pro(u8 *tmpinfo,u8 fDateModify, u8 hour, u8 min , u8 sec)
{
 uint8_t  year=0,mon=0,day=0;  
 TDateTime now, now_bak; 
 int i=0;

 
 day = (( tmpinfo[0] - 0x30 ) * 10 ) + ( tmpinfo[1] - 0x30 );
 mon = (( tmpinfo[2] - 0x30 ) * 10 ) + ( tmpinfo[3] - 0x30 );
 year = (( tmpinfo[4] - 0x30 ) * 10 ) + ( tmpinfo[5] - 0x30 );
 
 if(fDateModify){
	 //sscanf(tmpinfo,"%2d%2d%2d",&day,&mon,&year);
	 day++;
	 if(mon == 2){
		 if ( ( year % 4 ) == 0 ){
			 if ( day == 30 ){day = 1;mon++;}
		 }else if ( day == 29 ){ day = 1;mon++;}
	 }else if (( mon == 4 ) || ( mon == 6 ) || ( mon == 9 ) || ( mon == 11 )){
		 if ( day == 31 ){mon++;day = 1;}
	 }else{
		 if ( day == 32 ){mon++;day = 1;}	 
		 if( mon == 13 ){mon = 1;year++; }
	 }
 }

 //--------  年份过滤， 如果定位到以前时间不予处理，直接返回
 if(year<13)
 {
   //----- 状态改成 V  
   UDP_dataPacket_flag=0X03;
   StatusReg_GPS_V();

   Year_illigel=1; // 年份不合法
   
   return ;
 }
 else
 	Year_illigel=0; 
 
 GPRMC.utc_year=year;
 GPRMC.utc_mon=mon;
 GPRMC.utc_day=day;
 if((sec==30)&&(GPRMC.status=='A')){
	 now.year = year;
	 now.month = mon;	   
	 now.day = day;
	 now.hour = hour;
	 now.min = min;
	 now.sec = sec;
	 now.week=1;      
	 Device_RTC_set(now);  

	 //------  读取设置校验---
	 now_bak=Get_RTC();	 
	 i=memcmp((u8*)&now_bak,(u8*)&now,sizeof(now)); 
	 if(i!=0)
	  {
	        RT_Total_Config();		 	
		    Device_RTC_set(now);  
		 
		  rt_kprintf("\r\n 校验有误，重新设置RTC\r\n");  

	  }
	// else
	 //	rt_kprintf("\r\n RTC 配置校验 OK \r\n");
	 
 }
 //------------------------------------------------ 	
 if(GPRMC.status=='A')     //  记录定位时间
{
  Time2BCD(A_time);  
  //------- Debug 存储 每秒的经纬度  || 实际应该是 存储每分钟的位置  -----   
  //  内容持续55秒每秒更新，这寄存器中记录的是在每分钟内最后一包定位的经纬度 ,预留5秒用于存储上一小时的位置 
  if(sec<55)
  	{
      memcpy(Posit[min].latitude_BgEnd,Gps_Gprs.Latitude,4); //北纬
      memcpy(Posit[min].longitude_BgEnd,Gps_Gprs.Longitude,4); //经度	  
      Posit[min].longitude_BgEnd[0]|=0x80;//  东经
  	}
  if((min==59)&&(sec==55))
    { // 每个小时的位置信息               
       NandsaveFlg.MintPosit_SaveFlag=1; 
	}  
 }   
  //---- 存储当前的起始里程  跨天时------------
   if((hour==0)&&(min==0)&&(sec==3))   // 存储3次确保存储成功 
  	{ 
  	     //replace
	     // JT808Conf_struct.DayStartDistance_32=JT808Conf_struct.Distance_m_u32;
         // Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
         // by
		  DF_Write_RecordAdd(Distance_m_u32,DayStartDistance_32,TYPE_DayDistancAdd);
		  JT808Conf_struct.DayStartDistance_32=DayStartDistance_32;
		  JT808Conf_struct.Distance_m_u32=Distance_m_u32;		  
       } 
 
 //-------------------------------------------------
      //---------  天地通协议  -------
                       
						//if(systemTick_TriggerGPS==0)
 {
   Temp_Gps_Gprs.Date[0] = year;
   Temp_Gps_Gprs.Date[1] = mon;
   Temp_Gps_Gprs.Date[2] = day;   						
  }

  
 //-------------- 计算算平均速度 ----------------
 AvrgSpd_MintProcess(hour,min,sec); 
}


//---------  GGA --------------------------
void HDop_pro(u8 *tmpinfo)
{ 
 float dop;
 
  dop=atof((char *)tmpinfo);				        
  HDOP_value=dop;		 //  Hdop 数值

}

void  GPS_Delta_DurPro(void)    //告GPS 触发上报处理函数
{
    

 //    1.    定时上报相关
  if((1==JT808Conf_struct.SD_MODE.DUR_TOTALMODE)&&(Year_illigel==0))   // 定时上报模式
  {
	 	//----- 上一包数据记录的时间
		fomer_time_seconds = ( u32 ) ( BakTime[0] * 60 * 60 ) + ( u32 ) ( BakTime[1] * 60 ) + ( u32 ) BakTime[2];  
		
		//-----  当前数据记录的时间
		tmp_time_secnonds = ( u32 ) ( CurrentTime[0] * 60 * 60 ) + ( u32 ) ( CurrentTime[1] * 60 ) + ( u32 )  CurrentTime[2];
		
		//一天86400秒
		
		if ( tmp_time_secnonds > fomer_time_seconds )
		{
				delta_time_seconds = tmp_time_secnonds - fomer_time_seconds;
				//systemTickGPS_Clear();
		}
		else if(tmp_time_secnonds < fomer_time_seconds)
		{
				delta_time_seconds = 86400 - fomer_time_seconds + tmp_time_secnonds;
				//systemTickGPS_Clear();
		}
		else 
		 {
			 // systemTickGPS_Set();  
			  UDP_dataPacket_flag=0X03; 
			  StatusReg_GPS_V();
		 }	
		
		if((SleepState==1)&&(delta_time_seconds==(Current_SD_Duration-5)))  //  --  休眠时 先发鉴权
		   {
			  SleepConfigFlag=1;  //发送前5 发送一包鉴权
		   }            
		
		if((delta_time_seconds >= Current_SD_Duration))//limitSend_idle
		  {			  
			 if(Current_SD_Duration<=CURREN_LIM_Dur)   // 若发送间隔小于 10  则上报即时信息 
			  {     
			        Current_State=1;   //   使能发送标志位
				     PositionSD_Enable();
                     Current_UDP_sd=1;   // 使能发送操作执行
			  }
			 else
			 	 Current_State=0; 

			 
			 if (BD_ISP.ISP_running==0)
			    PositionSD_Enable();  
			 
			  memcpy(BakTime,CurrentTime,3); // update   
		  }
  	}  
  
    //------------------------------ do this every  second-----------------------------------------    
	memcpy((char*)&Gps_Gprs,(char*)&Temp_Gps_Gprs,sizeof(Temp_Gps_Gprs));     






  //  2.  判断行驶状态
     if((Speed_gps>100) || (Speed_cacu>100))
     {
            VDR_TrigStatus.Run_baseon_spd_10s_couter++;
			if(VDR_TrigStatus.Run_baseon_spd_10s_couter>20)  //  速度大于10km/h  超过 10s   这里判20s
				{
				   //  做循环判断处理
				   if(VDR_TrigStatus.Run_baseon_spd_10s_couter>65530)
				   	  VDR_TrigStatus.Run_baseon_spd_10s_couter=21;

                   //    11 H   连续驾驶开始时间   和起始位置     从60s 开始算驾驶起始
                    VDR_product_11H_Start();
                    
				   
				    VDR_TrigStatus.Running_state_enable=1;  //  处于行驶状态

				}
     }
     if((Speed_gps<100)&&(Speed_cacu<100))
     {
         //         11 H     相关
         VDR_product_11H_End();
	 
		 VDR_TrigStatus.Run_baseon_spd_10s_couter=0;	 	 
		 VDR_TrigStatus.Running_state_enable=0;  //  处于停止状态    
	 }




	 

   // 3.  行驶记录相关数据产生 触发,  且定位的情况下   
    VDR_product_08H_09H_10H();




   
   //4.   电子围栏 判断  ----------
  /*  if((Temp_Gps_Gprs.Time[2]%20)==0) //   认证时不检测圆形电子围栏
    {
        CycleRail_Judge(Temp_Gps_Gprs.Latitude,Temp_Gps_Gprs.Longitude);
		//rt_kprintf("\r\n --- 判断圆形电子围栏");
    }	*/
    // if((Temp_Gps_Gprs.Time[2]==5)||(Temp_Gps_Gprs.Time[2]==25)||(Temp_Gps_Gprs.Time[2]==45)) //  
    if(Temp_Gps_Gprs.Time[2]%2==0)//    认证时要求2 秒
   {
        RectangleRail_Judge(Temp_Gps_Gprs.Latitude,Temp_Gps_Gprs.Longitude);
		//rt_kprintf("\r\n -----判断矩形电子围栏"); 
    }
      if((Temp_Gps_Gprs.Time[2]%5)==0) //    
    {
          // printf("\r\n --- 判断圆形电子围栏");
          // RouteRail_Judge(Temp_Gps_Gprs.Latitude,Temp_Gps_Gprs.Longitude); 
          ;
    }
	//rt_kprintf("\r\n Delta_seconds %d \r\n",delta_time_seconds);    
  //----------------------------------------------------------------------------------------	
}

//---------------------------------------------------------------------------------------------------
void K_AdjustUseGPS(u32  sp_DISP)  // 通过GPS 校准  K 值  (车辆行驶1KM 的脉冲数目)  0.1 km/h 进入
{
      
  u32 Reg_distance=0;
  u32 Reg_plusNum=0;
  u16 i=0;
  
  if(JT808Conf_struct.DF_K_adjustState)   // 只有没校准时才有效
			return;
		
   Speed_Rec=(u8)(sp_DISP/10);	// GPS速度    单位:km/h
   // -------	要求速度在60到65km/h  -------------
   if(((Speed_Rec>=Speed_area)&&(Speed_Rec<=(Speed_area+8)))||((Speed_Rec>=40)&&(Speed_Rec<=(40+8)))||((Speed_Rec>=70)&&(Speed_Rec<=(70+8))))   // Speed_area=60   
 // if(Speed_Rec>=Speed_area) 
//   if((Speed_Rec>=40)&&(Speed_Rec<=48))   // Speed_area=60    
   {
	   Spd_adjust_counter++;    
	   if(Spd_adjust_counter>K_adjust_Duration)  //持续在速度在60~65下认为已经是匀速了
	   {
		   // 用获取到的匀速GPS速度作为基准，和根据传感器计算出来的速度，做K值得校准
		   Reg_distance=0;	// clear
		   Reg_plusNum=0;	// clear 
		   for(i=2;i<K_adjust_Duration;i++)  //剔除前2组数据(前两组数据不是很准确，因为什么时刻开始不确定)
		   {
			  Reg_distance+=Former_gpsSpd[i];  // 除以3.6km/h 表示该秒内走了多少米
			  Reg_plusNum+=Former_DeltaPlus[i]; 						   
		   }
           /*
                做一个判断  ， 如果速度传感器不管用， 那么返回， 
             */
           if(Reg_plusNum<20)
		   	 {
		   	   Spd_adjust_counter=0; 
			   rt_kprintf("\r\n    速度传感器 没有脉冲!\r\n");
		   	   return; 
           	 }  		   	
		   //===================================================================
		   // 转换成根据GPS速度计算行驶了多少米，(总距离) 先求和在除以3.6 ，为了计算方便 先x10  再除以36 
		   Reg_distance=(u32)(Reg_distance*10/36); // 转换成根据GPS速度计算行驶了多少米，(总距离)
		   // (Reg_plusNum/Reg_distance) 表示用脉冲总数除以距离(米)= 每米产生多少个脉冲 ，因为K值是1000米脉冲数，所以应该乘以1000
		   JT808Conf_struct.Vech_Character_Value=1000*Reg_plusNum/Reg_distance;
		   //-------  存储新的特征系数 -------------------------------- 						
		   JT808Conf_struct.DF_K_adjustState=1;					// clear  Flag
		   ModuleStatus|=Status_Pcheck;
		   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
		   
		  Spd_adjust_counter=0; // clear  counter
	   }
	   else
	   {   //-------- 记录规定时间内的脉冲数和GPS速度----------
		   Former_gpsSpd[Spd_adjust_counter]=Speed_Rec; 
		   Former_DeltaPlus[Spd_adjust_counter]=Delta_1s_Plus;
	   }
  
   }  
   else
	  Spd_adjust_counter=0;  // 只要速度超出预设范围 计数器清0
}


void  Delta_Speed_judge(void)
{
    /*
         Note: 在从传感器速度，且速度被校准的情况下，在GPS 速度大于40Km/h
                 的时候，判断速度传感器和实际速度的差值，如果速度差值连续续大于12 km/h
                ，且持续一段时间，说明以前脉冲系数已经不准确， 那么启动重新校准。
                     
       */
   if((JT808Conf_struct.DF_K_adjustState)&&(JT808Conf_struct.Speed_GetType)&&(Speed_gps>=400)&&(UDP_dataPacket_flag==0x02 ))
  {
     // rt_kprintf("\r\n  delta  check \r\n");
     if(abs(Speed_gps-Speed_cacu)>120)  // 速度相差12Km/h    这里的单位是0.1km/h
     {  
          Spd_Deltacheck_counter++;
		  if(Spd_Deltacheck_counter>30)  //  持续30s
		  {
		     Spd_Deltacheck_counter=0; // clear
             JT808Conf_struct.DF_K_adjustState=0;// disable              
			// rt_kprintf("\r\n  Re_adust!\r\n");
		  }
     }
	 else
	 	Spd_Deltacheck_counter=0; // clear   

   }

}
//==================================================================================================
// 第二部分 :   以下是外部传感器状态监测
//==================================================================================================
/*    
     -----------------------------
     2.1   和协议相关的功能函数
     ----------------------------- 
*/


/*    
     -----------------------------
    2.4  不同协议状态寄存器变化
     ----------------------------- 
*/

void StatusReg_WARN_Enable(void)
{
  //     紧急报警状态下 寄存器的变化
    Warn_Status[3] |= 0x01;//BIT( 0 );		   
}

void StatusReg_WARN_Clear(void)
{
  //     清除报警寄存器  
    Warn_Status[3] &= ~0x01;//BIT( 0 );  	  	
}

void StatusReg_ACC_ON(void)
{  //    ACC 开
  Car_Status[3]|=0x01; //  Bit(0)     Set  1  表示 ACC开

}

void StatusReg_ACC_OFF(void)
{  //    ACC 关  
  
  Car_Status[3]&=~0x01; //  Bit(0)     Set  01  表示 ACC关
}

void StatusReg_POWER_CUT(void)
{  //  主电源断开
   Warn_Status[2]	|= 0x01;//BIT( 0 );
   ModuleStatus|=Status_Battery;
}

void StatusReg_POWER_NORMAL(void)
{  // 主电源正常
   Warn_Status[2] &= ~0x01;//BIT( 0 );     
   ModuleStatus&=~Status_Battery;
}

void StatusReg_GPS_A(void)
{     // GPS 定位 
   if(GPS_getfirst==0)
   {	    
	 #ifdef LCD_5inch
                   DwinLCD.Type=LCD_SETTIME;       
	  #endif
   }	  
 GPS_getfirst=1;
 Car_Status[3]|=0x02;   //Bit(1)
 ModuleStatus |= Status_GPS;	
}

void StatusReg_GPS_V(void)
{    //  GPS 不定位  
  Car_Status[3]&=~0x02;   //Bit(1)
  ModuleStatus &= ~Status_GPS;
}
	
void StatusReg_SPD_WARN(void)
{    //  超速报警
   Warn_Status[3] |= 0x02;//BIT( 1 );
}

void StatusReg_SPD_NORMAL(void)
{    //  速度正常
  Warn_Status[3] &=~ 0x02;//BIT( 1 );
}

void StatusReg_Relay_Cut(void)
{  // 断油断电状态 

}

 void StatusReg_Relay_Normal(void)
{ //  断油电状态正常
 
}


void StatusReg_Default(void)
{    //   状态寄存器还原默认设置
  
   Warn_Status[0]=0x00; //HH
   Warn_Status[1]=0x00; //HL  
   Warn_Status[2]=0x00; //LH 
   Warn_Status[3]=0x00; //LL	      
}

//==================================================================================================
// 第三部分 :   以下是GPRS无线传输相关协议
//==================================================================================================
void  Save_GPS(void)     
{
   u16 counter_mainguffer,i;
   u8  lati_reg[4];//,regstatus;
   
		  if (PositionSD_Status())	
		  {
		         if(DF_LOCK==enable)    // 清除文件区域时 ，禁止操作DF
				 	return ;
				 if(Login_Menu_Flag!=1)  // 没有被设置前不存储相关位置信息，没有意义
				 	return;  
		     //-------------------------------------------------------	 
		     //1.   时间超前判断
                              //if(Time_FastJudge()==false)    
							//return ;		 
				  //----------------------- Save GPS --------------------------------------
				  memset(GPSsaveBuf,0,40);   
				  GPSsaveBuf_Wr=0;				  
				   //------------------------------- Stuff ----------------------------------------
					counter_mainguffer = GPSsaveBuf_Wr;
				   // 1. 告警状态   4 Bytes
				   memcpy( ( char * ) GPSsaveBuf + GPSsaveBuf_Wr, ( char * )Warn_Status,4 );   
				   GPSsaveBuf_Wr += 4;  
                   // 2. 车辆状态   4 Bytes
				   memcpy( ( char * ) GPSsaveBuf + GPSsaveBuf_Wr, ( char * )Car_Status,4 );   
				   GPSsaveBuf_Wr += 4; 
				   // 3.   纬度     4 Bytes
				   memcpy(lati_reg,Gps_Gprs.Latitude,4);
				   memcpy( ( char * ) GPSsaveBuf + GPSsaveBuf_Wr, ( char * )Gps_Gprs.Latitude, 4 );//纬度   modify by nathan
				   GPSsaveBuf_Wr += 4;
				   // 4.   经度     4 Bytes
				   memcpy( ( char * ) GPSsaveBuf + GPSsaveBuf_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	 //经度    东经  Bit 7->0	西经 Bit 7 -> 1
				   GPSsaveBuf_Wr += 4;
				   // 5.  高度	  2 Bytes    m
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(GPS_Hight>>8);	// High 			   
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(u8)GPS_Hight;  // Low  
				   // 6.  速度	  2 Bytes     0.1Km/h
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(Speed_gps>>8);	// High 			   
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(u8)Speed_gps;  // Low 
				   // 7.  方向	  2 Bytes	    1度 				   
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(GPS_direction>>8);	//High
				   GPSsaveBuf[GPSsaveBuf_Wr++]=GPS_direction; // Low				   
				   // 8.  日期时间	  6 Bytes 
				    GPSsaveBuf[GPSsaveBuf_Wr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);  	   	
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);    
                   //------------------------------------------------------------------ 
                   //   注:   因为长度有限所以存储时只存储传感器速度，不填写 类型和长度 
                  //----------- 附加信息  ------------
				    //  附加信息 1  -----------------------------    
					//	附加信息 ID
					//Original_info[Original_info_Wr++]=0x03; // 行驶记录仪的速度
					//	附加信息长度
					//Original_info[Original_info_Wr++]=2;
					//	类型
					//GPSsaveBuf[GPSsaveBuf_Wr++]=(u8)(Speed_cacu>>8);  
					//GPSsaveBuf[GPSsaveBuf_Wr++]=(u8)(Speed_cacu);	           
				   //--------------------------------------------------------------------
				if(strncmp(( char * )GPSsaveBuf + GPSsaveBuf_Wr-3,(const char*)Sdgps_Time,3)==0)						
					{
					/*if(strncmp((const char*)GPSsaveBuf + GPSsaveBuf_Wr-3,(const char*)Sdgps_Time,3)==0)
						{
						PositionSD_Disable();
						rt_kprintf("\r\n  -->不存储上报时间相同的车\r\n");
						return;
						}
					else*/
						//{    //-------- 用RTC 时钟 -----     
				        time_now=Get_RTC();  
						
					    if((time_now.year<100)&&(time_now.month<12)&&(time_now.day<31)&&(time_now.hour<24)&&(time_now.min<60)&&(time_now.sec<60))
					    {
						   Time2BCD(GPSsaveBuf + GPSsaveBuf_Wr-6);  
						   rt_kprintf("\r\n    启用RTC时间了! \r\n"); 
						  } 
					}
					memcpy(Sdgps_Time,GPSsaveBuf + GPSsaveBuf_Wr-3,3); //更新最新一次存储时间

                   //-------------  Caculate  FCS  -----------------------------------
					FCS_GPS_UDP=0;  
					for ( i = counter_mainguffer; i < 30; i++ )  
					{
							FCS_GPS_UDP ^= *( GPSsaveBuf + i ); 
					}			   //求上边数据的异或和
					GPSsaveBuf[30] = FCS_GPS_UDP;  	  	
			     //-------------------------------- Save  ------------------------------------------
			      if(Api_cycle_write(GPSsaveBuf,31))
			      { 
					  if(DispContent) 	
					       rt_kprintf("\r\n    GPS Save succed\r\n");
			      	}
				 else
			       {
				  	  if(DispContent) 
				     	         rt_kprintf("\r\n GPS save fail\r\n"); 
				 }
                 //---------------------------------------------------------------------------------
				  if(PositionSD_Status())
				         PositionSD_Disable();
				 if(SleepState==1)	   
				 {	 
					 rt_kprintf("\r\n休眠时存储时间 %d-%d-%d %02d:%02d:%02d\r\n", time_now.year+2000, time_now.month, time_now.day, \
					 time_now.hour, time_now.min, time_now.sec);
				 }
				  //-----------------------------------------------------
		  }
   

}
//----------------------------------------------------------------------
u8  Protocol_Head(u16 MSG_ID, u8 Packet_Type)
{
     u32    Reg_ID=0;
     //----  clear --------------
       Original_info_Wr=0;
	//	1. Head  
	
	    //  original info
	   Original_info[Original_info_Wr++]=(MSG_ID>>8); // 消息ID 
	   Original_info[Original_info_Wr++]=(u8)MSG_ID;
	   
	   Original_info[Original_info_Wr++]=0x00; // 分包、加密方式、状态位 
	   Original_info[Original_info_Wr++]=28; // 消息体长度   位置信息长度为28个字节
	   
	   memcpy(Original_info+Original_info_Wr,SIM_code,6); // 终端手机号 ，设备标识ID	BCD
	   Original_info_Wr+=6;  
	   


       if(Packet_Type==Packet_Divide) 
       	{
            switch (MediaObj.Media_Type)
	         {
	          case 0 : // 图像
	                              MediaObj.Media_totalPacketNum=Photo_sdState.Total_packetNum;  // 图片总包数
					  MediaObj.Media_currentPacketNum=Photo_sdState.SD_packetNum;  // 图片当前报数 
					  MediaObj.Media_ID=1;   //  多媒体ID
					  MediaObj.Media_Channel=Camera_Number;  // 图片摄像头通道号
					  
                                      Reg_ID=0xF000+Camera_Number*0x0100+Photo_sdState.SD_packetNum;
					   Original_info[Original_info_Wr++]=( Reg_ID>>8); //消息流水号
	                               Original_info[Original_info_Wr++]=  Reg_ID;				  
					  
			          break;
			  case 1 : // 音频
					  MediaObj.Media_totalPacketNum=Sound_sdState.Total_packetNum;	// 音频总包数
					  MediaObj.Media_currentPacketNum=Sound_sdState.SD_packetNum;  // 音频当前报数
					  MediaObj.Media_ID=1;	 //  多媒体ID
					  MediaObj.Media_Channel=1;  // 音频通道号   
					  
					  Reg_ID=0xE000+Sound_sdState.SD_packetNum;
					   Original_info[Original_info_Wr++]=( Reg_ID>>8); //消息流水号
	                               Original_info[Original_info_Wr++]=  Reg_ID;	

			          break;
			  case 2 : // 视频
					  MediaObj.Media_totalPacketNum=Video_sdState.Total_packetNum;	// 视频总包数
					  MediaObj.Media_currentPacketNum=Video_sdState.SD_packetNum;  // 视频当前报数
					  MediaObj.Media_ID=1;	 //  多媒体ID
					  MediaObj.Media_Channel=1;  // 视频通道号 
			          break;
			  default:
			  	      return false;
	         }
               //    当前包数
		 Original_info[Original_info_Wr++]=(MediaObj.Media_totalPacketNum&0xff00)>>8;//总block
		 Original_info[Original_info_Wr++]=(u8)MediaObj.Media_totalPacketNum;//总block
			 
		 //   总包数	 
	        Original_info[Original_info_Wr++]=((MediaObj.Media_currentPacketNum)&0xff00)>>8;//当前block
	        Original_info[Original_info_Wr++]=(u8)((MediaObj.Media_currentPacketNum)&0x00ff);//当前block

       	}  
            else
            	{
		     Original_info[Original_info_Wr++]=( JT808Conf_struct.Msg_Float_ID>>8); //消息流水号
	            Original_info[Original_info_Wr++]= JT808Conf_struct.Msg_Float_ID;		
            	}

			
	    return true;

}  

void Protocol_End(u8 Packet_Type,u8 LinkNum)   
{                   
 u16 packet_len=0;  
 u16  i=0;		//要发送的UDP 数据内容的长度
 u8   Gfcs=0;
 u16   Msg_bodyLen=0; //  协议里的消息只表示消息体     不包含消息头 消息头默认长度是12 , 分包消息头长度 20   
 
   Gfcs=0;				   //  计算从消息头开始到校验前数据的异或和  808协议校验  1Byte
   //---  填写信息长度 ---
   if(Packet_Normal==Packet_Type)
   {
     Msg_bodyLen=Original_info_Wr-12;
     Original_info[2]=(Msg_bodyLen>>8) ;
     Original_info[3]=Msg_bodyLen;
   }
   else
    if(Packet_Divide==Packet_Type)
   {
     Msg_bodyLen=Original_info_Wr-16;
	// rt_kprintf("\r\n Divide Infolen=%d  \r\n",Msg_bodyLen);  
     Original_info[2]=(Msg_bodyLen>>8)|0x20 ;  // Bit 13  0x20 就是Bit 13
     Original_info[3]=Msg_bodyLen;  
   }
   //---- 计算校验  -----
   for(i=0;i<Original_info_Wr;i++)  
		Gfcs^=Original_info[i]; 
   Original_info[Original_info_Wr++]=Gfcs;  // 填写G校验位     


  // 1.stuff start
   GPRS_infoWr_Tx=0;	
   GPRS_info[GPRS_infoWr_Tx++]=0x7e;   // Start 标识位
    if(Packet_Divide==Packet_Type)
   {   
       //rt_kprintf("\r\n Tx=%d  Divide Infolen=%d  \r\n",GPRS_infoWr_Tx,Original_info_Wr);   
	  /* rt_kprintf("\r\n PacketContent: ");
				   for(i=0;i<Original_info_Wr;i++)
                       rt_kprintf(" %X",Original_info[i]);    
				   rt_kprintf("\r\n");
	  */
   }
  // 2.  convert 
   packet_len=Protocol_808_Encode(GPRS_info+GPRS_infoWr_Tx,Original_info,Original_info_Wr);
   GPRS_infoWr_Tx+=packet_len;	   
    if(Packet_Divide==Packet_Type)
   {
	 //rt_kprintf("\r\n Divide  Send Infolen=%d  \r\n",packet_len);  
	 
	/* rt_kprintf("\r\n EncodeContent: ");
	 for(i=0;i<packet_len;i++)
                       rt_kprintf(" %X",GPRS_info[i+1]);      
	rt_kprintf("\r\n"); 
	*/
	//rt_kprintf("\r\n GPRStx  Send Infolen=%d  \r\n",GPRS_infoWr_Tx+1);    
   }
   GPRS_info[GPRS_infoWr_Tx++]=0x7e;  //  End  标识
  //  4. Send   

    // 4.1 发送信息内容1
     if(DispContent==2)
	 {
	    rt_kprintf("\r\n App to GSM info:");    
	    for(i=0;i<GPRS_infoWr_Tx;i++)
			 rt_kprintf(" %02X",GPRS_info[i]);  
		rt_kprintf("\r\n"); 
	 }
    // 4.2   MsgQueue
    WatchDog_Feed();
   Gsm_rxAppData_SemRelease(GPRS_info,GPRS_infoWr_Tx,LinkNum);  
   //--------消息序号 递增 --------
   JT808Conf_struct.Msg_Float_ID++;   
  //------------------------------ 
} 
//--------------------------------------------------------------------------------------
u8  Stuff_DevCommmonACK_0001H(void)      
{    
	// 1. Head
	  if(!Protocol_Head(MSG_0x0001,Packet_Normal))  return false;     //终端通用应答
	 // 2. content  is null 
	   //   float ID
	    Original_info[Original_info_Wr++]=(u8)(Centre_FloatID>>8);
	    Original_info[Original_info_Wr++]=(u8)Centre_FloatID;
	    //  cmd  ID
	    Original_info[Original_info_Wr++]=(u8)(Centre_CmdID>>8);
	    Original_info[Original_info_Wr++]=(u8)Centre_CmdID;   
		//   resualt
	    Original_info[Original_info_Wr++]=SD_ACKflag.f_CentreCMDack_resualt;
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	      rt_kprintf("\r\n	Common CMD ACK! \r\n");  
	 return true;    
}
//-------------------------------------------------------------------------------
u8  Stuff_RegisterPacket_0100H(u8  LinkNum)  
{  
  u8  i=0;
// 1. Head
  if(!Protocol_Head(MSG_0x0100,Packet_Normal)) 
 	return false; 	
    
  // 2. content 
    //  province ID
    Original_info[Original_info_Wr++]=(u8)(Vechicle_Info.Dev_ProvinceID>>8);
    Original_info[Original_info_Wr++]=(u8)Vechicle_Info.Dev_ProvinceID;  // 天津12     64  宁夏
    //  county  ID
    Original_info[Original_info_Wr++]=(u8)(Vechicle_Info.Dev_CityID>>8);   // 101  宁夏
    Original_info[Original_info_Wr++]=(u8)Vechicle_Info.Dev_CityID;        
	//  product Name	
	//  product Name	
    memcpy(Original_info+Original_info_Wr,"70420",5);   //北京中斗  70104      70523     70218
    Original_info_Wr+=5;
	//  终端型号 20 Bytes      -- 补充协议里做更改
	memcpy(Original_info+Original_info_Wr,"TW703",5);   //ZD-V01H  HVT100BD1   CM-10A-BD  YW3000-YM/MGB
    Original_info_Wr+=5;       
    for(i=0;i<15;i++)  
	 Original_info[Original_info_Wr++]=0x00;    	  
    //  终端ID   7 Bytes    ,    
    memcpy(Original_info+Original_info_Wr,SimID_12D+5,7);     // 这个就是终端ID 的后7 位    
    Original_info_Wr+=7;  
	//  车牌颜色  
	if(License_Not_SetEnable==1)       
	   Original_info[Original_info_Wr++]=0; //Vechicle_Info.Dev_Color;
	else
	   Original_info[Original_info_Wr++]=2; //Vechicle_Info.Dev_Color;
	
	if(License_Not_SetEnable==0) //  0  设置车牌号  
	{
		//  车牌
		memcpy(Original_info+Original_info_Wr,Vechicle_Info.Vech_Num,strlen((const char*)(Vechicle_Info.Vech_Num)));//13);  
		Original_info_Wr+=strlen((const char*)(Vechicle_Info.Vech_Num));
	}
	else
	{
	       rt_kprintf("\r\n  车辆颜色:0     Need Vin\r\n");
		// 车辆VIN 
		memcpy(Original_info+Original_info_Wr,Vechicle_Info.Vech_VIN,17);
		Original_info_Wr+=17;
    }

 
  if(DispContent)
  {      
       rt_kprintf("\r\n	SEND Reigster Packet! \r\n");  
	 rt_kprintf("\r\n  注册发送时间 %d-%d-%d %02d:%02d:%02d\r\n", time_now.year+2000, time_now.month, time_now.day, \
      time_now.hour, time_now.min, time_now.sec);
   

  }

 
 //  3. Send 
  Protocol_End(Packet_Normal,LinkNum);   
  if(DispContent)
 rt_kprintf("\r\n	SEND Reigster Packet! \r\n");  
 return true;  

}

//--------------------------------------------------------------------------------------
u8  Stuff_DeviceHeartPacket_0002H(void)      
{  
  
	// 1. Head
	if(!Protocol_Head(MSG_0x0002,Packet_Normal))  
 	  return false; 
	 // 2. content  is null
	 
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	Send Dev Heart! \r\n");    
	 return true;  

}
//--------------------------------------------------------------------------------------
u8  Stuff_DeviceDeregister_0101H(void)      
{    
	// 1. Head
	if(!Protocol_Head(MSG_0x0101,Packet_Normal)) 
 	  return false; //终端注销 
	 // 2. content  is null 
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	Deregister  注销! \r\n");  
	 return true;    
}
//------------------------------------------------------------------------------------
u8  Stuff_DevLogin_0102H(void)      
{    
	// 1. Head
	if(!Protocol_Head(MSG_0x0102,Packet_Normal))  
 	  return false; //终端鉴权 
	 // 2. content  
	  
	 memcpy(Original_info+Original_info_Wr,JT808Conf_struct.ConfirmCode,strlen((const char*)JT808Conf_struct.ConfirmCode));   // 鉴权码  string Type
	 Original_info_Wr+=strlen((const char*)JT808Conf_struct.ConfirmCode); 
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	 发送鉴权! \r\n");   
	 return true;    
}

//--------------------------------------------------------------------------------------
u8  Stuff_Normal_Data_0200H(void)  
{
  u8 spd_sensorReg[2];
  u32  Dis_01km=0;
 //  1. Head  
   if(!Protocol_Head(MSG_0x0200,Packet_Normal)) 
 	  return false;  
 // 2. content 
 WatchDog_Feed();
 if(Api_cycle_read(Original_info+Original_info_Wr, 31)==false)            
   return false;
 Original_info_Wr+=28;   // 内容只有28 
 memcpy(spd_sensorReg,Original_info+Original_info_Wr,2);// 把读取传感器速度 
 
 
  //----------- 附加信息  ------------ 
  //  附加信息 1  -----------------------------    
  //  附加信息 ID
  Original_info[Original_info_Wr++]=0x03; // 行驶记录仪的速度
  //  附加信息长度
  Original_info[Original_info_Wr++]=2;
  //  类型
  Original_info[Original_info_Wr++]=(u8)(Speed_cacu>>8); 
  Original_info[Original_info_Wr++]=(u8)(Speed_cacu);	   
  //rt_kprintf("\r\n GPS速度=%d km/h , 传感器速度=%d km/h\r\n",Speed_gps,Speed_cacu); 
   //  附加信息 2  -----------------------------	
   //  附加信息 ID
   Original_info[Original_info_Wr++]=0x01; // 车上的行驶里程
   //  附加信息长度
   Original_info[Original_info_Wr++]=4; 
   //  类型
   Dis_01km=JT808Conf_struct.Distance_m_u32/100;
   Original_info[Original_info_Wr++]=(Dis_01km>>24); 
   Original_info[Original_info_Wr++]=(Dis_01km>>16); 
   Original_info[Original_info_Wr++]=(Dis_01km>>8); 
   Original_info[Original_info_Wr++]=Dis_01km; 
 
 
  //  附加信息 3 
  if(Warn_Status[1]&0x10)
 {
   //  附加信息 ID
   Original_info[Original_info_Wr++]=0x12; //  进出区域/路线报警
   //  附加信息长度 
   Original_info[Original_info_Wr++]=6;
   //  类型
   Original_info[Original_info_Wr++]=InOut_Object.TYPE;
   Original_info[Original_info_Wr++]=(InOut_Object.ID>>24);
   Original_info[Original_info_Wr++]=(InOut_Object.ID>>16);
   Original_info[Original_info_Wr++]=(InOut_Object.ID>>8);
   Original_info[Original_info_Wr++]=InOut_Object.ID;
   Original_info[Original_info_Wr++]=InOut_Object.InOutState;  
   rt_kprintf("\r\n ----- 0x0200 附加信息 \r\n");	 
 }
 
  //  附件信息4
  if(Warn_Status[3]&0x02)
  { 	 
	//	附加信息 ID
	Original_info[Original_info_Wr++]=0x11; //	进出区域/路线报警
	//	附加信息长度 
	Original_info[Original_info_Wr++]=1; 
	//	类型
	Original_info[Original_info_Wr++]=0; //  无特定位置   
 
  }

  //---------- 附加信息 5 ----
  	 Original_info[Original_info_Wr++]=0x25; //扩展车辆信号状态
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=4; 
	 //  类型
	 Original_info[Original_info_Wr++]=0x00;
       Original_info[Original_info_Wr++]=0x00;
	 Original_info[Original_info_Wr++]=0x00;
       Original_info[Original_info_Wr++]=BD_EXT.Extent_IO_status; 

	   //  附加信息 6  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0xFE; //信号强度
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=2; 
	 //  类型
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_SignalValue;  
	 Original_info[Original_info_Wr++]=0x00;  //  保留 

        //if(DispContent)
         //     printf("\r\n---- Satelitenum: %d , CSQ:%d\r\n",Satelite_num,ModuleSQ);  

     //  附加信息 7  -----------------------------	   
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0xFF; //自定义模拟量上传
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=6; 
	 //  类型
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_1; 
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_2;  
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_0>>8);  // 模拟量 1
	 Original_info[Original_info_Wr++]=BD_EXT.AD_0;
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_1>>8);  // 模拟量 2
	 Original_info[Original_info_Wr++]=BD_EXT.AD_1;

 
 //  3. Send 
 Protocol_End(Packet_Normal ,0);
   	
 return true; 

}
//------------------------------------------------------------------------------------
u8  Stuff_Current_Data_0200H(void)   //  发送即时数据不存储到存储器中
{  
  
  u32  Dis_01km=0;
  
    if( Spd_Using <=( JT808Conf_struct.Speed_warn_MAX*10) )
		StatusReg_SPD_NORMAL(); 

 //  1. Head	
 if(!Protocol_Head(MSG_0x0200,Packet_Normal)) 
 	  return false; 
 // 2. content 
    //------------------------------- Stuff ----------------------------------------
	// 1. 告警标志  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Warn_Status,4 );    
	Original_info_Wr += 4;
	// 2. 状态  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Car_Status,4 );   
	Original_info_Wr += 4;
	// 3.  纬度
    memcpy( ( char * ) Original_info+ Original_info_Wr,( char * )  Gps_Gprs.Latitude, 4 );//纬度   modify by nathan
	Original_info_Wr += 4;
	// 4.  经度
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	  //经度    东经  Bit 7->0   西经 Bit 7 -> 1
	Original_info_Wr += 4;
	// 5.  高程
	Original_info[Original_info_Wr++]=(u8)(GPS_Hight<<8);
	Original_info[Original_info_Wr++]=(u8)GPS_Hight;
	// 6.  速度    0.1 Km/h
	Original_info[Original_info_Wr++]=(u8)(Speed_gps>>8);//(GPS_speed>>8); 
	Original_info[Original_info_Wr++]=(u8)(Speed_gps);//GPS_speed;     
	// 7. 方向   单位 1度
	Original_info[Original_info_Wr++]=(GPS_direction>>8);  //High 
	Original_info[Original_info_Wr++]=GPS_direction; // Low
	// 8.  日期时间	
	Original_info[Original_info_Wr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);		
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);	 

	//----------- 附加信息  ------------
    //  附加信息 1  -----------------------------    
	//	附加信息 ID
	Original_info[Original_info_Wr++]=0x03; // 行驶记录仪的速度
	//	附加信息长度
	Original_info[Original_info_Wr++]=2;
	//	类型
	Original_info[Original_info_Wr++]=(u8)(Speed_cacu>>8); 
	Original_info[Original_info_Wr++]=(u8)(Speed_cacu);	     
	//rt_kprintf("\r\n GPS速度=%d km/h , 传感器速度=%d km/h\r\n",Speed_gps,Speed_cacu); 
     //  附加信息 2  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0x01; // 车上的行驶里程
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=4; 
	 //  类型
	 Dis_01km=JT808Conf_struct.Distance_m_u32/100;
	 Original_info[Original_info_Wr++]=(Dis_01km>>24); 
	 Original_info[Original_info_Wr++]=(Dis_01km>>16); 
	 Original_info[Original_info_Wr++]=(Dis_01km>>8); 
	 Original_info[Original_info_Wr++]=Dis_01km; 
  
   
	//  附加信息 3 
	if(Warn_Status[1]&0x10)
   {
     //  附加信息 ID
     Original_info[Original_info_Wr++]=0x12; //  进出区域/路线报警
     //  附加信息长度 
     Original_info[Original_info_Wr++]=6;
	 //  类型
	 Original_info[Original_info_Wr++]=InOut_Object.TYPE;
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>24);
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>16);
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>8);
	 Original_info[Original_info_Wr++]=InOut_Object.ID;
	 Original_info[Original_info_Wr++]=InOut_Object.InOutState;  
	 rt_kprintf("\r\n ----- 0x0200 current 附加信息 \r\n");    
   }

    //  附件信息4
    if(Warn_Status[3]&0x02)
    {      
	  //  附加信息 ID
	  Original_info[Original_info_Wr++]=0x11; //  进出区域/路线报警
	  //  附加信息长度 
	  Original_info[Original_info_Wr++]=1; 
	  //  类型
	  Original_info[Original_info_Wr++]=0; //  无特定位置   

    }

rt_kprintf("\r\n ----- 0x0200 current 附加信息 \r\n"); 
//附加信息7
	 Original_info[Original_info_Wr++]=0x25; //扩展车辆信号状态
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=4; 
	 //  类型
	 Original_info[Original_info_Wr++]=0x00;
       Original_info[Original_info_Wr++]=0x00;
	 Original_info[Original_info_Wr++]=0x00;
       Original_info[Original_info_Wr++]=BD_EXT.Extent_IO_status; 
	   //  附加信息 5  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0xFE; //信号强度
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=2; 
	 //  类型
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_SignalValue; 
	 Original_info[Original_info_Wr++]=0x00;  //  保留 

        //if(DispContent)
         //     printf("\r\n---- Satelitenum: %d , CSQ:%d\r\n",Satelite_num,ModuleSQ);  

        //  附加信息 6  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0xFF; //自定义模拟量上传
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=6; 
	 //  类型
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_1; 
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_2;  
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_0>>8);  // 模拟量 1
	 Original_info[Original_info_Wr++]=BD_EXT.AD_0;
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_1>>8);  // 模拟量 2
	 Original_info[Original_info_Wr++]=BD_EXT.AD_1;
	
 //  3. Send 
 Protocol_End(Packet_Normal ,0);

  if(SleepState==1)  	
  {   
      rt_kprintf("\r\n休眠时发送时间 %d-%d-%d %02d:%02d:%02d\r\n", time_now.year+2000, time_now.month, time_now.day, \
      time_now.hour, time_now.min, time_now.sec);
  }
 return true; 

}
//-----------------------------------------------------------------------
u8  Stuff_Current_Data_0201H(void)   //   位置信息查询回应
{  
   u32  Dis_01km=0; 
 //  1. Head	
 if(!Protocol_Head(MSG_0x0201,Packet_Normal)) 
 	  return false; 
 // 2. content 
    //------------------------------- Stuff ----------------------------------------
    //   float ID                                                // 对应中心应答消息的流水号
	Original_info[Original_info_Wr++]=(u8)(Centre_FloatID>>8);
	Original_info[Original_info_Wr++]=(u8)Centre_FloatID;

	// 1. 告警标志  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Warn_Status,4 );    
	Original_info_Wr += 4;
	// 2. 状态  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Car_Status,4 );   
	Original_info_Wr += 4;
	// 3.  纬度
    memcpy( ( char * ) Original_info+ Original_info_Wr,( char * )  Gps_Gprs.Latitude, 4 );//纬度   modify by nathan
	Original_info_Wr += 4;
	// 4.  经度
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	  //经度    东经  Bit 7->0   西经 Bit 7 -> 1
	Original_info_Wr += 4;
	// 5.  高程
	Original_info[Original_info_Wr++]=(u8)(GPS_Hight<<8);
	Original_info[Original_info_Wr++]=(u8)GPS_Hight;
	// 6.  速度    0.1 Km/h
	Original_info[Original_info_Wr++]=(u8)(Speed_gps>>8);
	Original_info[Original_info_Wr++]=(u8)Speed_gps;   
	// 7. 方向   单位 1度
	Original_info[Original_info_Wr++]=(GPS_direction>>8);  //High 
	Original_info[Original_info_Wr++]=GPS_direction; // Low
	// 8.  日期时间	
	Original_info[Original_info_Wr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);		
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);	 

	
		//----------- 附加信息  ------------
    //  附加信息 1  -----------------------------    
	//	附加信息 ID
	Original_info[Original_info_Wr++]=0x03; // 行驶记录仪的速度
	//	附加信息长度
	Original_info[Original_info_Wr++]=2;
	//	类型
	Original_info[Original_info_Wr++]=(u8)(Speed_cacu>>8); 
	Original_info[Original_info_Wr++]=(u8)(Speed_cacu);	     
	//rt_kprintf("\r\n GPS速度=%d km/h , 传感器速度=%d km/h\r\n",Speed_gps,Speed_cacu); 
     //  附加信息 2  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0x01; // 车上的行驶里程
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=4; 
	 //  类型
	 Dis_01km=JT808Conf_struct.Distance_m_u32/100;
	 Original_info[Original_info_Wr++]=(Dis_01km>>24); 
	 Original_info[Original_info_Wr++]=(Dis_01km>>16); 
	 Original_info[Original_info_Wr++]=(Dis_01km>>8); 
	 Original_info[Original_info_Wr++]=Dis_01km; 
  
   
	//  附加信息 3 
	if(Warn_Status[1]&0x10)
   {
     //  附加信息 ID
     Original_info[Original_info_Wr++]=0x12; //  进出区域/路线报警
     //  附加信息长度 
     Original_info[Original_info_Wr++]=6;
	 //  类型
	 Original_info[Original_info_Wr++]=InOut_Object.TYPE;
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>24);
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>16);
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>8);
	 Original_info[Original_info_Wr++]=InOut_Object.ID;
	 Original_info[Original_info_Wr++]=InOut_Object.InOutState;  
	 rt_kprintf("\r\n ----- 0x0201 附加信息 \r\n");    
   }

    //  附件信息4
    if(Warn_Status[3]&0x02)
    {      
	  //  附加信息 ID
	  Original_info[Original_info_Wr++]=0x11; //  进出区域/路线报警
	  //  附加信息长度 
	  Original_info[Original_info_Wr++]=1; 
	  //  类型
	  Original_info[Original_info_Wr++]=0; //  无特定位置   

    }

    
	   //  附加信息 5  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0xFE; //信号强度
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=2; 
	 //  类型
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_SignalValue; 
	 Original_info[Original_info_Wr++]=0x00;  //  保留 

        //if(DispContent)
         //     printf("\r\n---- Satelitenum: %d , CSQ:%d\r\n",Satelite_num,ModuleSQ);  

        //  附加信息 6  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0xFF; //自定义模拟量上传
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=6; 
	 //  类型
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_1; 
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_2;  
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_0>>8);  // 模拟量 1
	 Original_info[Original_info_Wr++]=BD_EXT.AD_0;
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_1>>8);  // 模拟量 2
	 Original_info[Original_info_Wr++]=BD_EXT.AD_1;
	
 //  3. Send 
Protocol_End(Packet_Normal ,0);
 if(DispContent)
   rt_kprintf("\r\n	SEND GPS CMD=81H ! \r\n"); 
   	
 return true; 

}

u8 Paramater_0106_stuff(u32 cmdid, u8 *deststr)
{
    u8  reg_str[30];
	 u32  reg_u32=0;
	
   switch (cmdid)
   	{
		   //	参数列表
		case  MSG_0x0001:   
			  //  A.1 心跳包间隔   
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x01;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Heart_Dur>>24);   // 参数值 
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Heart_Dur>>16);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Heart_Dur>>8);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Heart_Dur);	 
			break;
	    case  MSG_0x0002:
			 //  A.2 TCP  消息应答间隔	 
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x02;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ACK_Dur>>24);	 // 参数值 
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ACK_Dur>>16);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ACK_Dur>>8);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ACK_Dur);	   
			  break;
	   case  MSG_0x0003:
			   //  A.3 TCP	消息重传次数   
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x03;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ReSD_Num>>24);   // 参数值 
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ReSD_Num>>16);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ReSD_Num>>8);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ReSD_Num); 		
	         break;
	   case  0x0004:
				  //  A.4	UDP 应答超时
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x04;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=(5>>24);	 // 参数值 
			Original_info[Original_info_Wr++]=(5>>16);
			Original_info[Original_info_Wr++]=(5>>8);
			Original_info[Original_info_Wr++]=(5);		
	         break;
	   case  0x0005:		   
				  //  A.5	UDP 重传次数
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x05;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.UDP_ReSD_Num>>24);   // 参数值 
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.UDP_ReSD_Num>>16);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.UDP_ReSD_Num>>8);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.UDP_ReSD_Num); 		
	        break;
	   case 0x0006:			   
				  //  A.6	SMS消息应答超时时间
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x06;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0;   // 参数值 
			Original_info[Original_info_Wr++]=0;
			Original_info[Original_info_Wr++]=0;
			Original_info[Original_info_Wr++]=5;		
	         break;
	   case  0x0007:    
	
				  //  A.7	SMS 消息重传次数
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x07;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0;   // 参数值 
			Original_info[Original_info_Wr++]=0;
			Original_info[Original_info_Wr++]=0;
			Original_info[Original_info_Wr++]=3;	
			break;
	   case  0x0010:
				 //   A.8  APN 字符串
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x10;
			Original_info[Original_info_Wr++]=strlen((const char*)APN_String); // 参数长度
			memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )APN_String,strlen((const char*)APN_String)); // 参数值
			Original_info_Wr+=strlen((const char*)APN_String);
	        break;
	   case 0x0011:		
				   //	A.9  APN 用户名
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x11;
			Original_info[Original_info_Wr++]=4; // 参数长度
			memcpy( ( char * ) Original_info+ Original_info_Wr, "user",4); // 参数值
			Original_info_Wr+=4;
	        break;
	   case 0x0012:		
				   //	A.10  APN 密码
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x12;
			Original_info[Original_info_Wr++]=4; // 参数长度
			memcpy( ( char * ) Original_info+ Original_info_Wr, "user",4); // 参数值
			Original_info_Wr+=4;
	        break;
       case 0x0013:			
				 //   A.11	 主服务器IP    
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x13;    
													 // 参数长度		
	
		   memset(reg_str,0,sizeof(reg_str));	  //  填写域名
		   memcpy(reg_str,"jt1.gghypt.net",strlen((char const*)"jt1.gghypt.net"));
		 // memcpy(reg_str,"fde.0132456.net",strlen((char const*)"jt1.gghypt.net"));
		   Original_info[Original_info_Wr++]=strlen((const char*)reg_str);
			  memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )reg_str,strlen((const char*)reg_str));	// 参数值	 
		   Original_info_Wr+=strlen((const char*)reg_str);
	       break;
	   case 0x0014:	   
					 //   A.12	 BAK  APN 字符串
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x14;
			Original_info[Original_info_Wr++]=strlen((const char*)"UNINET"); // 参数长度
			memcpy( ( char * ) Original_info+ Original_info_Wr,"UNINET",6); // 参数值
			Original_info_Wr+=6;
			break;
			
	  case 0x0015:
				   //	A.13 BAK APN 用户名
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x15;
			Original_info[Original_info_Wr++]=4; // 参数长度
			memcpy( ( char * ) Original_info+ Original_info_Wr, "user",4); // 参数值
			Original_info_Wr+=4;
			break;
	  case 0x0016:
				   //	A.14  BAK  APN 密码
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x16;
			Original_info[Original_info_Wr++]=4; // 参数长度
			memcpy( ( char * ) Original_info+ Original_info_Wr, "user",4); // 参数值
			Original_info_Wr+=4;
	        break;
     case 0x0017:			
						//	A.15   备用IP		 
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x17;    
													 // 参数长度				
		  memset(reg_str,0,sizeof(reg_str));	 //  填写域名
		  memcpy(reg_str,DomainNameStr_aux,strlen((char const*)DomainNameStr_aux));
		 // memcpy(reg_str,"fde.0132456.net",strlen((char const*)"jt1.gghypt.net"));
		   Original_info[Original_info_Wr++]=strlen((const char*)reg_str);
			  memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )reg_str,strlen((const char*)reg_str));	// 参数值	 
		   Original_info_Wr+=strlen((const char*)reg_str);
	      break;
			
     case 0x0018:			  
	
				 //  A.16  主服务TCP端口
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x18;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;   // 参数值
			Original_info[Original_info_Wr++]=0x00; 
			Original_info[Original_info_Wr++]=(7008>>8); 
			Original_info[Original_info_Wr++]=(u8)(7008); 
			break;
     case 0x0019:			
															
				//	A.17  备用服务TCP端口
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x19;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;   // 参数值
			Original_info[Original_info_Wr++]=0x00; 
			Original_info[Original_info_Wr++]=(7008>>8); 
			Original_info[Original_info_Wr++]=(u8)(7008); 
			break;

	case 0x001a:		
	
				 //  A.18  IC卡认证 主服务器地址
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x1A;
			Original_info[Original_info_Wr++]=0  ; // 参数长度
	
			//memcpy(( char * ) Original_info+ Original_info_Wr,"202.96.42.113",13);
			//Original_info_Wr+=13;
			 break;
	case 0x001b:
							//	A.19  IC卡认证 主服务器TCP
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x1B;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;   // 参数值
			Original_info[Original_info_Wr++]=0x00; 
			Original_info[Original_info_Wr++]=0x00; 
			Original_info[Original_info_Wr++]=0x00;  
			break;
	case 0x001c:
								//	A.20  IC卡认证 主服务器UDP
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x1C;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;   // 参数值
			Original_info[Original_info_Wr++]=0x00; 
			Original_info[Original_info_Wr++]=0x00; 
			Original_info[Original_info_Wr++]=0x00;  
			break;
	case 0x001d:		
	
					   //  A.21  IC卡认证  备用服务器地址
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes  
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x1D;
			Original_info[Original_info_Wr++]=0  ; // 参数长度
	
			//memcpy(( char * ) Original_info+ Original_info_Wr,"202.96.42.114",13);
			//Original_info_Wr+=13;
	         break;
	case 0x0020:		
	
								 //  A.22  位置汇报策略
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x20;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;   // 参数值
			Original_info[Original_info_Wr++]=0x00; 
			Original_info[Original_info_Wr++]=0x00; 
			Original_info[Original_info_Wr++]=JT808Conf_struct.SD_MODE.Send_strategy; 
			break;
	case 0x0021:
															   //  A.23  位置汇报方案
		    Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
		    Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x21;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;   // 参数值
			Original_info[Original_info_Wr++]=0x00; 
			Original_info[Original_info_Wr++]=0x00; 
			Original_info[Original_info_Wr++]=JT808Conf_struct.PositionSd_Stratage; 
			break;
	case 0x0022:
												  //  A.24 驾驶员未登录 汇报间隔
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x22;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;   // 参数值
			Original_info[Original_info_Wr++]=0x00; 
			Original_info[Original_info_Wr++]=0x00; 
			Original_info[Original_info_Wr++]=30; 
			break;
	case 0x0027:		
	
													   //  A.25  休眠时汇报间隔
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x27;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			JT808Conf_struct.DURATION.Sleep_Dur=300;
			Original_info[Original_info_Wr++]=(300>>24);   // 参数值  
			Original_info[Original_info_Wr++]=(300>>16);
			Original_info[Original_info_Wr++]=(300>>8);
			Original_info[Original_info_Wr++]=(u8)(300);	
			break;
	case 0x0028:		 
											//	A.26 紧急报警时 
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x28;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;   // 参数值
			Original_info[Original_info_Wr++]=0x00; 
			Original_info[Original_info_Wr++]=0x00; 
			Original_info[Original_info_Wr++]=10; 
			break;		 
	case 0x0029:
			 //   A.27	  缺省时间上报间隔
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x29;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>24);	 // 参数值 
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>16);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>8);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur);	   
			break;
	case 0x002c:
				 //   A.28	  缺省距离上报间隔
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x2c;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			//JT808Conf_struct.DISTANCE.Defalut_DistDelta=500;
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>24);   // 参数值 
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>16);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>8);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta);	 
	        break;
	case 0x002d:			 
					 //   A.29	  驾驶员未登录间隔
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x2d;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>24);   // 参数值 
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>16);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>8);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta);	 
			break;
	case 0x002e:
	
					 //   A.30	休眠时汇报距离间隔
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x2e;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>24);   // 参数值 
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>16);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>8);
			Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta);	 
			break;
	case 0x002f:
						 //   A.31	  紧急报警时 定距离
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x2f;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=(200>>24);   // 参数值 
			Original_info[Original_info_Wr++]=(200>>16);
			Original_info[Original_info_Wr++]=(200>>8);
			Original_info[Original_info_Wr++]=(200);	 
			break;
	case 0x0030:
						  //   A.32 	拐点补传角度
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x30;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=(45>>24);   // 参数值 
			Original_info[Original_info_Wr++]=(45>>16);
			Original_info[Original_info_Wr++]=(45>>8);
			Original_info[Original_info_Wr++]=(45); 	
	       break;
	case 0x0031:
				//	 A.33	  电子围栏半径
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x31;
			Original_info[Original_info_Wr++]=2  ; // 参数长度
		  // 参数值 
			Original_info[Original_info_Wr++]=(500>>8);
			Original_info[Original_info_Wr++]=(u8)(500);	 
	        break;
	case 0x0040:
				 //   A.34	  设置求助号码
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x40;
			Original_info[Original_info_Wr++]=0  ; // 参数长度
			break;
	case 0x0041:
					//	 A.35	 复位电话号码
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x41;
			Original_info[Original_info_Wr++]=0  ; // 参数长度
			break;
	
	case 0x0042:
				 //   A.36	  恢复出厂设置电话号码
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x42;
			Original_info[Original_info_Wr++]=0  ; // 参数长度
	         break;
	case 0x0043:
					   //  A37	监控平台SMS 短息号码
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x43;
			Original_info[Original_info_Wr++]=5  ; // 参数长度

	       
		   memcpy(( char * ) Original_info+ Original_info_Wr,JT808Conf_struct.SMS_RXNum,strlen((const char*)JT808Conf_struct.SMS_RXNum)); 
		   Original_info_Wr+=strlen(JT808Conf_struct.SMS_RXNum); 
	        break;
    case 0x0044:	
					
				 //   A.38	  sms 短息报警号码
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x44;
			Original_info[Original_info_Wr++]=0  ; // 参数长度
	        break;
	case 0x0045:
							//	 A.39	接听策略
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x45;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
				  Original_info[Original_info_Wr++]=1;	// ACC on 自动接听
	        break;
	case 0x0046:
	
								   //	A.40	 每次通话时长
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x46;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=(120>>24);   // 参数值 
			Original_info[Original_info_Wr++]=(120>>16);
			Original_info[Original_info_Wr++]=(120>>8);
			Original_info[Original_info_Wr++]=(120);  
			break;
	case 0x0047:
	
								  //   A.41    每月通话时长
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x47;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=(36000>>24);	 // 参数值 
			Original_info[Original_info_Wr++]=(36000>>16);
			Original_info[Original_info_Wr++]=(36000>>8);
			Original_info[Original_info_Wr++]=(36000);	 
			break;
	case 0x0048:
	
					 //   A42	 设置监听号码
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x48;
			Original_info[Original_info_Wr++]=0  ; // 参数长度
	        break;
	case 0x0049:
			  //   A.43    平台监管特权号码
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x49;
			Original_info[Original_info_Wr++]=0  ; // 参数长度
	         break;
	case 0x0050:
	
			//	 A.44	 报警屏蔽字
		            Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x50;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=Warn_MaskWord[0];
		Original_info[Original_info_Wr++]=Warn_MaskWord[1];
		Original_info[Original_info_Wr++]=Warn_MaskWord[2];
		Original_info[Original_info_Wr++]=Warn_MaskWord[3]; 
	        break;
	case 0x0051:
	
		  //   A.45  报警	发送Sms  开关
				Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				Original_info[Original_info_Wr++]=0x00;
		  Original_info[Original_info_Wr++]=0x00;
		  Original_info[Original_info_Wr++]=0x51;
				  Original_info[Original_info_Wr++]=4  ; // 参数长度
		  Original_info[Original_info_Wr++]=Text_MaskWord[0];
		  Original_info[Original_info_Wr++]=Text_MaskWord[1];
		  Original_info[Original_info_Wr++]=Text_MaskWord[2];
		  Original_info[Original_info_Wr++]=Text_MaskWord[3]; 
			break;
	case 0x0052:
	
	 //   A.46	  报警拍照开关
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x52;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x01;
			break;
	case 0x0053:
	
	 //   A.47	报警拍照存储标志
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x53;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			break;
	case 0x0054:
	
	 //   A.48	  关键报警标志
		   Original_info[Original_info_Wr++]=0x00;	 // 参数ID 4Bytes
		   Original_info[Original_info_Wr++]=0x00;
	 Original_info[Original_info_Wr++]=0x00;
	 Original_info[Original_info_Wr++]=0x54;
	 Original_info[Original_info_Wr++]=4  ; // 参数长度
	 Original_info[Original_info_Wr++]=Key_MaskWord[0];
	 Original_info[Original_info_Wr++]=Key_MaskWord[1];
	 Original_info[Original_info_Wr++]=Key_MaskWord[2];
	 Original_info[Original_info_Wr++]=Key_MaskWord[3];
			break;
	case 0x0055:
			
	   //	A.49   最大速度门限
			Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x55;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			JT808Conf_struct.Speed_warn_MAX=100;
			Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>24);	// 参数值
			Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>16);  
			Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>8);
			Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX);
			break;
	case 0x0056:
	
			   //	A.50	 超速持续时间
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x56;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=JT808Conf_struct.Spd_Exd_LimitSeconds;
			break;
	case 0x0057:
			
			 //  A.51	连续驾驶门限
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x57;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>24);	 // 参数值
			Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>16);	
			Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>8);
			Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec);
			break;
	case 0x0058:
	
			 //  A.52  当天累计驾驶门限
				 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
				 Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x58;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec>>24);	 // 参数值
			Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec>>16);	
			Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec>>8);
			Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec);
			break;
	case 0x0059:
	
			 //   A.53	 最小休息时间
			 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
			 Original_info[Original_info_Wr++]=0x00;
			 Original_info[Original_info_Wr++]=0x00;
			 Original_info[Original_info_Wr++]=0x59;
			 Original_info[Original_info_Wr++]=4  ; // 参数长度
			 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>24);	 // 参数值
			 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>16);	
			 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>8);
			 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec);		
			 break;
	case 0x005A:
	
						 //   A.54	最长停车时间
			 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
			 Original_info[Original_info_Wr++]=0x00;
			 Original_info[Original_info_Wr++]=0x00;
			 Original_info[Original_info_Wr++]=0x5A;
			 Original_info[Original_info_Wr++]=4  ; // 参数长度
			 TiredConf_struct.TiredDoor.Door_MaxParkingSec=3600;
			 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MaxParkingSec>>24);  // 参数值
			 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MaxParkingSec>>16); 
			 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MaxParkingSec>>8);
			 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MaxParkingSec); 	
			 break;
	case 0x005B:
	
						  //   A.55  超速报警预警差值
			 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
			 Original_info[Original_info_Wr++]=0x00;
			 Original_info[Original_info_Wr++]=0x00;
			 Original_info[Original_info_Wr++]=0x5B;
			 Original_info[Original_info_Wr++]=2  ; // 参数长度
			// 参数值
			 Original_info[Original_info_Wr++]=(JT808Conf_struct.BD_MaxSpd_preWarnValue>>8); // 100
			 Original_info[Original_info_Wr++]=(JT808Conf_struct.BD_MaxSpd_preWarnValue);	
			 break;
	case 0x005C:
	
	
						   //	A.56  疲劳驾驶预警差值
			 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
			 Original_info[Original_info_Wr++]=0x00;
			 Original_info[Original_info_Wr++]=0x00;
			 Original_info[Original_info_Wr++]=0x5C;
			 Original_info[Original_info_Wr++]=2  ; // 参数长度
			// 参数值
			 Original_info[Original_info_Wr++]=(JT808Conf_struct.BD_TiredDrv_preWarnValue>>8);
			 Original_info[Original_info_Wr++]=(JT808Conf_struct.BD_TiredDrv_preWarnValue);		
			 break;
	case 0x005D:
	
						//	 A.57  碰撞报警参数设置
			 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
			 Original_info[Original_info_Wr++]=0x00;
			 Original_info[Original_info_Wr++]=0x00;
			 Original_info[Original_info_Wr++]=0x5D;
			 Original_info[Original_info_Wr++]=2  ; // 参数长度
			// 参数值
			 Original_info[Original_info_Wr++]=(17924>>8);
			 Original_info[Original_info_Wr++]=(17924); 	
			 break;
	case 0x005E:	
						//	 A.58  侧翻报警参数设置
			 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
			 Original_info[Original_info_Wr++]=0x00;
			 Original_info[Original_info_Wr++]=0x00;
			 Original_info[Original_info_Wr++]=0x5E;
			 Original_info[Original_info_Wr++]=2  ; // 参数长度
			// 参数值
			 Original_info[Original_info_Wr++]=(30>>8);
			 Original_info[Original_info_Wr++]=(30);	
			 break;
	case 0x0064:	
	 //   A.59	 定时拍照控制
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x64;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			break;
	case 0x0065:
	
	 //   A.60	定距拍照控制
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x65;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			break;
	case 0x0070:
	
	 //   A.61 图像、视频质量
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x70;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x05;
			break;
	case 0x0071:
	
	 //   A.62	亮度
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x71;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=127;
			break;
	case 0x0072:
	
	 //   A.63	 对比度
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x72;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=64;
			break;
	case 0x0073:
	
	
	 //   A.64	饱和度
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x73;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=64;
			break;
	case 0x0074:
	
	
	 //   A.65	色度
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x74;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=127;
			break;
	case 0x0080:
	
	 //   A.66	 车辆里程表读数
		   Original_info[Original_info_Wr++]=0x00;	 // 参数ID 4Bytes
		   Original_info[Original_info_Wr++]=0x00;
	 Original_info[Original_info_Wr++]=0x00;
	 Original_info[Original_info_Wr++]=0x64;
	 Original_info[Original_info_Wr++]=4  ; // 参数长度
	 reg_u32=JT808Conf_struct.Distance_m_u32/100;
	 Original_info[Original_info_Wr++]=(reg_u32>>24);	 // 参数值
	 Original_info[Original_info_Wr++]=(reg_u32>>16);	
	 Original_info[Original_info_Wr++]=(reg_u32>>8);
	 Original_info[Original_info_Wr++]=(reg_u32);		  
			break;
	case 0x0081:
	
	
	
	 //   A.67	 车辆所在省域
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x81;
			Original_info[Original_info_Wr++]=2  ; // 参数长度
				 Original_info[Original_info_Wr++]=(u8)(10>>8);
				  Original_info[Original_info_Wr++]=(u8)10;
				  break;
	case 0x0082:
	
	//	 A.68	车辆所在市域
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x82;
			Original_info[Original_info_Wr++]=2  ; // 参数长?
			   //  county  ID
				  Original_info[Original_info_Wr++]=(u8)(1010>>8);
				  Original_info[Original_info_Wr++]=(u8)1010;	
				  break;
	case 0x0083:
	
														
		   //	A.69   车牌号
			Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x83;
			Original_info[Original_info_Wr++]=strlen((const char*)Vechicle_Info.Vech_Num); // 参数长度
			memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Vechicle_Info.Vech_Num,strlen((const char*)Vechicle_Info.Vech_Num) ); // 参数值
			Original_info_Wr+=strlen((const char*)Vechicle_Info.Vech_Num); 
			break;
	case 0x0084:
			
	//	 A.70	车辆颜色
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x84;
			Original_info[Original_info_Wr++]=1  ; // 参数长?
			Original_info[Original_info_Wr++]=Vechicle_Info.Dev_Color;
			break;
	case 0x0090:
	
	//	 A.71	GNSS  模式设置
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x90;
			Original_info[Original_info_Wr++]=1  ; // 参数长?
	
		   if( GpsStatus.Position_Moule_Status==1)
				 Original_info[Original_info_Wr++]=2;
		   
		   if( GpsStatus.Position_Moule_Status==2)
			  Original_info[Original_info_Wr++]=1;	 
		   
		   if( GpsStatus.Position_Moule_Status==3) 
			  Original_info[Original_info_Wr++]=3;	   
		   break;
	case 0x0091:
	
	//	 A.72	GNSS 波特率设置
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x91;
			Original_info[Original_info_Wr++]=1  ; // 参数长?
			Original_info[Original_info_Wr++]=2;
			break;
	case 0x0092:
	
	//	 A.73	GNSS nmea  输出更新频率
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x92;
			Original_info[Original_info_Wr++]=1  ; // 参数长?
			Original_info[Original_info_Wr++]=1;
			break;
	case 0x0093:
	
	 //   A74	车辆里程表读数
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x93;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=1;
			break;
	case 0x0094:
	
	//	 A.75  GNSS定位上传方式
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x94;
			Original_info[Original_info_Wr++]=1  ; // 参数长?
			Original_info[Original_info_Wr++]=0;
			break;
	case 0x0095:
	
	 //   A76	模块详细数据上传设置
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x95;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0;
			break;
	case 0x0100:
	
	 //   A77	CAN1   采集间隔
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x01;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			break;
	case 0x0101:
	
	 //   A78	CAN1   上传间隔
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x01;
			Original_info[Original_info_Wr++]=0x01;
			Original_info[Original_info_Wr++]=2  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			break;
	case 0x0102:
	
	 //   A79	 CAN2	采集间隔
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x01;
			Original_info[Original_info_Wr++]=0x02;
			Original_info[Original_info_Wr++]=4  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0;		
			break;
	case 0x0103:
	
	 //   A80  CAN2   上传间隔
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x01;
			Original_info[Original_info_Wr++]=0x03;
			Original_info[Original_info_Wr++]=2  ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
	        break;
	case 0x0110:
	
	
	 //   A81  CAN	总线ID 单独采集设置
				  Original_info[Original_info_Wr++]=0x00;	// 参数ID 4Bytes
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x01;
			Original_info[Original_info_Wr++]=0x10;
			Original_info[Original_info_Wr++]=8 ; // 参数长度
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0;
				  Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0x00;
			Original_info[Original_info_Wr++]=0;		 
			break;
	 default:
	 	    break;
   	}

}

//-----------------------------------------------------------------------
u8  Sub_stuff_AppointedPram_0106(void)
{    // 7E81060005013901234505004F0100000021B77E
     u16  i=0,len=0;
	
  
     //  1. Head     
      if(!Protocol_Head(MSG_0x0104,Packet_Normal)) 
 	  return false; // 终端参数上传

           //   float ID
	    Original_info[Original_info_Wr++]=(u8)(Centre_FloatID>>8);
	    Original_info[Original_info_Wr++]=(u8)Centre_FloatID; 
     //   参数个数	
	    Original_info[Original_info_Wr++]=Setting_Qry.Num_pram;

      for(i=0;i<Setting_Qry.Num_pram;i++)
	   {    
	   	    len=Paramater_0106_stuff(Setting_Qry.List_pram[i],Original_info);   //    填写信息	   	    
	   	    rt_kprintf("\r\n SendID:  %X  i=%d ",Setting_Qry.List_pram[i],i);     
            //  Original_info_Wr+=len;
       }
       
	// Paramater_0106_stuff(CMD_U32ID,Original_info);	//	  填写信息
	// rt_kprintf("\r\n SendID:  %4X ",CMD_U32ID);	   
	    
		  //  3. Send 
  Protocol_End(Packet_Normal ,0);
  if(DispContent)
	rt_kprintf("\r\n	发送单个参数查询信息! \r\n");    

  return true;
	 
}
u8  Stuff_SettingPram_0104H(void)
{
   u8  reg_str[30];
   u32  reg_u32=0;
  
  //  1. Head     
  if(!Protocol_Head(MSG_0x0104,Packet_Normal)) 
 	  return false; // 终端参数上传

  //  2. content 
       //   float ID
	    Original_info[Original_info_Wr++]=(u8)(Centre_FloatID>>8);
	    Original_info[Original_info_Wr++]=(u8)Centre_FloatID;
	   //   参数个数	
	    Original_info[Original_info_Wr++]=81;
	   //   参数列表
          //  A.1 心跳包间隔   
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x01;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Heart_Dur>>24);   // 参数值 
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Heart_Dur>>16);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Heart_Dur>>8);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Heart_Dur);     
         //  A.2 TCP  消息应答间隔   
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x02;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ACK_Dur>>24);   // 参数值 
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ACK_Dur>>16);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ACK_Dur>>8);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ACK_Dur);     

	       //  A.3 TCP  消息重传次数   
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x03;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ReSD_Num>>24);   // 参数值 
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ReSD_Num>>16);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ReSD_Num>>8);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.TCP_ReSD_Num);     	

              //  A.4   UDP 应答超时
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x04;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(5>>24);   // 参数值 
		Original_info[Original_info_Wr++]=(5>>16);
		Original_info[Original_info_Wr++]=(5>>8);
		Original_info[Original_info_Wr++]=(5);     	

           
              //  A.5   UDP 重传次数
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x05;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.UDP_ReSD_Num>>24);   // 参数值 
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.UDP_ReSD_Num>>16);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.UDP_ReSD_Num>>8);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.UDP_ReSD_Num);     	

               
              //  A.6   SMS消息应答超时时间
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x06;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0;   // 参数值 
		Original_info[Original_info_Wr++]=0;
		Original_info[Original_info_Wr++]=0;
		Original_info[Original_info_Wr++]=5;     	



              //  A.7   SMS 消息重传次数
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x07;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0;   // 参数值 
		Original_info[Original_info_Wr++]=0;
		Original_info[Original_info_Wr++]=0;
		Original_info[Original_info_Wr++]=3;     	

             //   A.8  APN 字符串
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x10;
		Original_info[Original_info_Wr++]=strlen((const char*)APN_String); // 参数长度
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )APN_String,strlen((const char*)APN_String)); // 参数值
		Original_info_Wr+=strlen((const char*)APN_String);

               //   A.9  APN 用户名
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x11;
		Original_info[Original_info_Wr++]=4; // 参数长度
		memcpy( ( char * ) Original_info+ Original_info_Wr, "user",4); // 参数值
		Original_info_Wr+=4;

               //   A.10  APN 密码
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x12;
		Original_info[Original_info_Wr++]=4; // 参数长度
		memcpy( ( char * ) Original_info+ Original_info_Wr, "user",4); // 参数值
		Original_info_Wr+=4;

             //   A.11   主服务器IP    
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x13;    
		                                         // 参数长度		

	   memset(reg_str,0,sizeof(reg_str));     //  填写域名
	   memcpy(reg_str,"jt1.gghypt.net",strlen((char const*)"jt1.gghypt.net"));
	 // memcpy(reg_str,"fde.0132456.net",strlen((char const*)"jt1.gghypt.net"));
	   Original_info[Original_info_Wr++]=strlen((const char*)reg_str);
          memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )reg_str,strlen((const char*)reg_str));  // 参数值	 
	   Original_info_Wr+=strlen((const char*)reg_str);

                 //   A.12   BAK  APN 字符串
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x14;
		Original_info[Original_info_Wr++]=strlen((const char*)"UNINET"); // 参数长度
		memcpy( ( char * ) Original_info+ Original_info_Wr,"UNINET",6); // 参数值
		Original_info_Wr+=6;

               //   A.13 BAK APN 用户名
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x15;
		Original_info[Original_info_Wr++]=4; // 参数长度
		memcpy( ( char * ) Original_info+ Original_info_Wr, "user",4); // 参数值
		Original_info_Wr+=4;

               //   A.14  BAK  APN 密码
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x16;
		Original_info[Original_info_Wr++]=4; // 参数长度
		memcpy( ( char * ) Original_info+ Original_info_Wr, "user",4); // 参数值
		Original_info_Wr+=4;

                    //  A.15   备用IP		 
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x17;    
		                                         // 参数长度		 	    
	  memset(reg_str,0,sizeof(reg_str));     //  填写域名
	  memcpy(reg_str,DomainNameStr_aux,strlen((char const*)DomainNameStr_aux));
	 // memcpy(reg_str,"fde.0132456.net",strlen((char const*)"jt1.gghypt.net"));
	   Original_info[Original_info_Wr++]=strlen((const char*)reg_str);
          memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )reg_str,strlen((const char*)reg_str));  // 参数值	 
	   Original_info_Wr+=strlen((const char*)reg_str);

		
          

             //  A.16  主服务TCP端口
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x18;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;   // 参数值
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=(7008>>8); 
		Original_info[Original_info_Wr++]=(u8)7008; 
													    
            //  A.17  备用服务TCP端口
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x19;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;   // 参数值
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=(7008>>8); 
		Original_info[Original_info_Wr++]=(u8)(7008); 

             //  A.18  IC卡认证 主服务器地址
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x1A;
		Original_info[Original_info_Wr++]=0  ; // 参数长度

		//memcpy(( char * ) Original_info+ Original_info_Wr,"202.96.42.113",13);
		//Original_info_Wr+=13;

                        //  A.19  IC卡认证 主服务器TCP
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x1B;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;   // 参数值
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=0x00;  

                            //  A.20  IC卡认证 主服务器UDP
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x1C;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;   // 参数值
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=0x00;  

                   //  A.21  IC卡认证  备用服务器地址
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes  
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x1D;
		Original_info[Original_info_Wr++]=0  ; // 参数长度

		//memcpy(( char * ) Original_info+ Original_info_Wr,"202.96.42.114",13);
		//Original_info_Wr+=13;


                             //  A.22  位置汇报策略
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x20;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;   // 参数值
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=JT808Conf_struct.SD_MODE.Send_strategy; 

                                                           //  A.23  位置汇报方案
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x21;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;   // 参数值
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=JT808Conf_struct.PositionSd_Stratage; 

                                              //  A.24 驾驶员未登录 汇报间隔
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x22;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;   // 参数值
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=30; 

			                                       //  A.25  休眠时汇报间隔
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x27;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		JT808Conf_struct.DURATION.Sleep_Dur=300;
		Original_info[Original_info_Wr++]=(300>>24);   // 参数值  
		Original_info[Original_info_Wr++]=(300>>16);
		Original_info[Original_info_Wr++]=(300>>8);
		Original_info[Original_info_Wr++]=(u8)(300);     	
		 
                                        //  A.26 紧急报警时 
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x28;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;   // 参数值
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=10; 
                 

  		 //   A.27    缺省时间上报间隔
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x29;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>24);   // 参数值 
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>16);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>8);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur);     

    		 //   A.28    缺省距离上报间隔
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x2c;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		//JT808Conf_struct.DISTANCE.Defalut_DistDelta=500;
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>24);   // 参数值 
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>16);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>8);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta);     

             
             	 //   A.29    驾驶员未登录间隔
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x2d;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>24);   // 参数值 
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>16);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>8);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta);     


               	 //   A.30  休眠时汇报距离间隔
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x2e;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>24);   // 参数值 
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>16);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta>>8);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DISTANCE.Defalut_DistDelta);     

                 	 //   A.31    紧急报警时 定距离
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x2f;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(200>>24);   // 参数值 
		Original_info[Original_info_Wr++]=(200>>16);
		Original_info[Original_info_Wr++]=(200>>8);
		Original_info[Original_info_Wr++]=(200);     

                      //   A.32     拐点补传角度
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x30;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(45>>24);   // 参数值 
		Original_info[Original_info_Wr++]=(45>>16);
		Original_info[Original_info_Wr++]=(45>>8);
		Original_info[Original_info_Wr++]=(45);     


            //   A.33     电子围栏半径
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x31;
		Original_info[Original_info_Wr++]=2  ; // 参数长度
	  // 参数值 
		Original_info[Original_info_Wr++]=(500>>8);
		Original_info[Original_info_Wr++]=(u8)(500);     


             //   A.34    设置求助号码
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x40;
		Original_info[Original_info_Wr++]=0  ; // 参数长度

                //   A.35    复位电话号码
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x41;
		Original_info[Original_info_Wr++]=0  ; // 参数长度


             //   A.36    恢复出厂设置电话号码
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x42;
		Original_info[Original_info_Wr++]=0  ; // 参数长度


                   //  A37  监控平台SMS 短息号码
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x43;
		Original_info[Original_info_Wr++]=5  ; // 参数长度

		memcpy(( char * ) Original_info+ Original_info_Wr,JT808Conf_struct.SMS_RXNum,strlen((const char*)JT808Conf_struct.SMS_RXNum)); 
		Original_info_Wr+=strlen((const char*)JT808Conf_struct.SMS_RXNum); 


                
             //   A.38    sms 短息报警号码
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x44;
		Original_info[Original_info_Wr++]=0  ; // 参数长度


                        //   A.39   接听策略
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x45;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
              Original_info[Original_info_Wr++]=1;  // ACC on 自动接听



                               //   A.40     每次通话时长
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x46;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(120>>24);   // 参数值 
		Original_info[Original_info_Wr++]=(120>>16);
		Original_info[Original_info_Wr++]=(120>>8);
		Original_info[Original_info_Wr++]=(120);   


                              //   A.41    每月通话时长
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x47;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(36000>>24);   // 参数值 
		Original_info[Original_info_Wr++]=(36000>>16);
		Original_info[Original_info_Wr++]=(36000>>8);
		Original_info[Original_info_Wr++]=(36000);   


                 //   A42    设置监听号码
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x48;
		Original_info[Original_info_Wr++]=0  ; // 参数长度


          //   A.43    平台监管特权号码
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x49;
		Original_info[Original_info_Wr++]=0  ; // 参数长度


        //   A.44    报警屏蔽字
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x50;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=Warn_MaskWord[0];
		Original_info[Original_info_Wr++]=Warn_MaskWord[1];
		Original_info[Original_info_Wr++]=Warn_MaskWord[2];
		Original_info[Original_info_Wr++]=Warn_MaskWord[3]; 


      //   A.45  报警   发送Sms  开关
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x51;
             	Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=Text_MaskWord[0];
		Original_info[Original_info_Wr++]=Text_MaskWord[1];
		Original_info[Original_info_Wr++]=Text_MaskWord[2];
		Original_info[Original_info_Wr++]=Text_MaskWord[3];

 //   A.46    报警拍照开关
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x52;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x01;

 //   A.47  报警拍照存储标志
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x53;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;

 //   A.48    关键报警标志
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x54;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=Key_MaskWord[0];
		Original_info[Original_info_Wr++]=Key_MaskWord[1];
		Original_info[Original_info_Wr++]=Key_MaskWord[2];
		Original_info[Original_info_Wr++]=Key_MaskWord[3];
		
   //   A.49   最大速度门限
        Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x55;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		JT808Conf_struct.Speed_warn_MAX=100;
		Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>24);   // 参数值
		Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>16);  
		Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>8);
		Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX);

	       //   A.50     超速持续时间
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x56;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=JT808Conf_struct.Spd_Exd_LimitSeconds;
		
		 //  A.51   连续驾驶门限
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x57;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>24);   // 参数值
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>16);  
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>8);
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec);

		 //  A.52  当天累计驾驶门限
             Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
             Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x58;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec>>24);   // 参数值
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec>>16);  
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec>>8);
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec);



		 //   A.53   最小休息时间
		 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x59;
		 Original_info[Original_info_Wr++]=4  ; // 参数长度
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>24);	 // 参数值
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>16);	
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>8);
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec);         

             		 //   A.54  最长停车时间
		 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x5A;
		 Original_info[Original_info_Wr++]=4  ; // 参数长度
		 TiredConf_struct.TiredDoor.Door_MaxParkingSec=3600;
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MaxParkingSec>>24);	 // 参数值
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MaxParkingSec>>16);	
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MaxParkingSec>>8);
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MaxParkingSec);         

                      //   A.55  超速报警预警差值
		 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x5B;
		 Original_info[Original_info_Wr++]=2  ; // 参数长度
		// 参数值
 		 Original_info[Original_info_Wr++]=(JT808Conf_struct.BD_MaxSpd_preWarnValue>>8); // 100
		 Original_info[Original_info_Wr++]=(JT808Conf_struct.BD_MaxSpd_preWarnValue);          


                       //   A.56  疲劳驾驶预警差值
		 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x5C;
		 Original_info[Original_info_Wr++]=2  ; // 参数长度
		// 参数值
 		 Original_info[Original_info_Wr++]=(JT808Conf_struct.BD_TiredDrv_preWarnValue>>8);
		 Original_info[Original_info_Wr++]=(JT808Conf_struct.BD_TiredDrv_preWarnValue);          



                    //   A.57  碰撞报警参数设置
		 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x5D;
		 Original_info[Original_info_Wr++]=2  ; // 参数长度
		// 参数值
 		 Original_info[Original_info_Wr++]=(17924>>8);
		 Original_info[Original_info_Wr++]=(17924);          



                    //   A.58  侧翻报警参数设置
		 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x5E;
		 Original_info[Original_info_Wr++]=2  ; // 参数长度
		// 参数值
 		 Original_info[Original_info_Wr++]=(30>>8);
		 Original_info[Original_info_Wr++]=(30);          


 //   A.59   定时拍照控制
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x80;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		reg_u32=JT808Conf_struct.Distance_m_u32/100;
		Original_info[Original_info_Wr++]=(reg_u32>>24);	// 参数值
		Original_info[Original_info_Wr++]=(reg_u32>>16);   
		Original_info[Original_info_Wr++]=(reg_u32>>8);
		Original_info[Original_info_Wr++]=(reg_u32);		 

 //   A.60  定距拍照控制
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x65;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;

 //   A.61 图像、视频质量
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x70;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x05;

 //   A.62  亮度
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x71;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=127;

 //   A.63   对比度
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x72;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=64;


 //   A.64  饱和度
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x73;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=64;


 //   A.65  色度
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x74;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=127;

 //   A.66   车辆里程表读数
	   Original_info[Original_info_Wr++]=0x00;	 // 参数ID 4Bytes
	   Original_info[Original_info_Wr++]=0x00;
 Original_info[Original_info_Wr++]=0x00;
 Original_info[Original_info_Wr++]=0x64;
 Original_info[Original_info_Wr++]=4  ; // 参数长度
 reg_u32=JT808Conf_struct.Distance_m_u32/100;
 Original_info[Original_info_Wr++]=(reg_u32>>24);	 // 参数值
 Original_info[Original_info_Wr++]=(reg_u32>>16);	
 Original_info[Original_info_Wr++]=(reg_u32>>8);
 Original_info[Original_info_Wr++]=(reg_u32);		  



 //   A.67   车辆所在省域
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x81;
		Original_info[Original_info_Wr++]=2  ; // 参数长度
             Original_info[Original_info_Wr++]=(u8)(10>>8);
              Original_info[Original_info_Wr++]=(u8)10;

//   A.68   车辆所在市域
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x82;
		Original_info[Original_info_Wr++]=2  ; // 参数长?
	       //  county  ID
              Original_info[Original_info_Wr++]=(u8)(1010>>8);
              Original_info[Original_info_Wr++]=(u8)1010;   

													
	   //   A.69   车牌号
        Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x83;
		Original_info[Original_info_Wr++]=strlen((const char*)Vechicle_Info.Vech_Num); // 参数长度
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Vechicle_Info.Vech_Num,strlen((const char*)Vechicle_Info.Vech_Num) ); // 参数值
		Original_info_Wr+=strlen((const char*)Vechicle_Info.Vech_Num); 
        
//   A.70   车辆颜色
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x84;
		Original_info[Original_info_Wr++]=1  ; // 参数长?
		Original_info[Original_info_Wr++]=Vechicle_Info.Dev_Color;

//   A.71   GNSS  模式设置
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x90;
		Original_info[Original_info_Wr++]=1  ; // 参数长?

	   if( GpsStatus.Position_Moule_Status==1)
		     Original_info[Original_info_Wr++]=2;
	   
	   if( GpsStatus.Position_Moule_Status==2)
		  Original_info[Original_info_Wr++]=1;   
	   
	   if( GpsStatus.Position_Moule_Status==3) 
		  Original_info[Original_info_Wr++]=3;     

//   A.72   GNSS 波特率设置
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x91;
		Original_info[Original_info_Wr++]=1  ; // 参数长?
		Original_info[Original_info_Wr++]=2;

//   A.73   GNSS nmea  输出更新频率
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x92;
		Original_info[Original_info_Wr++]=1  ; // 参数长?
		Original_info[Original_info_Wr++]=1;

 //   A74   车辆里程表读数
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x93;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=1;

//   A.75  GNSS定位上传方式
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x94;
		Original_info[Original_info_Wr++]=1  ; // 参数长?
		Original_info[Original_info_Wr++]=0;

 //   A76   模块详细数据上传设置
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x95;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0;

 //   A77   CAN1   采集间隔
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x01;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;

 //   A78   CAN1   上传间隔
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x01;
		Original_info[Original_info_Wr++]=0x01;
		Original_info[Original_info_Wr++]=2  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;

 //   A79    CAN2   采集间隔
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x01;
		Original_info[Original_info_Wr++]=0x02;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0;		

 //   A80  CAN2   上传间隔
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x01;
		Original_info[Original_info_Wr++]=0x03;
		Original_info[Original_info_Wr++]=2  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;



 //   A81  CAN  总线ID 单独采集设置
              Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x01;
		Original_info[Original_info_Wr++]=0x10;
		Original_info[Original_info_Wr++]=8 ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0;
              Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0;		   
  //  3. Send 
  Protocol_End(Packet_Normal ,0);
  if(DispContent)
	rt_kprintf("\r\n	发送参数查询信息! \r\n");    

  return true;
}


u8  Stuff_DeviceAttribute_BD_0107H(void)
{
  //   u16   infoLen=0;
   // 1. Head
	if(!Protocol_Head(MSG_0x0107,Packet_Normal)) 
 	  return false; 
	 // 2. content   
       Original_info[Original_info_Wr++]=(ProductAttribute._1_DevType>>8); 
	   Original_info[Original_info_Wr++]=ProductAttribute._1_DevType;
       memcpy( ( char * ) Original_info+ Original_info_Wr,(u8*)"70420",5);
       Original_info_Wr+=5;	
       memcpy( ( char * ) Original_info+ Original_info_Wr,(u8*)ProductAttribute._3_Dev_TYPENUM,20);
       Original_info_Wr+=20;
	    memcpy( ( char * ) Original_info+ Original_info_Wr,(u8*)SimID_12D+5,7);
       Original_info_Wr+=7;	
	   memcpy( ( char * ) Original_info+ Original_info_Wr,(u8*)ProductAttribute._5_Sim_ICCID,10);
       Original_info_Wr+=10;   
	   
	Original_info[Original_info_Wr++]=ProductAttribute._6_HardwareVer_Len;   
       memcpy( ( char * ) Original_info+ Original_info_Wr,(u8*)ProductAttribute._7_HardwareVer,ProductAttribute._6_HardwareVer_Len);
       Original_info_Wr+=ProductAttribute._6_HardwareVer_Len;   
	   
	//Original_info[Original_info_Wr++]=ProductAttribute._8_SoftwareVer_len;      
	//memcpy( ( char * ) Original_info+ Original_info_Wr,(u8*)ProductAttribute._9_SoftwareVer,ProductAttribute._8_SoftwareVer_len);
	//Original_info_Wr+=ProductAttribute._8_SoftwareVer_len;

	Original_info[Original_info_Wr++]=ProductAttribute._10_FirmWareVer_len;      
	memcpy( ( char * ) Original_info+ Original_info_Wr,(u8*)ProductAttribute._11_FirmWare,ProductAttribute._10_FirmWareVer_len);
	Original_info_Wr+=ProductAttribute._10_FirmWareVer_len; 

	Original_info[Original_info_Wr++]=ProductAttribute._12_GNSSAttribute;
	Original_info[Original_info_Wr++]=ProductAttribute._13_ComModuleAttribute;

	 /*
	  infoLen=sizeof(ProductAttribute);
	  memcpy( ( char * ) Original_info+ Original_info_Wr,(u8*)&ProductAttribute,infoLen);
	  Original_info_Wr+=infoLen;	 
        */
	  
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	发送终端属性 \r\n");    

    return true;
}
//--------------------------------------------------------------------------
u8  Stuff_EventACK_0301H(void)      
{    
	// 1. Head
	if(!Protocol_Head(MSG_0x0301,Packet_Normal)) 
 	  return false; 
	 // 2. content   
	   Original_info[Original_info_Wr++]=EventObj.Event_ID;// 返回事件ID
	    
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	 事件结果返回  \r\n");    
	 return true; 

}

u8  Stuff_ASKACK_0302H(void)      
{  
  
	// 1. Head
	if(!Protocol_Head(MSG_0x0302,Packet_Normal)) 
 	  return false; 
	 // 2. content   
	   //  应答流水号
	   Original_info[Original_info_Wr++]=(ASK_Centre.ASK_floatID>>8);// 返回事件ID
	   Original_info[Original_info_Wr++]=ASK_Centre.ASK_floatID;
       Original_info[Original_info_Wr++]=ASK_Centre.ASK_answerID;	    
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	发送提问选择结果 \r\n");     
	 return true; 

}

u8  Stuff_MSGACK_0303H(void)      
{  
 
	// 1. Head
	if(!Protocol_Head(MSG_0x0303,Packet_Normal)) 
 	  return false; 
	 // 2. content   
	   //  应答流水号
	   Original_info[Original_info_Wr++]=MSG_BroadCast_Obj.INFO_TYPE; 
	   Original_info[Original_info_Wr++]=MSG_BroadCast_Obj.INFO_PlyCancel;  //  0  取消  1 点播 
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	点播取消回复  \r\n");     
	 return true;    

}


void  Recorder_init(void)
{
  Recode_Obj.Float_ID=0;     //  命令流水号
  Recode_Obj.CMD=0;     //  数据采集 
  Recode_Obj.SD_Data_Flag=0; //  发送返回数返回标志
  Recode_Obj.CountStep=0;  //  发送数据需要一步一步发送 
  Recode_Obj.timer=0;
  //--------- add on  5-4 
  Recode_Obj.Devide_Flag=0;//  需要分包上传标志位
  Recode_Obj.Total_pkt_num=0;   // 分包总包数
  Recode_Obj.Current_pkt_num=0; // 当前发送包数 从 1  开始
  Recode_Obj.fcs=0;
  Recode_Obj.Error=0;
  rt_kprintf( "\r\n 记录仪定时发送---- init"); 
}




u8  Stuff_ControlACK_0500H(void)   //   车辆控制应答
{  

 //  1. Head	
   if(!Protocol_Head(MSG_0x0500,Packet_Normal)) 
 	  return false; 
 // 2. content 
    //------------------------------- Stuff ----------------------------------------
    //   float ID                                                // 对应中心应答消息的流水号
	Original_info[Original_info_Wr++]=(u8)(Vech_Control.CMD_FloatID>>8);
	Original_info[Original_info_Wr++]=(u8)Vech_Control.CMD_FloatID;  

	// 1. 告警标志  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Warn_Status,4 );    
	Original_info_Wr += 4;
	// 2. 状态  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Car_Status,4 );   
	Original_info_Wr += 4;
	// 3.  纬度
    memcpy( ( char * ) Original_info+ Original_info_Wr,( char * )  Gps_Gprs.Latitude, 4 );//纬度   modify by nathan
	Original_info_Wr += 4;
	// 4.  经度
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	  //经度    东经  Bit 7->0   西经 Bit 7 -> 1
	Original_info_Wr += 4;
	// 5.  高程
	Original_info[Original_info_Wr++]=(u8)(GPS_Hight<<8);
	Original_info[Original_info_Wr++]=(u8)GPS_Hight;
	// 6.  速度    0.1 Km/h
	Original_info[Original_info_Wr++]=(u8)(Speed_gps>>8);
	Original_info[Original_info_Wr++]=(u8)Speed_gps;   
	// 7. 方向   单位 1度
	Original_info[Original_info_Wr++]=(GPS_direction>>8);  //High 
	Original_info[Original_info_Wr++]=GPS_direction; // Low
	// 8.  日期时间	
	Original_info[Original_info_Wr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);		
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);	 

 //  3. Send 
 Protocol_End(Packet_Normal ,0);
 if(DispContent)
   rt_kprintf("\r\n	SEND  Vech  Control  ! \r\n"); 
   	
 return true; 

}

u8  Stuff_RecoderACK_0700H_Error(void) 
{
    
	u16  SregLen=0,Swr=0;//,Gwr=0; // S:serial	G: GPRS   	
	u8			Sfcs=0; 	
	u16  i=0;	
   
   //  1. Head
	   if( !Protocol_Head( MSG_0x0700, Packet_Normal ) )
	   {
		   return false;
	   }

        
		Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
		Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
		Original_info[Original_info_Wr++]	= Recode_Obj.CMD;	// 命令字
		Swr 								= Original_info_Wr;
		
		Original_info[Original_info_Wr++]	= 0x55; 			// 起始头
		Original_info[Original_info_Wr++]	= 0x7A;
		if(Recode_Obj.Error==1)
		   Original_info[Original_info_Wr++]	= 0xFA; 			//命令字
		else
		if(Recode_Obj.Error==2)	
		   Original_info[Original_info_Wr++]	= 0xFB; 			//命令字	
		Original_info[Original_info_Wr++]	= 0x00; 			// 保留字

     	//---------------  填写计算 A 协议	Serial Data   校验位  -------------------------------------

	Sfcs = 0;                            //  计算S校验 从Ox55 开始
	for( i = Swr; i < Original_info_Wr; i++ )
	{
		Sfcs ^= Original_info[i];
	}
	//Original_info[Original_info_Wr++] = Sfcs;               // 填写FCS

/*bitter:最后一包发送fcs*/


		Original_info[Original_info_Wr++] = Sfcs;               // 填写FCS
	

	//  3. Send
	Protocol_End( Packet_Normal, 0 );

	//  4.     如果是分包 判断结束
  return true; 
}


u8  Stuff_RecoderACK_0700H( u8 PaketType )  //   行车记录仪数据上传
{  

   u16	SregLen=0,Swr=0;//,Gwr=0; // S:serial  G: GPRS   
//   u16      Reg_len_position=0;  
   u8	       Sfcs=0;     
   u16	i=0;   
   u32	regdis=0,reg2=0;   
//   u8   	Reg[70];  
//   u8       QueryRecNum=0;  // 查询记录数目
   u16    stuff_len=0;

	//  1. Head
	if( !Protocol_Head( MSG_0x0700, PaketType ) )
	{
		return false;
	}

	switch( Recode_Obj.CMD )
	{
		case   0x00:                                                //  执行标准版本年号
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;   // 命令字
			Swr									= Original_info_Wr;

			Original_info[Original_info_Wr++]	= 0x55;             // 起始头
			Original_info[Original_info_Wr++]	= 0x7A;
			Original_info[Original_info_Wr++]	= 0x00;             //命令字
			SregLen								= 0x02;             // 信息长度
			Original_info[Original_info_Wr++]	= 0x00;             // Hi
			Original_info[Original_info_Wr++]	= 2;                // Lo

			Original_info[Original_info_Wr++]	= 0x00;             // 保留字
			Original_info[Original_info_Wr++]	= 0x12;             //  12  年标准 
			Original_info[Original_info_Wr++]	= 0x00;

			break;
		//---------------- 上传数类型  -------------------------------------
		case 0x01:
			//  01  当前驾驶员代码及对应的机动车驾驶证号
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;               // 命令字
			Swr									= Original_info_Wr;

			Original_info[Original_info_Wr++]	= 0x55;                         // 起始头
			Original_info[Original_info_Wr++]	= 0x7A;
			Original_info[Original_info_Wr++]	= 0x01;                         //命令字

			SregLen								= 0x00;                         // 信息长度
			Original_info[Original_info_Wr++]	= 0x00;                         // Hi
			Original_info[Original_info_Wr++]	= 18;                           // Lo

			Original_info[Original_info_Wr++] = 0x00;                           // 保留字

            /*
			memcpy( Original_info + Original_info_Wr, "120221123456789", 15 );  //信息内容
			Original_info_Wr					+= 15;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			*/

             memcpy( Original_info + Original_info_Wr,JT808Conf_struct.Driver_Info.DriverCard_ID, 18 );  //信息内容
			 Original_info_Wr					+= 18;			
			break;
		case 0x02:                                                              //  02  采集记录仪的实时时钟
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;               // 命令字
			Swr									= Original_info_Wr;

			Original_info[Original_info_Wr++]	= 0x55;                         // 起始头
			Original_info[Original_info_Wr++]	= 0x7A;
			Original_info[Original_info_Wr++]	= 0x02;                         //命令字

			SregLen								= 0x00;                         // 信息长度
			Original_info[Original_info_Wr++]	= 0x00;                         // Hi
			Original_info[Original_info_Wr++]	= 6;                            // Lo

			Original_info[Original_info_Wr++] = 0x00;                           // 保留字

             time_now=Get_RTC();     
			Time2BCD( Original_info + Original_info_Wr );                       //记录仪时间
			Original_info_Wr += 6;
			break;
		case 0x03:                                                              // 03 采集360h内里程
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;               // 命令字
			Swr									= Original_info_Wr;

			Original_info[Original_info_Wr++]	= 0x55;                         // 起始头
			Original_info[Original_info_Wr++]	= 0x7A;
			Original_info[Original_info_Wr++]	= 0x03;                         //命令字

			SregLen								= 0x00;                         // 信息长度
			Original_info[Original_info_Wr++]	= 0x00;                         // Hi
			Original_info[Original_info_Wr++]	= 20;                           // Lo

			Original_info[Original_info_Wr++] = 0x00;                           // 保留字
			//	信息内容
			Time2BCD( Original_info + Original_info_Wr );                       // 记录仪实时时间
			Original_info_Wr					+= 6;
			Original_info[Original_info_Wr++]	= JT808Conf_struct.FirstSetupDate[0];                         // 初次安装时间
			Original_info[Original_info_Wr++]	= JT808Conf_struct.FirstSetupDate[1];
			Original_info[Original_info_Wr++]	= JT808Conf_struct.FirstSetupDate[2];
			Original_info[Original_info_Wr++]	= JT808Conf_struct.FirstSetupDate[3];
			Original_info[Original_info_Wr++]	= JT808Conf_struct.FirstSetupDate[4];
			Original_info[Original_info_Wr++]	= JT808Conf_struct.FirstSetupDate[5];
			//--- 初始里程
			Original_info[Original_info_Wr++]	= chushilicheng[0];
			Original_info[Original_info_Wr++]	= chushilicheng[1];
			Original_info[Original_info_Wr++]	= chushilicheng[2];
			Original_info[Original_info_Wr++]	= chushilicheng[3];

			/*Original_info[Original_info_Wr++]	= leijilicheng[0];
			Original_info[Original_info_Wr++]	= leijilicheng[1];
			Original_info[Original_info_Wr++]	= leijilicheng[2];
			Original_info[Original_info_Wr++]	= leijilicheng[3]; */
			// -- 累积里程 3个字节 单位0.1km    6位
			regdis								= JT808Conf_struct.Distance_m_u32 / 100; //单位0.1km
			reg2								= regdis / 100000;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= ( reg2 << 4 ) + ( regdis % 100000 / 10000 );
			Original_info[Original_info_Wr++]	= ( ( regdis % 10000 / 1000 ) << 4 ) + ( regdis % 1000 / 100 );
			Original_info[Original_info_Wr++]	= ( ( regdis % 100 / 10 ) << 4 ) + ( regdis % 10 );
			
			break;

		case 0x04:                                                                  // 04  采集记录仪脉冲系数
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;                   // 命令字
			Swr									= Original_info_Wr;
			Original_info[Original_info_Wr++]	= 0x55;                             // 起始头
			Original_info[Original_info_Wr++]	= 0x7A;

			Original_info[Original_info_Wr++] = 0x04;                               //命令字

			SregLen								= 0x00;                             // 信息长度
			Original_info[Original_info_Wr++]	= 0x00;                             // Hi
			Original_info[Original_info_Wr++]	= 8;                                // Lo

			Original_info[Original_info_Wr++] = 0x00;                               // 保留字

			//  信息内容
			Time2BCD( Original_info + Original_info_Wr );                           // 记录仪实时时间
			Original_info_Wr					+= 6;
			Original_info[Original_info_Wr++]	= (u8)( JT808Conf_struct.Vech_Character_Value >> 8 );
			Original_info[Original_info_Wr++]	= (u8)( JT808Conf_struct.Vech_Character_Value );
			break;
		case 0x05:                                                                  //05      车辆信息采集
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;                   // 命令字
			Swr									= Original_info_Wr;
			Original_info[Original_info_Wr++]	= 0x55;                             // 起始头
			Original_info[Original_info_Wr++]	= 0x7A;

			Original_info[Original_info_Wr++] = 0x05;                               //命令字

			SregLen								= 41;                               // 信息长度
			Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 );             // Hi
			Original_info[Original_info_Wr++]	= (u8)SregLen;                      // Lo	  65x7

			Original_info[Original_info_Wr++] = 0x00;                               // 保留字
			//-----------	信息内容  --------------
			//memcpy( Original_info + Original_info_Wr, "LGBK22E70AY100102", 17 );    //信息内容
			memcpy( Original_info + Original_info_Wr,Vechicle_Info.Vech_VIN,17);
			Original_info_Wr += 17;
			memcpy( Original_info + Original_info_Wr,Vechicle_Info.Vech_Num, 8 );               // 车牌号
			Original_info_Wr					+= 8;
			//Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			//车辆类型
			/*Original_info[Original_info_Wr++]	= 0x03; //小型车
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;*/
			/*
			memcpy( Original_info + Original_info_Wr, "小型车", 6 );   
			Original_info_Wr+=6;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;*/
			 memcpy(Original_info + Original_info_Wr,Vechicle_Info.Vech_Type,12); 
			Original_info_Wr+=12; 
			//---- 去掉了一个多余的字节
			break;
		case   0x06:                                                    // 06-09
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;       // 命令字
			Swr									= Original_info_Wr;
			//  06 信号配置信息
			Original_info[Original_info_Wr++]	= 0x55;                 // 起始头
			Original_info[Original_info_Wr++]	= 0x7A;

			Original_info[Original_info_Wr++] = 0x06;                   //命令字

			SregLen								= 87;                   // 信息长度
			Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 ); // Hi
			Original_info[Original_info_Wr++]	= (u8)SregLen;          // Lo

			Original_info[Original_info_Wr++] = 0x00;                   // 保留字

			Time2BCD( Original_info + Original_info_Wr );               //记录仪实时时间
			Original_info_Wr += 6;
			//-------  状态字个数----------------------
			Original_info[Original_info_Wr++] = 1;  
			//---------- 状态字内容-------------------


			/*
			   -------------------------------------------------------------
			          F4  行车记录仪 TW703   管脚定义
			   -------------------------------------------------------------
			   遵循  GB10956 (2012)  Page26  表A.12  规定
			   -------------------------------------------------------------
			 | Bit  |      Note       |  必备|   MCUpin  |   PCB pin  |   Colour | ADC
			   ------------------------------------------------------------
			    D7      刹车           *            PE11             9                棕
			    D6      左转灯     *             PE10            10               红
			    D5      右转灯     *             PC2              8                白
			    D4      远光灯     *             PC0              4                黑
			    D3      近光灯     *             PC1              5                黄
			    D2      雾灯          add          PC3              7                绿      *
			    D1      车门          add          PA1              6                灰      *
			    D0      预留
			 */
			/*memcpy( Original_info + Original_info_Wr, "预留      ", 10 );       // D0
			Original_info_Wr += 10;
			memcpy( Original_info + Original_info_Wr, "车门      ", 10 );       // D1
			Original_info_Wr += 10;
			memcpy( Original_info + Original_info_Wr, "雾灯      ", 10 );       // D2
			Original_info_Wr += 10;
			memcpy( Original_info + Original_info_Wr, "近光灯    ", 10 );       // D3
			Original_info_Wr += 10;
			memcpy( Original_info + Original_info_Wr, "远光灯    ", 10 );       // D4
			Original_info_Wr += 10;
			memcpy( Original_info + Original_info_Wr, "右转灯    ", 10 );       // D5
			Original_info_Wr += 10;
			memcpy( Original_info + Original_info_Wr, "左转灯    ", 10 );       // D6
			Original_info_Wr += 10;
			memcpy( Original_info + Original_info_Wr, "刹车      ", 10 );       // D7
			Original_info_Wr += 10;*/
			memcpy( Original_info + Original_info_Wr, Setting08, 80 );    
			Original_info_Wr += 80;
			break;

		case 0x07:                                                              //07  记录仪唯一编号
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;               // 命令字
			Swr									= Original_info_Wr;

			Original_info[Original_info_Wr++]	= 0x55;                         // 起始头
			Original_info[Original_info_Wr++]	= 0x7A;

			Original_info[Original_info_Wr++] = 0x07;                           //命令字

			SregLen								= 30;                           //206;		 // 信息长度
			Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 );         // Hi
			Original_info[Original_info_Wr++]	= (u8)SregLen;                  // Lo

			Original_info[Original_info_Wr++] = 0x00;                           // 保留字
			//------- 信息内容 ------
			memcpy( Original_info + Original_info_Wr, "7654321", 7 );           // 3C 认证代码
			Original_info_Wr += 7;
			memcpy( Original_info + Original_info_Wr, "TW703   TW703   ", 16 ); // 产品型号
			Original_info_Wr					+= 16;
			Original_info[Original_info_Wr++]	= 0x13;                         // 生产日期
			Original_info[Original_info_Wr++]	= 0x03;
			Original_info[Original_info_Wr++]	= 0x01;
			Original_info[Original_info_Wr++]	= 0x00;                         // 生产流水号
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= IMSI_CODE[14] - 0x30;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			//Original_info[Original_info_Wr++]	= 0x00;
			break;

		case 0x08:                                                              //  08   采集指定的行驶速度记录
			//if( ( PaketType == Packet_Divide ) && ( Recode_Obj.Current_pkt_num == 1 ) )
			{
				Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
				Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
				Original_info[Original_info_Wr++]	= Recode_Obj.CMD;           // 命令字
				Swr									= Original_info_Wr;

				Original_info[Original_info_Wr++]	= 0x55;                     // 起始头
				Original_info[Original_info_Wr++]	= 0x7A;
				Original_info[Original_info_Wr++]	= 0x08;                     //命令字

               	if(Vdr_Wr_Rd_Offset.V_08H_Write>5)
				       SregLen= 630;                      // 信息长度       630
				else
					   SregLen=Vdr_Wr_Rd_Offset.V_08H_Write*126;
				
				Original_info[Original_info_Wr++]	= SregLen >> 8;             // Hi
				Original_info[Original_info_Wr++]	= SregLen;                  // Lo

				Original_info[Original_info_Wr++] = 0x00;                       // 保留字
			}
			//	信息内容
			//WatchDog_Feed( );
			//get_08h( Original_info + Original_info_Wr );                        //126*5=630        num=576  packet

			if(Vdr_Wr_Rd_Offset.V_08H_Write>5)
				stuff_len=stuff_drvData(0x08,Vdr_Wr_Rd_Offset.V_08H_Write-5,5,Original_info + Original_info_Wr);  
			else
				stuff_len=stuff_drvData(0x08,0,Vdr_Wr_Rd_Offset.V_08H_Write,Original_info + Original_info_Wr); 

			Original_info_Wr += stuff_len; 
			//  后续需要分包处理  -----nate
			break;
		case   0x09:                                                            // 09   指定的位置信息记录
			//if( ( PaketType == Packet_Divide ) && ( Recode_Obj.Current_pkt_num == 1 ) )
			{
				Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
				Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
				Original_info[Original_info_Wr++]	= Recode_Obj.CMD;           // 命令字
				Swr									= Original_info_Wr;
				Original_info[Original_info_Wr++]	= 0x55;                     // 起始头
				Original_info[Original_info_Wr++]	= 0x7A;

				Original_info[Original_info_Wr++] = 0x09;                       //命令字

                if(Vdr_Wr_Rd_Offset.V_09H_Write>0)
				   SregLen								= 666;  //666 * 2;                  // 信息长度
				else
				   SregLen=0;
				Original_info[Original_info_Wr++]	= SregLen >> 8;             // Hi      666=0x29A
				Original_info[Original_info_Wr++]	= SregLen;                  // Lo

				//Original_info[Original_info_Wr++]=0;    // Hi      666=0x29A
				//Original_info[Original_info_Wr++]=0;	   // Lo

				Original_info[Original_info_Wr++] = 0x00;                       // 保留字
			}
			//	信息内容
			WatchDog_Feed( );

			if(Vdr_Wr_Rd_Offset.V_09H_Write>0)
			 {	
			      stuff_len=stuff_drvData(0x09,Vdr_Wr_Rd_Offset.V_09H_Write-1,1,Original_info + Original_info_Wr); 
			     Original_info_Wr += stuff_len;      
			 }
			 
			break;
		case   0x10:                                                            // 10-13     10   事故疑点采集记录
			//事故疑点数据
			//if( ( PaketType == Packet_Divide ) && ( Recode_Obj.Current_pkt_num == 1 ) )
			{
				Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
				Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
				Original_info[Original_info_Wr++]	= Recode_Obj.CMD;           // 命令字
				Swr									= Original_info_Wr;

				Original_info[Original_info_Wr++]	= 0x55;                     // 起始头
				Original_info[Original_info_Wr++]	= 0x7A;

				Original_info[Original_info_Wr++] = 0x10;                       //命令字

				if(Vdr_Wr_Rd_Offset.V_10H_Write>0)
							   SregLen					= 234 ;//*100;                //0		 // 信息长度
				else
							   SregLen=0;
				Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 );     // Hi
				Original_info[Original_info_Wr++]	= (u8)SregLen;              // Lo

				Original_info[Original_info_Wr++] = 0x00;                       // 保留字
			}
			//------- 信息内容 ------
			//WatchDog_Feed( );
			delay_ms( 3 );
			
			if(Vdr_Wr_Rd_Offset.V_10H_Write>0) 
			{
			   stuff_len=stuff_drvData(0x10,Vdr_Wr_Rd_Offset.V_10H_Write-1,1,Original_info + Original_info_Wr); 
			    Original_info_Wr += stuff_len;   
			}

			break;

		case  0x11:                                                             // 11 采集指定的的超时驾驶记录
			//if( ( PaketType == Packet_Divide ) && ( Recode_Obj.Current_pkt_num == 1 ) )
			{
				Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
				Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
				Original_info[Original_info_Wr++]	= Recode_Obj.CMD;           // 命令字
				Swr									= Original_info_Wr;

				Original_info[Original_info_Wr++]	= 0x55;                     // 起始头
				Original_info[Original_info_Wr++]	= 0x7A;

				Original_info[Original_info_Wr++] = 0x11;                       //命令字

			//	SregLen								= 50 * 10;                 // 信息长度
					if(Vdr_Wr_Rd_Offset.V_11H_Write>10)
				       SregLen= 500;                      // 信息长度      50*10
				else
					   SregLen=Vdr_Wr_Rd_Offset.V_11H_Write*50;
				Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 );     // Hi
				Original_info[Original_info_Wr++]	= (u8)SregLen;              // Lo    65x7

				Original_info[Original_info_Wr++] = 0x00;                       // 保留字
			}



			/*
			       每条 50 bytes  ，100 条    获取的是每10 条打一包  500 packet    Totalnum=10
			 */
			 WatchDog_Feed( );
			//get_11h( Original_info + Original_info_Wr );                        //50  packetsize      num=100
			//Original_info_Wr += 500; 

			if(Vdr_Wr_Rd_Offset.V_11H_Write>10)
				stuff_len=stuff_drvData(0x11,Vdr_Wr_Rd_Offset.V_11H_Write-10,10,Original_info + Original_info_Wr);  
			else
				stuff_len=stuff_drvData(0x11,0,Vdr_Wr_Rd_Offset.V_11H_Write,Original_info + Original_info_Wr); 

			Original_info_Wr += stuff_len; 
			break;
			
		case  0x12:                                                             // 12 采集指定驾驶人身份记录  ---Devide
			//if( ( PaketType == Packet_Divide ) && ( Recode_Obj.Current_pkt_num == 1 ) )
			{
				Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
				Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
				Original_info[Original_info_Wr++]	= Recode_Obj.CMD;           // 命令字
				Swr									= Original_info_Wr;

				Original_info[Original_info_Wr++]	= 0x55;                     // 起始头
				Original_info[Original_info_Wr++]	= 0x7A;

				Original_info[Original_info_Wr++] = 0x12;                       //命令字

			   if(Vdr_Wr_Rd_Offset.V_12H_Write>20)
				       SregLen= 500;                      // 信息长度      50*10
				else
					   SregLen=Vdr_Wr_Rd_Offset.V_12H_Write*25; 
				Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 );     // Hi
				Original_info[Original_info_Wr++]	= (u8)SregLen;              // Lo    65x7

				Original_info[Original_info_Wr++]  = 0x00;                      // 保留字
			}
			//------- 信息内容 ------


			/*
			        驾驶员身份登录记录  每条25 bytes      200条      20条一包 500 packetsize  totalnum=10
			 */
			 WatchDog_Feed( );
			if(Vdr_Wr_Rd_Offset.V_12H_Write>20)
				stuff_len=stuff_drvData(0x12,Vdr_Wr_Rd_Offset.V_12H_Write-20,20,Original_info + Original_info_Wr);  
			else
				stuff_len=stuff_drvData(0x12,0,Vdr_Wr_Rd_Offset.V_12H_Write,Original_info + Original_info_Wr); 

			Original_info_Wr += stuff_len; 
			break;
		case  0x13:                                                     // 13 采集记录仪外部供电记录
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;       // 命令字
			Swr									= Original_info_Wr;
			Original_info[Original_info_Wr++]	= 0x55;                 // 起始头
			Original_info[Original_info_Wr++]	= 0x7A;

			Original_info[Original_info_Wr++] = 0x13;                   //命令字

            
			//SregLen								= 700;                  // 信息长度
			if(Vdr_Wr_Rd_Offset.V_13H_Write>100)
				       SregLen= 700;                      // 信息长度      50*10
			else
					   SregLen=Vdr_Wr_Rd_Offset.V_13H_Write*7;
				
			Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 ); // Hi
			Original_info[Original_info_Wr++]	= (u8)SregLen;          // Lo    65x7

			Original_info[Original_info_Wr++] = 0x00;                   // 保留字
			//------- 信息内容 ------


			/*
			   外部供电记录   7 个字节  100 条       一个完整包
			 */
			 WatchDog_Feed( );
			//get_13h( Original_info + Original_info_Wr );
			//Original_info_Wr += 700;
			
			if(Vdr_Wr_Rd_Offset.V_13H_Write>100)
				stuff_len=stuff_drvData(0x13,Vdr_Wr_Rd_Offset.V_13H_Write-100,100,Original_info + Original_info_Wr);  
			else
				stuff_len=stuff_drvData(0x13,0,Vdr_Wr_Rd_Offset.V_13H_Write,Original_info + Original_info_Wr); 

			Original_info_Wr += stuff_len;   

			break;
		case   0x14:
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;       // 命令字
			Swr									= Original_info_Wr;
			// 14 记录仪参数修改记录
			Original_info[Original_info_Wr++]	= 0x55;                 // 起始头
			Original_info[Original_info_Wr++]	= 0x7A;

			Original_info[Original_info_Wr++] = 0x14;                   //命令字

			//SregLen								= 700;                  // 信息长度
            if(Vdr_Wr_Rd_Offset.V_14H_Write>100)
				       SregLen= 700;                      // 信息长度      50*10
			else
					   SregLen=Vdr_Wr_Rd_Offset.V_14H_Write*7; 
				 
			
			Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 ); // Hi
			Original_info[Original_info_Wr++]	= (u8)SregLen;          // Lo    65x7

			Original_info[Original_info_Wr++] = 0x00;                   // 保留字
			//------- 信息内容 ------


			/*
			       每条 7 个字节   100 条    1个完整包
			 */
			 WatchDog_Feed( );
			// get_14h( Original_info + Original_info_Wr );
			// Original_info_Wr += 700;
			
			
			if(Vdr_Wr_Rd_Offset.V_14H_Write>100)
				stuff_len=stuff_drvData(0x14,Vdr_Wr_Rd_Offset.V_14H_Write-100,100,Original_info + Original_info_Wr);  
			else
				stuff_len=stuff_drvData(0x14,0,Vdr_Wr_Rd_Offset.V_14H_Write,Original_info + Original_info_Wr); 

			Original_info_Wr += stuff_len;   
			
			break;

		case    0x15:                                                      // 15 采集指定的速度状态日志     --------Divde
			//if( ( PaketType == Packet_Divide ) && ( Recode_Obj.Current_pkt_num == 1 ) )
		//	if( ( Recode_Obj.Current_pkt_num == 1 ) ) 
			{
				Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
				Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
				Original_info[Original_info_Wr++]	= Recode_Obj.CMD;       // 命令字
				Swr									= Original_info_Wr;
				Original_info[Original_info_Wr++]	= 0x55;                 // 起始头
				Original_info[Original_info_Wr++]	= 0x7A;

				Original_info[Original_info_Wr++] = 0x15;                   //命令字

				//SregLen								= 133*2;             // 信息长度
				if(Vdr_Wr_Rd_Offset.V_15H_Write>2)
				       SregLen=266;                      // 信息长度      50*10
			    else
					   SregLen=Vdr_Wr_Rd_Offset.V_15H_Write*133;
				Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 ); // Hi
				Original_info[Original_info_Wr++]	= (u8)SregLen;          // Lo    65x7

				Original_info[Original_info_Wr++] = 0x00;                   // 保留字 
			}


			/*
			       每条 133 个字节   10 条    1个完整包     5*133=665        totalnum=2
			 */
			WatchDog_Feed( );
			//get_15h( Original_info + Original_info_Wr );
			//Original_info_Wr += 133;
            
			if(Vdr_Wr_Rd_Offset.V_15H_Write>2)
				stuff_len=stuff_drvData(0x15,Vdr_Wr_Rd_Offset.V_15H_Write-2,2,Original_info + Original_info_Wr);  
			else
				stuff_len=stuff_drvData(0x15,0,Vdr_Wr_Rd_Offset.V_15H_Write,Original_info + Original_info_Wr); 

			Original_info_Wr += stuff_len;   
			break;
		default:         
			
			break;
	}
	//---------------  填写计算 A 协议	Serial Data   校验位  -------------------------------------

	Sfcs = 0;                            //  计算S校验 从Ox55 开始
	for( i = Swr; i < Original_info_Wr; i++ )
	{
		Sfcs ^= Original_info[i];
	}
	//Original_info[Original_info_Wr++] = Sfcs;               // 填写FCS

/*bitter:最后一包发送fcs*/
#if 1
	if( PaketType == Packet_Divide )
	{
		Recode_Obj.fcs ^= Sfcs;
		if( Recode_Obj.Current_pkt_num == Recode_Obj.Total_pkt_num )
		{
			Original_info[Original_info_Wr++] = Recode_Obj.fcs; // 填写FCS
		}
	}else
	{
		Original_info[Original_info_Wr++] = Sfcs;               // 填写FCS
	}
#endif

	//  3. Send
	Protocol_End( PaketType, 0 );

	//  4.     如果是分包 判断结束
    if( Recode_Obj.Devide_Flag== 1 )   // 不给应答
	{
		if(Recode_Obj.Current_pkt_num >=Recode_Obj.Total_pkt_num )
		{
			Recorder_init( );           //  clear
		}else
		{
			Recode_Obj.Current_pkt_num++;
		}
	}

	if( DispContent )
	{
		rt_kprintf( "\r\n	SEND Recorder Data ! \r\n");
	}

	return true;
}


//------------------------------------------------------------------
u8  Stuff_DataTransTx_0900H(void)      
{  
	
	// 1. Head
	if(!Protocol_Head(MSG_0x0900,Packet_Normal)) 
 	  return false; 
	 // 2. content   
	 //  应答流水号
	Original_info[Original_info_Wr++]=DataTrans.TYPE;// 返回透传数据的类型
	memcpy(Original_info+Original_info_Wr,DataTrans.Data_Tx,DataTrans.Data_TxLen);
	Original_info_Wr+=DataTrans.Data_TxLen;
	 //  3. Send 
	Protocol_End(Packet_Normal ,0);
    if(DispContent)
	     rt_kprintf("\r\n	发送透传  \r\n");    
	return true; 

}

//----------------------------------------------------------------------
u8  Stuff_CentreTakeACK_BD_0805H( void )
{
	// 1. Head
	if( !Protocol_Head( MSG_0x0805, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	//   float ID
	Original_info[Original_info_Wr++]	= (u8)( Centre_FloatID >> 8 );      //  应答流水号
	Original_info[Original_info_Wr++]	= (u8)Centre_FloatID;
	Original_info[Original_info_Wr++]	= (u8)SingleCamra_TakeResualt_BD;   // 中心应答结果
	Original_info[Original_info_Wr++]	= 0x00;                             //  成功拍照多媒体个数    1
	Original_info[Original_info_Wr++]	= 1;
	Original_info[Original_info_Wr++]	= 0;                                // 多媒体ID 列表
	Original_info[Original_info_Wr++]	= 0;
	Original_info[Original_info_Wr++]	= 0;
	Original_info[Original_info_Wr++]	= 1;
	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	摄像头立即拍照应答 \r\n");
	}
 
	return true;  
}

//----------------------------------------------------------------------

u8  Stuff_MultiMedia_InfoSD_0800H(void)      
{  
  
	// 1. Head
	if(!Protocol_Head(MSG_0x0800,Packet_Normal)) 
 	  return false; 
	 // 2. content          
		 switch (MediaObj.Media_Type)
		  {
		   case 0 : // 图像
				   MediaObj.Media_totalPacketNum=Photo_sdState.Total_packetNum;  // 图片总包数
				   MediaObj.Media_currentPacketNum=Photo_sdState.SD_packetNum;	// 图片当前报数
				   MediaObj.Media_ID=1;   //  多媒体ID
				   MediaObj.Media_Channel=Camera_Number;  // 图片摄像头通道号
				   break;
		   case 1 : // 音频
		          #ifdef REC_VOICE_ENABLE
				  MediaObj.Media_totalPacketNum=Sound_sdState.Total_packetNum;	// 图片总包数
				  MediaObj.Media_currentPacketNum=Sound_sdState.SD_packetNum;  // 图片当前报数
				  MediaObj.Media_ID=1;	 //  多媒体ID
				  MediaObj.Media_Channel=1;  // 图片摄像头通道号
		         // rt_kprintf(" \r\n 申请上传音频信息 \r\n");  
		         #endif
				   break;
		   default:
				   return false;
		  }
	 
	   //  MediaID
	   Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>24);// 返回事件ID
	   Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>16);
       Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>8);    	
	   Original_info[Original_info_Wr++]=MediaObj.Media_ID;
	   //  Type	   
	   Original_info[Original_info_Wr++]=MediaObj.Media_Type;
	   //  MediaCode Type 
	    Original_info[Original_info_Wr++]=MediaObj.Media_CodeType;
	    Original_info[Original_info_Wr++]=MediaObj.Event_Code;
	    Original_info[Original_info_Wr++]=MediaObj.Media_Channel;
	   
	   
	 //  3. Send 
	Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	发送多媒体事件信息上传  \r\n");    
	 return true; 

}

//--------------------------------------------------------------------------
u8  Stuff_MultiMedia_Data_0801H(void)      
{  
	u16 inadd=0,readsize=0;//,soundpage=0,sounddelta=0;
//	u8  instr[SpxGet_Size];  

	  
	//  rt_kprintf("\r\n  1--- pic_total_num:  %d	current_num:  %d  MediaObj.Media_Type: %d \r\n ",MediaObj.Media_totalPacketNum,MediaObj.Media_currentPacketNum,MediaObj.Media_Type);
	 // 1. Head 
	 if(!Protocol_Head(MSG_0x0801,Packet_Divide))   
 	  return false;    
	 // 2. content1  ==>  MediaHead   
	 if(MediaObj.Media_currentPacketNum==1)
	 {
	   //  MediaID
	   Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>24);//  多媒体ID
	   Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>16);
       Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>8);    	
	   Original_info[Original_info_Wr++]=MediaObj.Media_ID;
	   //  Type	   
	   Original_info[Original_info_Wr++]=MediaObj.Media_Type;    // 多媒体类型
	   //  MediaCode Type 
	    Original_info[Original_info_Wr++]=MediaObj.Media_CodeType; // 多媒体编码格式
	    Original_info[Original_info_Wr++]=MediaObj.Event_Code;     // 多媒体事件编码
	    Original_info[Original_info_Wr++]=MediaObj.Media_Channel;  // 通道ID  

	    //  Position Inifo	    
		//  告警标志  4
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Warn_Status,4 );    
		Original_info_Wr += 4;
		// . 状态  4
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Car_Status,4 );   
		Original_info_Wr += 4;
		//   纬度
	    memcpy( ( char * ) Original_info+ Original_info_Wr,( char * )  Gps_Gprs.Latitude, 4 );//纬度   modify by nathan
		Original_info_Wr += 4;
		//   经度
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	  //经度    东经  Bit 7->0   西经 Bit 7 -> 1
		Original_info_Wr += 4;
		//   高程
		Original_info[Original_info_Wr++]=(u8)(GPS_Hight<<8);
		Original_info[Original_info_Wr++]=(u8)GPS_Hight;
		//   速度    0.1 Km/h
		Original_info[Original_info_Wr++]=(u8)(Speed_gps>>8);//(GPS_speed>>8); 
		Original_info[Original_info_Wr++]=(u8)(Speed_gps);//GPS_speed;     
		//   方向   单位 1度
		Original_info[Original_info_Wr++]=(GPS_direction>>8);  //High 
		Original_info[Original_info_Wr++]=GPS_direction; // Low
		//   日期时间	
		Original_info[Original_info_Wr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);		
		Original_info[Original_info_Wr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
		Original_info[Original_info_Wr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
		Original_info[Original_info_Wr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
		Original_info[Original_info_Wr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
		Original_info[Original_info_Wr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);	 
		//------------
	 } 	
      // 4. content3  ==> Media Info
     switch (MediaObj.Media_Type) 
         {
          case 0 : // 图像
                   
				   if(((Photo_sdState.photo_sending)==enable)&&((Photo_sdState.SD_flag)==enable))
				   {
					   Photo_sdState.SD_flag=disable;// clear    
				   }
				   else				   	
				   if((1==MediaObj.RSD_State)&&(Photo_sdState.SD_flag==enable)) //  补报相关
				   {
					   Photo_sdState.SD_flag=disable;// clear    
				   }
				   else
				   	 return false; 
				   //  ---------------  填写内容  ---------------
				   Photo_sdState.photo_sendTimeout=0;  // clear timeout   
				   //			read		Photo_sdState.SD_packetNum从1开始计数
				   //			content_startoffset 	picpage_offset				 contentpage_offset
                  if(TF_Card_Status()==0) 
                  {
                   if(Camera_Number==1)
				      Api_DFdirectory_Read(camera_1, Original_info + Original_info_Wr, 512,1,MediaObj.Media_currentPacketNum); 
                   else
				   if(Camera_Number==2)
				      Api_DFdirectory_Read(camera_2, Original_info + Original_info_Wr, 512,1,MediaObj.Media_currentPacketNum); 
                   else
				   if(Camera_Number==3)
				      Api_DFdirectory_Read(camera_3, Original_info + Original_info_Wr, 512,1,MediaObj.Media_currentPacketNum); 
                   else
		     if(Camera_Number==4)
				      Api_DFdirectory_Read(camera_4, Original_info + Original_info_Wr, 512,1,MediaObj.Media_currentPacketNum);  
 
				   	
                       DF_delay_ms(10); 
				      inadd=(Photo_sdState.SD_packetNum-1)<<9; //乘以512
				   	  if(PicFileSize>inadd)
				   	  	{
                            if((PicFileSize-inadd)>512)
                                 readsize=512;
							else
							 {	 
							    readsize=PicFileSize-inadd; // 最后一包  
							    rt_kprintf("\r\n   最后一包 readsize =%d \r\n",readsize);
							 }
				   	  	}
					  else
					  	 return false;
                  	}
				  else
				   if(TF_Card_Status()==1)  
				   	{  ;
				   	 /* inadd=(Photo_sdState.SD_packetNum-1)<<9; //乘以512
				   	  if(PicFileSize>inadd)
				   	  	{
                            if((PicFileSize-inadd)>512)
                                 readsize=512;
							else
							 {	 
							    readsize=PicFileSize-inadd; // 最后一包  
							    rt_kprintf("\r\n   最后一包 readsize =%d \r\n",readsize);
							 }
				   	  	}
					  else
					  	 return false;
                      i=read_file(PictureName,inadd,readsize,Original_info + Original_info_Wr); 
					  if(i==false)
					  	{
                          rt_kprintf("\r\n 图片文件: %s   读取失败\r\n",PictureName); 
                          return false;
					  	} */
				   	} 
			 Original_info_Wr+=readsize;		 //         
		          break;
		  case 1 : // 音频
		   #ifdef REC_VOICE_ENABLE
			  if(((Sound_sdState.photo_sending)==enable)&&((Sound_sdState.SD_flag)==enable))
			   {
				   Sound_sdState.SD_flag=disable;// clear	   
			   }
			   else
				 return false; 
                       //------------------------------------------------------------------------
			   //  ---------------  填写内容  ---------------
			   //			read		Photo_sdState.SD_packetNum从1开始计数
			   //			content_startoffset 	picpage_offset				 contentpage_offset
	                  if(TF_Card_Status()==0) 
	                  {
				       Api_DFdirectory_Read(voice, Original_info + Original_info_Wr, 512,1,MediaObj.Media_currentPacketNum);  
					inadd=(Sound_sdState.SD_packetNum-1)<<9; //乘以512
					if(SrcFileSize>inadd)
					 {
			                           if((SrcFileSize-inadd)>512)
			                                 readsize=512;
							else
							 {	 
							     readsize=SrcFileSize-inadd; // 最后一包  
							     rt_kprintf("\r\n   最后一包 readsize =%d \r\n",readsize);  
							 }
					  }
					  else
						  return false;    
	                  	}
				 rt_kprintf("\r\n Sound_sdState.SD_packetNum= %d   filesize=%d  readsize=%d  \r\n",Sound_sdState.SD_packetNum,SrcFileSize,SrcFileSize-inadd);  
                              Original_info_Wr+=readsize;	  


		  #endif
		          break; 
		  default:
		  	      return false;
         }	  

	 
	 if(MediaObj.Media_currentPacketNum>MediaObj.Media_totalPacketNum)
	 {
	 	return false;
	 }	
	 //  5. Send 
	 
		/*     debug   丢包情况   模拟
		       if((MediaObj.Media_currentPacketNum==3)&&(MediaObj.RSD_State==0))
			 	    rt_kprintf("\r\n	jump 3  packet \r\n"); 
			 else
			   if((MediaObj.Media_currentPacketNum==5)&&(MediaObj.RSD_State==0))
			 	    rt_kprintf("\r\n	jump 5  packet \r\n");	 
			 else  
			   if((MediaObj.Media_currentPacketNum==8)&&(MediaObj.RSD_State==0))
			 	    rt_kprintf("\r\n	jump 8  packet \r\n");  
			 else
			     if((MediaObj.Media_currentPacketNum==11)&&(MediaObj.RSD_State==0))
			 	    rt_kprintf("\r\n	jump 11  packet \r\n");	
			 else	 
		*/
	     Protocol_End(Packet_Divide,0);
	 
	 if(DispContent)
	     rt_kprintf("\r\n	Send Media Data \r\n");     
	//  else
	  {
		 rt_kprintf("\r\n pic_total_num:  %d   current_num:  %d   \r\n ",MediaObj.Media_totalPacketNum,MediaObj.Media_currentPacketNum);
		 if(MediaObj.Media_currentPacketNum>=MediaObj.Media_totalPacketNum)
		 {				    
			rt_kprintf("\r\n Media 最后一个block\r\n");
			
			if(0==MediaObj.RSD_State)	// 如果在顺序传输模式下，则改为停止状态,等待中心下重传
			 {
			    MediaObj.RSD_State=2;	
			    MediaObj.RSD_Timer=0;			   
			 } 
			
		 }							 

	  }  
     //----------  累加发送报数 --------------------
     if(0==MediaObj.RSD_State)
     {
		 if(MediaObj.Media_currentPacketNum<MediaObj.Media_totalPacketNum)
		  {
		    //  图片
		    if(Photo_sdState.photo_sending==enable)
		       Photo_sdState.SD_packetNum++;
			//  音频 
			if(Sound_sdState.photo_sending==enable)
			Sound_sdState.SD_packetNum++;
			//视频
			if(Video_sdState.photo_sending==enable)
			 Video_sdState.SD_packetNum++; 
		  }
     }
	 else
	 if(1==MediaObj.RSD_State)
	 {
	      rt_kprintf("\r\n  MediaObj.RSD_Reader =%d\r\n",MediaObj.RSD_Reader); 
	      MediaObj.RSD_Reader++;
	      if(MediaObj.RSD_Reader==MediaObj.RSD_total)
	   	     MediaObj.RSD_State=2; //  置位等待状态，等待着中心再发重传指令
	 }	
	 //----------  返回  -------------------
	 return true; 

}
//----------------------------------------------------------------------
u8  Stuff_MultiMedia_IndexAck_0802H(void)      
{  
   u16  totalNum=0, lenregwr=0;
   u16  i=0;  
   
   
	// 1. Head
	if(!Protocol_Head(MSG_0x0802,Packet_Normal)) 
 	  return false; 
	 // 2. content     
        //   float ID  应答流水号 
	    Original_info[Original_info_Wr++]=(u8)(Centre_FloatID>>8);
	    Original_info[Original_info_Wr++]=(u8)Centre_FloatID;
          
		//------- 多媒体总项数 ----
		lenregwr=Original_info_Wr;		
	    Original_info[Original_info_Wr++]=(u8)(totalNum>>8); // 临时占上位置
	    Original_info[Original_info_Wr++]=(u8)totalNum;

		//----- 查找有效效位置 ----
		totalNum=0;
		  for(i=0;i<8;i++)
		  {
			  if(SD_ACKflag.f_MediaIndexACK_0802H==1)  // 图像
			  {		
				 Api_RecordNum_Read(pic_index,i,(u8*)&MediaIndex, sizeof(MediaIndex));   
			  }
			  else
			  if(SD_ACKflag.f_MediaIndexACK_0802H==2) // 音频
			  {
			       Api_RecordNum_Read(voice_index,i,(u8*)&MediaIndex, sizeof(MediaIndex));   
			  }  
			 // rt_kprintf("\r\n Effective_Flag %d  f_QueryEventCode %d  EventCode %d  \r\n",MediaIndex.Effective_Flag,SD_ACKflag.f_QueryEventCode,MediaIndex.EventCode);
			  if((MediaIndex.Effective_Flag==1)&&(SD_ACKflag.f_QueryEventCode==MediaIndex.EventCode))
			  { //  查找有效的索引和相对应类型的索引
			    Original_info[Original_info_Wr++]=(u8)(MediaIndex.MediaID>>24); //  多媒体ID dworrd
			    Original_info[Original_info_Wr++]=(u8)(MediaIndex.MediaID>>16);
				Original_info[Original_info_Wr++]=(u8)(MediaIndex.MediaID>>8);
				Original_info[Original_info_Wr++]=(u8)(MediaIndex.MediaID);
			    Original_info[Original_info_Wr++]=MediaIndex.Type; //  多媒体类型
				Original_info[Original_info_Wr++]=MediaIndex.ID;   //  通道
			    Original_info[Original_info_Wr++]=MediaIndex.EventCode;
				memcpy(Original_info+Original_info_Wr,MediaIndex.PosInfo,28);
			    Original_info_Wr+=28;    
				totalNum++;    
			  }
			  
		  }

          //---------   补上总项数  -------- 
          Original_info[lenregwr]=(u8)(totalNum>>8);
		  Original_info[lenregwr+1]=totalNum;     

	 //  3. Send 
	Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	Send Media Index \r\n");    
	 return true; 

}
//--------------------------------------------------------------------------------------
u8  Stuff_DriverInfoSD_0702H(void)      
{  
    u8 i=0;
	 // 1. Head
	 if(!Protocol_Head(MSG_0x0702,Packet_Normal))  
 	    return false; 
	
	 // 2. content     
        //   驾驶员姓名长度
        i=strlen((const char*) JT808Conf_struct.Driver_Info.DriveName);
	    Original_info[Original_info_Wr++]=i;
	    memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.DriveName,i);// name
		Original_info_Wr+=i;
        memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.Driver_ID,20);// 身份证号码
        Original_info_Wr+=20;
		memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.Drv_CareerID,40);//从业资格证
        Original_info_Wr+=40;
		i=strlen((const char*)JT808Conf_struct.Driver_Info.Comfirm_agentID); // 机构名称
		Original_info[Original_info_Wr++]=i;
		memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.Comfirm_agentID,i);
		Original_info_Wr+=i;
		
	 // 3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	Send Driver Info \r\n");    
	 return true; 

}
u8  Stuff_ISP_Resualt_BD_0108H(void)
{
   // 1. Head
	if(!Protocol_Head(MSG_0x0108,Packet_Normal)) 
 	  return false; 
	 // 2. content   
	 BD_ISP.ISP_running=0; // clear
         //----------------------------------------------------
         Original_info[Original_info_Wr++]=BD_ISP.Update_Type; // 升级类型
	  Original_info[Original_info_Wr++]=SD_ACKflag.f_BD_ISPResualt_0108H-1;//BD_ISP.Update_Type;  // 升级结果
	 
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	远程升级结果上报   %d \r\n",SD_ACKflag.f_BD_ISPResualt_0108H-1);      
  
		return true;
}
u8  Stuff_BatchDataTrans_BD_0704H(void)
{
   // 1. Head
	if(!Protocol_Head(MSG_0x0704,Packet_Normal)) 
 	     return false; 
	 // 2. content   
             // 2.1   数据项个数
             Original_info[Original_info_Wr++]=1;   
	       // 2.2   位置数据类型
             Original_info[Original_info_Wr++]=0;   //  0: 正常上报  1: 盲区补报
	      // 2.3   位置汇报数据项
	      	// 1. 告警标志  4
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Warn_Status,4 );    
		Original_info_Wr += 4;
		// 2. 状态  4
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Car_Status,4 );   
		Original_info_Wr += 4;
		// 3.  纬度
	       memcpy( ( char * ) Original_info+ Original_info_Wr,( char * )  Gps_Gprs.Latitude, 4 );//纬度   modify by nathan
		Original_info_Wr += 4;
		// 4.  经度
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	  //经度    东经  Bit 7->0   西经 Bit 7 -> 1
		Original_info_Wr += 4;
		// 5.  高程
		Original_info[Original_info_Wr++]=(u8)(GPS_Hight<<8);
		Original_info[Original_info_Wr++]=(u8)GPS_Hight;
		// 6.  速度    0.1 Km/h
		Original_info[Original_info_Wr++]=(u8)(Speed_gps>>8);//(GPS_speed>>8); 
		Original_info[Original_info_Wr++]=(u8)(Speed_gps);//GPS_speed;     
		// 7. 方向   单位 1度
		Original_info[Original_info_Wr++]=(GPS_direction>>8);  //High 
		Original_info[Original_info_Wr++]=GPS_direction; // Low
		// 8.  日期时间	
		Original_info[Original_info_Wr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);		
		Original_info[Original_info_Wr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
		Original_info[Original_info_Wr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
		Original_info[Original_info_Wr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
		Original_info[Original_info_Wr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
		Original_info[Original_info_Wr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);	 
     
	 //  3. Send 
	  Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	          rt_kprintf("\r\n	定位数据批量上传\r\n");    
   return true;
}


u8  Stuff_CANDataTrans_BD_0705H(void)
{
      u16   DataNum=0,i=0;
      u32   read_rd=0; 	  
   // 1. Head
	if(!Protocol_Head(MSG_0x0705,Packet_Normal)) 
 	  return false; 
	 // 2. content   
        // 数据项个数
        DataNum=(CAN_trans.canid_1_SdWr>>3);  // 除以8 
        Original_info[Original_info_Wr++]=(DataNum>>8);
	 Original_info[Original_info_Wr++]=DataNum;
	 Can_sdnum+=DataNum;  
	 //接收时间
        Original_info[Original_info_Wr++]=((time_now.hour/10)<<4)+(time_now.hour%10);
	 Original_info[Original_info_Wr++]=((time_now.min/10)<<4)+(time_now.min%10);
	 Original_info[Original_info_Wr++]=((time_now.sec/10)<<4)+(time_now.sec%10);    
	 Original_info[Original_info_Wr++]=(Can_same%10);//0x00;  // ms 毫秒  
	 Original_info[Original_info_Wr++]=0x00;    
	 //  CAN 总线数据项
	 read_rd=0;
        for(i=0;i<DataNum;i++)
        {	
           /*
		Original_info[Original_info_Wr++]= (CAN_trans.canid_1_Filter_ID>>24)|0x40;// 返回透传数据的类型 
		Original_info[Original_info_Wr++]=(CAN_trans.canid_1_Filter_ID>>16);
		Original_info[Original_info_Wr++]=(CAN_trans.canid_1_Filter_ID>>8);
		Original_info[Original_info_Wr++]=CAN_trans.canid_1_Filter_ID;        
	  */     
		
		Original_info[Original_info_Wr++]= (CAN_trans.canid_1_ID_SdBUF[i]>>24)|0x40;// 返回透传数据的类型 
		Original_info[Original_info_Wr++]=(CAN_trans.canid_1_ID_SdBUF[i]>>16);
		Original_info[Original_info_Wr++]=(CAN_trans.canid_1_ID_SdBUF[i]>>8);
		Original_info[Original_info_Wr++]=CAN_trans.canid_1_ID_SdBUF[i];   
		
		//--------------------------------------------------------------------------
		memcpy(Original_info+Original_info_Wr,CAN_trans.canid_1_Sdbuf+read_rd,8);    
		Original_info_Wr+=8;
		read_rd+=8;  
        }	 
	 CAN_trans.canid_1_SdWr=0;   
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	CAN 总线数据上传 数据项=%d   Rxnum=%d  Sd_num=%d  Can_loudiao=%d  Can_notsame=%d Same=%d\r\n",DataNum,Can_RXnum,Can_sdnum,Can_loudiao,Can_same,Can_notsame);       
   return true;
}

//---------------------------------------------------------------------------------
u8  Stuff_Worklist_0701H(void)      
{  
    u32  listlen=215;
	 // 1. Head
	 if(!Protocol_Head(MSG_0x0701,Packet_Normal)) 
 	    return false; 
	
	 // 2. content     
        //   信息长度
	     listlen=207;
		 Original_info[Original_info_Wr++]=(listlen>>24);// 返回事件ID
		 Original_info[Original_info_Wr++]=(listlen>>16);
		 Original_info[Original_info_Wr++]=(listlen>>8);		  
		 Original_info[Original_info_Wr++]=listlen;
	    
        memcpy(Original_info+Original_info_Wr,"托运单位:天津七一二通信广播有限公司 电话:022-26237216  ",55);
        Original_info_Wr+=55;		
        memcpy(Original_info+Original_info_Wr,"承运单位:天津物流运输公司 电话:022-86692666  ",45);
        Original_info_Wr+=45;
		memcpy(Original_info+Original_info_Wr,"物品名称:GPS车载终端  包装方式:  箱式   每箱数量: 20   总量: 30箱  ",67);
        Original_info_Wr+=67;
		memcpy(Original_info+Original_info_Wr,"车型:箱式小货车 运达日期 :  2012-1-11   ",40); 
        Original_info_Wr+=40; 
		
	 // 3. Send 
	Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	Send Worklist  \r\n");    
	 return true; 

}
 //-------------------- ISP Check  ---------------------------------------------
void  ISP_file_Check(void)
{
   u8  FileHead[100];
   u8  ISP_judge_resualt=0;
   u32  HardVersion=0;
   	
      memset(ISP_buffer,0,sizeof(ISP_buffer));
	  SST25V_BufferRead(ISP_buffer,ISP_StartArea,256); 
      //---判断文件更新标志---------------------
      if(ISP_buffer[32]!=ISP_BYTE_CrcPass)   //  计算校验通过后  更新标志改成0xE1     以前是0xF1
      	{
      	     rt_kprintf("\r\n 厂商型号正确\r\n");  
             return;  
      	}
	  
	  /*
		   序号   字节数	名称			  备注
		  1 		  1    更新标志 	 1 表示需要更新   0 表示不需要更新
		  2-5			  4   设备类型				 0x0000 0001  ST712   TWA1
											0x0000 0002   STM32  103  新A1
											0x0000 0003   STM32  101  简易型
											0x0000 0004   STM32  A3  sst25
											0x0000 0005   STM32  行车记录仪 
		  6-9		 4	   软件版本 	 每个设备类型从  0x0000 00001 开始根据版本依次递增
		  10-29 	  20	日期		' mm-dd-yyyy HH:MM:SS'
		  30-31 	  2    总页数		   不包括信息页
		  32-35 	  4    程序入口地址    
		  36-200	   165	  预留	  
		  201-		  n    文件名  
	 
	  */
	  //------------   Type check  ---------------------
	  memset(FileHead,0,sizeof(FileHead));
	  memcpy(FileHead,ISP_buffer,32);
	  rt_kprintf( "\r\n FileHead:%s\r\n",FileHead ); 


	  //------    文件格式判断
	  if(strncmp(ISP_buffer+32+13,"70420TW703",10)==0) //判断厂商和型号
		 {
			 ISP_judge_resualt++;// step 1
			 rt_kprintf("\r\n 厂商型号正确\r\n");  

			 // hardware 
			 HardVersion=(ISP_buffer[32+38]<<24)+(ISP_buffer[32+39]<<16)+(ISP_buffer[32+40]<<8)+ISP_buffer[32+41]; 
			 HardWareVerion=HardWareGet();
			 if((HardWareVerion==HardVersion)||(HardWareVerion==7))	// 要兼容以前的老板子 全1									
			  {
				  ISP_judge_resualt++;// step 2
				  rt_kprintf("\r\n 硬件版本:%d\r\n",HardVersion);  
			  }
             else
				  rt_kprintf("\r\n 硬件版本不匹配!\r\n");    
			 //firmware
			 if(strncmp(ISP_buffer+32+42,"TJGGHYPT",8)==0) 
			 {
				ISP_judge_resualt++;// step 3
				rt_kprintf("\r\n  固件版本:TJGGHYPT\r\n");	 
			 }
			 // operater
			 if(strncmp(ISP_buffer+32+62,"TJGTGS",6)==0) 
			 {
				ISP_judge_resualt++;// step 4
				rt_kprintf("\r\n  固件版本:TJGTGS\r\n");  	  
			 }
			 
		 }

     //ISP_judge_resualt=4;
	 if(ISP_judge_resualt==4) 
	 {
		  //------- enable  flag -------------
		 SST25V_BufferRead( FileHead,0x001000,100 );
		 FileHead[32] = ISP_BYTE_Rdy2Update;    //-----  文件更新标志  使能启动时更新
		 SST25V_BufferWrite( FileHead,0x001000,100);   
		
		 {
			Systerm_Reset_counter= (Max_SystemCounter-5);	 // 准备重启更新最新程序
			ISP_resetFlag=1;//准备重启 
			rt_kprintf( "\r\n 准备重启更新程序!\r\n" );	  
		 } 	
		
		// rt_kprintf( "\r\n 升级完成了，但不更新等待判断 校验测试 \r\n" );	   
	 }
	else
	{
	        //------- enable  flag -------------
		 SST25V_BufferRead( FileHead,0x001000,100 );
		 FileHead[32] = ISP_BYTE_TypeNotmatch;    //-----   文件校验通过，但 类型不匹配  
		 SST25V_BufferWrite( FileHead,0x001000,100);   
	      BD_ISP.ISP_running=0; // recover normal  
          rt_kprintf( "\r\n 相关参数不匹配不与更新!\r\n" );	 
	}
   
 }
// FINSH_FUNCTION_EXPORT(ISP_file_Check, ISP_file_Check);   



//----------------------------------------------------------------------------------
void Stuff_O200_Info_Only( u8* Instr) 
{
   u8  Infowr=0;
   
	// 1. 告警标志  4
	memcpy( ( char * ) Instr+ Infowr, ( char * )Warn_Status,4 );    
	Infowr += 4;
	// 2. 状态  4
	memcpy( ( char * ) Instr+ Infowr, ( char * )Car_Status,4 );   
	Infowr += 4;
	// 3.  纬度
    memcpy( ( char * ) Instr+ Infowr,( char * )  Gps_Gprs.Latitude, 4 );//纬度   modify by nathan
	Infowr += 4;
	// 4.  经度
	memcpy( ( char * ) Instr+ Infowr, ( char * )  Gps_Gprs.Longitude, 4 );	  //经度    东经  Bit 7->0   西经 Bit 7 -> 1
	Infowr += 4;
	// 5.  高程
	Instr[Infowr++]=(u8)(GPS_Hight<<8);
	Instr[Infowr++]=(u8)GPS_Hight;
	// 6.  速度    0.1 Km/h
	Instr[Infowr++]=(u8)(Speed_gps>>8);
	Instr[Infowr++]=(u8)Speed_gps;   
	// 7. 方向   单位 1度
	Instr[Infowr++]=(GPS_direction>>8);  //High 
	Instr[Infowr++]=GPS_direction; // Low
	// 8.  日期时间	
	Instr[Infowr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);		
	Instr[Infowr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
	Instr[Infowr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
	Instr[Infowr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
	Instr[Infowr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
	Instr[Infowr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);	

}
//-----------------------------------------------------
u8  Save_MediaIndex( u8 type, u8* name, u8 ID,u8 Evencode)
{
  u8   i=0;

  if((type!=1)&&(type!=0))
    return false;
  
  //----- 查找无效位置 ----
  for(i=0;i<8;i++)
  {
	  if(type==0)  // 图像
	  {		
		   Api_RecordNum_Read(pic_index,i,(u8*)&MediaIndex, sizeof(MediaIndex));    
	  }
	  else
	  if(type==1) // 音频
	  {
	         Api_RecordNum_Read(voice_index,1,(u8*)&MediaIndex, sizeof(MediaIndex));    
	  }    
	  if(MediaIndex.Effective_Flag==0)
	  	 break;
  }
   if(i==8)  // 如果都满了则从第一个开始
   	   i=0;
  //----  填写信息 -------------
  memset((u8*)&MediaIndex,0,sizeof(MediaIndex));
  MediaIndex.MediaID= JT808Conf_struct.Msg_Float_ID;   
  MediaIndex.Type=type;
  MediaIndex.ID=ID;
  MediaIndex.Effective_Flag=1;
  MediaIndex.EventCode=Evencode;
  memcpy(MediaIndex.FileName,name,strlen((const char*)name));
  Stuff_O200_Info_Only(MediaIndex.PosInfo);
  
  if(type==0)  // 图像
  {
        Api_RecordNum_Write(pic_index,i,(u8*)&MediaIndex, sizeof(MediaIndex));    
  }
  else
  if(type==1) // 音频
  {
       Api_RecordNum_Write(voice_index,i,(u8*)&MediaIndex, sizeof(MediaIndex));    
  }
  return true;
  
}
//------------------------------------------------------------------
u8  CentreSet_subService_8103H(u32 SubID, u8 infolen, u8 *Content )
{  
 
  u8    i=0;
  u8    reg_str[80];
  u8    reg_in[20];
  u32   resualtu32=0;
  

   rt_kprintf("\r\n    收到中心设置命令 SubID=%X \r\n",SubID);  
	   
  switch(SubID)
   {
     case 0x0001:  // 终端心跳包发送间隔  单位:s
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.Heart_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                                   rt_kprintf("\r\n 心跳包间隔: %d s\r\n",JT808Conf_struct.DURATION.Heart_Dur);
                    break;
	 case 0x0002:  // TCP 消息应答超时时间  单位:s	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.TCP_ACK_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n TCP消息应答间隔: %d s\r\n",JT808Conf_struct.DURATION.TCP_ACK_Dur); 
	                break;
	 case 0x0003:  //  TCP 消息重传次数	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.TCP_ReSD_Num=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n TCP重传次数: %d\r\n",JT808Conf_struct.DURATION.TCP_ReSD_Num); 
	                break;
	 case 0x0004:  // UDP 消息应答超时时间  单位:s	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.UDP_ACK_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                    rt_kprintf("\r\n UDP应答超时: %d\r\n",JT808Conf_struct.DURATION.UDP_ACK_Dur);
	                break;
	 case 0x0005:  //  UDP 消息重传次数	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.UDP_ReSD_Num=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n UDP重传次数: %d\r\n",JT808Conf_struct.DURATION.UDP_ReSD_Num); 
					break;
	 case 0x0010:  //  主服务器APN 
                                    if(infolen==0)
					  	break;
					  memset(APN_String,0,sizeof(APN_String));					  
					  memcpy(APN_String,(char*)Content,infolen);  
					  memset((u8*)SysConf_struct.APN_str,0,sizeof(APN_String));	
					  memcpy((u8*)SysConf_struct.APN_str,(char*)Content,infolen);  
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));


                                     DataLink_APN_Set(APN_String,1); 
									 
	                break; 
	 case 0x0013:  //  主服务器地址  IP 或域名
                      memset(reg_in,0,sizeof(reg_in)); 
					  memcpy(reg_in,Content,infolen);
                        rt_kprintf("\r\n  不允许修改 主IP 和域名: %s \r\n",reg_in);
                       Fail_Flag=1;
                       break;// 平台不让修改域名和IP 
					  
					 //----------------------------	
					 
					 i =str2ip((char*)reg_in, RemoteIP_main);
					 if (i <= 3)
					 {
					  rt_kprintf("\r\n  域名: %s \r\n",reg_in); 
					  
					  memset(DomainNameStr,0,sizeof(DomainNameStr));					  
					  memset(SysConf_struct.DNSR,0,sizeof(DomainNameStr));
					  memcpy(DomainNameStr,(char*)Content,infolen);   
					  memcpy(SysConf_struct.DNSR,(char*)Content,infolen);   
					  Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));

                                     //----- 传给 GSM 模块------
                                    DataLink_DNSR_Set(SysConf_struct.DNSR,1); 

									 
					  SD_ACKflag.f_CentreCMDack_0001H=1 ;// 2 DataLink_EndFlag=1; //AT_End(); 先返回结果再挂断  
					  break;
					 }
					 memset(reg_str,0,sizeof(reg_str));
					 IP_Str((char*)reg_str, *( u32 * ) RemoteIP_main); 		  
					 strcat((char*)reg_str, " :");		 
					 sprintf((char*)reg_str+strlen((const char*)reg_str), "%u\r\n", RemotePort_main);  
					 memcpy(SysConf_struct.IP_Main,RemoteIP_main,4);
					 SysConf_struct.Port_main=RemotePort_main;
					 
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					 rt_kprintf("\r\n 中心设置主服务器 IP \r\n");
					 rt_kprintf("\r\n SOCKET :");  
					 rt_kprintf((char*)reg_str);   
					  //-----------  Below add by Nathan  ----------------------------				  
					  rt_kprintf("\r\n		   备用IP: %d.%d.%d.%d : %d \r\n",RemoteIP_aux[0],RemoteIP_aux[1],RemoteIP_aux[2],RemoteIP_aux[3],RemotePort_main);   
					 
					  //-----------  Below add by Nathan  ----------------------------
					 DataLink_MainSocket_set(RemoteIP_main,RemotePort_main,1);
	                              //-------------------------------------------------------------	
					   
					   SD_ACKflag.f_CentreCMDack_0001H=1 ;//DataLink_EndFlag=1; //AT_End(); 先返回结果再挂断  

	                break;
	 case 0x0014:  // 备份服务器 APN

	                break;
	 case 0x0017:  // 备份服务器  IP     


	  
				  memset(reg_in,0,sizeof(reg_in)); 
				  memcpy(reg_in,Content,infolen);

				     rt_kprintf("\r\n  不允许修改 备用IP 和域名: %s \r\n",reg_in);
                      Fail_Flag=1;
                       break;// 平台不让修改域名和IP  
					  
				 //---------------------------- 
				  i =str2ip((char*)reg_in, RemoteIP_aux);
				  if (i <= 3)
				 {
					  rt_kprintf("\r\n  域名aux: %s \r\n",reg_in); 					  
					 memset(DomainNameStr_aux,0,sizeof(DomainNameStr_aux));					  
					 memset(SysConf_struct.DNSR_Aux,0,sizeof(DomainNameStr_aux));     
					  memcpy(DomainNameStr_aux,(char*)Content,infolen);   
					  memcpy(SysConf_struct.DNSR_Aux,(char*)Content,infolen);    
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
                                     //----- 传给 GSM 模块------
                                    DataLink_DNSR2_Set(SysConf_struct.DNSR_Aux,1);
									 
					  SD_ACKflag.f_CentreCMDack_0001H=1 ;//DataLink_EndFlag=1; //AT_End(); 先返回结果再挂断  
					  break;
				 }  				  
				  memset(reg_str,0,sizeof(reg_str));
				  IP_Str((char*)reg_str, *( u32 * ) RemoteIP_aux);		   
				  strcat((char*)reg_str, " :"); 	  
				  sprintf((char*)reg_str+strlen((const char*)reg_str), "%u\r\n", RemotePort_aux);  
				 
				  Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
				  rt_kprintf("\r\n 中心设置备用服务器 IP \r\n");
				  rt_kprintf("\r\nUDP SOCKET :");  
				  rt_kprintf((char*)reg_str);  
				  DataLink_AuxSocket_set(RemoteIP_aux,RemotePort_aux,1);
				   //-----------  Below add by Nathan  ----------------------------				   
				   rt_kprintf("\r\n 		备用IP: %d.%d.%d.%d : %d \r\n",RemoteIP_aux[0],RemoteIP_aux[1],RemoteIP_aux[2],RemoteIP_aux[3],RemotePort_aux);	
	                break;
	 case 0x0018:  //  服务器 TCP 端口
                    //----------------------------	
                      if(infolen!=4)
						break;	
					  RemotePort_main=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];

					  Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct)); 
					  rt_kprintf("\r\n 中心设置主服务器 PORT :%d\r\n",RemotePort_main);
					  rt_kprintf("\r\nUDP SOCKET :");  
					  rt_kprintf((char*)reg_str);  
					   //-----------  Below add by Nathan  ----------------------------
                                     DataLink_MainSocket_set(RemoteIP_main,RemotePort_main,1);					   //-------------------------------------------------------------		   
					   SD_ACKflag.f_CentreCMDack_0001H=1 ;//DataLink_EndFlag=1; //AT_End(); 先返回结果再挂断	
	                break;
	 case 0x0019:  //  服务器 UDP 端口
                    
					if(infolen!=4)
					  break;  
					RemotePort_aux=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
					
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					rt_kprintf("\r\n 中心设置UDP服务器 PORT \r\n");
					rt_kprintf("\r\nUDP SOCKET :");   
					rt_kprintf((char*)reg_str);    
					rt_kprintf("\r\n		 备用IP: %d.%d.%d.%d : %d \r\n",RemoteIP_aux[0],RemoteIP_aux[1],RemoteIP_aux[2],RemoteIP_aux[3],RemotePort_aux);	 
	                break;
     case 0x0020:  //  汇报策略  0 定时汇报  1 定距汇报 2 定时和定距汇报
                    if(infolen!=4)
						break;	
                    resualtu32=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
					
					JT808Conf_struct.SD_MODE.Send_strategy=resualtu32;					
					switch(resualtu32)
						{
						   case 0:rt_kprintf("\r\n 定时汇报 \r\n");
						          JT808Conf_struct.SD_MODE.DUR_TOTALMODE=1;  // 使能定时发送
                                  JT808Conf_struct.SD_MODE.Dur_DefaultMode=1; //  缺省方式上报

								    JT808Conf_struct.SD_MODE.DIST_TOTALMODE=0;
                                  JT808Conf_struct.SD_MODE.Dist_DefaultMode=0;
						   	      break;
						   case 1:rt_kprintf("\r\n 定距汇报 \r\n");
						          JT808Conf_struct.SD_MODE.DIST_TOTALMODE=1;
                                  JT808Conf_struct.SD_MODE.Dist_DefaultMode=1;

								    JT808Conf_struct.SD_MODE.DUR_TOTALMODE=0;  // 使能定时发送
                                   JT808Conf_struct.SD_MODE.Dur_DefaultMode=0; //  缺省方式上报
						   	      break;
						   case 2:rt_kprintf("\r\n 定时和定距汇报\r\n"); 
						            JT808Conf_struct.SD_MODE.DIST_TOTALMODE=1;
                                  JT808Conf_struct.SD_MODE.Dist_DefaultMode=1;

								   JT808Conf_struct.SD_MODE.DUR_TOTALMODE=1;  // 使能定时发送
                                  JT808Conf_struct.SD_MODE.Dur_DefaultMode=1; //  缺省方式上报
								  
						   	      break;
						   default:
						   	      break;

						}
					
					Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
	                break;
	 case 0x0021:  //  位置汇报方案  0 根据ACC上报  1 根据ACC和登录状态上报 
                    JT808Conf_struct.PositionSd_Stratage=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
	                break;
       //--------
					
	 case 0x0022:  //  驾驶员未登录 汇报时间间隔 单位:s    >0	                  
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.NoDrvLogin_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                                   rt_kprintf("\r\n 驾驶员未登录汇报间隔: %d\r\n",JT808Conf_struct.DURATION.NoDrvLogin_Dur);  
	                break;
	 case 0x0027:   //  休眠时汇报时间间隔，单位 s  >0	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.Sleep_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                    rt_kprintf("\r\n 休眠汇报时间间隔: %d \r\n",JT808Conf_struct.DURATION.Sleep_Dur);   
	                break;
	 case 0x0028:   //  紧急报警时汇报时间间隔  单位 s	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.Emegence_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
                    rt_kprintf("\r\n 紧急报警时间间隔: %d \r\n",JT808Conf_struct.DURATION.Emegence_Dur);   
	                break;
	 case 0x0029:   //  缺省时间汇报间隔  单位 s
	                if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.Default_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
					rt_kprintf("\r\n 缺省汇报时间间隔: %d \r\n",JT808Conf_struct.DURATION.Default_Dur);   
	                break;
       //---------
					
	 case 0x002C:   //  缺省距离汇报间隔  单位 米
	                if(infolen!=4)
						break;					
					JT808Conf_struct.DISTANCE.Defalut_DistDelta=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n 缺省距离汇报间隔: %d m\r\n",JT808Conf_struct.DISTANCE.Defalut_DistDelta); 
	                break;
	 case 0x002D:   //  驾驶员未登录汇报距离间隔 单位 米
	               if(infolen!=4)
						break;					
					JT808Conf_struct.DISTANCE.NoDrvLogin_Dist=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
					rt_kprintf("\r\n 驾驶员未登录汇报距离: %d m\r\n",JT808Conf_struct.DISTANCE.NoDrvLogin_Dist); 
	                break;
	 case 0x002E:   //  休眠时汇报距离间隔  单位 米
	               if(infolen!=4)
						break;					
					JT808Conf_struct.DISTANCE.Sleep_Dist=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n 休眠时定距上报间隔: %d m\r\n",JT808Conf_struct.DISTANCE.Sleep_Dist); 
	                break;
	 case 0x002F:   //  紧急报警时汇报距离间隔  单位 米
	               if(infolen!=4)
						break;					
					JT808Conf_struct.DISTANCE.Emergen_Dist=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n 紧急报警时定距上报间隔: %d m\r\n",JT808Conf_struct.DISTANCE.Emergen_Dist); 
	                break;
	 case 0x0030:   //  拐点补传角度 , <180
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.SD_Delta_maxAngle=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n 拐点补传角度: %d 度\r\n",JT808Conf_struct.DISTANCE.Emergen_Dist);  
	                break;
  
	 case 0x0040:   //   监控平台电话号码  
	                 if(infolen==0)
					 	 break;
					i=strlen((const char*)JT808Conf_struct.LISTEN_Num);
					rt_kprintf("\r\n old: %s \r\n",JT808Conf_struct.LISTEN_Num);
					 
					memset(JT808Conf_struct.LISTEN_Num,0,sizeof(JT808Conf_struct.LISTEN_Num));
					memcpy(JT808Conf_struct.LISTEN_Num,Content,infolen);											
					Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));  
 					rt_kprintf("\r\n new: %s \r\n",JT808Conf_struct.LISTEN_Num);
                     
	                //CallState=CallState_rdytoDialLis;  // 准备开始拨打监听号码 
	                rt_kprintf("\r\n 设置监控平台号码: %s \r\n",JT808Conf_struct.LISTEN_Num);   

	                break;
	 case 0x0041:   //   复位电话号码，可采用此电话号码拨打终端电话让终端复位
                    if(infolen==0)
					 	 break;
					memset(reg_str,0,sizeof(reg_str)); 
					memcpy(reg_str,Content,infolen);											
 					rt_kprintf("\r\n 复位电话号码 %s \r\n",reg_str);  
	                break;
	 case 0x0042:   //   恢复出厂设置电话，可采用该电话号码是终端恢复出厂设置

	                break;
	 case 0x0045:   //  终端电话接听策略 0 自动接听  1 ACC ON自动接听 OFF时手动接听

	                break;
	 case 0x0046:   //  每次通话最长时间 ，单位  秒
                    
	                break;
	 case 0x0047:   //  当月最长通话时间，单位  秒

	                break;
	 case 0x0048:   //  监听电话号码                    
					if(infolen==0)
						 break;
					memset(JT808Conf_struct.LISTEN_Num,0,sizeof(JT808Conf_struct.LISTEN_Num));
					memcpy(JT808Conf_struct.LISTEN_Num,Content,infolen); 										
					Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));  
					CallState=CallState_rdytoDialLis;  // 准备开始拨打监听号码 
					rt_kprintf("\r\n 立即拨打监听号码: %s \r\n",JT808Conf_struct.LISTEN_Num);  
	                break;  

	    //----------				
	 case 0x0050:  //  报警屏蔽字， 与位置信息中报警标志相对应。相应位为1时报警被屏蔽---
                    
                    if(infolen!=4)
						break;	
                    Warn_MaskWord[0]=Content[0];
					Warn_MaskWord[1]=Content[1];
					Warn_MaskWord[2]=Content[2];
					Warn_MaskWord[3]=Content[3];  
				    resualtu32=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3]; 
                    rt_kprintf("\r\n 报警屏蔽字: %x \r\n",resualtu32);    //
					break;
	 case 0x0052:  //  报警拍照开关， 与报警标志对应的位1时，拍照

	                break;
	 case 0x0053:  //  报警拍照存储  	与报警标志对应的位1时，拍照存储 否则实时上传

	                break;
	 case 0x0054:  //  关键标志  		与报警标志对应的位1  为关键报警
	                     if(infolen!=4)
						break;	
                    Key_MaskWord[0]=Content[0];
					Key_MaskWord[1]=Content[1];
					Key_MaskWord[2]=Content[2];
					Key_MaskWord[3]=Content[3];  
				    resualtu32=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3]; 
                    rt_kprintf("\r\n 关键报警标志: %x \r\n",resualtu32);    // 
		 

	                break;
        //---------  
					
	 case 0x0055:  //  最高速度   单位   千米每小时
                   if(infolen!=4)
						break;
				    JT808Conf_struct.Speed_warn_MAX=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)	+Content[3];
				   memset(reg_str,0,sizeof(reg_str));
				   memcpy(reg_str,& JT808Conf_struct.Speed_warn_MAX,4);
				   memcpy(reg_str+4,&JT808Conf_struct.Spd_Exd_LimitSeconds,4);
				    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
				   rt_kprintf("\r\n 最高速度: %d km/h \r\n", JT808Conf_struct.Speed_warn_MAX); 
				   Spd_ExpInit();
	                break;
	 case 0x0056:  //  超速持续时间    单位 s
                   if(infolen!=4)
						break;
				   JT808Conf_struct.Spd_Exd_LimitSeconds=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)	+Content[3];
				   memset(reg_str,0,sizeof(reg_str));
				   memcpy(reg_str,& JT808Conf_struct.Speed_warn_MAX,4);
				   memcpy(reg_str+4,&JT808Conf_struct.Spd_Exd_LimitSeconds,4); 
				    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
				   rt_kprintf("\r\n 超时持续时间: %d s \r\n",JT808Conf_struct.Spd_Exd_LimitSeconds); 
				   Spd_ExpInit();
	                break;
	 case 0x0057:  //  连续驾驶时间门限 单位  s
					 if(infolen!=4)
						  break;
				   TiredConf_struct.TiredDoor.Door_DrvKeepingSec=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)   +Content[3];
                               Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct));				  
				   Warn_Status[3]&=~0x04;  //BIT(2)	接触疲劳驾驶报警 
				   TIRED_Drive_Init();    // 清除疲劳驾驶状态      
				    rt_kprintf("\r\n 连续驾驶时间门限: %d s \r\n",TiredConf_struct.TiredDoor.Door_DrvKeepingSec);  
	               break;
	 case 0x0058:  //  当天累计驾驶时间门限  单位  s
				   if(infolen!=4)
						break;
				   TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)   +Content[3];
                               Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct));			
				   TiredConf_struct.Tired_drive.ACC_ONstate_counter=0;
				   TiredConf_struct.Tired_drive.ACC_Offstate_counter=0;
                                Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
                               rt_kprintf("\r\n 当天累计驾驶时间: %d s \r\n",TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec); 
	                break;
     case 0x0059:  //  最小休息时间  单位 s
				   if(infolen!=4)
						break;
				    TiredConf_struct.TiredDoor.Door_MinSleepSec=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)   +Content[3];
                               Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct));				  
					rt_kprintf("\r\n 最小休息时间: %d s \r\n",TiredConf_struct.TiredDoor.Door_MinSleepSec);  
	                break;
	 case 0x005A:  //  最长停车时间   单位 s
					 if(infolen!=4)
						  break;
					TiredConf_struct.TiredDoor.Door_MaxParkingSec=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)	 +Content[3];
                                   Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct));				
					TiredConf_struct.TiredDoor.Parking_currentcnt=0; 
					 Warn_Status[1]&=~0x08;  // 清除超时触发 
					rt_kprintf("\r\n 最长停车时间: %d s \r\n",TiredConf_struct.TiredDoor.Door_MaxParkingSec);   
	                break;
	     //--------- 
	 case  0x0070: //  图像/视频质量  1-10  1 最好

	                break;
	 case  0x0071: //  亮度  0-255

	                break;
	 case  0x0072: //  对比度  0-127

	                break;
	 case  0x0073: // 饱和度  0-127

	                break;
	 case  0x0074: // 色度   0-255

	                break;
		  //---------
	 case  0x0080: // 车辆里程表读数   1/10 km
	                // resualtu32=JT808Conf_struct.Distance_m_u32/100;
					 
					 resualtu32=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)  +Content[3];					
                   //  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                      DayStartDistance_32=resualtu32;
				      Distance_m_u32=0; 
					  DF_Write_RecordAdd(Distance_m_u32,DayStartDistance_32,TYPE_DayDistancAdd);
					  JT808Conf_struct.DayStartDistance_32=DayStartDistance_32;
					  JT808Conf_struct.Distance_m_u32=Distance_m_u32;	 
					 rt_kprintf("\r\n 中心设置里程:  %d  1/10km/h\r\n",resualtu32);  
					

	                break;
	 case  0x0081: // 车辆所在的省域ID 
	                if(infolen!=2)
						  break;
					Vechicle_Info.Dev_ProvinceID=(Content[0]<<8)+Content[1];
                    DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					rt_kprintf("\r\n 车辆所在省域ID: 0x%X \r\n",Vechicle_Info.Dev_ProvinceID); 
	                break;
	 case  0x0082: // 车辆所在市域ID
	                if(infolen!=2)
						  break;
					Vechicle_Info.Dev_ProvinceID=(Content[0]<<8)+Content[1];
                    DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));  
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					rt_kprintf("\r\n 车辆所在市域ID: 0x%X \r\n",Vechicle_Info.Dev_ProvinceID); 
	                break;
	 case  0x0083: // 公安交通管理部门颁发的机动车号牌
	                if(infolen<4)
						  break;					
					memset(Vechicle_Info.Vech_Num,0,sizeof(Vechicle_Info.Vech_Num));
					memcpy(Vechicle_Info.Vech_Num,Content,infolen);
					DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));	
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					rt_kprintf("\r\n 机动车驾驶证号: %s  \r\n",Vechicle_Info.Vech_Num);  
	                break;
	 case  0x0084: // 车牌颜色  按照国家规定
	                if(infolen!=1)
						  break;
					Vechicle_Info.Dev_Color=Content[0];           
					DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));	
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					rt_kprintf("\r\n 车辆颜色: %d  \r\n",Vechicle_Info.Dev_Color);    

					//  redial();
					// idip("clear");
	                break;
       //--------------- BD  新增----------------------------------
       case  0x001A: //  IC卡 主服务器或域名
                                    memset(reg_in,0,sizeof(reg_in)); 
					  memcpy(reg_in,Content,infolen);
					 //----------------------------	
					 
					 i =str2ip((char*)reg_in, SysConf_struct.BD_IC_main_IP);
					 if (i <= 3)
					 {
					  rt_kprintf("\r\n IC  主 域名: %s \r\n",reg_in); 
					  
					  memset(SysConf_struct.BD_IC_DNSR,0,sizeof(SysConf_struct.BD_IC_DNSR));
					  memcpy(SysConf_struct.BD_IC_DNSR,(char*)Content,infolen);   
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					  break;
					 }
					 memset(reg_str,0,sizeof(reg_str));
					 IP_Str((char*)reg_str, *( u32 * ) SysConf_struct.BD_IC_main_IP); 		  
				       Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					rt_kprintf("\r\n IC  主IP: %d.%d.%d.%d \r\n",SysConf_struct.BD_IC_main_IP[0],SysConf_struct.BD_IC_main_IP[1],SysConf_struct.BD_IC_main_IP[2],SysConf_struct.BD_IC_main_IP[3]);    
			break;
       case  0x001B:// IC 卡  主TCP端口
                                    SysConf_struct.BD_IC_TCP_port=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)+Content[3];
                                 	 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					  rt_kprintf("\r\n IC  TCP Port: %d \r\n",SysConf_struct.BD_IC_TCP_port); 	
                                     //  IC 卡中心
					  DataLink_IC_Socket_set(SysConf_struct.BD_IC_main_IP,SysConf_struct.BD_IC_TCP_port,0); 
	              break;
       case  0x001C:// IC卡   主UDP 端口
                                     SysConf_struct.BD_IC_UDP_port=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)+Content[3];
                                    Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					 rt_kprintf("\r\n IC  UDP Port : %d \r\n",SysConf_struct.BD_IC_UDP_port); 	 			
	              break;
       case  0x001D:// IC 卡备用服务器和端口
                                    memset(reg_in,0,sizeof(reg_in)); 
					  memcpy(reg_in,Content,infolen);
					 //----------------------------						 
					 i =str2ip((char*)reg_in, SysConf_struct.BD_IC_Aux_IP);
					 if (i <= 3)
					 {
					  rt_kprintf("\r\n IC  主 域名: %s \r\n",reg_in); 
					  
					  memset(SysConf_struct.BD_IC_DNSR_Aux,0,sizeof(SysConf_struct.BD_IC_DNSR_Aux));
					  memcpy(SysConf_struct.BD_IC_DNSR_Aux,(char*)Content,infolen);   
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					  break;
					 }
					 memset(reg_str,0,sizeof(reg_str));
					 IP_Str((char*)reg_str, *( u32 * ) SysConf_struct.BD_IC_Aux_IP); 		  
				        Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					 rt_kprintf("\r\n IC  备用IP: %d.%d.%d.%d \r\n",SysConf_struct.BD_IC_Aux_IP[0],SysConf_struct.BD_IC_Aux_IP[1],SysConf_struct.BD_IC_Aux_IP[2],SysConf_struct.BD_IC_Aux_IP[3]);    

	              break;
	case   0x0031://  电子围栏半径(非法位移阈值)
                            if(infolen!=2)
						break;					
				JT808Conf_struct.BD_CycleRadius_DoorValue=(Content[0]<<8)+Content[1]; 
	                     Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
		  		rt_kprintf("\r\n 电子围栏半径(非法移动阈值): %d m\r\n",JT808Conf_struct.BD_CycleRadius_DoorValue);  
			break;
	case   0x005B: // 超速报警预警差值   1/10 KM/h
	                     if(infolen!=2)
						break;					
				JT808Conf_struct.BD_MaxSpd_preWarnValue=(Content[0]<<8)+Content[1]; 
	                     Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
		  		rt_kprintf("\r\n 超速报警预警差值: %d x 0.1km/h\r\n",JT808Conf_struct.BD_MaxSpd_preWarnValue);

	              break;
	case   0x005C: // 疲劳驾驶阈值  单位:s
	                    if(infolen!=2)
						break;					
				JT808Conf_struct.BD_TiredDrv_preWarnValue=(Content[0]<<8)+Content[1]; 
	                     Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
		  		rt_kprintf("\r\n 疲劳驾驶阈值: %d s\r\n",JT808Conf_struct.BD_TiredDrv_preWarnValue);	
	              break;
	case   0x005D:// 碰撞报警参数设置 
	        	       if(infolen!=2)
						break;					
				JT808Conf_struct.BD_Collision_Setting=(Content[0]<<8)+Content[1]; 
	                     Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
		  		rt_kprintf("\r\n 碰撞报警参数设置 : %x \r\n",JT808Conf_struct.BD_MaxSpd_preWarnValue);	
	              break;
	case   0x005E: // 侧翻报警参数设置   默认30度
		              if(infolen!=2)
						break;					
				JT808Conf_struct.BD_Laydown_Setting=(Content[0]<<8)+Content[1]; 
	                     Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
		  		rt_kprintf("\r\n侧翻报警参数设置: %x r\n",JT808Conf_struct.BD_Laydown_Setting);	
	              break;
       //---  CAMERA 
	case   0x0064://  定时拍照控制
	                        if(infolen!=4)
						break;
	                         JT808Conf_struct.BD_CameraTakeByTime_Settings=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)+Content[3];
                                Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
				    rt_kprintf("\r\n 定时拍照控制: %X\r\n",JT808Conf_struct.BD_CameraTakeByTime_Settings); 			
	              break;
	case   0x0065://  定距离拍照控制
                               if(infolen!=4)
						break;
				    JT808Conf_struct.BD_CameraTakeByDistance_Settings=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)+Content[3];
                                Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
				    rt_kprintf("\r\n  定距离拍照控制: %X\r\n",JT808Conf_struct.BD_CameraTakeByDistance_Settings); 	
                     break;
	 //--- GNSS
	case    0x0090: // GNSS 定位模式 
		           	
                                   JT808Conf_struct.BD_EXT.GNSS_Mode=Content[0]; 
					  rt_kprintf("\r\n  GNSS Value= 0x%2X	\r\n",Content[0]);		   
				        switch(JT808Conf_struct.BD_EXT.GNSS_Mode)
				      	{
		                         case 0x01:  // 单 GPS 定位模式                        
								   gps_mode("2");
							          Car_Status[1]&=~0x0C; // clear bit3 bit2      1100
							          Car_Status[1]|=0x04; // Gps mode   0100
								 break;
					    case  0x02:  // 	单BD2 定位模式
					                        gps_mode("1");
								    Car_Status[1]&=~0x0C; // clear bit3 bit2 
							           Car_Status[1]|=0x08; // BD mode	1000		
						               break;
					    case  0x03: //  BD2+GPS 定位模式 
								   gps_mode("3");
								    Car_Status[1]&=~0x0C; // clear bit3 bit2 
							           Car_Status[1]|=0x0C; // BD+GPS  mode	1100	
						               break;							   	
				      	}				
                                  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
	              break;
       case    0x0091: // GNSS 波特率
	                      JT808Conf_struct.BD_EXT.GNSS_Baud=Content[0];
				switch(JT808Conf_struct.BD_EXT.GNSS_Baud)
				{ 
				    case 0x00:  //  4800
                                                  JT808Conf_struct.BD_EXT.GNSS_Baud_Value=4800;    
							//rt_thread_delay(5);						  
							//gps_write("$PCAS01,0*1C\r\n",14);	
							//rt_thread_delay(5);	
							// gps_baud( 4800 );
						        break;
				     case 0x01: //9600   --default 
				                     JT808Conf_struct.BD_EXT.GNSS_Baud_Value=9600;
							//rt_thread_delay(5);			 
                                                 //gps_write("$PCAS01,1*1D\r\n",14);	
							//rt_thread_delay(5);
							//gps_baud( 9600 );
						        break;
				     case  0x02: // 19200
				                     JT808Conf_struct.BD_EXT.GNSS_Baud_Value=19200;
							//rt_thread_delay(5);			 
                                                 //gps_write("$PCAS01,2*1E\r\n",14);	
							//rt_thread_delay(5);	
							//gps_baud( 19200 );
					               break;
				     case  0x03://  38400
                                                  JT808Conf_struct.BD_EXT.GNSS_Baud_Value=38400;
							 //rt_thread_delay(5);						  
							 //gps_write("$PCAS01,3*1F\r\n",14);	
							 //rt_thread_delay(5);	
							 //gps_baud( 38400 ); 
					               break;
				     case  0x04:// 57600
				                      JT808Conf_struct.BD_EXT.GNSS_Baud_Value=57600;
							 //rt_thread_delay(5);			  
                                                  //gps_write("$PCAS01,4*18\r\n",14);	
							 //rt_thread_delay(5);		
							 //gps_baud( 57600 );
					               break;
				     case   0x05:// 115200
				                      JT808Conf_struct.BD_EXT.GNSS_Baud_Value=115200; 
							//rt_thread_delay(5);			  
                                                 // gps_write("$PCAS01,5*19\r\n",14);	
							 //rt_thread_delay(5);	
							// gps_baud( 115200 ); 
					               break;				    				

				}
		      	 //---UART_GPS_Init(baud);  //   修改串口波特率 
		      	 rt_thread_delay(20);
                      // BD_EXT_Write();   
			rt_kprintf("\r\n 中心设置GNSS 波特率:  %d s\r\n",JT808Conf_struct.BD_EXT.GNSS_Baud_Value); 

	              break;
       case    0x0092: // GNSS 模块详细定位数据输出频率

	              break;
       case    0x0093://  GNSS 模块详细定位数据采集频率  1

	              break;
       case    0x0094:// GNSS 模块详细定位数据上传方式
                   if(Content[0]==0x01)
                       rt_kprintf("\r\n 按时间上传");
				   if(Content[0]==0x00)
                       rt_kprintf("\r\n 本地存储不上传"); 
				   
	              break;
       case    0x0095://  GNSS 模块详细上传设置
                        // GNSS_rawdata.SendDuration=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];;  // 
	              break;
	 //----CAN--
	case      0x0100://  CAN  总线通道 1  采集间隔              0   : 表示不采集 
                     if(infolen!=4)
						break;					
			CAN_trans.can1_sample_dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                     Can_RXnum=0;
			Can_sdnum=0;	
			Can_same=0; 
		     break;
	case    0x0101://  CAN  总线通道 1 上传时间间隔    0 :  表示不上传
                    if(infolen!=2)
						break;					
			CAN_trans.can1_trans_dur=(Content[0]<<8) +Content[1];
	           
	              break;
	case    0x0102://  CAN  总线通道 2  采集间隔              0   : 表示不采集 
                     if(infolen!=4)
						break;					
			CAN_trans.can2_sample_dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
	              break;
	case    0x0103://  CAN  总线通道 2 上传时间间隔    0 :  表示不上传
                     if(infolen!=2)
						break;					
			CAN_trans.can2_trans_dur=(Content[0]<<8) +Content[1];	           
	              break;
	case    0x0110://  CAN 总线ID 单独采集设置
                     CAN_trans.canid_2_NotGetID= ((Content[4]&0x1F)<<24)+(Content[5]<<16)+(Content[6]<<8) +Content[7];
		       rt_kprintf("\r\n不采集ID  0x0110= %08X\r\n",CAN_trans.canid_2_NotGetID);   		  

	            break;	
	case    0x0111://  CAN 总线ID 单独采集设置 其他
                    if(infolen!=8)
						break;		
			OutPrint_HEX("0x0111",Content,8);			
		       memcpy(	CAN_trans.canid_1,Content,8);
			memset(CAN_trans.canid_1_Rxbuf,0,sizeof(CAN_trans.canid_1_Rxbuf));
			CAN_trans.canid_1_RxWr=0;    // clear  write
			CAN_trans.canid_timer=0;
			CAN_trans.canid_0705_sdFlag=0;
           
		        //------ 解析赋值 --------
                      CAN_trans.canid_1_sample_dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                      if(Content[4]&0x40) // bit 30   
			 	   CAN_trans.canid_1_ext_state=1;
			 else
			 	   CAN_trans.canid_1_ext_state=0;
                       CAN_trans.canid_1_Filter_ID=((Content[4]&0x1F)<<24)+(Content[5]<<16)+(Content[6]<<8) +Content[7];

                       rt_kprintf("\r\n FilterID=%08X, EXTstate: %d   can1_samle=%d ms   canid_1_sample_dur=%dms    Trans_dur=%d s\r\n", CAN_trans.canid_1_Filter_ID, CAN_trans.canid_1_ext_state,CAN_trans.can1_sample_dur,CAN_trans.canid_1_sample_dur,CAN_trans.can1_trans_dur);
			 
	             break;
	 default:
				return false;
   }

    return true;
}

//--------------------------------------------------------------------
u8  CentreSet_subService_8105H(u32 Control_ID, u8 infolen, u8 *Content )
{

   switch(Control_ID) 
   {
      case 1:  //  无线升级参数  参数之间采用分号分隔   指令格式如下:
             /*
URL 地址；拨号名称；拨号用户名；拨号密码；地址；TCP端口；UDP端口；制造商ID; 硬件版本；固件版本；连接到指定服务器指定是服务器时限；
           若某个参数无数值，则放空
               */
             rt_kprintf("\r\n 无线升级 \r\n");  
			 rt_kprintf("\r\n 内容: %s\r\n",Content); 
		     break;
	  case 2:  // 控制终端连接指定服务器
	        /*
连接控制；监管平台鉴权码；拨号点名称； 拨号用户名；拨号密码；地址；TCP端口；UDP端口；连接到指定服务器时限
           若每个参数无数值，则放空
	         */  
	         rt_kprintf("\r\n 终端控制连接指定服务器\r\n");  
			 rt_kprintf("\r\n 内容: %s\r\n",Content);
			 break;
	  case 3:  //  终端关机
              SD_ACKflag.f_CentreCMDack_0001H=5; 
			  rt_kprintf("\r\n 终端关机 \r\n");  
	         break;
      case 4:  //  终端复位
               SD_ACKflag.f_CentreCMDack_0001H=3;
			   rt_kprintf("\r\n 终端复位 \r\n");          
	         break;
	  case 5: //   终端恢复出厂设置		 		  	       
	           /* if(SysConf_struct.Version_ID==SYSID)   //  check  wether need  update  or not 
	          	{                    
					SysConf_struct.Version_ID=SYSID+1;   
					Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					Systerm_Reset_counter=Max_SystemCounter;
					ISP_resetFlag=2;    //   借助远程下载重启机制复位系统 
	          	}
				rt_kprintf("\r\n 恢复出厂设置 \r\n"); */ 
			 break;
	  case 6: //   关闭数据通信
             SD_ACKflag.f_CentreCMDack_0001H=5; 
			 rt_kprintf("\r\n 关闭数据通信 \r\n"); 
	         break;
	  case 7: //   关闭所有无线通信
             SD_ACKflag.f_CentreCMDack_0001H=5; 
			 rt_kprintf("\r\n 关闭所有通信 \r\n");   
		     break;
	  default:
	  	     return false;			 

   }
      return  true;
}

//-------------------------------------------------------------------
void CenterSet_subService_8701H(u8 cmd,  u8*Instr)
{  
   TDateTime now;
   u32  reg_dis=0,regdis=0,reg2=0; 
  
  switch(cmd)
   {
	 case 0x82: //	  中心设置车牌号  
				memset(Vechicle_Info.Vech_VIN,0,sizeof(Vechicle_Info.Vech_VIN));
				memset(Vechicle_Info.Vech_Num,0,sizeof(Vechicle_Info.Vech_Num));
				memset(Vechicle_Info.Vech_Type,0,sizeof(Vechicle_Info.Vech_Type));	
				
				 //-----------------------------------------------------------------------
				 memcpy(Vechicle_Info.Vech_VIN,Instr,17);
				 memcpy(Vechicle_Info.Vech_Num,Instr+17,12);
				 memcpy(Vechicle_Info.Vech_Type,Instr+29,12); 

				DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));   
				WatchDog_Feed();
				DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
				WatchDog_Feed();
				DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
				break;
	 case 0xC2: //设置记录仪时钟
			    // 没啥用，给个回复就行，俺有GPS校准就够了 
			    //  年月日 时分秒 BCD
			    now.year=(Instr[0]>>4)*10+(Instr[0]&0x0F);  
				now.month=(Instr[1]>>4)*10+(Instr[1]&0x0F);   
				now.day=(Instr[2]>>4)*10+(Instr[2]&0x0F);  
				now.hour=(Instr[3]>>4)*10+(Instr[3]&0x0F);  
				now.min=(Instr[4]>>4)*10+(Instr[4]&0x0F);  
				now.sec=(Instr[5]>>4)*10+(Instr[5]&0x0F);      
				now.week=1;      
	            Device_RTC_set(now);    
		        break;

	 case 0xC3: //车辆速度脉冲系数（特征系数）
	              // 前6 个是当前时间
	                  now.year=(Instr[0]>>4)*10+(Instr[0]&0x0F);  
				now.month=(Instr[1]>>4)*10+(Instr[1]&0x0F);   
				now.day=(Instr[2]>>4)*10+(Instr[2]&0x0F);  
				now.hour=(Instr[3]>>4)*10+(Instr[3]&0x0F);  
				now.min=(Instr[4]>>4)*10+(Instr[4]&0x0F);  
				now.sec=(Instr[5]>>4)*10+(Instr[5]&0x0F);      
				now.week=1;      
	            Device_RTC_set(now);   
				 JT808Conf_struct.Vech_Character_Value=(Instr[6]<<8)+(u32)Instr[7]; // 特征系数  速度脉冲系数
				 JT808Conf_struct.DF_K_adjustState=0; 		
		                ModuleStatus&=~Status_Pcheck; 
	                      Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
		        break;
	 case 0x83: //  记录仪初次安装时间
                
			    memcpy(JT808Conf_struct.FirstSetupDate,Instr,6); 
				Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
	            break;
	 case 0x84: // 设置信号量配置信息
	            memcpy(Setting08,Instr,80);
				break;
	  
	 case 0xC4: //   设置初始里程	
	          
            /*   Time2BCD( Original_info + Original_info_Wr );                       // 记录仪实时时间
				Original_info_Wr					+= 6;
				Original_info[Original_info_Wr++]	= 0x13;                         // 初次安装时间
				Original_info[Original_info_Wr++]	= 0x03;
				Original_info[Original_info_Wr++]	= 0x01;
				Original_info[Original_info_Wr++]	= 0x08;
				Original_info[Original_info_Wr++]	= 0x30;
				Original_info[Original_info_Wr++]	= 0x26; 
			*/
                 now.year=(Instr[0]>>4)*10+(Instr[0]&0x0F);  
				now.month=(Instr[1]>>4)*10+(Instr[1]&0x0F);   
				now.day=(Instr[2]>>4)*10+(Instr[2]&0x0F);  
				now.hour=(Instr[3]>>4)*10+(Instr[3]&0x0F);  
				now.min=(Instr[4]>>4)*10+(Instr[4]&0x0F);  
				now.sec=(Instr[5]>>4)*10+(Instr[5]&0x0F);      
				now.week=1;      
	            Device_RTC_set(now);   


			
			memcpy(JT808Conf_struct.FirstSetupDate,Instr+6,6);  
			//Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
			     // instr     + 12      设置初始里程的位置   BCD
			     
			reg_dis=(Instr[12]>>4)*10000000+(Instr[12]&0x0F)*1000000+(Instr[13]>>4)*100000+(Instr[13]&0x0F)*10000 \
			       +(Instr[14]>>4)*1000+(Instr[14]&0x0F)*100+(Instr[15]>>4)*10+(Instr[15]&0x0F);  

			  Distance_m_u32=reg_dis*100;  
			  DF_Write_RecordAdd(Distance_m_u32,DayStartDistance_32,TYPE_DayDistancAdd);
			  JT808Conf_struct.DayStartDistance_32=DayStartDistance_32;
			  JT808Conf_struct.Distance_m_u32=Distance_m_u32;	
			//Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
			//--- 初始里程
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			// -- 累积里程 3个字节 单位0.1km    6位
			regdis								= JT808Conf_struct.Distance_m_u32 / 100; //单位0.1km
			reg2								= regdis / 100000;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= ( reg2 << 4 ) + ( regdis % 100000 / 10000 );
			Original_info[Original_info_Wr++]	= ( ( regdis % 10000 / 1000 ) << 4 ) + ( regdis % 1000 / 100 );
			Original_info[Original_info_Wr++]	= ( ( regdis % 100 / 10 ) << 4 ) + ( regdis % 10 );
			
			
		/*	chushilicheng[0]	=Instr[12];
			chushilicheng[1]	=Instr[13];
			chushilicheng[2]    =Instr[14];
			chushilicheng[3]	= Instr[15];  

			leijilicheng[0]	=Instr[16];
			leijilicheng[1]	=Instr[17];
			leijilicheng[2] =Instr[18];
			leijilicheng[3]	= Instr[19];*/
	              
	             break;
	 default:
	 	        Recode_Obj.Error=2; // 设置错误
				break;
   }

   if(Recode_Obj.Error!=2)
        VDR_product_14H(cmd);     
    
}
//-------------------------------------------
void  Media_Start_Init( u8  MdType , u8  MdCodeType)
{  
  MediaObj.Media_Type=MdType;//	指定当前传输多媒体的类型   0  表示图片
  MediaObj.Media_CodeType=MdCodeType;  //  多媒体编码格式   0  表示JPEG	格式 
  MediaObj.SD_media_Flag=1;   //  置多媒体事件信息发送标志位  ，开始图片传输  
  MediaObj.SD_Eventstate=1;		  //  开始处于发送状态 
  MediaObj.RSD_State=0;  
  MediaObj.RSD_Timer=0;
  MediaObj.RSD_total=0;
  MediaObj.RSD_Reader=0;
  MediaObj.SD_Data_Flag=0; 
  MediaObj.Media_transmittingFlag=0;      
   //------------  add  for debuging   --------------
  // Media_Clear_State();  //  clear  
 /*  if(0==MediaObj.Media_Type)
	{
	   Photo_sdState.photo_sending=enable; 
	   Photo_sdState.SD_packetNum=1; // 第一包开始  
	   rt_kprintf("\r\n 开始上传照片! ....\r\n");  												
    }
  */
   //---------------------------------------------------- 
}

void Media_Clear_State(void)
{
     // 不清楚Meia Type
    MediaObj.MaxSd_counter=0;
    MediaObj.SD_Eventstate=0;
    MediaObj.SD_timer=0;
    MediaObj.SD_media_Flag=0;	
	MediaObj.SD_Data_Flag=0; 
	MediaObj.RSD_State=0;
	MediaObj.RSD_Timer=0;
	MediaObj.RSD_total=0;
	MediaObj.RSD_Reader=0;
	MediaObj.Media_transmittingFlag=0;
}
void  Media_Timer(void)
{
  if(1==MediaObj.SD_Eventstate) 
  	{
  	  MediaObj.SD_timer++;
      if(MediaObj.SD_timer>6) 
      	{
            MediaObj.SD_timer=0;			
            MediaObj.SD_media_Flag=1; 
			MediaObj.MaxSd_counter++;
			if(MediaObj.MaxSd_counter>5)
                   Media_Clear_State();  
      	}

  	}
}
void Media_RSdMode_Timer(void)
{
    if((1==MediaObj.RSD_State))
	   	{ 
	        MediaObj.RSD_Timer++;
			if(MediaObj.RSD_Timer>12)
				{
	              MediaObj.RSD_Timer=0;
				  MediaObj.SD_Data_Flag=1; // 置重传发送多媒体信息标志位
				  switch(MediaObj.Media_Type)    //   图片重传包数
				  	{
				  	 case 0://  图像
				  	         Photo_sdState.SD_packetNum=MediaObj.Media_ReSdList[MediaObj.RSD_Reader];
	                         Photo_sdState.SD_flag=1;
							  rt_kprintf("\r\n 补报 %d 包\r\n",Photo_sdState.SD_packetNum); 
							 break;
					 case 1:// 音频                             
				  	         Sound_sdState.SD_packetNum=MediaObj.Media_ReSdList[MediaObj.RSD_Reader];
	                         Sound_sdState.SD_flag=1;  
					         break;
				  #if 0			 
					 case 2:// 视频
							 Video_sdState.SD_packetNum=MediaObj.Media_ReSdList[MediaObj.RSD_Reader];
							 Video_sdState.SD_flag=1;  					          
					         break;
				 #endif			 
				     default:
					 	     break;

				  	}	
				}
	   	} 
      else
	  if(2==MediaObj.RSD_State)
	  {
          MediaObj.RSD_Timer++;		  
		  if(MediaObj.RSD_Timer>180)   //   如果状态一直在等待且超过30s择清除状态 
		  { 
		     switch (MediaObj.Media_Type)
	         {
	          case 0 : // 图像
	                  Photo_send_end();  // 拍照上传结束
	                  
			          break;
			  case 1 : // 音频
			         #ifdef REC_VOICE_ENABLE
			           Sound_send_end();  
			         #endif
			          break;
			  case 2 : // 视频
                            //   Video_send_end();
			          break;
			  default:
			  	      break;
	         } 
            Media_Clear_State();
			rt_kprintf("\r\n 信息重传超时结束! \r\n");
			
			Check_MultiTakeResult_b4Trans();  // 多路摄像头拍照状态检测
			
		  }

	  }
   


	
}
#ifdef   MEDIA
//-------- photo send -------------------------------------
void Photo_send_start(u16 Numpic)  
{
  
//  UINT ByteRead;  
 // FIL FileCurrent; 

     rt_kprintf("   \r\n  Photo_send_start =%d \r\n",Numpic);        
	 Photo_sdState.photo_sending=other;	//disable        
	 Photo_sdState.SD_flag=0;  
	 Photo_sdState.SD_packetNum=1; // 从1 开始
     Photo_sdState.Exeption_timer=0;
	 
	if(Camera_Number==1)
	             PicFileSize=Api_DFdirectory_Query(camera_1, 1);
	else
	if(Camera_Number==2)
		     PicFileSize=Api_DFdirectory_Query(camera_2, 1);	   
	else
	if(Camera_Number==3)
		     PicFileSize=Api_DFdirectory_Query(camera_3, 1);   
	else
	if(Camera_Number==4)
		     PicFileSize=Api_DFdirectory_Query(camera_4, 1);    		
		
		
      //	 DF_ReadFlash(pic_current_page, 0,PictureName, 23);
     //      Camera_Number=PictureName[18]; 
     //      memcpy((u8*)&PicFileSize,PictureName+19,4); 
	 //pic_current_page++; //指向文件内容 
	 //pic_PageIn_offset=0;// 图片读取页内偏移地址 
	// rt_kprintf("\r\n    open Pic =%s",PictureName); 
	  
	  if(PicFileSize%512)
	     Photo_sdState.Total_packetNum=PicFileSize/512+1;
	  else
	  	 Photo_sdState.Total_packetNum=PicFileSize/512;

	 rt_kprintf("\r\n    Camera %d  ReadpicStart total :%d ，Pagesize: %d Bytes\r\n\r\n",Camera_Number,Photo_sdState.Total_packetNum,PicFileSize);    
	 if((Camera_Number==0)||(Photo_sdState.Total_packetNum==0))
	 {
	   Photo_send_end(); // clear  state 
       rt_kprintf("\r\n  图片总包数为空 ，摄像头序号为0 ，发送拍照失败到中心 \r\n");    
	 }


      // -------  MultiMedia Related --------
      Media_Start_Init(0,0);  // 0: 图片格式 0: JPEG 文件格式      

}

#ifdef REC_VOICE_ENABLE
u8  Sound_send_start(void)
{
  u8 sound_name[20]; 
//  u16  i;
//  u8  oldstate=0;
//  u16 i2,j; 
 // u8  WrieEnd=0,LeftLen=0; 
  

  Sound_sdState.photo_sending=disable;  
  Sound_sdState.SD_flag=0;  
  Sound_sdState.SD_packetNum=1; // 从1 开始
  Sound_sdState.Exeption_timer=0;
   
 //---  Speex_Init();	 // speachX 初始化	 
  // 1. 查找最新的文件
/*  memset((u8*)&MediaIndex,0,sizeof(MediaIndex)); 
  for(i=0;i<8;i++)
  {
	 Api_RecordNum_Read(voice_index, i+1, (u8*)&MediaIndex, sizeof(MediaIndex));
        if(MediaIndex.Effective_Flag==1)
        {  
            break;
	  }       
  }
  if(MediaIndex.Effective_Flag)  	
  { 
	rt_kprintf("\r\n 索引filename:%s\r\n",MediaIndex.FileName);
  }	
  else
  	{
  	   rt_kprintf("\r\n 没有已存储的音频文件 \r\n");
  	   return false	;
  	}

*/
    
	//	2.	创建wav 文件   

	//	3. 文件大小
	  // file name
	memset(sound_name,0,sizeof(sound_name));
	DF_ReadFlash(SoundStart_offdet, 4, sound_name,20);
       SrcFileSize=Api_DFdirectory_Query(voice, 1);
     //  Sound_sdState.Total_packetNum=(SrcFileSize/512); // 每包100个字节
        if(SrcFileSize%512)
	     Sound_sdState.Total_packetNum=SrcFileSize/512+1;
	  else
	  	 Sound_sdState.Total_packetNum=SrcFileSize/512; 
	rt_kprintf("\r\n	文件名: %s大小: %d Bytes  totalpacketnum=%d \r\n",sound_name,SrcFileSize,Sound_sdState.Total_packetNum);
  
    // -------  MultiMedia Related --------
      Media_Start_Init(1,3); // 音频  wav 格式   0:JPEG ;   1: TIF ;   2:MP3;  3:WAV  4: WMV  其他保留   
                                           //    5   amr
     return true;  
}
#endif

void Photo_send_TimeOut(void)
{ 
   if(Photo_sdState.photo_sending)
   {
       Photo_sdState.photo_sendTimeout++;
	   if(Photo_sdState.photo_sendTimeout>150)
	   	{
	   	   Photo_sdState.photo_sendTimeout=0;   
           Photo_send_end();// clear
	   	}
   }
}


void Photo_send_end(void)
{
        Photo_sdState.photo_sending=0;
	 Photo_sdState.SD_flag=0;
	 Photo_sdState.SD_packetNum=0;
	 Photo_sdState.Total_packetNum=0;
	 Photo_sdState.Exeption_timer=0;
	 MediaObj.Media_transmittingFlag=0;  // clear  
	 Media_Clear_State(); 
} 

#ifdef REC_VOICE_ENABLE
void Sound_send_end(void) 
{
       Sound_sdState.photo_sending=0;
	 Sound_sdState.SD_flag=0;
	 Sound_sdState.SD_packetNum=0;
	 Sound_sdState.Total_packetNum=0;
	  Sound_sdState.Exeption_timer=0;
	 MediaObj.Media_transmittingFlag=0;  // clear  
	 mp3_sendstate=0;
	#ifdef REC_VOICE_ENABLE
	 VocREC.running=0;   // clear
  #endif
	 Media_Clear_State(); 
} 
#endif


#ifdef REC_VOICE_ENABLE
void Sound_Timer(void)
{
  
  if((Sound_sdState.photo_sending==enable)&&(1==MediaObj.Media_Type)) // 音频
  {
	 if((Sound_sdState.SD_packetNum<=Sound_sdState.Total_packetNum+1)&&(0==MediaObj.RSD_State))  
	  {  //  一下定时器在	在顺序发送过过程中	 和   收到重传开始后有效
		  Sound_sdState.Data_SD_counter++;
		  if( Sound_sdState.Data_SD_counter>14)         
			  {
				 Sound_sdState.Data_SD_counter=0;
				  Sound_sdState.Exeption_timer=0;
				 Sound_sdState.SD_flag=1;
				 MediaObj.SD_Data_Flag=1; 
				 
				 //rt_kprintf("\r\n Sound  Transmit set Flag \r\n"); 				
			  }
	  }   
  }
}
#endif

void Photo_Timer(void)   
{
	if((Photo_sdState.photo_sending==enable)&&(0==MediaObj.Media_Type))
	{
	   if((Photo_sdState.SD_packetNum<=Photo_sdState.Total_packetNum+1)&&(0==MediaObj.RSD_State))  
	   	{  //  一下定时器在   在顺序发送过过程中   和   收到重传开始后有效 
	        Photo_sdState.Data_SD_counter++;
			if( Photo_sdState.Data_SD_counter>14)  //40   12     
				{
	               Photo_sdState.Data_SD_counter=0;
				   Photo_sdState.Exeption_timer=0;			
	               Photo_sdState.SD_flag=1;
				   MediaObj.SD_Data_Flag=1;   				   
				}
	   	} 	
	}
}

void Meida_Trans_Exception(void) 
{
    u8  resualt=0;
	
        if(Photo_sdState.photo_sending==enable)
	 {
	     if( Photo_sdState.Exeption_timer++>50)
	     	{
                  Photo_send_end();
		    resualt=1;		  
	     	}
        }
	#ifdef REC_VOICE_ENABLE	
	 else 
	 if(Sound_sdState.photo_sending==enable)	 
	 { 
	    if( Sound_sdState.Exeption_timer++>50)
	     	{
                  Sound_send_end();
		    resualt=2;		  
	     	}
	}
	#endif

	if(resualt)
		rt_kprintf("\r\n   Media  Trans  Timeout  resualt: %d\r\n", resualt);
		
}

 void Media_Timer_Service(void)
{
       //----------------------------------
   if(DataLink_Status()&&(DEV_Login.Operate_enable==2))
   {
     if(Photo_sdState.photo_sending==enable)
	  Photo_Timer(); 
	 
	 #ifdef REC_VOICE_ENABLE
	 else 
	 if(Sound_sdState.photo_sending==enable)	 
	  Sound_Timer(); 
	 #endif
	// else
	 // Video_Timer();  	 
	  Media_RSdMode_Timer(); 
   }   
}

#endif

//------------------------------------------------------------
void DataTrans_Init(void)
{
   DataTrans.Data_RxLen=0; 
   DataTrans.Data_TxLen=0;
   DataTrans.Tx_Wr=0; 
   memset(DataTrans.DataRx,0,sizeof((const char*)DataTrans.DataRx));   
   memset(DataTrans.Data_Tx,0,sizeof((const char*)DataTrans.Data_Tx)); 
}
//------------------------------------------------------------
void DoorCameraInit(void)
{
   DoorOpen.currentState=0;
   DoorOpen.BakState=0;
}
//-----------------------------------------------------------
void Spd_ExpInit(void)
{
   speed_Exd.current_maxSpd=0;
   speed_Exd.dur_seconds=0;
   speed_Exd.excd_status=0;   
   memset((char*)(speed_Exd.ex_startTime),0,5);
   speed_Exd.speed_flag=0;
}

//  河北天地通多媒体事件信息上传报文应答不好，所以单独处理  
 void Multimedia_0800H_ACK_process(void)
 {
      						 Media_Clear_State();  //  clear 
											 
                                            if(0==MediaObj.Media_Type)
											{
											   MediaObj.Media_transmittingFlag=1; 
											   PositionSD_Enable(); 
											   Current_UDP_sd=1;
											   
											   Photo_sdState.photo_sending=enable; 
											   Photo_sdState.SD_packetNum=1; // 第一包开始 
											   PositionSD_Enable();  //   使能上报
											   rt_kprintf("\r\n 开始上传照片! ....\r\n");   												
                            	            }
											else
											if(1==MediaObj.Media_Type)
											{
											   MediaObj.Media_transmittingFlag=1;  
											   
											   Sound_sdState.photo_sending=enable; 
											   Sound_sdState.SD_packetNum=1; // 第一包开始 
											   PositionSD_Enable();  //   使能上报 
											   Current_UDP_sd=1;
											   rt_kprintf("\r\n 开始上传音频! ....\r\n");    												
                            	            }	
										/*	else
											if(2==MediaObj.Media_Type)
											{
											   MediaObj.Media_transmittingFlag=1;   
											   PositionSD_Enable();  //   使能上报
											   Current_UDP_sd=1;
											   Video_sdState.photo_sending=enable; 
											   Video_sdState.SD_packetNum=1; // 第一包开始   
											   rt_kprintf("\r\n 开始上传视频! ....\r\n");    	
									   
                            	            }	
										*/	

 }

u16  Instr_2_GBK(u8 *SrcINstr, u16 Inlen, u8* DstOutstr )
{
  u16 i=0,j=0;
  
	
	//对非GBK编码处理------------------------------------
	for(i=0,j=0;i<Inlen;i++)
		{
		if((SrcINstr[i]>=0xA1)&&(SrcINstr[i+1]>=0xA0))
			{
			DstOutstr[j]=SrcINstr[i];
			DstOutstr[j+1]=SrcINstr[i+1];
			j+=2;
			i++;
			}
		else
			{
			DstOutstr[j]=' ';
			DstOutstr[j+1]=SrcINstr[i];
			j+=2;
			}
		}
	return   j;		    					
}


//-----------------------------------------------------------
void TCP_RX_Process( u8  LinkNum)  //  ---- 808  标准协议 
{
  	  u16	i=0,j=0;//,DF_PageAddr;
	  u16  infolen=0,contentlen=0; 
	  u8  ISP_judge_resualt=0;
	  u32 HardVersion=0;
	 // u8   ireg[5]; 
	  u8   Ack_Resualt=1;
	  u16  Ack_CMDid_8001=0;
	  u8   Total_ParaNum=0;        // 中心设置参数总数 
	  u8   Process_Resualt=0;  //  bit 表示   bit0 表示 1  bit 1 表示2
	  u8   ContentRdAdd=0;   // 当前读取到的地址
	  u8   SubInfolen=0;     // 子信息长度
	  u8   Reg_buf[22];
	  u8   CheckResualt=0;
      u32  reg_u32=0;
	  u16  GB19056infolen=0;  
  //----------------      行车记录仪808 协议 接收处理   -------------------------- 

  //  0.  Decode     
  Protocol_808_Decode();
  //  1.  fliter head
  if(UDP_HEX_Rx[0]!=0x7e)           //   过滤头 
  	   return; 
  //  2.  check Centre Ack
  Centre_CmdID=(UDP_HEX_Rx[1]<<8)+UDP_HEX_Rx[2];  // 接收到中心消息ID   
  Centre_FloatID=(UDP_HEX_Rx[11]<<8)+UDP_HEX_Rx[12];  // 接收到中心消息流水号        


  //  3.   get infolen    ( 长度为消息体的长度)    不分包的话  消息头长度为12 则参与计算校验的长度 =infolen+12
  //infolen =( u16 )((UDP_HEX_Rx[3]&0x3f) << 8 ) + ( u16 ) UDP_HEX_Rx[4];  
  infolen =( u16 )((UDP_HEX_Rx[3]&0x03) << 8 ) + ( u16 ) UDP_HEX_Rx[4];  
  contentlen=infolen+12;    //  参与校验字节的长度   

  //  4.   Check  Fcs 
  
	  FCS_RX_UDP=0; 
  	  //nop;nop;
	  for ( i=0;i<(UDP_DecodeHex_Len-3); i++ ) //先算出收到数据的异或和   
		  {
			  FCS_RX_UDP^=UDP_HEX_Rx[1+i];	  
		  } 
	  //nop;
	  // ------- FCS filter -----------------
	  if( UDP_HEX_Rx[UDP_DecodeHex_Len-2]!=FCS_RX_UDP ) //判断校验结果	    
		  {
			  rt_kprintf("\r\n  Protocolinfolen=%d   UDP_hexRx_len=%d",infolen,UDP_hexRx_len);  
			  rt_kprintf("\r\n808协议校验错误	  Caucate %x  ,RX  %x\r\n",FCS_RX_UDP,UDP_HEX_Rx[UDP_DecodeHex_Len-2]); 
			 // OutPrint_HEX("UDP_HEX_RX",UDP_HEX_Rx,UDP_DecodeHex_Len);
				//-----------------  memset  -------------------------------------
			  memset(UDP_HEX_Rx, 0, sizeof(UDP_HEX_Rx));    
			  UDP_hexRx_len= 0; 
			  return; 
		  }		  
	//  else		  
		 // rt_kprintf("\r\n 808协议校验正确	  Caucate %x  ,RX  %x\r\n",FCS_RX_UDP,UDP_HEX_Rx[UDP_DecodeHex_Len-2]);	   
   
 //   5 .  Classify  Process 
         rt_kprintf("\r\n           CentreCMD = 0x%X  \r\n",Centre_CmdID);  // add for  debug  
         
     switch(Centre_CmdID)
     	{
     	   case  0x8001:  //平台通用应答
     	             // 若没有分包处理的话  消息头长12  从0开始计算第12个字节是消息体得主体

					  //  13 14  对应的终端消息流水号  
					  //  15 16  对应终端的消息
                       Ack_CMDid_8001=(UDP_HEX_Rx[15]<<8)+UDP_HEX_Rx[16];

					   switch(Ack_CMDid_8001)   // 判断对应终端消息的ID做区分处理
					   	{ 
					   	   case 0x0200:    //  对应位置消息的应答
                                      //----- 判断确认是否成功
						              if(0x00!=UDP_HEX_Rx[17])
									  	 break;
									  //--------------------------------
									  if (Warn_Status[3]&0x01) 
									  {
											  StatusReg_WARN_Clear();			  
											  f_Exigent_warning = 0;
											  warn_flag=0;
											  Send_warn_times=0; 
											   StatusReg_WARN_Clear();	
											   rt_kprintf( "\r\n紧急报警收到应答，得清除!\r\n");   
									  }

									  if((Warn_Status[0]&0x20)==0x20) 
								     {
                                             Warn_Status[0]&=~0x20; 
											 rt_kprintf( "\r\n碰撞侧翻收到应答，得清除!\r\n");  
									 } 
									  //------------------------------------  
										rt_kprintf( "\r\nCentre ACK!\r\n");  	
									    Api_cycle_Update();   
									   //-------------------------------------------------------------------
									  /* cycle_read++;   //  收到应答才递增
									   if(cycle_read>=Max_CycleNum)
											   cycle_read=0;
									   ReadCycle_status=RdCycle_Idle;
                                                                 */
                                        //--------------  多媒体上传相关  --------------                                       
									   if(MediaObj.Media_transmittingFlag==1)  // clear									      
									      {
									         MediaObj.Media_transmittingFlag=2;   
											 if(Duomeiti_sdFlag==1)
											  { 
											      Duomeiti_sdFlag=0; 
											      Media_Clear_State();
												Photo_send_end();
												#ifdef REC_VOICE_ENABLE
												Sound_send_end();
												#endif
                                                                                     rt_kprintf("\r\n  手动上报多媒体上传处理\r\n");
											  }	
											 rt_kprintf("\r\n  多媒体信息前的多媒体发送完毕 \r\n");  
									   	  }	
									   
								       break;
				  case 0x0002:  //  心跳包的应答
                                       //  不用判结果了  ---     
                                        JT808Conf_struct.DURATION.TCP_ACK_DurCnter=0;//clear
                                        JT808Conf_struct.DURATION.TCP_SD_state=0; //clear
                                         rt_kprintf( "\r\n  Centre  Heart ACK!\r\n");  
							           break;
                            case 0x0101:  //  终端注销应答
                            case 0x0003:
                                        if(0==UDP_HEX_Rx[17])
                                        {  // 注销成功
                                          
										 memset(Reg_buf,0,sizeof(Reg_buf));
 										 memcpy(Reg_buf,JT808Conf_struct.ConfirmCode,20);
 										 JT808Conf_struct.Regsiter_Status=0; 
 										 Reg_buf[20]=JT808Conf_struct.Regsiter_Status; 
 										Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
										 rt_kprintf("\r\n  终端注销成功!  \r\n"); 
                                         redial();
										 idip("clear");
                                        }  

							           break;
						    case 0x0102:  //  终端鉴权
						                
 						                rt_kprintf("\r\n 收到鉴权结果: %x \r\n",UDP_HEX_Rx[17]); 
						                if(0==UDP_HEX_Rx[17])
                                         {  // 鉴权成功 
                                                          DEV_Login.Operate_enable=2; // 鉴权完成
					                        if(DataLink_Status())   
							                   DataLinkOK_Process();
											 rt_kprintf("\r\n  终端鉴权成功!  \r\n");   
						                }
									   break;
                            case 0x0800:  // 多媒体事件信息上传
                                             rt_kprintf("\r\n 多媒体事件信息上传回应! \r\n");											 
											 Media_Clear_State();  //  clear 
											 
                                            if(0==MediaObj.Media_Type)
											{
											   MediaObj.Media_transmittingFlag=1; 
											   PositionSD_Enable(); 
											   Current_UDP_sd=1;
											   
											   Photo_sdState.photo_sending=enable; 
											   Photo_sdState.SD_packetNum=1; // 第一包开始 
											   PositionSD_Enable();  //   使能上报
											   rt_kprintf("\r\n 开始上传照片! ....\r\n");   												
                            	            }
											else
											if(1==MediaObj.Media_Type)
											{
											   MediaObj.Media_transmittingFlag=1;  
											   
											   Sound_sdState.photo_sending=enable; 
											   Sound_sdState.SD_packetNum=1; // 第一包开始 
											   PositionSD_Enable();  //   使能上报 
											   Current_UDP_sd=1;
											   rt_kprintf("\r\n 开始上传音频! ....\r\n");     												
                            	            }	
											else
											if(2==MediaObj.Media_Type)
											{
											   MediaObj.Media_transmittingFlag=1;   
											   PositionSD_Enable();  //   使能上报
											   Current_UDP_sd=1;
											   Video_sdState.photo_sending=enable; 
											   Video_sdState.SD_packetNum=1; // 第一包开始   
											   rt_kprintf("\r\n 开始上传视频! ....\r\n");    												
                            	            }	
											break;
						    case 0x0702: 
                                          rt_kprintf("\r\n  驾驶员信息上报---中心应答!  \r\n"); 							       
											 
							           break;
							case 0x0701: 
                                           rt_kprintf("\r\n  电子运单上报---中心应答!  \r\n"); 							       
											 
							           break;		
						   case 0x0705: 	//
															  rt_kprintf("\r\n can-ack"); 
								   break; 
							 		   
									   
							default	 :
								       break;
					   	}


					    //-------------------------------
				      break;
           case  0x8100:    //  监控中心对终端注册消息的应答		
                       //-----------------------------------------------------------
					   switch(UDP_HEX_Rx[15])
                        	{
                               case 0: rt_kprintf("\r\n   ----注册成功\r\n");
							           memset(JT808Conf_struct.ConfirmCode,0,sizeof(JT808Conf_struct.ConfirmCode));
                                       memcpy(JT808Conf_struct.ConfirmCode,UDP_HEX_Rx+16,infolen-3);

									   memset(Reg_buf,0,sizeof(Reg_buf));
							           memcpy(Reg_buf,JT808Conf_struct.ConfirmCode,20);
									   JT808Conf_struct.Regsiter_Status=1; 
							           Reg_buf[20]=JT808Conf_struct.Regsiter_Status;
                                       Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
									   rt_kprintf("鉴权码: %s\r\n		   鉴权码长度: %d\r\n",JT808Conf_struct.ConfirmCode,strlen((const char*)JT808Conf_struct.ConfirmCode)); 
                                       //-------- 开始鉴权 ------
									   DEV_Login.Operate_enable=1;  
									   break;
							   case 1: rt_kprintf("\r\n   ----车辆已被注册\r\n");
									   break;	
							   case 2: rt_kprintf("\r\n   ----数据库中无该车辆\r\n");
									   break;
							   case 3: rt_kprintf("\r\n   ----终端已被注册\r\n");  
							           if(0==JT808Conf_struct.Regsiter_Status)  
                                        {
                                          ;//JT808Conf_struct.Regsiter_Status=2;  // not  1
                                          //DEV_regist.DeRegst_sd=1;
							           	}  
									   else
									   if(1==JT808Conf_struct.Regsiter_Status) 
							               DEV_Login.Operate_enable=1; //开始鉴权 
							          
									   break;		    
                               case 4: rt_kprintf("\r\n   ----数据库中无该终端\r\n");  
									   break;
                        	}					    
		              break;
		   case  0x8103:    //  设置终端参数   
		               //  Ack_Resualt=0;
		                Fail_Flag=0;
                      if(contentlen)
                       {                       // 和中心商议好了每次只下发操作一项设置
                         Total_ParaNum=UDP_HEX_Rx[13]; // 中心设置参数总数
                         rt_kprintf("\r\n Set ParametersNum =%d  \r\n",Total_ParaNum);
						 //------------------------------------------------------------------- 
						 ContentRdAdd=14;
						 Process_Resualt=0;  // clear resualt
						 for(i=0;i<Total_ParaNum;i++)
						 {
						   //  数类型是DWORD 4 个字节
						   SubCMD_8103H=(UDP_HEX_Rx[ContentRdAdd]<<24)+(UDP_HEX_Rx[ContentRdAdd+1]<<16)+(UDP_HEX_Rx[ContentRdAdd+2]<<8)+UDP_HEX_Rx[ContentRdAdd+3]; 
                           //  子信息长度
						   SubInfolen=UDP_HEX_Rx[ContentRdAdd+4];
						   //  处理子信息 如果设置成功把相应Bit 位置为 1 否则保持 0
						   if(CentreSet_subService_8103H(SubCMD_8103H,SubInfolen,UDP_HEX_Rx+ContentRdAdd+5))
                                Process_Resualt|=(0x01<<i);						   
						   //  移动偏移地址
						   ContentRdAdd+=5+UDP_HEX_Rx[ContentRdAdd+4]; // 偏移下标
						 } 						   

                         //--------------判断所有的设置结果  ---------------
                        /* for(i=0;i<Total_ParaNum;i++)
                         {
                             if(!((Process_Resualt>>0)&0x01))
						     	{
                                  Ack_Resualt=1; //  1  表示失败
                                  break; 
						     	}
							 if(i==(Total_ParaNum-1))  //  设置到最后一个确认成功
							 	Ack_Resualt=0;  //  成功/确认
                         }*/
						 
						 if(Fail_Flag==0)
						     Ack_Resualt=0;
						 else
						 	 Ack_Resualt=1;// 返回失败 
						 
                      }	
					  
						 //-------------------------------------------------------------------
						 if(SD_ACKflag.f_CentreCMDack_0001H==2)
						 {                  
						    Ack_Resualt=0;
							SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
						 }
						 else
                         if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
                         } 	 
						  rt_kprintf("\r\n  Set Device !\r\n");   
						  
		              break;  
		   case  0x8104:    //  查询终端参数
                      SD_ACKflag.f_SettingPram_0104H=1;  // 不管什么内容回复统一结果
                      rt_kprintf("\r\n  中心查询终端参数 !\r\n");   
			          break;
           case  0x8105:     // 终端控制
                    // Ack_Resualt=0; 
                        rt_kprintf("\r\ny  终端控制 -1!\r\n"); 
                     if(contentlen)
                      {                       // 和中心商议好了每次只下发操作一项设置
                         Total_ParaNum=UDP_HEX_Rx[13]; //  终端控制命令字 
                         rt_kprintf("\r\n Set ParametersNum =%d  \r\n",Total_ParaNum); 
						 //------------------------------------------------------------------- 
						 if(CentreSet_subService_8105H(Total_ParaNum,contentlen-1,UDP_HEX_Rx+14)) 
                                Ack_Resualt=0;   // 返回成功						   
                       }	
					 
						 //-------------------------------------------------------------------
						 Ack_Resualt=0;
					   if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
                         }	 
						  rt_kprintf("\r\ny  终端控制 !\r\n"); 
						  
		              break;
		   case  0x8201:     // 位置信息查询    位置信息查询消息体为空
		               SD_ACKflag.f_CurrentPosition_0201H=1;		
					   rt_kprintf("\r\n  位置信息查询 !\r\n");  
		              break;
		   case  0x8202:     // 临时位置跟踪控制
		                 Ack_Resualt=0;

						 //  13 14  时间间隔 
					     JT808Conf_struct.RT_LOCK.Lock_Dur=(UDP_HEX_Rx[13]<<8)+UDP_HEX_Rx[14];
					    //  15 16  17 18 对应终端的消息
                         JT808Conf_struct.RT_LOCK.Lock_KeepDur=(UDP_HEX_Rx[15]<<24)+(UDP_HEX_Rx[16]<<16)+(UDP_HEX_Rx[17]<<8)+UDP_HEX_Rx[18];

                         JT808Conf_struct.RT_LOCK.Lock_state=1;    // Enable Flag
						 JT808Conf_struct.RT_LOCK.Lock_KeepCnter=0;  //  保持计数器
						 Current_SD_Duration=JT808Conf_struct.RT_LOCK.Lock_Dur;  //更改发送间隔						 

						 JT808Conf_struct.SD_MODE.DUR_TOTALMODE=1;   // 更新定时相关状态位
						 JT808Conf_struct.SD_MODE.Dur_DefaultMode=1; 
						 //  保存配置
						 Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
						 
		                // if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     Ack_Resualt=0;
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;      
                         }
                      rt_kprintf("\r\n  临时位置跟踪控制!\r\n"); 
		              break;
		   case  0x8300:    //  文本信息下发
		                  Ack_Resualt=0;
                          TextInfo.TEXT_FLAG=UDP_HEX_Rx[13];
						  if(TextInfo.TEXT_FLAG&0x09)  // 检测是否给TTS终端  ，紧急也给TTS播报
						  {
						      
							  Dev_Voice.CMD_Type='2';
							  memset(Dev_Voice.Play_info,0,sizeof(Dev_Voice.Play_info));
                              memcpy(Dev_Voice.Play_info,UDP_HEX_Rx+14,infolen-1);
							  Dev_Voice.info_sdFlag=1;

							//#ifdef LCD_5inch
							   //DwinLCD.Type=LCD_SDTXT;
							 // memset(DwinLCD.TXT_content,0,sizeof(DwinLCD.TXT_content)); 
                                                   //DwinLCD.TXT_contentLen=AsciiToGb(DwinLCD.TXT_content,infolen-1,UDP_HEX_Rx+14);  						  
							//#endif   
                                                   //  TTS  
							   TTS_Get_Data(UDP_HEX_Rx+14,infolen-1);

						  }
						 if((TextInfo.TEXT_FLAG&0x04)||(TextInfo.TEXT_FLAG&0x01))  // 检测是否给终端显示器
						 {   
						     //    1. 短信设置命令
						       if( strncmp( (char*)UDP_HEX_Rx+14, "TW703#", 6 ) == 0 )                                                //短信修改UDP的IP和端口
								{
									//-----------  自定义 短息设置修改 协议 ----------------------------------
									SMS_protocol( (UDP_HEX_Rx+14)+ 5,(infolen-1)- 5 ,SMS_ACK_none);  
								}   
							   else
							   	{
								     //     2. 正常短信
									memset( TextInfo.TEXT_Content,0,sizeof(TextInfo.TEXT_Content));
									memcpy(TextInfo.TEXT_Content,UDP_HEX_Rx+14,infolen-1);
									//---------------------									
									//对非GBK编码处理------------------------------------
                                    contentlen=Instr_2_GBK(UDP_HEX_Rx+14,infolen-1,TextInfo.TEXT_Content);					
									//-------------
									TextInfo.TEXT_SD_FLAG=1;	// 置发送给显示屏标志位  // ||||||||||||||||||||||||||||||||||

									//========================================
									TextInforCounter++;
									rt_kprintf("\r\n写入收到的第 %d 条信息,消息长度=%d,消息:%s",TextInforCounter,contentlen,TextInfo.TEXT_Content);
									TEXTMSG_Write(TextInforCounter,1,contentlen,TextInfo.TEXT_Content);	 
							   	}
							//========================================
						 } 

						  #ifdef LCD_5inch
						           //======  信息都在屏幕上显示  
                                   DwinLCD.Type=LCD_SDTXT;
							       memset(DwinLCD.TXT_content,0,sizeof(DwinLCD.TXT_content)); 
                                   DwinLCD.TXT_contentLen=AsciiToGb(DwinLCD.TXT_content,infolen-1,UDP_HEX_Rx+14);  						  
   						  #endif    

                          //------- 返回 ----
						 //  if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {   
						      Ack_Resualt=0;
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
                         }
						   rt_kprintf("\r\n 文本信息: %s\r\n",TextInfo.TEXT_Content);
		              break; 
		   case  0x8301:    //  事件设置 
					   if(contentlen)
                      {                       
                         //--- 设置类型--
                         switch(UDP_HEX_Rx[13])
                         {
                            case 0 :  //  删除终端现有所有事件，该命令后不带后继字符
                                      Event_Init(1); 	
                                      rt_kprintf("\r\n 删除所有事件\r\n");
                                      break; 
							case 1:  // 更新事件
									 if(UDP_HEX_Rx[13]==1)
									 	rt_kprintf("\r\n 更新事件\r\n");
							         //break;
							case 2:  // 追加事件
                                     if(UDP_HEX_Rx[13]==2)
									 	rt_kprintf("\r\n 追加事件\r\n");
							          //break;
							case 3:  // 修改事件
                                     if(UDP_HEX_Rx[13]==3)
									 	 rt_kprintf("\r\n 修改事件\r\n");  
							          //break;
							case 4:  // 删除特定事件       
							         if(UDP_HEX_Rx[13]==4)
									 	 rt_kprintf("\r\n 删除特点事件\r\n"); 
									  Total_ParaNum=UDP_HEX_Rx[14]; // 中心设置参数总数
									  rt_kprintf("\r\n Set ParametersNum =%d  \r\n",Total_ParaNum);
									  if(Total_ParaNum!=1)
										 break;
									  //-------------------------------
									  if((UDP_HEX_Rx[15]>8)&&(UDP_HEX_Rx[15]==0))
									     EventObj.Event_ID=0;
									  else
									  	 EventObj.Event_ID=UDP_HEX_Rx[15];
									  
									 // EventObj.Event_Len=UDP_HEX_Rx[16];
									  memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
									  //----  Instr  转 GBK ----------------------
									  EventObj.Event_Len=Instr_2_GBK(UDP_HEX_Rx+17,UDP_HEX_Rx[16],EventObj.Event_Str);	 		
									  EventObj.Event_Effective=1; 
                                      Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj));									  rt_kprintf("\r\n 事件内容:%s\r\n",EventObj.Event_Str);   
									  rt_kprintf("\r\n 事件内容:%s\r\n",EventObj.Event_Str);   
							          break;
							default:
								      break;

                         }

                          //---------返回 -------
                         // if(SD_ACKflag.f_CentreCMDack_0001H==0) // 一般回复
 					      {
		  						 SD_ACKflag.f_CentreCMDack_0001H=1;
		  						 Ack_Resualt=0;
		  						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					       }
						  /*
						  if(SD_ACKflag.f_CurrentEventACK_0301H==0)
						  {
							  SD_ACKflag.f_CurrentEventACK_0301H=1; 
						  }
                              */
					   }
                          
		              break;			  
		   case  0x8302:    // 提问下发
						  if(UDP_HEX_Rx[13]&0x04)  // 检测标志是否给显示终端
						  	{
		                              	  if(TextInfo.TEXT_FLAG&0x09)  // 检测是否给TTS终端  ，紧急也给TTS播报
								  {
								      
									  Dev_Voice.CMD_Type='2';
									  memset(Dev_Voice.Play_info,0,sizeof(Dev_Voice.Play_info));
		                              memcpy(Dev_Voice.Play_info,UDP_HEX_Rx+15,UDP_HEX_Rx[14]);
									  Dev_Voice.info_sdFlag=1;
		                                                   //  TTS  
									   TTS_Get_Data(UDP_HEX_Rx+15,infolen-1);

								  }
						  	}
						   rt_kprintf("\r\n  中心下发提问 \r\n"); 
						  {
						     //ASK_Centre.ASK_infolen=UDP_HEX_Rx[14];
							 memset(ASK_Centre.ASK_info,0,sizeof(ASK_Centre.ASK_info));
							 //----  Instr  转 GBK ----------------------
							 ASK_Centre.ASK_infolen=Instr_2_GBK(UDP_HEX_Rx+15,UDP_HEX_Rx[14],ASK_Centre.ASK_info);	
							// memcpy(ASK_Centre.ASK_info,UDP_HEX_Rx+15,ASK_Centre.ASK_infolen);
							 rt_kprintf("\r\n  问题: %s \r\n",ASK_Centre.ASK_info); 
							 memset(ASK_Centre.ASK_answer,0,sizeof(ASK_Centre.ASK_answer));
							 //----  Instr  转 GBK ----------------------
							 contentlen=Instr_2_GBK(UDP_HEX_Rx+15+UDP_HEX_Rx[14],infolen-2-UDP_HEX_Rx[14],ASK_Centre.ASK_answer);	
							// memcpy(ASK_Centre.ASK_answer,UDP_HEX_Rx+15+ASK_Centre.ASK_infolen,infolen-2-ASK_Centre.ASK_infolen);	 

							 ASK_Centre.ASK_SdFlag=1;   // ||||||||||||||||||||||||||||||||||
							 ASK_Centre.ASK_floatID=Centre_FloatID; // 备份 FloatID	 
                                                 
							 rt_kprintf("\r\n 提问Answer:%s\r\n",ASK_Centre.ASK_answer+3);       
                             Api_RecordNum_Write(ask_quesstion,1, (u8*)&ASK_Centre,sizeof(ASK_Centre)); 

							 ASK_Centre.ASK_disp_Enable=1;
                                                  
						  }
						   
						// if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     Ack_Resualt=0; 
						     SD_ACKflag.f_CentreCMDack_0001H=1;
					          SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;  
                                           } 

					
		              break;
           case  0x8303:    //  信息点播菜单设置
                              //--- 设置类型--
                         switch(UDP_HEX_Rx[13])
                         {
                            case 0 :  //  删除终端现有所有信息
                                      MSG_BroadCast_Init(1); 
                                      rt_kprintf("\r\n 删除信息\r\n");
                                      break;
							case 1:  // 更新菜单
									  if(UDP_HEX_Rx[13]==1)
									 	  rt_kprintf("\r\n 更新菜单\r\n");
							         //break;
							case 2:  // 追加菜单
                                       if(UDP_HEX_Rx[13]==2)
									 	  rt_kprintf("\r\n 追加菜单\r\n");
							          //break;
							case 3:  // 修改菜单
							          if(UDP_HEX_Rx[13]==3)
									 	  rt_kprintf("\r\n 修改菜单\r\n");
									  Total_ParaNum=UDP_HEX_Rx[14];           // 消息项总数
									  rt_kprintf("\r\n Set ParametersNum =%d  \r\n",Total_ParaNum);
									  if(Total_ParaNum!=1)
										 break;
									  //-------------------------------
									  if((UDP_HEX_Rx[15]>8)&&(UDP_HEX_Rx[15]==0))
									     MSG_BroadCast_Obj.INFO_TYPE=0;
									  else
									  	  MSG_BroadCast_Obj.INFO_TYPE=UDP_HEX_Rx[15];
									  
									 contentlen=(UDP_HEX_Rx[16]<<8)+UDP_HEX_Rx[17];
									  memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
									  //----  Instr  转 GBK ----------------------
							          MSG_BroadCast_Obj.INFO_LEN=Instr_2_GBK(UDP_HEX_Rx+18,contentlen,MSG_BroadCast_Obj.INFO_STR);
									  //memcpy(MSG_BroadCast_Obj.INFO_STR,UDP_HEX_Rx+18,MSG_BroadCast_Obj.INFO_LEN);  
									  MSG_BroadCast_Obj.INFO_Effective=1;
									  MSG_BroadCast_Obj.INFO_PlyCancel=1;  
	                                  Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
									  rt_kprintf("\r\n 信息点播内容:%s\r\n",MSG_BroadCast_Obj.INFO_STR); 
							          break;
							default:
								      break;

                         }

                          //---------返回 -------
                          // if(SD_ACKflag.f_CentreCMDack_0001H==0) // 一般回复
 					      {
		  						 SD_ACKflag.f_CentreCMDack_0001H=1;
		  						 Ack_Resualt=0;
		  						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					       }
						   
                        /* 
                                            if(SD_ACKflag.f_MsgBroadCast_0303H==0)
						 {
						     SD_ACKflag.f_MsgBroadCast_0303H=1;  
		                             }
		                             */ 
		              break;
		   case  0x8304:    //  信息服务
                          Ack_Resualt=0;
                          MSG_BroadCast_Obj.INFO_TYPE=UDP_HEX_Rx[13];  //  信息类型
                          MSG_BroadCast_Obj.INFO_LEN=(UDP_HEX_Rx[14]<<8)+UDP_HEX_Rx[15];
						  memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
						  
						  //----  Instr  转 GBK ----------------------
						 // contentlen=Instr_2_GBK(UDP_HEX_Rx+16,infolen-3,TextInfo.TEXT_Content);						  
						  memcpy(MSG_BroadCast_Obj.INFO_STR,UDP_HEX_Rx+16,infolen-3); 

                          
						  MSG_BroadCast_Obj.INFO_SDFlag=1;    // ||||||||||||||||||||||||||||||||||
						  Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 


						  
                           //------------------------------
						 /*  Dev_Voice.CMD_Type='2';
						   memset(Dev_Voice.Play_info,0,sizeof(Dev_Voice.Play_info));
                                             memcpy(Dev_Voice.Play_info,UDP_HEX_Rx+16,infolen-3);
						   Dev_Voice.info_sdFlag=1;
						   */

                           // --------  发送给文本信息  --------------    
						   memset( TextInfo.TEXT_Content,0,sizeof(TextInfo.TEXT_Content));
						   
						   //----  Instr  转 GBK ----------------------
						   contentlen=Instr_2_GBK(UDP_HEX_Rx+16,infolen-3,TextInfo.TEXT_Content);						   
						  // memcpy(TextInfo.TEXT_Content,UDP_HEX_Rx+16,infolen-3);
						   TextInfo.TEXT_SD_FLAG=1;    // 置发送给显示屏标志位	// ||||||||||||||||||||||||||||||||||
						   
                           
						   rt_kprintf("\r\n 信息服务内容:%s\r\n",TextInfo.TEXT_Content); 
                          //------- 返回 ----
						 //  if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     Ack_Resualt=0;
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
                         }
		              break;	
		   case  0x8400:    //  电话回拨
                       
					   if(infolen==0)
							 break;
					   if(0==UDP_HEX_Rx[13])   // 普通通话
					   	{   Speak_ON;rt_kprintf("\r\n   电话回拨-->普通通话\r\n");}
					   else
					   if(1==UDP_HEX_Rx[13])  //  监听
					    {  Speak_OFF;rt_kprintf("\r\n   电话回拨-->监听"); } 
					   else
					       break;
	 					  memset(JT808Conf_struct.LISTEN_Num,0,sizeof(JT808Conf_struct.LISTEN_Num));
	 					  memcpy(JT808Conf_struct.LISTEN_Num,UDP_HEX_Rx+14,infolen-1);  										  
                                            Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
                                            CallState=CallState_rdytoDialLis;  // 准备开始拨打监听号码 

						 // if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=0;      
                         }
		              break;
		   case  0x8401:  //   设置电话本
                      
					  switch(UDP_HEX_Rx[13])
					  {
						 case 0 :  //  删除终端现有所有信息
								  // PhoneBook_Init(1); 
								   rt_kprintf("\r\n 删除电话本\r\n"); 
								   break;
						 case 1:  // 更新菜单
						          if(UDP_HEX_Rx[13]==1)
									 	 rt_kprintf("\r\n 更新电话本\r\n"); 
						 case 3:  // 修改菜单
						           if(UDP_HEX_Rx[13]==3)
									 	 rt_kprintf("\r\n 修改电话本\r\n"); 
						           Rx_PhoneBOOK.CALL_TYPE=UDP_HEX_Rx[15]; // 标志 ，呼入呼出
                                   Rx_PhoneBOOK.NumLen=UDP_HEX_Rx[16];								   
								   memset(Rx_PhoneBOOK.NumberStr,0,sizeof(Rx_PhoneBOOK.NumberStr)); 
								   memcpy(Rx_PhoneBOOK.NumberStr,UDP_HEX_Rx+17,Rx_PhoneBOOK.NumLen);
								   
								   contentlen=UDP_HEX_Rx[17+Rx_PhoneBOOK.NumLen]; 								   
								   memset(Rx_PhoneBOOK.UserStr,0,sizeof(Rx_PhoneBOOK.UserStr));	
								     //----  Instr  转 GBK ----------------------
						           Rx_PhoneBOOK.UserLen=Instr_2_GBK(UDP_HEX_Rx+18+Rx_PhoneBOOK.NumLen,contentlen,Rx_PhoneBOOK.UserStr);	
								   //memcpy(Rx_PhoneBOOK.UserStr,UDP_HEX_Rx+18+Rx_PhoneBOOK.NumLen,Rx_PhoneBOOK.UserLen);
								   
								   for(i=0;i<8;i++)
								   {
                                    PhoneBook.CALL_TYPE=2; //类型定义为输出 
								    PhoneBook.NumLen=0;    // 号码长度
								    memset(PhoneBook.NumberStr,0,sizeof(PhoneBook.NumberStr));  
								    PhoneBook.UserLen=0;		
								    memset(PhoneBook.UserStr,0,sizeof(PhoneBook.UserStr)); 	 
									   Api_RecordNum_Write(phonebook, i+1, (u8*)&PhoneBook,sizeof(PhoneBook)); 
									if(strncmp((char*)PhoneBook.UserStr,(const char*)Rx_PhoneBOOK.UserStr,Rx_PhoneBOOK.UserLen) == 0) 
									{ // 找到相同名字的把以前的删除用新的代替
									   Api_RecordNum_Write(phonebook, i+1, (u8*)&Rx_PhoneBOOK,sizeof(Rx_PhoneBOOK)); 
									   break;  // 跳出for 
									}

								   }
								   break;
						 case 2:  // 追加菜单	
						          if(UDP_HEX_Rx[13]==2)
									 	 rt_kprintf("\r\n 追加电话本\r\n"); 
						           Rx_PhoneBOOK.CALL_TYPE=UDP_HEX_Rx[15]; // 标志 ，呼入呼出
                                   Rx_PhoneBOOK.NumLen=UDP_HEX_Rx[16];								   
								   memset(Rx_PhoneBOOK.NumberStr,0,sizeof(Rx_PhoneBOOK.NumberStr)); 
								   memcpy(Rx_PhoneBOOK.NumberStr,UDP_HEX_Rx+17,Rx_PhoneBOOK.NumLen);
								 //  Rx_PhoneBOOK.UserLen=UDP_HEX_Rx[17+Rx_PhoneBOOK.NumLen]; 								   
								 //  memset(Rx_PhoneBOOK.UserStr,0,sizeof(Rx_PhoneBOOK.UserStr));		
								   Rx_PhoneBOOK.Effective_Flag=1; // 有效标志位  
								  // memcpy(Rx_PhoneBOOK.UserStr,UDP_HEX_Rx+18+Rx_PhoneBOOK.NumLen,Rx_PhoneBOOK.UserLen);	
								   //---------------------------------------------------  
								   contentlen=UDP_HEX_Rx[17+Rx_PhoneBOOK.NumLen]; 								   
								   memset(Rx_PhoneBOOK.UserStr,0,sizeof(Rx_PhoneBOOK.UserStr));	
								     //----  Instr  转 GBK ----------------------
						           Rx_PhoneBOOK.UserLen=Instr_2_GBK(UDP_HEX_Rx+18+Rx_PhoneBOOK.NumLen,contentlen,Rx_PhoneBOOK.UserStr);	
								   //memcpy(Rx_PhoneBOOK.UserStr,UDP_HEX_Rx+18+Rx_PhoneBOOK.NumLen,Rx_PhoneBOOK.UserLen);
                                   //-------------------------------------------
								   
								   Api_RecordNum_Write(phonebook, UDP_HEX_Rx[14], (u8*)&Rx_PhoneBOOK, sizeof(Rx_PhoneBOOK)); 
                                                           rt_kprintf("\r\n Name:%s\r\n",Rx_PhoneBOOK.UserStr); 
								   rt_kprintf("\r\n Number:%s\r\n",Rx_PhoneBOOK.NumberStr);
								   break;
						 default:
								   break;
					  
					  }

					   
					// if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
					 
			          break;
           case  0x8500:    //  车辆控制
                      Vech_Control.Control_Flag=UDP_HEX_Rx[13];
					  if(UDP_HEX_Rx[13]&0x01)
					  	{ // 车门加锁       bit 12
                            Car_Status[2]|=0x10;     // 需要控制继电器
							rt_kprintf("\r\n  车辆加锁 \r\n"); 
					  	}
					  else
					  	{ // 车门解锁
                            Car_Status[2]&=~0x10;    // 需要控制继电器
                            rt_kprintf("\r\n  车辆解锁 \r\n"); 
					  	}					  
					  Vech_Control.ACK_SD_Flag=1;    
		              break;
		   case  0x8600:    //  设置圆形区域
                       rt_kprintf("\r\n  设置圆形区域 \r\n"); 
		              if(UDP_HEX_Rx[14]==1)  //  现在支持设置一个区域
		              { 
	                       switch(UDP_HEX_Rx[13])
	                       	{	                       	      
		                          case 1:  // 追加区域
                                           for(i=0;i<8;i++)
                                           	{                                                
												memset((u8*)&Rail_Cycle,0,sizeof(Rail_Cycle));
												Api_RecordNum_Write(Rail_cycle,Rail_Cycle.Area_ID, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); 	 
                                                Rails_Routline_Read();  
												if(Rail_Cycle.Area_attribute) // 找出来未使用的
													break;
                                           	}
										   if(8==i)  //  如果都满了，那么用 0
										   	{
										   	  i=0;
										   	}
								         
		                          case 0:  // 更新区域
		                          case 2:  // 修改区域                                      
										   memset((u8*)&Rail_Cycle,0,sizeof(Rail_Cycle));
                                           Rail_Cycle.Area_ID=(UDP_HEX_Rx[15]<<24)+(UDP_HEX_Rx[16]<<16)+(UDP_HEX_Rx[17]<<8)+UDP_HEX_Rx[18];
										   Rail_Cycle.Area_attribute=(UDP_HEX_Rx[19]<<8)+UDP_HEX_Rx[20];
										   Rail_Cycle.Center_Latitude=(UDP_HEX_Rx[21]<<24)+(UDP_HEX_Rx[22]<<16)+(UDP_HEX_Rx[23]<<8)+UDP_HEX_Rx[24];
										   Rail_Cycle.Center_Longitude=(UDP_HEX_Rx[25]<<24)+(UDP_HEX_Rx[26]<<16)+(UDP_HEX_Rx[27]<<8)+UDP_HEX_Rx[28]; 
										   Rail_Cycle.Radius=(UDP_HEX_Rx[29]<<24)+(UDP_HEX_Rx[30]<<16)+(UDP_HEX_Rx[31]<<8)+UDP_HEX_Rx[32]; 
										   memcpy(Rail_Cycle.StartTimeBCD,UDP_HEX_Rx+33,6);
										   memcpy(Rail_Cycle.EndTimeBCD,UDP_HEX_Rx+39,6);   
										   Rail_Cycle.MaxSpd=(UDP_HEX_Rx[45]<<8)+UDP_HEX_Rx[46]; 
										   Rail_Cycle.KeepDur=UDP_HEX_Rx[47];  
										   Rail_Cycle.Effective_flag=1; 

										   if((Rail_Cycle.Area_ID>8)||(Rail_Cycle.Area_ID==0))
										   	Rail_Cycle.Area_ID=1;										   
										    Api_RecordNum_Write(Rail_cycle,Rail_Cycle.Area_ID, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); 	 
									   
									       Rails_Routline_Read();
								           break;
									default:
										    break;

	                       	}    
					   
		              	} 
                		 //------- 返回 ----
						//   if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 Ack_Resualt=0;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
                         }
				      break;
           case  0x8601:    //  删除圆形区域 
                         rt_kprintf("\r\n  删除圆形区域 \r\n"); 
						 if(0==UDP_HEX_Rx[13])   // 区域数
                                                    RailCycle_Init();  // 删除所有区域  
						 else
						 	{						 	  
							  memset((u8*)&Rail_Cycle,0,sizeof(Rail_Cycle));  //  clear all  first
						 	  for(i=0;i<UDP_HEX_Rx[13];i++)
						 	  {
						 	    Rail_Cycle.Area_ID=(UDP_HEX_Rx[14+i]<<24)+(UDP_HEX_Rx[15+i]<<16)+(UDP_HEX_Rx[16+i]<<8)+UDP_HEX_Rx[17+i];
                                if((Rail_Cycle.Area_ID>8)||(Rail_Cycle.Area_ID==0))
										   	Rail_Cycle.Area_ID=1;	
								Rail_Cycle.Effective_flag=0; // clear
								 Api_RecordNum_Write(Rail_cycle,Rail_Cycle.Area_ID, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); // 删除对应的围栏
                                Rails_Routline_Read();  
						 	  }

						 	}

                        //----------------
                    //   if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;
		   case  0x8602:    //  设置矩形区域
		               rt_kprintf("\r\n  设置矩形区域 \r\n"); 
                        if(UDP_HEX_Rx[14]==1)  //  现在支持设置一个区域
		              { 
	                       switch(UDP_HEX_Rx[13])
	                       	{	                       	      
		                          case 1:  // 追加区域
                                  case 0:  // 更新区域
		                          case 2:  // 修改区域                                      
										   memset((u8*)&Rail_Rectangle,0,sizeof(Rail_Rectangle));
                                           Rail_Rectangle.Area_ID=(UDP_HEX_Rx[15]<<24)+(UDP_HEX_Rx[16]<<16)+(UDP_HEX_Rx[17]<<8)+UDP_HEX_Rx[18];
										   Rail_Rectangle.Area_attribute=(UDP_HEX_Rx[19]<<8)+UDP_HEX_Rx[20];
										   Rail_Rectangle.LeftUp_Latitude=(UDP_HEX_Rx[21]<<24)+(UDP_HEX_Rx[22]<<16)+(UDP_HEX_Rx[23]<<8)+UDP_HEX_Rx[24];
										   Rail_Rectangle.LeftUp_Longitude=(UDP_HEX_Rx[25]<<24)+(UDP_HEX_Rx[26]<<16)+(UDP_HEX_Rx[27]<<8)+UDP_HEX_Rx[28]; 
										   Rail_Rectangle.RightDown_Latitude=(UDP_HEX_Rx[29]<<24)+(UDP_HEX_Rx[30]<<16)+(UDP_HEX_Rx[31]<<8)+UDP_HEX_Rx[32];
										   Rail_Rectangle.RightDown_Longitude=(UDP_HEX_Rx[33]<<24)+(UDP_HEX_Rx[34]<<16)+(UDP_HEX_Rx[35]<<8)+UDP_HEX_Rx[36]; 
										   memcpy(Rail_Rectangle.StartTimeBCD,UDP_HEX_Rx+37,6);
										   memcpy(Rail_Rectangle.EndTimeBCD,UDP_HEX_Rx+43,6);   
										   Rail_Rectangle.MaxSpd=(UDP_HEX_Rx[49]<<8)+UDP_HEX_Rx[50]; 
										   Rail_Rectangle.KeepDur=UDP_HEX_Rx[51];   
										   Rail_Rectangle.Effective_flag=1;

										   if((Rail_Rectangle.Area_ID>8)||(Rail_Rectangle.Area_ID==0))
										   	Rail_Rectangle.Area_ID=1;										   
										   Api_RecordNum_Write(Rail_rect,Rail_Rectangle.Area_ID, (u8*)&Rail_Rectangle,sizeof(Rail_Rectangle)); 	 

										   rt_kprintf("\r\n   中心设置  矩形围栏 leftLati=%d leftlongi=%d\r\n",Rail_Rectangle.LeftUp_Latitude,Rail_Rectangle.LeftUp_Longitude); 
										   
										   rt_kprintf("\r\n    attribute:%X         矩形围栏 rightLati=%d rightlongi=%d\r\n",Rail_Rectangle.Area_attribute,Rail_Rectangle.RightDown_Latitude,Rail_Rectangle.RightDown_Longitude);   
                                           Rails_Routline_Read();   
										   break;
									default:
										    break;

	                       	}    
					   
		              	} 
                        //----------------
                     //if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;
		   case  0x8603:    //  删除矩形区域
                           rt_kprintf("\r\n  删除矩形区域 \r\n");    
					   	  if(0==UDP_HEX_Rx[13])   // 区域数
                           {
                              RailRect_Init();  // 删除所有区域
                              Rails_Routline_Read();  
					   	   }
						   else
						 	{						 	  
							  memset((u8*)&Rail_Rectangle,0,sizeof(Rail_Rectangle));  //  clear all  first
						 	  for(i=0;i<UDP_HEX_Rx[13];i++)
						 	  {
						 	    Rail_Rectangle.Area_ID=(UDP_HEX_Rx[14+i]<<24)+(UDP_HEX_Rx[15+i]<<16)+(UDP_HEX_Rx[16+i]<<8)+UDP_HEX_Rx[17+i];
                                if((Rail_Rectangle.Area_ID>8)||(Rail_Rectangle.Area_ID==0))
										   	Rail_Rectangle.Area_ID=1;	
								Rail_Rectangle.Effective_flag=0;
								Api_RecordNum_Write(Rail_rect,Rail_Rectangle.Area_ID, (u8*)&Rail_Rectangle,sizeof(Rail_Rectangle)); // 删除对应的围栏
                                Rails_Routline_Read();  
							  }  
						 	}


                         //----------------
                     //  if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
			          break;
		   case  0x8604:	//  多边形区域
		                     rt_kprintf("\r\n  设置多边形区域 \r\n");
			                 if(UDP_HEX_Rx[14]==1)  //  现在支持设置一个区域
							 { 
								  switch(UDP_HEX_Rx[13])
								   {								 
										 case 1:  // 追加区域
										 case 0:  // 更新区域
										 case 2:  // 修改区域									   
												  memset((u8*)&Rail_Polygen,0,sizeof(Rail_Polygen));   
												  Rail_Polygen.Area_ID=(UDP_HEX_Rx[15]<<24)+(UDP_HEX_Rx[16]<<16)+(UDP_HEX_Rx[17]<<8)+UDP_HEX_Rx[18];
												  Rail_Polygen.Area_attribute=(UDP_HEX_Rx[19]<<8)+UDP_HEX_Rx[20];
												  memcpy(Rail_Polygen.StartTimeBCD,UDP_HEX_Rx+20,6);
												  memcpy(Rail_Polygen.EndTimeBCD,UDP_HEX_Rx+26,6);   
												  Rail_Polygen.MaxSpd=(UDP_HEX_Rx[32]<<8)+UDP_HEX_Rx[33]; 
												  Rail_Polygen.KeepDur=UDP_HEX_Rx[34];   
                                                                                      Rail_Polygen.Acme_Num=UDP_HEX_Rx[35];												  
												  Rail_Polygen.Acme1_Latitude=(UDP_HEX_Rx[36]<<24)+(UDP_HEX_Rx[37]<<16)+(UDP_HEX_Rx[38]<<8)+UDP_HEX_Rx[39];
												  Rail_Polygen.Acme1_Longitude=(UDP_HEX_Rx[40]<<24)+(UDP_HEX_Rx[41]<<16)+(UDP_HEX_Rx[42]<<8)+UDP_HEX_Rx[43]; 
												  Rail_Polygen.Acme2_Latitude=(UDP_HEX_Rx[44]<<24)+(UDP_HEX_Rx[45]<<16)+(UDP_HEX_Rx[46]<<8)+UDP_HEX_Rx[47];
												  Rail_Polygen.Acme2_Longitude=(UDP_HEX_Rx[48]<<24)+(UDP_HEX_Rx[49]<<16)+(UDP_HEX_Rx[50]<<8)+UDP_HEX_Rx[51];
												  Rail_Polygen.Acme3_Latitude=(UDP_HEX_Rx[52]<<24)+(UDP_HEX_Rx[53]<<16)+(UDP_HEX_Rx[54]<<8)+UDP_HEX_Rx[55];
												  Rail_Polygen.Acme3_Longitude=(UDP_HEX_Rx[56]<<24)+(UDP_HEX_Rx[57]<<16)+(UDP_HEX_Rx[58]<<8)+UDP_HEX_Rx[59];  
			   
												  if((Rail_Polygen.Area_ID>8)||(Rail_Polygen.Area_ID==0))
												   Rail_Polygen.Area_ID=1;		
												  Rail_Polygen.Effective_flag=1;
                                                                                       Api_RecordNum_Write(Rail_polygen,Rail_Polygen.Area_ID, (u8*)&Rail_Polygen,sizeof(Rail_Polygen)); 	 
												  break;
										   default:
												   break;
			   
								   }	
							  
							   } 

                          //----------------
                      // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					     {
	 						 SD_ACKflag.f_CentreCMDack_0001H=1;
	 						 Ack_Resualt=0;
	 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					      }
		              break;
		   case  0x8605:    //  删除多边区域
                         rt_kprintf("\r\n  删除多边形区域 \r\n");
                        if(0==UDP_HEX_Rx[13])   // 区域数
                             RailPolygen_Init();  // 删除所有区域
						 else
						 	{						 	  
							  memset((u8*)&Rail_Polygen,0,sizeof(Rail_Polygen));  //  clear all  first
						 	  for(i=0;i<UDP_HEX_Rx[13];i++)
						 	  {
						 	    Rail_Polygen.Area_ID=(UDP_HEX_Rx[14+i]<<24)+(UDP_HEX_Rx[15+i]<<16)+(UDP_HEX_Rx[16+i]<<8)+UDP_HEX_Rx[17+i];
                                if((Rail_Polygen.Area_ID>8)||(Rail_Polygen.Area_ID==0))
										   	Rail_Polygen.Area_ID=1;	
								Rail_Polygen.Effective_flag=0;   
                                                         Api_RecordNum_Write(Rail_polygen,Rail_Polygen.Area_ID, (u8*)&Rail_Polygen,sizeof(Rail_Polygen)); 	 
						 	  }   
						 	}

                          //----------------
                      // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 } 
		              break;
		   case  0x8606:    //  设置路线
		                 rt_kprintf("\r\n  设置路线 \r\n");
						memset((u8*)&ROUTE_Obj,0,sizeof(ROUTE_Obj));  //  clear all  first			
                                          ROUTE_Obj.Route_ID=(UDP_HEX_Rx[13]<<24)+(UDP_HEX_Rx[14]<<16)+(UDP_HEX_Rx[15]<<8)+UDP_HEX_Rx[16];  
 						ROUTE_Obj.Route_attribute=(UDP_HEX_Rx[17]<<8)+UDP_HEX_Rx[18];  						
						memcpy(ROUTE_Obj.StartTimeBCD,UDP_HEX_Rx+19,6); 
						memcpy(ROUTE_Obj.EndTimeBCD,UDP_HEX_Rx+25,6); 
						ROUTE_Obj.Points_Num=(UDP_HEX_Rx[31]<<8)+UDP_HEX_Rx[32];  
						rt_kprintf("\r\n ROUTE_Obj.ID:  %d  \r\n ",ROUTE_Obj.Route_ID);    
						rt_kprintf("\r\n ROUTE_Obj.ID:  %04X  \r\n ",ROUTE_Obj.Route_attribute); 
						rt_kprintf("\r\n ROUTE_Obj.Points_Num:  %d  \r\n ",ROUTE_Obj.Points_Num);   
						if(ROUTE_Obj.Points_Num<3) 
						{ 
 						   SD_ACKflag.f_CentreCMDack_0001H=1;
 						   Ack_Resualt=0;
 						    SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
						  break;
						}  
						reg_u32=33;
						for(i=0;i<6;i++)  // 拐点数目
						{						  
						   // if((infolen+32)<reg_u32)   
							//	 break;   
							   
						  ROUTE_Obj.RoutePoints[i].POINT_ID=(UDP_HEX_Rx[reg_u32]<<24)+(UDP_HEX_Rx[reg_u32+1]<<16)+(UDP_HEX_Rx[reg_u32+2]<<8)+UDP_HEX_Rx[reg_u32+3]; 
						  reg_u32+=4;
						  rt_kprintf("\r\n PointID=%08x\r\n",ROUTE_Obj.RoutePoints[i].POINT_ID);
						  ROUTE_Obj.RoutePoints[i].Line_ID=(UDP_HEX_Rx[reg_u32]<<24)+(UDP_HEX_Rx[reg_u32+1]<<16)+(UDP_HEX_Rx[reg_u32+2]<<8)+UDP_HEX_Rx[reg_u32+3]; 
						  reg_u32+=4;
                                            rt_kprintf("\r\n LineID=%08x\r\n",ROUTE_Obj.RoutePoints[i].Line_ID);
						  ROUTE_Obj.RoutePoints[i].POINT_Latitude=(UDP_HEX_Rx[reg_u32]<<24)+(UDP_HEX_Rx[reg_u32+1]<<16)+(UDP_HEX_Rx[reg_u32+2]<<8)+UDP_HEX_Rx[reg_u32+3]; 
						  reg_u32+=4;
                                             rt_kprintf("\r\n LatiID=%08x\r\n",ROUTE_Obj.RoutePoints[i].POINT_Latitude); 
						  ROUTE_Obj.RoutePoints[i].POINT_Longitude=(UDP_HEX_Rx[reg_u32]<<24)+(UDP_HEX_Rx[reg_u32+1]<<16)+(UDP_HEX_Rx[reg_u32+2]<<8)+UDP_HEX_Rx[reg_u32+3]; 
						  reg_u32+=4;
                                                rt_kprintf("\r\n LongID=%08x\r\n",ROUTE_Obj.RoutePoints[i].POINT_Longitude); 
						  ROUTE_Obj.RoutePoints[i].Width=UDP_HEX_Rx[reg_u32++];  
						     rt_kprintf("\r\n Width=%02x\r\n",ROUTE_Obj.RoutePoints[i].Width); 
						  ROUTE_Obj.RoutePoints[i].Atribute=UDP_HEX_Rx[reg_u32++]; 
						   rt_kprintf("\r\n atrit=%02x\r\n\r\n",ROUTE_Obj.RoutePoints[i].Atribute);         
						   if(ROUTE_Obj.RoutePoints[i].Atribute==0)
                                                            ;
						   else
						    if(ROUTE_Obj.RoutePoints[i].Atribute==1) 
                                                      ROUTE_Obj.RoutePoints[i].MaxSpd=(UDP_HEX_Rx[reg_u32++]<<8)+UDP_HEX_Rx[reg_u32++]; 
                                             else
							 { 
									  ROUTE_Obj.RoutePoints[i].TooLongValue=(UDP_HEX_Rx[reg_u32++]<<8)+UDP_HEX_Rx[reg_u32++]; 
									  ROUTE_Obj.RoutePoints[i].TooLessValue=(UDP_HEX_Rx[reg_u32++]<<8)+UDP_HEX_Rx[reg_u32++];   
									  ROUTE_Obj.RoutePoints[i].MaxSpd=(UDP_HEX_Rx[reg_u32++]<<8)+UDP_HEX_Rx[reg_u32++]; 
									  ROUTE_Obj.RoutePoints[i].KeepDur=(UDP_HEX_Rx[reg_u32++]<<8)+UDP_HEX_Rx[reg_u32++];    
						     	}

						}	
                           
						  if((ROUTE_Obj.Route_ID>Route_Mum)||(ROUTE_Obj.Route_ID==0))
									  ROUTE_Obj.Route_ID=1;		
						  ROUTE_Obj.Effective_flag=1;
						  Api_RecordNum_Write(route_line,ROUTE_Obj.Route_ID, (u8*)&ROUTE_Obj,sizeof(ROUTE_Obj));// 删除对应的围栏 



                           //----------------
                    // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;
		   case  0x8607:    //  删除路线
                         rt_kprintf("\r\n  删除路线 \r\n");  
                         if(0==UDP_HEX_Rx[13])   // 区域数
                             RouteLine_Init();  // 删除所有区域
						 else
						 	{						 	  
							  memset((u8*)&ROUTE_Obj,0,sizeof(ROUTE_Obj));  //  clear all  first
						 	  for(i=0;i<UDP_HEX_Rx[13];i++)
						 	  {
						 	    ROUTE_Obj.Route_ID=(UDP_HEX_Rx[14+i]<<24)+(UDP_HEX_Rx[15+i]<<16)+(UDP_HEX_Rx[16+i]<<8)+UDP_HEX_Rx[17+i];
                                if((ROUTE_Obj.Route_ID>Route_Mum)||(ROUTE_Obj.Route_ID==0))
										   	ROUTE_Obj.Route_ID=1;	
								ROUTE_Obj.Effective_flag=0;       
								     Api_RecordNum_Write(route_line,ROUTE_Obj.Route_ID, (u8*)&ROUTE_Obj,sizeof(ROUTE_Obj)); 	  // 删除对应的围栏
						 	  }   
						 	}  

                           //----------------
                    // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;        
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;
           case  0x8700:    //  行车记录仪数据采集命令
                    			 /*
															 行车记录仪发送出发开始
								   */
								  rt_kprintf( "\r\n  记录仪采集命令 \r\n" );
								  Recode_Obj.Float_ID = Centre_FloatID;

                                  /*
                                                             exam1  gghypt: 7E 87 00 00 15 01 39 01 23 45 03 01 A1 09 AA 75 09 00 0E 00 13 01 01 01 01 01 14 01 01 01 01 01 0A  D5 67 7E
                                                             exam2   cctic:   7E 87 00 00 01 01 36 01 30 00 01 C9 47 15 1A 7E
                                                            */

								   //------ 判断有没有起始时间 -----------
								   Recode_Obj.Get_withDateFlag=0;   // 初始化没有 GB19056 报文 
                                  if((UDP_HEX_Rx[14]==0xAA)&&(UDP_HEX_Rx[15]==0x75)&&(UDP_HEX_Rx[13]==UDP_HEX_Rx[16])) 
                                  {   // 判断该字段是否有 GB19056 2012 的字段内容  ，协议里写可为空，
                                      // 然后判断中心下发的命令字和GB19056 里的命令字是否一致
                                      GB19056infolen=(UDP_HEX_Rx[17]<<8)+UDP_HEX_Rx[18];
									  if(GB19056infolen==0x0E)   // 长度是14  
									  switch(UDP_HEX_Rx[16])
									  	{  // 判断ID 是否是  08 -15 
									  	    case 0x08:
										    case 0x09:
										    case 0x10:
										    case 0x11:
										    case 0x12:
										    case 0x13:
										    case 0x14:
										    case 0x15: 	  
												for(i=0;i<6;i++)
													{
										  	            Recode_Obj.Get_startDate[i]=UDP_HEX_Rx[20+i];  
														Recode_Obj.Get_endDate[i]=UDP_HEX_Rx[26+i];  

													}
											  Recode_Obj.Get_recNum=(UDP_HEX_Rx[31]<<8)+UDP_HEX_Rx[32];  // 获取的条数
											  Recode_Obj.Get_withDateFlag= 1; 

                                                      // ---  记录仪获取采集指定日志时间-- 
											     if(UDP_HEX_Rx[16]==0x15)
															VDR_get_15H_StartEnd_Time(UDP_HEX_Rx+20,UDP_HEX_Rx+26);
							

											  rt_kprintf( "\r\n  记录仪8700 带AA 75 start   %2X-%2X-%2X  %2X:%2X:%2X\r\n",UDP_HEX_Rx[20],UDP_HEX_Rx[21],UDP_HEX_Rx[22],UDP_HEX_Rx[23],UDP_HEX_Rx[24],UDP_HEX_Rx[25]);
											  rt_kprintf( "\r\n                     end     %2X-%2X-%2X  %2X:%2X:%2X  GetNUM: %d \r\n",UDP_HEX_Rx[26],UDP_HEX_Rx[27],UDP_HEX_Rx[28],UDP_HEX_Rx[29],UDP_HEX_Rx[30],UDP_HEX_Rx[31],Recode_Obj.Get_recNum); 
											  break;
										   default:
                                                      Recode_Obj.Get_withDateFlag=0;
											          break;
										}
                                         


								  
                                  }
								  
								  //Recode_Obj.CMD=UDP_HEX_Rx[13];   
								  //	Stuff  Hardly
								  switch( UDP_HEX_Rx[13] )
								  {
									  case 0x00:
									  case 0x01:
									  case 0x02:
									  case 0x03:	
									  case 0x04:
									  case 0x05:
									  case 0x06:
									  case 0x07:	
									  case 0x08:
									  case 0x09:
									  case 0x10:
									  case 0x11:
									  case 0x12:
									  case 0x13:
									  case 0x14:
									  case 0x15: 	  
									  	  Recode_Obj.CMD   = UDP_HEX_Rx[13]; 
										  Recode_Obj.SD_Data_Flag = 1;
										  Recode_Obj.CountStep	  = 1; 
										  break;
									  //--------- Lagre  block -------
								  
								  
									  /*
									        cmd 		maxnum
									        15H	       2
									        12H 	10
										 11H 	10
										 10H  	100
										 09H 	360
										 08H 	576								  
									   */
									/*  case	0x08:  Recode_Obj.Total_pkt_num   = 20;//720;
										  Recode_Obj.Current_pkt_num		  = 1;
										  Recode_Obj.Devide_Flag			  = 1;
										  //--------------------------------
										  Recode_Obj.CMD		  = UDP_HEX_Rx[13];
										  Recode_Obj.SD_Data_Flag = 1;
										  Recode_Obj.CountStep	  = 1;
										  MediaObj.Media_Type	  = 3; //行驶记录仪
										  break;
								  
									  case	0x09:  Recode_Obj.Total_pkt_num   =15; // 720;
										  Recode_Obj.Current_pkt_num		  = 0; //0
										  Recode_Obj.Devide_Flag			  = 1;
										  //--------------------------------
										  Recode_Obj.CMD		  = UDP_HEX_Rx[13];
										  Recode_Obj.SD_Data_Flag = 1;
										  Recode_Obj.CountStep	  = 1;
										  MediaObj.Media_Type	  = 3; //行驶记录仪
										  break;
									  case	0x10:
										  Recode_Obj.Total_pkt_num	  =10;// 100;
										  Recode_Obj.Current_pkt_num  = 1;
										  Recode_Obj.Devide_Flag	  = 1;
										  //--------------------------------
										  Recode_Obj.CMD		  = UDP_HEX_Rx[13];
										  Recode_Obj.SD_Data_Flag = 1;
										  Recode_Obj.CountStep	  = 1;
										  Recode_Obj.fcs			  = 0;
										  MediaObj.Media_Type	  = 3; //行驶记录仪
										  break;
									  case	0x11:
										  Recode_Obj.Total_pkt_num	  = 10;//100;
										  Recode_Obj.Current_pkt_num  = 1;
										  Recode_Obj.Devide_Flag	  = 1;
										  Recode_Obj.CMD			  = UDP_HEX_Rx[13];
										  Recode_Obj.SD_Data_Flag	  = 1;
										  Recode_Obj.CountStep		  = 1;
										  Recode_Obj.fcs			  = 0;
										  MediaObj.Media_Type		  = 3; //行驶记录仪
										  break;
									  case	0x12:
										  Recode_Obj.Total_pkt_num	  = 10;
										  Recode_Obj.Current_pkt_num  = 1;
										  Recode_Obj.Devide_Flag	  = 1;
										  Recode_Obj.CMD			  = UDP_HEX_Rx[13];
										  Recode_Obj.SD_Data_Flag	  = 1;
										  Recode_Obj.CountStep		  = 1;
										  Recode_Obj.fcs			  = 0;
										  MediaObj.Media_Type		  = 3; //行驶记录仪
										  break;
									  case	0x13:
										  //--------------------------------
										  Recode_Obj.CMD		  = UDP_HEX_Rx[13];
										  Recode_Obj.SD_Data_Flag = 1;
										  Recode_Obj.CountStep	  = 1;
										  break;
									  case	0x14:
										  //--------------------------------
										  Recode_Obj.CMD		  = UDP_HEX_Rx[13];
										  Recode_Obj.SD_Data_Flag = 1;
										  Recode_Obj.CountStep	  = 1;
										  break;
									  case	0x15:
										  Recode_Obj.Current_pkt_num  = 1;
										  Recode_Obj.Devide_Flag	  = 1;
										  Recode_Obj.CMD			  = UDP_HEX_Rx[13];
										  Recode_Obj.SD_Data_Flag	  = 1;
										  Recode_Obj.CountStep		  = 1;
										  Recode_Obj.Total_pkt_num	  = 2;
										  Recode_Obj.fcs			  = 0;
										  MediaObj.Media_Type		  = 3;						  //行驶记录仪
										  break;*/
									   default:
									   	      Recode_Obj.Error=1;  //  采集错误
												  break;
										 }
		              break;
		   case  0x8701:    //  行驶记录仪参数下传命令
		               rt_kprintf("\r\n  记录仪参数下传 \r\n");
		               Recode_Obj.Float_ID=Centre_FloatID;
					   Recode_Obj.CMD=UDP_HEX_Rx[13];    
					   Recode_Obj.SD_Data_Flag=1;  
					   CenterSet_subService_8701H(Recode_Obj.CMD,UDP_HEX_Rx+14); //跳过2B长度和1 保留字
			          break; 
		   case  0x8800:    //   多媒体数据上传应答
		              if(infolen==5)
		              	{  //  判断是否有重传ID列表，如果没有则表示中心接收完成!						
                                switch (MediaObj.Media_Type)
						         {
						          case 0 : // 图像
						                  Photo_send_end();  // 拍照上传结束
						                  rt_kprintf("\r\n 图像传输结束! \r\n");
                                          //------------多路拍照处理  -------
                                          CheckResualt=Check_MultiTakeResult_b4Trans();  
						                 				   
								          break;
								  case 1 : // 音频
								         #ifdef REC_VOICE_ENABLE
								          Sound_send_end();
										  rt_kprintf("\r\n 音频传输结束! \r\n");
										  #endif
								          break;
							
								  default:
								  	      break;
						         }
						 if(CheckResualt==0)	
						   	   Media_Clear_State();  
		              	}
					  else
					  	{  //  重传包ID 列表 
					  	  //if(UDP_HEX_Rx[17]!=0)
						  if((UDP_HEX_Rx[17]!=0)&&(MultiTake.Taking==0))
					  	  {
						  	   MediaObj.RSD_State=1;   //   进入重传状态
						  	   MediaObj.RSD_Timer=0;   //   清除重传定时器   
						  	   MediaObj.RSD_Reader=0;
							   MediaObj.RSD_total=UDP_HEX_Rx[17];    // 重传包总数 


						  	   memset(MediaObj.Media_ReSdList,0,sizeof(MediaObj.Media_ReSdList));                       
						       //   获取重传列表 
							   j=0;			
                               // 根据总包数后边第一个字节区分是老版本还是新版本
                               if(UDP_HEX_Rx[18]!=0)   // 老版本
                               {
									   for(i=0;i<MediaObj.RSD_total;i++)
									   	{
											//----- 天地通-----									
		                                    MediaObj.Media_ReSdList[i]=UDP_HEX_Rx[18+i]; 
										}		
                               	}
							   else
							   	{          //  新版本协议                                   
									 for(i=0;i<MediaObj.RSD_total;i++)
									  {
										  // formal
										  MediaObj.Media_ReSdList[i]=(UDP_HEX_Rx[18+j]<<8)+UDP_HEX_Rx[19+j];
										  j+=2;
									  } 
							   	}
							   
	                           rt_kprintf("\r\n  8800 重传列表Total=%d: ",MediaObj.RSD_total);
							   for(i=0;i<MediaObj.RSD_total;i++)
							         rt_kprintf("%d,",MediaObj.Media_ReSdList[i]);   
							   rt_kprintf("\r\n");   

					  	   } 
						}
                        
		              break;
					  //---------  BD  add -------------------------------
			 case  0x8003: // BD--8.4 分包请求上传
												// nouse	Detach_PKG.Original_floatID=(UDP_HEX_Rx[13]<<8)+UDP_HEX_Rx[14];//原始消息流水号
								  MediaObj.RSD_State=1;   //   进入重传状态
						  	      MediaObj.RSD_Timer=0;   //   清除重传定时器   
						  	      MediaObj.RSD_Reader=0;				
								  MediaObj.RSD_total=UDP_HEX_Rx[15];
								  //  重传列表
								  reg_u32=0;// 借用做下标计数器
								  for(i=3;i<infolen;i+=2)
								  {
									 MediaObj.Media_ReSdList[reg_u32]=(UDP_HEX_Rx[16+2*reg_u32]<<24)+UDP_HEX_Rx[17+2*reg_u32+1];
									 reg_u32++; 
								  }

								 rt_kprintf("\r\n  8003 重传列表Total=%d: ",MediaObj.RSD_total);
							     for(i=0;i<MediaObj.RSD_total;i++)
							         rt_kprintf("%d,",MediaObj.Media_ReSdList[i]);  
							     rt_kprintf("\r\n");  
					  
									   {
									   SD_ACKflag.f_CentreCMDack_0001H=1;
									   Ack_Resualt=0;
									   SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
									 }	  
								   
								break;	  		   
					  
		   case  0x8801:   //    摄像头立即拍照		  

				     Camera_Obj.Channel_ID=UDP_HEX_Rx[13];     //   通道
					 Camera_Obj.Operate_state=UDP_HEX_Rx[18];  //   是否保存标志位  
		            //----------------------------------
				   
					if((Camera_Take_Enable())&&(Photo_sdState.photo_sending==0)) //图片传输中不能拍
					{
					   Camera_Number=UDP_HEX_Rx[13];
					   if((Camera_Number>Max_CameraNum)&&(Camera_Number<1)) 
						  break;
					 
					   Start_Camera(Camera_Number);   //开始拍照  
					   SingleCamera_TakeRetry=0;   // clear 
					}	   
					rt_kprintf("\r\n   中心及时拍照  Camera: %d    \r\n",Camera_Number);	

                                    if(UDP_HEX_Rx[18]==0x01)    // 拍照不上传
                                      {    
                                          Camera_Take_not_trans=1;
                                           rt_kprintf("\r\n   拍照不上传\r\n");
                                      }				
					else
						Camera_Take_not_trans=0;  
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt; 
 					 }
                      break;
		   case  0x8802:   //    存储多媒体数据检索
		                 SD_ACKflag.f_QueryEventCode=UDP_HEX_Rx[15];    
                         switch(UDP_HEX_Rx[13])
                         {
                            case 0:  // 图像
                                     SD_ACKflag.f_MediaIndexACK_0802H=1;
									 rt_kprintf("\r\n  中心查询图像索引 \r\n");   
							       break;
							case 1:  //  音频
                                     SD_ACKflag.f_MediaIndexACK_0802H=2;
									  rt_kprintf("\r\n  中心查询音频索引 \r\n"); 	 
							case 2:  //  视频
							         SD_ACKflag.f_MediaIndexACK_0802H=3;
						    default:
								     break; 
						}						 
						
		              break;
		   case  0x8803:   //    存储多媒体数据上传命令
                      rt_kprintf("\r\n 多媒体数据上传\r\n"); 
                       switch(UDP_HEX_Rx[13])
                         {
                            case 0:  // 图像
									 rt_kprintf("\r\n   上传固有图片\r\n");    
							       break;
							case 1:  //  音频
                                    // MP3_send_start();
									  rt_kprintf("\r\n  上传固有音频 \r\n"); 
								   break;	  
							case 2:  //  视频
							        // Video_send_start();
							         // MP3_send_start();
									  rt_kprintf("\r\n  上传固有视频  不操作了 用音频\r\n"); 
								   break;	  
						    default:
								     break; 
						}				
					  
                     //----------------------------------------------------------
                     // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;
		   case  0x8804:   //    录音开始命令
		   
		            //#if  1			     
                       switch(UDP_HEX_Rx[13])
                       	{ 
                       	  case 0x00:  // 停录音

						            break;
						  case 0x01:  // 开始录音
                                     #ifdef REC_VOICE_ENABLE                  
								       VOC_REC_Start();	 
									 #endif		
							        break;

                       	}
					  
					      
					  //------------------------------
					 //if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;

		 case  0x8900:   //    数据下行透传                      
                                   switch(UDP_HEX_Rx[13]) // 透传消息类型  // BD
                                   {
                                         case  0x41 : // 串口1  透传消息
                                               rt_kprintf("\r\n 串口1 透传:"); 
                                                  for(i=0;i<(infolen-1);i++)
                                                       rt_kprintf("%c",UDP_HEX_Rx[14+i]);   
											   rt_kprintf("\r\n");  	  
												  	   
								         break;
									case  0x42:  //  串口2 透传消息
									           rt_kprintf("\r\n 串口2 透传:"); 
                                               for(i=0;i<(infolen-1);i++)
                                                       rt_kprintf("%c",UDP_HEX_Rx[14+i]);   
											   rt_kprintf("\r\n");  

									                   break;
									case  0x0B:  //  IC  卡 信息
                                    /*                         memcpy(IC_MOD.IC_Rx,UDP_HEX_Rx+14,infolen-1);
								     rt_kprintf("\r\n IC 卡透传len=%dBytes  RX:",infolen-1);
									 for(i=0;i<infolen-1;i++)
									 	rt_kprintf("%2X ",UDP_HEX_Rx[14+i]); 
								     rt_kprintf("\r\n");	 
                                                             //------ 直接发送给IC 卡模块-----
                                                             Reg_buf[0]=0x00;
								     memcpy(Reg_buf,IC_MOD.IC_Rx,infolen-1);
                                                             DeviceData_Encode_Send(0x0B,0x40,Reg_buf,infolen); 									 
						                   return;  */
						                   break;

                                    }
		                       //---------------------------------------------------   
		                      if(LinkNum==0)
		                     {
                                     DataTrans.TYPE=UDP_HEX_Rx[13];
					  memset(DataTrans.DataRx,0,sizeof(DataTrans.DataRx)); 
                                     memcpy(DataTrans.DataRx,UDP_HEX_Rx+14,infolen-1);
					  DataTrans.Data_RxLen=infolen-1;   

					  //--------- 送给小屏幕---------- 	
					  	memset( TextInfo.TEXT_Content,0,sizeof(TextInfo.TEXT_Content));
						AsciiToGb(TextInfo.TEXT_Content,infolen-1,UDP_HEX_Rx+14); 
						TextInfo.TEXT_SD_FLAG=1;	// 置发送给显示屏标志位  // ||||||||||||||||||||||||||||||||||

						//========================================
						   

						     #ifdef LCD_5inch
						 
							 DwinLCD.Type=LCD_SDTXT;
							  memset(DwinLCD.TXT_content,0,sizeof(DwinLCD.TXT_content)); 
                                                   DwinLCD.TXT_contentLen=AsciiToGb(DwinLCD.TXT_content,infolen-1,UDP_HEX_Rx+14);  						  
				   
							#endif  
					 // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		   	        }
				 else
				 {
					  	rt_kprintf("\r\n透传数据类型错误UDP_HEX_Rx[13]=%d",UDP_HEX_Rx[13]);
				 }
		              break;
		   case  0x8A00:   //    平台RSA公钥

		   
                    // if(SD_ACKflag.f_CentreCMDack_0001H==0) 
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 } 
				      break;	
			case  0x8106://BD--8.11 查询指定终端参数
			                //7E 81 06 00 05 01 39 01 23 45 05 00 2A 01 00 00 00 21 D2 7E
			                //7E81060005013901234505002A0100000021D27E
			                //7E81060005013901234505002A03000000210000009400000055D27E
			                Setting_Qry.Num_pram=UDP_HEX_Rx[13];// 个数
			                if(Setting_Qry.Num_pram>90)
								     Setting_Qry.Num_pram=90;
                            reg_u32=0;// 借用做下标计数器

                          //  CMD_U32ID=(UDP_HEX_Rx[14+4*reg_u32]<<24)+(UDP_HEX_Rx[14+4*reg_u32+1]<<16)+(UDP_HEX_Rx[14+4*reg_u32+2]<<8)+(UDP_HEX_Rx[14+4*reg_u32+3]);
							
							for(i=0;i<Setting_Qry.Num_pram;i++)  
							{
	                           Setting_Qry.List_pram[reg_u32]=(UDP_HEX_Rx[14+4*reg_u32]<<24)+(UDP_HEX_Rx[14+4*reg_u32+1]<<16)+(UDP_HEX_Rx[14+4*reg_u32+2]<<8)+(UDP_HEX_Rx[14+4*reg_u32+3]); 
							   rt_kprintf("ID:  %4X ",Setting_Qry.List_pram[reg_u32]); 
							   reg_u32++;     
							}
							

							

						     SD_ACKflag.f_SettingPram_0104H=2;	// 中心查询指定参数	 		   
			                 break;
			case  0x8107:// BD--8.14 查询终端属性   
                                             SD_ACKflag.f_BD_DeviceAttribute_8107=1; // 消息体为空
			                       break;
             case   0x8108://BD--8.16 下发终端升级包   远程升级  (重要) 

                            //   1.  分包判断bit 位  
                            if(UDP_HEX_Rx[3]&0x20) 
						  	{   //  分包判断
						  	     BD_ISP.Total_PacketNum=(UDP_HEX_Rx[13]<<8)+UDP_HEX_Rx[14];  // 总包数
						         BD_ISP.CurrentPacket_Num=(UDP_HEX_Rx[15]<<8)+UDP_HEX_Rx[16]; // 当前包序号从1 开始
						  	} 
						    else
						 	{
							 	//------------   ACK   Flag -----------------
							  	 SD_ACKflag.f_CentreCMDack_0001H=1;
		 						 Ack_Resualt=0;
		 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
						 	}	
						  //  exception	
                          if(BD_ISP.Total_PacketNum==0)
						  	{
                               rt_kprintf("\r\n 总包数为0  异常\r\n"); 
							    break;
                          	}
							
						//   2.  检测 
						if(BD_ISP.CurrentPacket_Num==1)
						{	
						          ISP_judge_resualt=0;
	                              BD_ISP.Update_Type=UDP_HEX_Rx[17];  //升级包类型
		                                            //----  Debug ------
								  switch(BD_ISP.Update_Type)
								  {
								      case  0:   
									  	      rt_kprintf("\r\n 升级类型: 终端\r\n");
										      break;
								      case  12:   
									  	      rt_kprintf("\r\n 升级类型: IC 卡读卡器\r\n");
										      break;
								      case  52:   
									  	      rt_kprintf("\r\n 升级类型: 北斗模块\r\n");
										      break;
								  }
	                             //----------------------
	                              memcpy(BD_ISP.ProductID,UDP_HEX_Rx+18,5);// 制造商ID
	                              BD_ISP.Version_len=UDP_HEX_Rx[23]; // 版本号长度
								  memcpy(BD_ISP.VersionStr,UDP_HEX_Rx+20,BD_ISP.Version_len);	// 软件版本号
								  i=24+BD_ISP.Version_len;
								  BD_ISP.Content_len=(UDP_HEX_Rx[i]<<24)+(UDP_HEX_Rx[i+1]<<16)+(UDP_HEX_Rx[i+2]<<8)+UDP_HEX_Rx[i+3];// 升级包长度
								  i+=4;  
								  rt_kprintf("\r\n 制造商ID:%s\r\n",BD_ISP.ProductID); 
		                          rt_kprintf("\r\n 升级包长度:%d Bytes\r\n",BD_ISP.Content_len); 

								  infolen=infolen-11-BD_ISP.Version_len;// infolen 					 

								  BD_ISP.PacketRX_wr=0; // clear
								  BD_ISP.ISP_running=1;			

                                   i=17+24;

                                  //   判断自己文件
								    memcpy(BD_ISP.ContentData,UDP_HEX_Rx+i,infolen); // 升级包内容	
								    //SST25V_BufferRead(ISP_buffer,ISP_StartArea,256); //---------------
									//OutPrint_HEX("第一包",BD_ISP.ContentData,infolen);
                                  if(strncmp(BD_ISP.ContentData+32+13,"70420TW703",10)==0) //判断厂商和型号
                                   //if(strncmp(BD_ISP.ContentData+32+13,ISP_buffer+14,10)==0) //判断厂商和型号
                                   	{
                                   	    ISP_judge_resualt++;// step 1
                                        rt_kprintf("\r\n 厂商型号正确\r\n");  

                                        // hardware 
										HardVersion=(BD_ISP.ContentData[32+38]<<24)+(BD_ISP.ContentData[32+39]<<16)+(BD_ISP.ContentData[32+40]<<8)+BD_ISP.ContentData[32+41]; 
									     rt_kprintf("\r\n 硬件版本:%d\r\n",HardVersion); 
										 HardWareVerion=HardWareGet();
										 if((HardWareVerion==HardVersion)||(HardWareVerion==7))	// 要兼容以前的老板子 全1								 	
	                                          ISP_judge_resualt++;// step 2
	                                     else
										 	 rt_kprintf("\r\n 硬件版本不匹配!\r\n");   
                                        
										//firmware
										if(strncmp(BD_ISP.ContentData+32+42,"TJGGHYPT",8)==0)  
										{
										   ISP_judge_resualt++;// step 3
										   rt_kprintf("\r\n  固件版本:TJGGHYPT\r\n");   
										}
										// operater
										if(strncmp(BD_ISP.ContentData+32+62,"TJGTGS",6)==0)  
										{
										   ISP_judge_resualt++;// step 4
										   rt_kprintf("\r\n  固件版本:TJGTGS\r\n");       
										}
										
                                   	}

								  
								  //  根据判断是否是本机的然后进行操作，如不是返回取消    
								  // 若是 第一包，擦除相关区域
								  if(ISP_judge_resualt==4) 
								     DF_Erase();  
								  else
								  	{									  	
	                                  SD_ACKflag.f_BD_ISPResualt_0108H=3;  // 取消
	                                  rt_kprintf("\r\n 类型不匹配失败!\r\n");  
									  break; 
								  	}

								  
						}		
						else
							 i=17; 
						   //3. --------  升级包内容  -------------		
                           if(BD_ISP.CurrentPacket_Num!=BD_ISP.Total_PacketNum)
								 BD_ISP.PacketSizeGet=infolen;
						   
						   if(infolen) //  升级内容以后根据类型往不同的存储区域写
                            {
                                  memcpy(BD_ISP.ContentData,UDP_HEX_Rx+i,infolen); // 升级包内容
                                  BD_ISP.PacketRX_wr+=infolen;
                                  //---- 替换更新标志-----
                                  if(BD_ISP.CurrentPacket_Num==1)
								  	     BD_ISP.ContentData[32]=ISP_BYTE_StartValue;        
								  
								  // OutPrint_HEX("Write",BD_ISP.ContentData,infolen);   
                                  //  判断写入地址
                                  if(((BD_ISP.CurrentPacket_Num-1)*BD_ISP.PacketSizeGet)>257024) //257024=251*1024
                                  	{                                  	
									     SD_ACKflag.f_BD_ISPResualt_0108H=3;  // 取消
									    rt_kprintf("\r\n 地址偏出区域 --异常 ，失败!\r\n"); 
								  	    break;
                                  	}	
								  
                                  //  write area   
                                  WatchDog_Feed();
                                  SST25V_BufferWrite(BD_ISP.ContentData,ISP_StartArea+(BD_ISP.CurrentPacket_Num-1)*BD_ISP.PacketSizeGet,infolen);
								  //delay_ms(160);
								  rt_thread_delay(12);  
								  WatchDog_Feed();
                            

								  // read 
								  memset(ISP_buffer,0,sizeof(ISP_buffer));
								  SST25V_BufferRead(ISP_buffer,ISP_StartArea+(BD_ISP.CurrentPacket_Num-1)*BD_ISP.PacketSizeGet,infolen); 
								  //delay_ms(120);
								  rt_thread_delay(8); 

								  for(i=0;i<infolen;i++)
								  {
                                     if(BD_ISP.ContentData[i]!=ISP_buffer[i])
                                     	{
                                          rt_kprintf("\r\n ISP error at :%d   wr=0x%2X rd=0x%2X\r\n",i,BD_ISP.ContentData[i],ISP_buffer[i]);  
                                          break;
                                     	}

								  }
								  if(i!=infolen)
								  	{
                                      rt_kprintf("\r\n DF 读写 有出入!  \r\n");
									  return;
								  	}
                                //  OutPrint_HEX("Read",ISP_buffer,infolen);   
								  
						    }
						   
						   rt_kprintf("\r\n  ISP  当前包数:%d	--- 总包数:%d    contentlen=%d Bytes \r\n",BD_ISP.CurrentPacket_Num,BD_ISP.Total_PacketNum,infolen);
						   
                           //4.----------- ISP state update ---------------------- 
                           BD_ISP.ISP_running=1;
                           BD_ISP.ISP_runTimer=0;     

						   //5. --------  ISP ACK process  -----------
						   if((BD_ISP.CurrentPacket_Num==BD_ISP.Total_PacketNum)&&(i==infolen))
						   {
						        // File_CRC_Get(); 
						       SD_ACKflag.f_BD_ISPResualt_0108H=1;   //正常
							   rt_kprintf("\r\n 最后一包\r\n"); 
						   }
						   else
						   if(BD_ISP.CurrentPacket_Num>BD_ISP.Total_PacketNum)
						   	{
                                
                                  SD_ACKflag.f_BD_ISPResualt_0108H=3;  // 取消
                                 rt_kprintf("\r\n 当前包大于总包数 --异常 ，失败!\r\n"); 
						   	}
						   else
						   	{
                                
								//------------	 ACK   Flag -----------------
		  					    SD_ACKflag.f_CentreCMDack_0001H=1;
							    Ack_Resualt=0;
							    SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt; 
						    	 //------------------------------------------------------------------------   

						   	} 
                            break;
            case   0x8203://BD--8.22  人工确认报警消息
                                     rt_kprintf( "\r\n收到8203!\r\n"); 
                             	    if (Warn_Status[3]&0x01)  //紧急报警
									  {
											  StatusReg_WARN_Clear();			  
											  f_Exigent_warning = 0;
											  warn_flag=0;
											  Send_warn_times=0; 
											   StatusReg_WARN_Clear();	
											   rt_kprintf( "\r\n紧急报警收到应答，得清除!\r\n");   
									  }
			 
                                            HumanConfirmWarn.Warn_FloatID=(UDP_HEX_Rx[13]<<8)+UDP_HEX_Rx[14];
                                            HumanConfirmWarn.ConfirmWarnType=(UDP_HEX_Rx[15]<<24)+(UDP_HEX_Rx[16]<<16)+(UDP_HEX_Rx[17]<<8)+UDP_HEX_Rx[18];
                                            if(HumanConfirmWarn.Warn_FloatID==0x00)
                                            	{    //  如果为0 表示所有消息
                                            	     Warn_Status[3] &= ~0x01;  // bit 0
							     Warn_Status[3]&=~0x08;	// bit3
							     Warn_Status[1]&=~0x10;	// bit20 进出区域报警 
							     Warn_Status[1]&=~0x20;    // bit 21 进出线路报警
							     Warn_Status[1]&=~0x40; 	// bit 22  路段行驶时间不足报警 
							     Warn_Status[0]&=~0x08;    // bit 27  确认车辆非法点火报警
							     Warn_Status[0]&=~0x10;    // bit 28  确认车辆非法报警
                                                      Warn_Status[0]&=~0x80;    //bit 31  非法开门 (终端不设置区域时，车门开关无效) 
                                            	}
						  else
						  	{
						  	     	  //--------------------------------
									  if ((Warn_Status[3]&0x01)&&(0x00000001&HumanConfirmWarn.ConfirmWarnType))  //紧急报警
									  {
											  StatusReg_WARN_Clear();		 	  
											  f_Exigent_warning = 0;
											  warn_flag=0;
											  Send_warn_times=0; 
											   StatusReg_WARN_Clear();	
											   rt_kprintf( "\r\n紧急报警收到应答，得清除!\r\n");   
									  }
			                                      	  if( (Warn_Status[3]&0x08)&&(0x00000008&HumanConfirmWarn.ConfirmWarnType))   //危险报警-BD
									    {
														  Warn_Status[3]&=~0x08;
									     }
									  
									  //------------------------------------
				                                     if((Warn_Status[1]&0x10)&&(0x00001000&HumanConfirmWarn.ConfirmWarnType)) // 进出区域报警
				                                     {
										                  InOut_Object.TYPE=0;//圆形区域
										                  InOut_Object.ID=0; //  ID
										                  InOut_Object.InOutState=0;//  进报警 
										                  Warn_Status[1]&=~0x10;
				                                     } 
									if((Warn_Status[1]&0x20)&&(0x00002000&HumanConfirmWarn.ConfirmWarnType)) // 进出路线报警
				                                     {
										                  InOut_Object.TYPE=0;//圆形区域
										                  InOut_Object.ID=0; //  ID
										                  InOut_Object.InOutState=0;//  进报警 
										                  Warn_Status[1]&=~0x20;  
				                                     } 
									 if((Warn_Status[1]&0x40)&&(0x00004000&HumanConfirmWarn.ConfirmWarnType))  // 路段行驶时间不足、过长
									 {
				                                                  Warn_Status[1]&=~0x40; 
									 }
									  //-----------------------------------------
									   if((Warn_Status[0]&0x08)&&(0x08000000&HumanConfirmWarn.ConfirmWarnType))  //车辆非法点火
									  {
                                                                              Warn_Status[0]&=~0x08; 
									  }
									   if((Warn_Status[0]&0x10)&&(0x10000000&HumanConfirmWarn.ConfirmWarnType))  //车辆非法位移
									  {
                                                                              Warn_Status[0]&=~0x10; 
									  } 
									      if((Warn_Status[0]&0x80)&&(0x80000000&HumanConfirmWarn.ConfirmWarnType))  //非法开门报警(终端未设置区域时，不判断非法开门)
                                                                {              
                                                                        Warn_Status[0]&=~0x80; 
									  }
									  //------------------------------------  
						  	}

                             		   //------------   ACK   Flag -----------------
						  	 SD_ACKflag.f_CentreCMDack_0001H=1;
	 						 Ack_Resualt=0;
	 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;	
							    //---------  报警确认后发送，立即发送定位包				
                             PositionSD_Enable();
							    Current_UDP_sd=1;	
						  break;					
                     case   0x8702:// BD-8.47-  上报驾驶员身份信息请求
                                             SD_ACKflag.f_DriverInfoSD_0702H=1; // 消息体为空
					         break;
		       case  0x8805:   //BD--8.55    单条存储多媒体数据检索上传命令	---- 补充协议要求

	                                    reg_u32=(UDP_HEX_Rx[13]<<24)+(UDP_HEX_Rx[14]<<16)+(UDP_HEX_Rx[15]<<8)+UDP_HEX_Rx[16];
						  rt_kprintf("\r\n单条存储多媒体数据检索上传 MeidiaID=%d 删除标志: %d ",reg_u32,UDP_HEX_Rx[17]);

                                           Camera_Number=1; 
                                           Photo_send_start(Camera_Number);  //在不是多路拍照的情况下拍完就可以上传了
							  //------------------------------
						 // if(SD_ACKflag.f_CentreCMDack_0001H==0)
	 					 {
	 						 SD_ACKflag.f_CentreCMDack_0001H=1; 
	 						 Ack_Resualt=0;
	 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
	 					 }
		                   break;
		    default:
				      break;  	    
	    } 

		//-----------------  memset  ------------------------------------- 
		//memset(UDP_HEX_Rx, 0, sizeof(UDP_HEX_Rx));	
		//UDP_hexRx_len= 0; 
        return;  

}


void Time2BCD(u8 *dest)
 {
    if(UDP_dataPacket_flag==0x02)
   {
		 
		   dest[0]=((Temp_Gps_Gprs.Date[0]/10)<<4)+(Temp_Gps_Gprs.Date[0]%10);  
		   dest[1]=((Temp_Gps_Gprs.Date[1]/10)<<4)+(Temp_Gps_Gprs.Date[1]%10);//Temp_Gps_Gprs.Date[1];
		   dest[2]=((Temp_Gps_Gprs.Date[2]/10)<<4)+(Temp_Gps_Gprs.Date[2]%10);//Temp_Gps_Gprs.Date[2]; 
		   
		   dest[3]=((Temp_Gps_Gprs.Time[0]/10)<<4)+(Temp_Gps_Gprs.Time[0]%10);//Temp_Gps_Gprs.Time[0]; 
		   dest[4]=((Temp_Gps_Gprs.Time[1]/10)<<4)+(Temp_Gps_Gprs.Time[1]%10);//Temp_Gps_Gprs.Time[1]; 
		   dest[5]=((Temp_Gps_Gprs.Time[2]/10)<<4)+(Temp_Gps_Gprs.Time[2]%10);//Temp_Gps_Gprs.Time[2];   
	   
   }
   else
   {
		dest[0]=((time_now.year/10)<<4)+(time_now.year%10);  	   	
		dest[1]=((time_now.month/10)<<4)+(time_now.month%10); 
		dest[2]=((time_now.day/10)<<4)+(time_now.day%10);
		dest[3]=((time_now.hour/10)<<4)+(time_now.hour%10);
		dest[4]=((time_now.min/10)<<4)+(time_now.min%10);
		dest[5]=((time_now.sec/10)<<4)+(time_now.sec%10);     
	}

 }

void AvrgSpd_MintProcess(u8 hour,u8 min, u8 sec)  
{
	   //------- 系统启动时处理   若 标志位为0 且年月日都为 0，表面是系统刚刚启动，这时获取当前时间把当前时间给BAK------
	if((0==Avrgspd_Mint.saveFlag)&&(Avrgspd_Mint.datetime_Bak[0]==0)&&(Avrgspd_Mint.datetime_Bak[1]==0)&&(Avrgspd_Mint.datetime_Bak[2]==0))
		{
		Time2BCD(Avrgspd_Mint.datetime);  
		memcpy(Avrgspd_Mint.datetime_Bak,Avrgspd_Mint.datetime,6);//将当前时间给BAK
		avgspd_Mint_Wr=Temp_Gps_Gprs.Time[1];//time_now.min;
		//rt_kprintf("\r\n-------------()avgspd_Mint_Wr=%d",avgspd_Mint_Wr);
		}	
	else
		{     // 若不是系统启动开始   
		if(min==0) //  把秒数写为非0 ，以便留出时间去存储上1小时的数据 
			{
			if(sec==5)  //存储上一小时每分钟的平均速度 ==2是因为要保证第59分钟速度已经被存储到相应的寄存器中
				{
				memcpy(Avrgspd_Mint.datetime_Bak,Avrgspd_Mint.datetime,6);//将当前时间给BAK
				Avrgspd_Mint.datetime_Bak[4]=0;// 分钟为0
				Avrgspd_Mint.datetime_Bak[5]=0;// 秒也为0 
				Avrgspd_Mint.saveFlag=1;//使能存储      到NandFlag  标志位
				Time2BCD(Avrgspd_Mint.datetime);//获取新的时间
				//rt_kprintf("\r\n存储前一小时平均速度,min=%d",min);
				} 
			if(sec==8)    //存储完后  清除时间下标  
				avgspd_Mint_Wr=0;
			}
		}
	
	if(sec==0)  // 秒为0时存储上1分钟的数据
		{
		//rt_kprintf("\r\n下一分钟开始,PerMinSpdTotal=%d,AspdCounter=%d",PerMinSpdTotal,AspdCounter);
		if(AspdCounter)
			{			
			Avrgspd_Mint.avgrspd[avgspd_Mint_Wr]=PerMinSpdTotal/AspdCounter;//第几分钟的平均速度			
			//---rt_kprintf("\r\n--------------stuff spd=%d PerMinSpdTotal=%d,AspdCounter=%d",Avrgspd_Mint.avgrspd[avgspd_Mint_Wr], PerMinSpdTotal,AspdCounter);  
			//=============================================================
			//为了好测试暂时把1分钟内速度总和当作平均速度来统计
			//Avrgspd_Mint.avgrspd[avgspd_Mint_Wr]=PerMinSpdTotal;
			//=============================================================
			}
		else
			{
			Avrgspd_Mint.avgrspd[avgspd_Mint_Wr]=0; 
			//rt_kprintf("\r\n-------------(速度无效)");
			}
		//rt_kprintf("\r\n--------------%d分,平均速度=%d \r\n", avgspd_Mint_Wr,Avrgspd_Mint.avgrspd[avgspd_Mint_Wr]);  
		//avgspd_Mint_Wr++;//=(Avrgspd_Mint.datetime[4]>>4)*10+(0x0f&Avrgspd_Mint.datetime[4]);  
		
		PerMinSpdTotal=0;  // clear   
		AspdCounter=0;	   // clear  
		
		//rt_kprintf("\r\n清0，速度总和=%d,次数=%d\r\n",PerMinSpdTotal,AspdCounter);
		
		}
	else
		{ 
		if((UDP_dataPacket_flag==0x02)||((JT808Conf_struct.DF_K_adjustState==1)&&(JT808Conf_struct.Speed_GetType==1)))
			{
			avgspd_Mint_Wr=Temp_Gps_Gprs.Time[1];;//++;//=time_now.min+1;
			
			AspdCounter++;
			PerMinSpdTotal+=Spd_Using/10;   // 只要求精确到 km/h   所以要除 10 
		   // PerMinSpdTotal+=(Spd_Using+Spd_add_debug)/10;   // 只要求精确到 km/h   所以要除 10 	  
			  
			//if(AspdCounter%10==0)
				//rt_kprintf("\r\nAspdCounter=%d,Spd_Using=%d,PerMinSpdTotal=%d \r\n",AspdCounter,Spd_Using/10,PerMinSpdTotal);  
		
			} 
		}  
}

//==================================================================================================
// 第四部分 :   以下是行车记录仪相关协议 即 附录A   
//==================================================================================================

//  1.  上载数据命令字相关  

//------------------------------------------------------------------
void  Process_GPRSIN_DeviceData(u8 *instr, u16  infolen)
{
   u8  fcs=0;
   u16 i=0;



   //   caculate  and   check fcs
   for(i=0;i<infolen-1;i++)
   	  fcs^=instr[i];
   
   if(fcs!=instr[infolen-1])
   	  return;
   //  classify  cmd             
   switch(instr[2])   // AAH 75H CMD
   	{
   	  //  上行
      case 0x00:  // 采集行车记录仪执行标准版本号
                 Adata_ACKflag.A_Flag__Up_Ver_00H=0xff;
				 break;
	  case 0x01:  // 采集当前驾驶人信息
	             Adata_ACKflag.A_Flag_Up_DrvInfo_01H=instr[2];
				 break;
	  case 0x02:  // 采集记录仪的实时时钟
	             Adata_ACKflag.A_Flag_Up_RTC_02H=instr[2];
				 break;
	  case 0x03:  // 采集行驶里程
	             Adata_ACKflag.A_Flag_Up_Dist_03H=instr[2];
				 break;
	  case 0x04:  // 采集记录仪速度脉冲系数
	             Adata_ACKflag.A_Flag_Up_PLUS_04H=instr[2];
				 break;
	  case 0x06:  // 采集车辆信息
	             Adata_ACKflag.A_Flag_Up_VechInfo_06H=instr[2];
				 break;
	  case 0x08:  // 采集记录仪状态信号配置信息
	             Adata_ACKflag.A_Flag_Up_SetInfo_08H=instr[2];
				 break;
	  case 0x16:  // 采集记录仪唯一编号
	             Adata_ACKflag.A_Flag_Up_DevID_16H=instr[2];
				 break;
	  case 0x09:  // 采集指定的每秒钟平均速度记录            
	             Adata_ACKflag.A_Flag_Up_AvrgSec_09H=instr[2];  // 有起始结束时间
	  	         break;
	  case 0x05: // 采集指定的每分钟平均速度记录
	             Adata_ACKflag.A_Flag_Up_AvrgMin_05H=instr[2]; // 有起始结束时间
	             break;
	  case 0x13: // 采集指定的位置信息记录
	             Adata_ACKflag.A_Flag_Up_Posit_13H=instr[2];
				 break;
	  case 0x07: // 采集事故疑点记录
	             Adata_ACKflag.A_Flag_Up_Doubt_07H=instr[2];  // 有起始结束时间
	             break;
	  case 0x11: // 采集指定的疲劳驾驶记录
	             Adata_ACKflag.A_Flag_Up_Tired_11H=instr[2]; // 有起始结束时间
				 break;
	  case 0x10: // 采集指定的登录退出记录
	             Adata_ACKflag.A_Flag_Up_LogIn_10H=instr[2]; // 有起始结束时间
	             break;
	  case 0x14: // 采集指定的记录仪外部供电记录
	            Adata_ACKflag.A_Flag_Up_Powercut_14H=instr[2]; // 有起始结束时间
				break;
	  case 0x15: // 采集指定的记录仪参数修改记录
	            Adata_ACKflag.A_Flag_Up_SetMdfy_15H=instr[2];
				break;
	  //  下行
	  case 0x82: // 设置车辆信息                 
				    memset(Vechicle_Info.Vech_VIN,0,sizeof(Vechicle_Info.Vech_VIN));
				    memset(Vechicle_Info.Vech_Num,0,sizeof(Vechicle_Info.Vech_Num));
				    memset(Vechicle_Info.Vech_Type,0,sizeof(Vechicle_Info.Vech_Type)); 	
                 
					 //-----------------------------------------------------------------------
					 memcpy(Vechicle_Info.Vech_VIN,instr,17);
					 memcpy(Vechicle_Info.Vech_Num,instr+17,12);
					 memcpy(Vechicle_Info.Vech_Type,instr+29,12); 
					 
			
				 	DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));      
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					WatchDog_Feed();
				    DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 

                 Adata_ACKflag.A_Flag_Dn_DrvInfo_82H=instr[2];
				    
					Settingchg_Status=0x82;//设置车辆信息 					
					NandsaveFlg.Setting_SaveFlag=1;  //存储参数修改记录
	  case 0x83:  // 设置初次安装日期
                             memcpy(JT808Conf_struct.FirstSetupDate,instr,6); 
				  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                 Adata_ACKflag.A_Flag_Dn_SetupDate_83H=instr[2];
				 Settingchg_Status=0x83;//设置车辆信息 				 
				 NandsaveFlg.Setting_SaveFlag=1;  //存储参数修改记录
				 break;
	  case 0x84:  // 设置状态量信息	
	             Settingchg_Status=0x84;//设置车辆信息 	             
				 NandsaveFlg.Setting_SaveFlag=1;  //存储参数修改记录
	             break;
	  case 0xc2: // 设置记录仪时钟
                 Adata_ACKflag.A_Flag_Dn_RTC_C2H=instr[2];
				 Settingchg_Status=0xc2;//设置车辆信息 				 
				 NandsaveFlg.Setting_SaveFlag=1;  //存储参数修改记录
				 break;
	  case 0xc3: // 设置记录仪速度脉冲系数                 
				  JT808Conf_struct.Vech_Character_Value=(u32)(instr[0]<<24)+(u32)(instr[1]<<16)+(u32)(instr[2]<<8)+(u32)(instr[3]); // 特征系数	速度脉冲系数
				  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
		         Adata_ACKflag.A_Flag_Dn_Plus_C3H=instr[2];   
				 Settingchg_Status=0xc3;//设置车辆信息  				 
				 NandsaveFlg.Setting_SaveFlag=1;  //存储参数修改记录
				 break;
	  default:

		         break;
				 
				 	


   	}
}

#if 0

u8 RecordSerial_output_Str(const char *fmt,...) 
{
    u8 regstr[100],fcs=0;
	u16 reglen=0,i=0;
	
	va_list args;
	va_start(args, fmt);
	  memset(regstr,0,sizeof(regstr));  
	regstr[0]=0x55; // 协议头    
    regstr[1]=0x7A;
	regstr[2]=0xFE; // 命令字 ，用预留命令字表示调试输出	
	//  3,4 为长度字节最后填写
	regstr[5]=0x00; // 备用字  0x00

	reglen= vsprintf((char*)regstr+6, fmt, args);
	va_end(args); 
	regstr[3]=(u8)(reglen>>8);  // 填写长度  ，长度为信息内容的长度 
	regstr[4]=(u8)reglen;   
	
	reglen+=6;
	fcs=0;	
	for(i=0;i<reglen;i++)
		fcs^=regstr[i];
	regstr[reglen]=fcs;
	reglen++; 
	for(i=0;i<reglen;i++)
	     rt_kprintf("%c",regstr[i]);  
	
	return 1;
}

#endif

void SpeedWarnJudge(void)  //  速度报警判断 
{
  					 //--------  速度报警  -------	 
					 if(  JT808Conf_struct.Speed_warn_MAX >0 )   //> 0 
					 {
			            
						 //----- GPS  即时速度	0.1 km/h  ---------
							 if  ( Spd_Using>( JT808Conf_struct.Speed_warn_MAX*10) )							 	
							// if( DebugSpd > ( JT808Conf_struct.Speed_warn_MAX*10) ) 
							 {
											 speed_Exd.dur_seconds++;
											 if ( speed_Exd.dur_seconds >JT808Conf_struct.Spd_Exd_LimitSeconds)   
											 {
													 speed_Exd.dur_seconds = 0;  
													 if ( speed_Exd.speed_flag!= 1 )
													 {				   
														 speed_Exd.speed_flag = 1;			 
														 //PositionsSD_Enable();	  //  回报GPS 信息	 

														 StatusReg_SPD_WARN(); //  超速报警状态
														 rt_kprintf("\r\n  超速报警\r\n"); 
													 } 
													 //---------------------------------------------
													 Time2BCD(speed_Exd.ex_startTime); //记录超速报警起始时间
													 if(speed_Exd.current_maxSpd<Spd_Using) //找最大速度
														  speed_Exd.current_maxSpd=Spd_Using;
													 speed_Exd.excd_status=1;
													 speed_Exd.dur_seconds++;		
			 
												   //---------------------------------------------- 
											 }
											 
											 if(speed_Exd.excd_status==1) // 使能flag 后开始计时 
											  {
												 speed_Exd.dur_seconds++; 
												 if(speed_Exd.current_maxSpd<Spd_Using) //找最大速度
														  speed_Exd.current_maxSpd=Spd_Using;
											   } 
									 
							 }
							 else
							 { 
							              StatusReg_SPD_NORMAL(); //  清除速度报警状态寄存器  
										  
										  if(speed_Exd.excd_status!=2)
										  {
										    StatusReg_SPD_NORMAL(); //  清除速度报警状态寄存器
											speed_Exd.dur_seconds = 0;
											speed_Exd.speed_flag = 0; 	
										  } 											   
											//----------------------------------------------
											if(speed_Exd.excd_status==1)  
		                                    {	
											    Time2BCD(speed_Exd.ex_endTime); //记录超速报警结束时间 
												speed_Exd.excd_status=2;
							 	            }
											else
											if(speed_Exd.excd_status==0)
												Spd_ExpInit();
											//---------------------------------------------- 
							 }
					  }//------- 速度报警 over	 ---


}


u16  Protocol_808_Encode(u8 *Dest,u8 *Src, u16 srclen)
{
  u16  lencnt=0,destcnt=0;

  for(lencnt=0;lencnt<srclen;lencnt++)
  {
     if(Src[lencnt]==0x7e)  // 7e 转义
     {
       Dest[destcnt++]=0x7d;
	   Dest[destcnt++]=0x02;
     }
     else
	 if(Src[lencnt]==0x7d) //  7d  转义 	
	 {
       Dest[destcnt++]=0x7d;
	   Dest[destcnt++]=0x01; 
	 }
	 else
        Dest[destcnt++]=Src[lencnt]; // 原始信息
  }

  return destcnt; //返回转义后的长度

}
//-------------------------------------------------------------------------------
void Protocol_808_Decode(void)  // 解析指定buffer :  UDP_HEX_Rx  
{
	//-----------------------------------
	  u16 i=0;

    // 1.  clear  write_counter
	  UDP_DecodeHex_Len=0;//clear DecodeLen

	// 2   decode process   
	for(i=0;i<UDP_hexRx_len;i++)
	 {
		if((UDP_HEX_Rx[i]==0x7d)&&(UDP_HEX_Rx[i+1]==0x02))
		{
		   UDP_HEX_Rx[UDP_DecodeHex_Len]=0x7e;
		   i++;
		}
		else
		if((UDP_HEX_Rx[i]==0x7d)&&(UDP_HEX_Rx[i+1]==0x01))
		{
		   UDP_HEX_Rx[UDP_DecodeHex_Len]=0x7d;
		   i++;
		}
		else  
		{
		  UDP_HEX_Rx[UDP_DecodeHex_Len]=UDP_HEX_Rx[i];   
		}
	    UDP_DecodeHex_Len++;
	 }	
    //  3.  The  End
}
//---------  拐点补传测试程序  ---------------------------
//#if 0
/*
void Inflexion_Process(void)
{            //
  u16  once_delta=0;


	Inflexion_Current=GPS_direction;  //  update new
   //----------------------------------------------------------------------- 
    if(Inflexion_Current>Inflexion_Bak)   // 初步判断大小  
    	{  // 增大
            if((Inflexion_Current-Inflexion_Bak)>300)  // 判断是否倒置减小
			{   //  如果差值大于300 ，说明是小于
			     once_delta=Inflexion_Bak+360-Inflexion_Current;  //判断差值绝对值
			     InflexDelta_Accumulate+=once_delta;
				 if((once_delta>=15)&&(once_delta<=60)) // 角度最小变化率不得小于15度  拐点补传角度不大于180 要求连续3s  所以每秒不大于60
				 {
					    if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==2))  //判断之前是否一直是小于
					    	{
		                        Inflexion_chgcnter++;
								if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
									{     //要求至少持续3s	累计拐角补传角度不得大于180
                                           InflexLarge_or_Small=0;
										   Inflexion_chgcnter=0;
										   InflexDelta_Accumulate=0;
										   PositionSD_Enable=1; // 发送拐点标志位
										   rt_kprintf("\r\n 拐点上报 --1\r\n");
									}
								else
								  InflexLarge_or_Small=2; // 这次是小于
					    	}
						else
							{
							   InflexLarge_or_Small=2;  // 这是第一次小于 
							   Inflexion_chgcnter=1;
							   InflexDelta_Accumulate=once_delta;
							}
				 }
				 else
				 {    //  小于 15 就算等于
				   InflexLarge_or_Small=0;
				   Inflexion_chgcnter=0;
				   InflexDelta_Accumulate=0;
				 }
				 	
            } 
			else		// current真真正正的比Bak 大
			{  
			   once_delta=Inflexion_Current-Inflexion_Bak;  //判断差值绝对值
			   InflexDelta_Accumulate+=once_delta;
			   if((once_delta>=15)&&(once_delta<=60)) // 角度最小变化率不得小于15度  拐点补传角度不大于180 要求连续3s  所以每秒不大于60
			   {
	               if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==1)) //判断之前是否一直大于
				   {
				       Inflexion_chgcnter++;					   
					   if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
						   {	 //要求至少持续3s	累计拐角补传角度不得大于180
								  InflexLarge_or_Small=0;
								  Inflexion_chgcnter=0;
								  InflexDelta_Accumulate=0;
								  PositionSD_Enable(); // 发送拐点标志位
								  rt_kprintf("\r\n 拐点上报 --2\r\n");
						   }
					   else
					     InflexLarge_or_Small=1; // 这次是大于
	               }	
				   else
				   	{  
                       InflexLarge_or_Small=1;  // 这是第一次大于
					   Inflexion_chgcnter=1;
					   InflexDelta_Accumulate=once_delta; 
				   	}
			   }
			    else
				 {     // 小于15度就算等于
				   InflexLarge_or_Small=0;
				   Inflexion_chgcnter=0;
				   InflexDelta_Accumulate=0;
				 }

			}  
    	}
	else
	 if(Inflexion_Current<Inflexion_Bak)	
	 	{  // 减小
               if((Inflexion_Bak-Inflexion_Current)>300)  // 判断是否倒置增大
               { //  如果差值大于300 ，说明是大于
                  once_delta=Inflexion_Current+360-Inflexion_Bak;  //判断差值绝对值
			      InflexDelta_Accumulate+=once_delta;
				  if((once_delta>=15)&&(once_delta<=60)) // 角度最小变化率不得小于15度	拐点补传角度不大于180 要求连续3s  所以每秒不大于60
                  {   // 最小变化率 不小于 15
                     if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==1))  //判断之前是否一直是大于
					    	{
		                        Inflexion_chgcnter++;
								if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
									{     //要求至少持续3s	累计拐角补传角度不得大于180
                                           InflexLarge_or_Small=0;
										   Inflexion_chgcnter=0;
										   InflexDelta_Accumulate=0;
										   PositionSD_Enable(); // 发送拐点标志位
										   rt_kprintf("\r\n 拐点上报 --3\r\n");
									}
								else
								  InflexLarge_or_Small=1; // 这次是大于
					    	}
						else
							{
							   InflexLarge_or_Small=1;  // 这是第一次大于 
							   Inflexion_chgcnter=1;
							   InflexDelta_Accumulate=once_delta;
							}

                  }
				  else
				  {    //  小于 15 就算等于
				   InflexLarge_or_Small=0;
				   Inflexion_chgcnter=0;
				   InflexDelta_Accumulate=0;
				  }
			  }//---------------------------
			   else 	   // current 真真正正的比Bak 小
			   {
				  once_delta=Inflexion_Bak-Inflexion_Current;  //判断差值绝对值
				  InflexDelta_Accumulate+=once_delta;
				  if((once_delta>=15)&&(once_delta<=60)) // 角度最小变化率不得小于15度	拐点补传角度不大于180 要求连续3s  所以每秒不大于60
				  {
					  if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==2)) //判断之前是否一直小于
					  {
						  Inflexion_chgcnter++; 					  
						  if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
							  { 	//要求至少持续3s	累计拐角补传角度不得大于180
									 InflexLarge_or_Small=0;
									 Inflexion_chgcnter=0;
									 InflexDelta_Accumulate=0;
									 PositionSD_Enable(); // 发送拐点标志位
									 rt_kprintf("\r\n 拐点上报 --4\r\n"); 
							  }
						  else
						   InflexLarge_or_Small=2; // 这次是小于
					  }    
					  else
					   {  
						  InflexLarge_or_Small=2;  // 这是第一次小于
						  Inflexion_chgcnter=1;
						  InflexDelta_Accumulate=once_delta; 
					   }
				  }
				   else
					{	  // 小于15度就算等于
					  InflexLarge_or_Small=0;
					  Inflexion_chgcnter=0;
					  InflexDelta_Accumulate=0;
					}
			   
			   }
			  	
	

	 	}
	else
		{
		  InflexLarge_or_Small=0;
		  Inflexion_chgcnter=0;
		  InflexDelta_Accumulate=0;
		}  

    //--------------------------------------------------------
    Inflexion_Bak=Inflexion_Current; //  throw  old	 to  Bak

}*/
//#endif 

void  Sleep_Mode_ConfigEnter(void)
{
   if(SleepState==0)
		{
		   	  SleepCounter++;
			  if(SleepCounter>15)   // 防抖处理
			 {
			      SleepCounter=0; 
			   	   SleepState=1;		
				   if(JT808Conf_struct.RT_LOCK.Lock_state!=1)
				   Current_SD_Duration=JT808Conf_struct.DURATION.Sleep_Dur; // 5分钟 

				   //JT808Conf_struct.DURATION.Heart_SDCnter=25; 
				   //JT808Conf_struct.DURATION.Heart_Dur=320; 
				  /*   天地通要求休眠不重新计时  ，不  
				    memcpy(BakTime,CurrentTime,3); // update  //gps开始计时 
				    systemTick_trigGPS_counter=0; // 系统定时 清除
				  */  
				   if(DataLink_Status())
	                                PositionSD_Enable();  //  在线就发送    河北天地通要求数据完整 
				    rt_kprintf("\r\n 进入休眠状态! \r\n");          
			 }  
		 }
}

void  Sleep_Mode_ConfigExit(void)
{
    if(SleepState==1)
		   	{
		   	   SleepState=0;
			   rt_kprintf("\r\n 车台唤醒! \r\n");      
		   	}
   if(JT808Conf_struct.RT_LOCK.Lock_state!=1)
       Current_SD_Duration=JT808Conf_struct.DURATION.Default_Dur; 
   JT808Conf_struct.DURATION.Heart_Dur=300;  
   SleepState=0;
   SleepCounter=0;
}

#if 0
u16 WaveFile_EncodeHeader(u32 inFilesize ,u8* DestStr)
{
   u32 Filesize=0,i=0;     // Header Len  =44Bytes  

    
	//  1. RIFF
    memcpy(DestStr,"RIFF",4); 
	i+=4;
	//  2. Wave 文件 大小  小端模式 
	Filesize=0x24+(inFilesize<<3); // 乘以16 加 36 wave 文件大小 
	rt_kprintf("\r\n .wav 文件大小: %d Rawdata: %d \r\n ",Filesize,(inFilesize<<3)); 
	DestStr[i++]=Filesize; // LL
	DestStr[i++]=(Filesize>>8);//LH
	DestStr[i++]=(Filesize>>16);//HL
	DestStr[i++]=(Filesize>>24);//HH
	//  3. WAVE string	
    memcpy(DestStr+i,"WAVE",4);
	i+=4;
	//  4. fmt string	
    memcpy(DestStr+i,"fmt ",4);
	i+=4;
    //  5. PCM Code    
	DestStr[i++]=0x10; // LL
	DestStr[i++]=0x00;//LH
	DestStr[i++]=0x00;//HL
	DestStr[i++]=0x00;//HH   
	//  6. Audio Format  PCM=1	
	DestStr[i++]=0x01; // L
	DestStr[i++]=0x00;//H
	//  7. NumChannels  通道数
	DestStr[i++]=0x01; // L
	DestStr[i++]=0x00;//H
	//  8. SampleRate     8000<=>0x00001F40    16000<=>0x00003E80      
	DestStr[i++]=0x40;//0x40; // LL
	DestStr[i++]=0x1F;//0x1F;//LH
	DestStr[i++]=0x00;//HL
	DestStr[i++]=0x00;//HH    
    //  9.ByteRate       == SampleRate * NumChannels * BitsPerSample/8  ==8000x1x8/8 
	DestStr[i++]=0x40;//0x40; // LL
	DestStr[i++]=0x1F;//0x1F;//LH
	DestStr[i++]=0x00;//HL
	DestStr[i++]=0x00;//HH   

	// 10.BlockAlign   	== NumChannels * BitsPerSample/8
	DestStr[i++]=0x01;//0x02;//0x01; // L 
	DestStr[i++]=0x00;//H
	// 11.BitsPerSample
	DestStr[i++]=0x08;//0x10;//0x08; // L
	DestStr[i++]=0x00;//H
    // 12.data string	
    memcpy(DestStr+i,"data",4);
	i+=4;
	// 13 .datasize
	Filesize=(inFilesize<<3); // 乘以16 加 36 wave 文件大小 
	DestStr[i++]=Filesize; // LL
	DestStr[i++]=(Filesize>>8);//LH
	DestStr[i++]=(Filesize>>16);//HL
	DestStr[i++]=(Filesize>>24);//HH

    return i;
}
#endif

//-----------  starttime[6]
u8 CurrentTime_Judge( u8*startTime ,u8* endTime)
{
   u32 daycaculate_current=0,daycaculate_start=0,daycaculate_end=0;
   u32 secondcaculate_current=0,secondcaculate_start=0,secondcaculate_end=0;
  

   daycaculate_start=((startTime[0]>>4)*10+(startTime[0]&0x0f))*365+((startTime[1]>>4)*10+(startTime[1]&0x0f))*30+((startTime[2]>>4)*10+(startTime[2]&0x0f));
   secondcaculate_start=((startTime[3]>>4)*10+(startTime[3]&0x0f))*60+((startTime[4]>>4)*10+(startTime[4]&0x0f))*60+((startTime[5]>>4)*10+(startTime[5]&0x0f)); 

   
   daycaculate_end=((endTime[0]>>4)*10+(endTime[0]&0x0f))*365+((endTime[1]>>4)*10+(endTime[1]&0x0f))*30+((endTime[2]>>4)*10+(endTime[2]&0x0f));
   secondcaculate_end=((endTime[3]>>4)*10+(endTime[3]&0x0f))*60+((endTime[4]>>4)*10+(endTime[4]&0x0f))*60+((endTime[5]>>4)*10+(endTime[5]&0x0f)); 

   daycaculate_current=(time_now.year)*365+time_now.month*30+time_now.day;
   secondcaculate_current=time_now.hour*60+time_now.min*60+time_now.sec;

   if((daycaculate_current>daycaculate_start)&&(daycaculate_current<daycaculate_end))
   {
        return  true;
   }
   else
   if((secondcaculate_current>=secondcaculate_start)&&(secondcaculate_current<=secondcaculate_end))
   	{     
        return true;
   	}
   else   
   	    return false;

   

}

void CycleRail_Judge(u8* LatiStr,u8* LongiStr) 
{
   /*
       纬度没有差值    1纬度  111km
       40度纬度上 1经度为  85.3km   (北京地区)
   */
   u8 i=0;
   u32 Latitude=0,Longitude=0;
   u32 DeltaLatiDis=0,DeltaLongiDis=0,CacuDist=0;
   u8  InOutState=0;   //   0 表示 in   1  表示Out

   //  1. get value
      Latitude=(LatiStr[0]<<24)+(LatiStr[1]<<16)+(LatiStr[2]<<8)+LatiStr[3];
      Longitude=(LongiStr[0]<<24)+(LongiStr[1]<<16)+(LongiStr[2]<<8)+LongiStr[3]; 


 
  for(i=0;i<8;i++)
  {
         InOutState=0; 
		 memset((u8*)&Rail_Cycle,0,sizeof(Rail_Cycle));
		//Api_RecordNum_Read(Rail_cycle,i+1, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); 	 
		memcpy((u8*)&Rail_Cycle,(u8*)&Rail_Cycle_multi[i],sizeof(Rail_Cycle));
		// rt_kprintf("\r\n\r\n 圆形围栏 有效状态:%d  TYPE: %d    atrri=%d  lati: %d  longiti:%d  radicus:%d  maxspd: %d  keepdur:%d \r\n",Rail_Cycle.Effective_flag,Rail_Cycle.Area_ID,Rail_Cycle.Area_attribute,Rail_Cycle.Center_Latitude,Rail_Cycle.Center_Longitude,Rail_Cycle.Radius,Rail_Cycle.MaxSpd,Rail_Cycle.KeepDur);  
         

      if(Rail_Cycle.Effective_flag==1) 
      {
         
			 DeltaLatiDis=abs(Latitude-Rail_Cycle.Center_Latitude)/9; //  a/1000000*111000=a/9.009	除以一百万得到度数 再乘以 111000 米得到实际距离 
             
			 DeltaLongiDis=abs(Longitude-Rail_Cycle.Center_Longitude)*853/10000; // a/1000000*85300=a 853/10000 m 	  
			 
			 CacuDist=sqrt((DeltaLatiDis*DeltaLatiDis)+(DeltaLongiDis*DeltaLongiDis));  
             
			 rt_kprintf("\r\n  TemperLati  %d  TemperLongi	%d	  Centerlati %d  center longi %d\r\n",Latitude,Longitude,Rail_Cycle.Center_Latitude,Rail_Cycle.Center_Longitude); 
			 rt_kprintf("\r\n  he=%d heng=%d   shu=%d   juli=%d\r\n",abs(Longitude-Rail_Cycle.Center_Longitude),DeltaLongiDis,DeltaLatiDis,CacuDist);   
			 
		   if(DeltaLatiDis>Rail_Cycle.Radius ) 
		   {  // 如果纬度距离大于 半径肯定出
		     InOutState=1;
		   }	 
		    else
		   {
			   DeltaLongiDis=abs(Longitude-Rail_Cycle.Center_Longitude)*853/10000; // a/1000000*85300=a 853/10000 m		
			   if(DeltaLongiDis>Rail_Cycle.Radius )	 
			   	{  // 如果经度距离大于半径肯定出
                   InOutState=1;
			   	}
			   else  //  计算两点见距离
			    CacuDist=sqrt((DeltaLatiDis*DeltaLatiDis)+(DeltaLongiDis*DeltaLongiDis));
		    } 


		     // 1. 判断属性
		     if(Rail_Cycle.Area_attribute &0x0001) //Bit 0 根据时间
		     {
                if(CurrentTime_Judge(Rail_Cycle.StartTimeBCD,Rail_Cycle.EndTimeBCD)==false)
				{
				  rt_kprintf("\r\n 时段没在区间内 \r\n");
				  return;
                }  
			   //continue;
		     }
			 if(Rail_Cycle.Area_attribute &0x0002) //Bit 1 限速
			 {
                if(Spd_Using>Rail_Cycle.MaxSpd) 
                	{
                          StatusReg_SPD_WARN(); //  超速报警状态
						  rt_kprintf("\r\n  设定围栏超速报警\r\n"); 
                	}
				else
					 StatusReg_SPD_NORMAL();
	           //continue;
			 }
	         if(Rail_Cycle.Area_attribute &0x0004) //Bit 2 进区域报警给驾驶员
	         {


	           //continue;
	         }
			 if(Rail_Cycle.Area_attribute &0x0008) //Bit 3 进区域报警给平台 
			 {
               if((InOutState==0)&&(CacuDist<Rail_Cycle.Radius )&&(Rail_Cycle.MaxSpd>(Speed_gps/10)))
               {
                  Warn_Status[1]|=0x10;// 进出区域报警
                  InOut_Object.TYPE=1;//圆形区域
                  InOut_Object.ID=i; //  ID
                  InOut_Object.InOutState=0;//  进报警	
                  rt_kprintf("\r\n -----圆形电子围栏--入报警");   
			      break;
               }			 
	           //continue;
			 }
			  if(Rail_Cycle.Area_attribute &0x0010) //Bit 4 出区域报警给司机
			 {
	           ;   
	           //continue;
			 }
			 if((Rail_Cycle.Area_attribute &0x0020)&&(Rail_Cycle.MaxSpd>(Speed_gps/10))) //Bit 5 出区域报警给平台 
			 {
			   if((InOutState==1)||(CacuDist>Rail_Cycle.Radius )) 
			   	{
			   	  Warn_Status[1]|=0x10;// 进出区域报警
                  InOut_Object.TYPE=1;//圆形区域
                  InOut_Object.ID=i; //  ID
                  InOut_Object.InOutState=1;//  出报警 
                  rt_kprintf("\r\n -----圆形电子围栏--出报警");        
			      break;
			   	}
						 
	           //continue;
			 } 
      	}

  }



}

void RectangleRail_Judge(u8* LatiStr,u8* LongiStr)
{  
	u8 i=0;
	u32 Latitude=0,Longitude=0;
//	u32 DeltaLatiDis=0,DeltaLongiDis=0,CacuDist=0;
	u8	InOutState=1;	//	 0 表示 in	 1	表示Out
	
  
  //  1. get value
	 Latitude=(LatiStr[0]<<24)+(LatiStr[1]<<16)+(LatiStr[2]<<8)+LatiStr[3];
	 Longitude=(LongiStr[0]<<24)+(LongiStr[1]<<16)+(LongiStr[2]<<8)+LongiStr[3]; 

	 
	// rt_kprintf("\r\n  1---TemperLati  %d  TemperLongi	%d	 res %d\r\n",Latitude,Longitude,InOutState);  

	for(i=0;i<8;i++)
	{
	    InOutState=1;
		// Api_RecordNum_Read(Rail_rect,i+1, (u8*)&Rail_Rectangle, sizeof(Rail_Rectangle)); 
		 memcpy((u8*)&Rail_Rectangle,(u8*)&Rail_Rectangle_multi[i],sizeof(Rail_Rectangle));		
        
		if(Rail_Rectangle.Effective_flag==1) 
		{
		 
		//  rt_kprintf("\r\n\r\n 判断矩形形围栏 有效:%d ID: %d  atrri=%X  leftlati: %d  leftlongiti:%d    rightLati:%d   rightLongitu: %d	\r\n",Rail_Rectangle.Effective_flag,i+1,Rail_Rectangle.Area_attribute,Rail_Rectangle.LeftUp_Latitude,Rail_Rectangle.LeftUp_Longitude,Rail_Rectangle.RightDown_Latitude,Rail_Rectangle.RightDown_Longitude);	
            if((Latitude>Rail_Rectangle.RightDown_Latitude)&&(Latitude<Rail_Rectangle.LeftUp_Latitude)&&(Longitude>Rail_Rectangle.LeftUp_Longitude)&&(Longitude<Rail_Rectangle.RightDown_Longitude))
		       InOutState=0;
            
			//rt_kprintf("\r\n  TemperLati  %d  TemperLongi  %d   res %d\r\n",Latitude,Longitude,InOutState); 
		     
			// 1. 判断属性
			if(Rail_Rectangle.Area_attribute &0x0001) //Bit 0 根据时间
			{
			
			  //continue;
			}
			if(Rail_Rectangle.Area_attribute &0x0002) //Bit 1 限速
			{
			
			  //continue;
			}
			if(Rail_Rectangle.Area_attribute &0x0004) //Bit 2 进区域报警给驾驶员
			{
			
			
			  //continue;
			}
			if(Rail_Rectangle.Area_attribute &0x0008) //Bit 3 进区域报警给平台 
			{
			  if(InOutState==0)
			  {
				 Warn_Status[1]|=0x10;// 进出区域报警
				 InOut_Object.TYPE=2;//矩形区域
				 InOut_Object.ID=i; //	ID
				 InOut_Object.InOutState=0;//  进报警		
				 rt_kprintf("\r\n -----矩形电子围栏--入报警"); 
				 break;
			  } 			
			  //continue;
			}
			 if(Rail_Rectangle.Area_attribute &0x0010) //Bit 4 出区域报警给司机
			{
			
			
			 // continue;
			}
			if(Rail_Rectangle.Area_attribute &0x0020) //Bit 5 出区域报警给平台 
			{
			  if(InOutState==1)
			   {
				 Warn_Status[1]|=0x10;// 进出区域报警
				 InOut_Object.TYPE=2;//矩形区域
				 InOut_Object.ID=i; //	ID
				 InOut_Object.InOutState=1;//  出报警		
				 rt_kprintf("\r\n -----矩形电子围栏--出报警"); 
				 break;
			   }
						
			 // continue;  
			} 

 
		}
		
	} 




}

void RouteRail_Judge(u8* LatiStr,u8* LongiStr)  
{
   /*
       纬度没有差值    1纬度  111km
       40度纬度上 1经度为  85.3km   (北京地区)
   */
     u8 i=0;
      u8 route_cout=0, seg_count=0,seg_num=0; 
      u32 Latitude=0,Longitude=0;
    // u32 DeltaLatiDis=0,DeltaLongiDis=0,CacuDist=0;
    // u8  InOutState=0;   //   0 表示 in   1  表示Out
     u32  Route_Status=0;   // 每个bit 表示 一个路线 偏航状态默认为0
     u32  Segment_Status=0;  //  当前线路中，对应端的偏航情况， 默认为0
     u32  Distance=0;
//     u8    InAreaJudge=0; //  判断是否在判断区域 bit 0 经度范围 bit  1 纬度范围
     u32  Distance_Array[6]; //存储当条线路的最小距离，默认是个大数值

       //  1. get value
	 Latitude=(LatiStr[0]<<24)+(LatiStr[1]<<16)+(LatiStr[2]<<8)+LatiStr[3];
	 Longitude=(LongiStr[0]<<24)+(LongiStr[1]<<16)+(LongiStr[2]<<8)+LongiStr[3]; 

	 // rt_kprintf("\r\n 当前---->  Latitude:   %d     Longitude: %d\r\n",Latitude,Longitude); 

      //  2.  Judge 
      for(route_cout=0;route_cout<Route_Mum;route_cout++)      // 读取路线
     { 
                 
	        // 2.1  --------   读取路线-----------	        
	        memset((u8*)&ROUTE_Obj,0,sizeof(ROUTE_Obj));  //  clear all  first        
	        DF_ReadFlash(DF_Route_Page+route_cout, 0,(u8*)&ROUTE_Obj, sizeof(ROUTE_Obj));	  
		 DF_delay_us(20);  
		//rt_kprintf("\r\n -----> ROUTE_Obj.RouteID:   %d \r\n",ROUTE_Obj.Route_ID);   
              // 2.2  -----  判断是否有效  -------
	       if((ROUTE_Obj.Effective_flag==1) &&(ROUTE_Obj.Points_Num>1)) //  判断是否有效且有拐点，若无效不处理
	      {     
	              // 2.2.0    当前段距离付给一个大的数值
	              for(i=0;i<6;i++)
				  	  Distance_Array[i]=ROUTE_DIS_Default;  
		      // 2.2.1      计算段数
                     seg_num=ROUTE_Obj.Points_Num-1; // 线路段数目
                    //  2.2.2    判断路线中每一段的状态
                     Segment_Status=0;  // 清除段判断状态，每个线路重新开始一次
			for(seg_count=0;seg_count<seg_num;seg_count++)
			{      
			          if((ROUTE_Obj.RoutePoints[seg_count+1].POINT_Latitude==0)&&(ROUTE_Obj.RoutePoints[seg_count+1].POINT_Longitude==0))
				      	{
				      	     rt_kprintf("\r\n  该点为0 ，jump\r\n"); 
					     continue;
				      	}
			      //----- 开始做距离计算, 在没在区域在函数里边做了判断
                           Distance_Array[seg_count]=Distance_Point2Line(Latitude, Longitude,ROUTE_Obj.RoutePoints[seg_count].POINT_Latitude,ROUTE_Obj.RoutePoints[seg_count].POINT_Longitude,ROUTE_Obj.RoutePoints[seg_count+1].POINT_Latitude,ROUTE_Obj.RoutePoints[seg_count+1].POINT_Longitude);   

			}
		//=========================================================	
              //  2.4 ------  打印显示距离，找出最小数值----
              Distance=Distance_Array[0];  // 最小距离  
               for(i=0;i<6;i++)
		   {
		       if(Distance>=Distance_Array[i])  
                                    Distance=Distance_Array[i];			   
		      // rt_kprintf("\r\n  Distance[%d]=%d",i,Distance_Array[i]);
                 }	 
		  rt_kprintf("\r\n MinDistance =%d  Width=%d \r\n",Distance,(ROUTE_Obj.RoutePoints[seg_num].Width>>1));	  // 

		if(Distance<ROUTE_DIS_Default)
	      {
		       //  ---- 和路段宽度做对比	   
	                 if(Distance>(ROUTE_Obj.RoutePoints[seg_num].Width>>1))
	                {
	                       rt_kprintf("\r\n 路线偏离\r\n");   
	                        Segment_Status|=(1<<seg_num);   //  把相应的bit  置位    
	                }  
		}

                     //                    
	      }  
		// 2.4  根据 2.2 结果判断当期路线状态
		if(Segment_Status)
			Route_Status|=(1<<route_cout);   //  把相应的bit  置位 
     }
     // 3.  Result
     		if(Route_Status) 
		 {
		    if( (Warn_Status[1]&0x80)==0)   //  如果以前没触发，那么及时上报
			{	  
                          PositionSD_Enable();  
				 Current_UDP_sd=1;   
		    	}
			
		    Warn_Status[1]|=0x80;// 路线偏航报警 
		    rt_kprintf("\r\n    路径偏航触发 !\r\n");           
		   
		}
		else
		{
		     if (Warn_Status[1]&0x80)   //  如果以前没触发，那么及时上报
			{	  
                             PositionSD_Enable();  
				 Current_UDP_sd=1;   
		    	}

		    Warn_Status[1]&=~0x80;// 路线偏航报警      
		}
	  

}

//--------  D点到直线距离计算-------
/*
     P1(x1,y1)   P2(x2,y2)  ,把点P(x1,y2)作为坐标原点，即x1=0，y2=0；

     那么两点P1，P2 确定的直线方程(两点式)为:
             (x-x1)/(x2-x1) =(y-y1)/(y2-y1)                          (1)
             
    注:  标准式直线方程为 AX+BY+C=0;
             那么平面上任意一点P(x0,y0) 到直线的距离表示为
             d=abs(Ax0+By0+C)/sqrt(A^2+B^2)

    其中把方程式(1) 转换成标准式为:
            (y2-y1)x+(x1-x2)y+x1(y1-y2)+y1(x2-x1)=0; 

   由于点(x1,y2)为原点  即x1=0，y2=0；  P1(0,y1) , P2(x2,0)
    所以   A=-y1 ,  B=-x2, C=y1x2
    那么 直线的方程:
                  -y1x-x2y+y1x2=0;  (2)  
 
  =>     d=abs(-y1x0-x2y0+y1x2)/sqrt(y1^2+x2^2)       (3)

         其中 (3)  为最终应用的公式 

        注:  先根据经纬度折合计算出 x0，y0，x1,y1,x2,y2  的数值单位为: 米     
=>  区域判断:
           根据(2) 可以求出  过 P1(0,y1) , P2(x2,0) 点与已知直线垂直的两条直线方程
              P1(0,y1) :      x2x-y1y+y1^2=0  (4)
              P2(x2,0) :      x2x-y1y-x2^2=0  (5)

          如果 y1 >=0      直线(4)    在直线(5)  的上边
          那么 点在线段区域内的判断方法是
                       (4) <=  0    且  (5)  >=0 
       另                
           如果 y1 <=0      直线(5)    在直线(4)  的上边
          那么 点在线段区域内的判断方法是
                       (4) >=  0    且  (5)  <=0 
   //------------------------------------------------------------------------------------------     
     
       纬度没有差值    1纬度  111km
       40度纬度上 1经度为  85.3km   (北京地区)   

       X 轴为 经度(longitude) 差值
       Y 轴为纬度 (latitude)  差值
                
     
   //------------------------------------------------------------------------------------------
*/

u32   Distance_Point2Line(u32 Cur_Lat, u32  Cur_Longi,u32 P1_Lat,u32 P1_Longi,u32 P2_Lat,u32 P2_Longi)
{   //   输入当前点 ，返回点到既有直线的距离
      long  x0=0,y0=0,Line4_Resualt=0,Line5_Resualt=0;  // 单位: 米
      long  y1=0; 
      long  x2=0;	
      long   distance=0;	 
     // long  Rabs=0;
//      long  Rsqrt=0;  
   //  long  DeltaA1=0,DeltaA2=0,DeltaO1=0,DeltaO2=0; //  DeltaA : Latitude     DeltaO:  Longitude 	   
     // u32   Line4_Resualt2=0,Line5_Resualt2=0; 
      double   fx0=0,fy0=0,fy1=0,fx2=0;	  
      double   FLine4_Resualt2=0,FLine5_Resualt2=0,fRabs=0,fRsqrt=0; 	  

	  // 0.   先粗略的判断 
	    //   DeltaA1=abs(Cur_Lat-P1_Lat); 
	    //   DeltaA2=abs(Cur_Lat-P2_Lat); 
	//	DeltaO1=abs(Cur_Lat-P1_Longi); 
	     //  DeltaO2=abs(Cur_Lat-P2_Longi); 
	   /* if((DeltaA1>1000000) &&(DeltaA2>1000000))	    
            {  
                rt_kprintf("\r\n  Latitude 差太大\r\n");
                return   ROUTE_DIS_Default; 
	    }	
	     if((DeltaO1>1000000) &&(DeltaO2>1000000))	    
            {  
                rt_kprintf("\r\n  Longitude 差太大\r\n"); 
                return   ROUTE_DIS_Default; 
	    }	  	
       */
	 // 1.  获取  P1(0,y1)   P2(x2,0) ,和P(x0,y0)    P(x1,y2)为原点  即x1=0，y2=0；  P1(0,y1) , P2(x2,0)
	  x2=abs(P2_Longi-P1_Longi); // a/1000000*85300=a 853/10000 m =a x 0.0853
         if(P2_Longi<P1_Longi)
		 	x2=0-x2;
	  fx2=(double)((double)x2/1000);
	 //rt_kprintf("\r\n P2_L=%d,P1_L=%d   delta=%d \r\n",P2_Longi,P1_Longi,(P2_Longi-P1_Longi));
	 // if(P2_Longi
         y1=abs(P2_Lat-P1_Lat); //  a/1000000*111000=a/9.009	除以一百万得到度数 再乘以 111000 米得到实际距离 
         if(P2_Lat<P1_Lat) 
		 	y1=0-y1;
	   fy1=(double)((double)y1/1000);	  
	  //rt_kprintf("\r\n P2_LA=%d,P1_LA=%d   delta=%d \r\n",P2_Lat,P1_Lat,(P2_Lat-P1_Lat));

      //   rt_kprintf("\r\n 已知两点坐标: P1(0,%d)   P2(%d,0) \r\n", y1,x2); 
       //    当前点
	  x0=abs(Cur_Longi-P1_Longi);     
	   if(Cur_Longi<P1_Longi)
	   	x0=0-x0;
	   fx0=(double)((double)x0/1000);  
         //rt_kprintf("\r\n Cur_L=%d,P1_L=%d   delta=%d \r\n",Cur_Longi,P1_Longi,(Cur_Longi-P1_Longi));
 
         y0=abs(Cur_Lat-P2_Lat); //  a/1000000*111000=a/9.009	    
         if(Cur_Lat<P2_Lat)
		   y0=0-y0;         
	 fy0=(double)((double)y0/1000);	 
          // rt_kprintf("\r\n Cur_La=%d,P2_La=%d   delta=%d \r\n",Cur_Lat,P2_Lat,(Cur_Lat-P2_Lat)); 
        //   rt_kprintf("\r\n当前点坐标: P0(%d,%d)    \r\n", x0,y0);  
	  // 2. 判断y1  的大小， 求出过 P1(0,y1)   P2(x2,0) ,和已知直线的方程，并判断
	  //     当前点是否在路段垂直范围内
	  
             //  2.1   将当前点带入， 过 P1(0,y1)   的 直线方程(4)  求出结果
                                  Line4_Resualt=(x2*x0)-(y1*y0)+(y1*y1);
	                            FLine4_Resualt2=fx2*fx0-fy1*fy0+fy1*fy1;
	   //     rt_kprintf("\r\n Line4=x2*x0-y1*y0+y1*y1=(%d)*(%d)-(%d)*(%d)+(%d)*(%d)=%ld     x2*x0=%d    y1*y0=%d   y1*y1=%d  \r\n",x2,x0,y1,y0,y1,y1,Line4_Resualt,x2*x0,y1*y0,y1*y1); 
          //     rt_kprintf("\r\n FLine4=fx2*fx0-fy1*fy0+fy1*fy1=(%f)*(%f)-(%f)*(%f)+(%f)*(%f)=%f      fx2*fx0=%f    fy1*fy0=%f   fy1*fy1=%f  \r\n",fx2,fx0,fy1,fy0,fy1,fy1,FLine4_Resualt2,fx2*fx0,fy1*fy0,fy1*fy1); 
  
	      //   2.2   将当前点带入， 过P2(x2,0) 的 直线方程(5)  求出结果 
                                  Line5_Resualt=(x2*x0)-y1*y0-x2*x2;
		                    FLine5_Resualt2=fx2*fx0-fy1*fy0-fx2*fx2;    
		//rt_kprintf("\r\n Line5=x2*x0-y1*y0-x2*x2=(%d)*(%d)-(%d)*(%d)-(%d)*(%d)=%ld     Se : %ld   \r\n",x2,x0,y1,y0,x2,x2,Line5_Resualt,Line5_Resualt2); 
          //    rt_kprintf("\r\n FLine5=fx2*fx0-fy1*fy0-fx2*fx2=(%f)*(%f)-(%f)*(%f)-(%f)*(%f)=%f      fx2*fx0=%f    fy1*fy0=%f   fx2*fx2=%f  \r\n",fx2,fx0,fy1,fy0,fx2,fx2,FLine5_Resualt2,fx2*fx0,fy1*fy0,fx2*fx2);   
 	      // rt_kprintf("\r\n  Line4_Resualt=%d     Line5_Resualt=%d  \r\n",Line4_Resualt,Line5_Resualt);    

	     if(fy1>=0)      //  直线(4) 在上发
	  	{
               
			//   2.3   判断区域    (4) <=  0    且  (5)  >=0     // 判断条件取反
			     if((FLine4_Resualt2>0) ||(FLine5_Resualt2<0))
				 	 return   ROUTE_DIS_Default;      //  不满足条件返回最大数值
	  	}
	     else
	     	{      //  直线(5)

                   	//   2.4   判断区域     (4) >=  0    且  (5)  <=0     // 判断条件取反 
			     if((FLine4_Resualt2<0) ||(FLine5_Resualt2>0)) 
				 	 return   ROUTE_DIS_Default;      //  不满足条件返回最大数值 

	     	}	

              rt_kprintf("\r\n In judge area \r\n");  
		//rt_kprintf("\r\n   Current== Latitude:   %d     Longitude: %d     Point1== Latitude:   %d     Longitude: %d     Point2== Latitude:   %d     Longitude: %d\r\n",Cur_Lat,Cur_Longi,P1_Lat,P1_Longi,P2_Lat,P2_Longi);   

        //  3. 将差值差算成实际距离
             #if 0
		    x2=x2*0.0853; // a/1000000*85300=a 853/10000 m =a x 0.0853
                 y1=y1/9; //  a/1000000*111000=a/9.009	除以一百万得到度数 再乘以 111000 米得到实际距离 
                 x0=x0*0.0853;     
		   y0=y0/9; //  a/1000000*111000=a/9.009	    
	      #else
		   fx2=fx2*0.0853; // a/1000000*85300=a 853/10000 m =a x 0.0853
                 fy1=fy1/9; //  a/1000000*111000=a/9.009	除以一百万得到度数 再乘以 111000 米得到实际距离 
                 fx0=fx0*0.0853;     
		   fy0=fy0/9; //  a/1000000*111000=a/9.009	      
             #endif

	 //  4. 计算距离 
	       //Rabs=0-y1*x0-x2*y0+y1*x2;  
              // rt_kprintf("\r\n Test -y1*x0=%d -y0*x2=%d  y1*x2=%d   Rabs=%d  \r\n",0-y1*x0,0-y0*x2,0-y1*x2,Rabs);          
	  #if 0
	       Rabs=abs(-y1*x0-x2*y0+y1*x2);   
	       Rsqrt=sqrt(y1*y1+x2*x2); 
	        // distance=abs(-y1*x0-x2*y0-y1*x2)/sqrt(y1*y1+x2*x2); 
	       distance=Rabs/Rsqrt;
	  // rt_kprintf("\r\n Rabs=%d    Rsqrt=%d   d=%d",Rabs,Rsqrt,distance);        
        #else
             	fRabs=abs(-fy1*fx0-fx2*fy0+fy1*fx2);   
	       fRsqrt=sqrt(fy1*fy1+fx2*fx2); 
	        // distance=abs(-y1*x0-x2*y0-y1*x2)/sqrt(y1*y1+x2*x2); 
	       distance=(long) ((fRabs/fRsqrt)*1000);
	       // rt_kprintf("\r\n Rabs=%d    Rsqrt=%d   d=%d",Rabs,Rsqrt,distance);      
	  #endif
	  
	     
     return   distance;
}

/*
void  ISP_DF_Status(void)
{
  
   BD_ISP.ISP_status_Read=0;  // 上电读取 DF 状态
   memset(ISP_buffer,0,sizeof(ISP_buffer));
   SST25V_BufferRead(ISP_buffer,ISP_StartArea,256);      
   if(ISP_buffer[32]==ISP_B

}
*/

 unsigned short int File_CRC_Get(void)  
{
 
 u8   buffer_temp[514]; 
 unsigned short int i=0;
 u16  packet_num=0,leftvalue=0;  // 512    per packet
 u32  File_size=0; 

 DF_LOCK=1;
   memset(buffer_temp,0,sizeof(buffer_temp));   

   //  获取  文件头信息
   SST25V_BufferRead( buffer_temp,ISP_StartArea,  256 );     
   File_size=(buffer_temp[114]<<24)+(buffer_temp[115]<<16)+(buffer_temp[116]<<8)+buffer_temp[117];
   
   leftvalue=File_size%512;
   rt_kprintf("\r\n 文件大小: %d Bytes  leftvalue=%d \r\n",File_size,leftvalue);  			
   FileTCB_CRC16=(buffer_temp[134]<<8)+buffer_temp[135];
   rt_kprintf("\r\n Read CRC16: 0x%X Bytes\r\n",FileTCB_CRC16);  

  // OutPrint_HEX("1stpacket",buffer_temp,256);  
   
    if(leftvalue)   // 除以 512
          packet_num=(File_size>>9)+1; 
	else 
          packet_num=(File_size>>9);

	
   for(i=0;i<packet_num;i++)
	{
		if(i==0)   //第一包
			{
			Last_crc=0;   // clear first  
			crc_fcs=0; 
			SST25V_BufferRead(buffer_temp,ISP_StartArea+256,512);   // 0x001000+256=0x001100   ISP_StartArea+256
			delay_ms(50);
			WatchDog_Feed();		 	
			Last_crc=CRC16_1(buffer_temp,512,0xffff);
			//rt_kprintf("\r\n                  i=%d,Last_crc=0x%X",i+1,Last_crc);  
			
			//rt_kprintf("\r\n //----------   %d     packet    len=%d  ",i+1,512); 
			//OutPrint_HEX("1stpacket",buffer_temp,512);  
			}
		else 
		if(i==(packet_num-1))  //最后一包  
			{
             SST25V_BufferRead(buffer_temp,ISP_StartArea+256+i*512,leftvalue);    
			 delay_ms(50);
			 WatchDog_Feed();			 
			// rt_kprintf("\r\n //----------   %d     packet    len=%d  ",i+1,leftvalue); 
			// OutPrint_HEX("endstpacket",buffer_temp,leftvalue);  
			 crc_fcs=CRC16_1(buffer_temp,leftvalue,Last_crc);
			 rt_kprintf("\r\n                  i=%d,Last_crc=0x%X ReadCrc=0x%X ",i+1,crc_fcs,FileTCB_CRC16);   
			} 
		else  
			{             // 中间的包
			  SST25V_BufferRead(buffer_temp,ISP_StartArea+256+i*512,512);    
			  delay_ms(50); 
			  WatchDog_Feed();			  
			 // rt_kprintf("\r\n //----------   %d	 packet    len=%d  ",i+1,512); 			  
			 // OutPrint_HEX("midstpacket",buffer_temp,512);     
			  Last_crc=CRC16_1(buffer_temp,512,Last_crc);  			  
			 // rt_kprintf("\r\n                 i=%d,Last_crc=0x%X",i+1,Last_crc);    
			}
	   }

      DF_LOCK=0; 
      rt_kprintf("\r\n  校验结果 0x%X \r\n",crc_fcs);  

	  if(FileTCB_CRC16==crc_fcs)
	  	{
	  	    SST25V_BufferRead( buffer_temp,0x001000,100 );
		    buffer_temp[32] = ISP_BYTE_CrcPass;    //-----   文件校验通过 
		    SST25V_BufferWrite( buffer_temp,0x001000,100);  
	  	    rt_kprintf("\r\n  校验正确! \r\n",crc_fcs);
		   return true;
	  	}
	  else
	  	{
           rt_kprintf("\r\n  校验失败! \r\n",crc_fcs);
		   return false;
	  	} 
}
//FINSH_FUNCTION_EXPORT(File_CRC_Get, File_CRC_Get);      

//-----------------------------------------------------------------------------------
void Save_AvrgSpdPerMin(void)  
{ /*
     Note: 存储单位小时每分钟平均速度 
  */
  u8   content[70];
  u8   saveLen=0,FCS=0,i;   
    
	 saveLen=0; 
	 memset(content,0,70);
	 memcpy(content+saveLen,Avrgspd_Mint.datetime_Bak,5);    //用BAKtime
	 saveLen+=5;
	 memcpy(content+saveLen,Avrgspd_Mint.avgrspd,60);    //用BAKtime
	 saveLen+=60;  
  //---------------------------------------------------------------------------------------
  FCS = 0;
  for ( i = 0; i < 69; i++ ) 
  {
	 FCS ^=content[i];    
  } 			 //求上边数据的异或和   
  content[69] = FCS;	 // 第8字节	       
  //----------------------------------------------------------------------------------------  
  Api_DFdirectory_Write(spdpermin, content, 70); 
  //-----------------------------------------------------------------
  memset(Avrgspd_Mint.avgrspd,0,60); // 存储后清除记录     
   if(DispContent)
        rt_kprintf("\r\n   存储单位小时每分钟平均速度   Save Time %X-%X-%X %x:%x     \r\n",\
  	Avrgspd_Mint.datetime_Bak[0],Avrgspd_Mint.datetime_Bak[1],\
  	Avrgspd_Mint.datetime_Bak[2],Avrgspd_Mint.datetime_Bak[3],Avrgspd_Mint.datetime_Bak[4]);   
  
}

  void Spd_Exp_Wr(void)
  {	
     u8  content[40];
     u8  wr_add=0,i=0,FCS=0; 	 

	 memset(content,0,sizeof(content));
	 
	memcpy(content+wr_add,JT808Conf_struct.Driver_Info.DriveName,18);
	wr_add+=18;
	memcpy(content+wr_add,speed_Exd.ex_startTime,6);
	wr_add+=6;
	memcpy(content+wr_add,speed_Exd.ex_endTime,6);  
	wr_add+=6; 
	content[wr_add++]=speed_Exd.current_maxSpd/10; 

       FCS = 0;
	   for ( i = 0; i < 32; i++ )  
	   {
			 FCS ^=content[i];
	   }			  //求上边数据的异或和
	   content[wr_add++] = FCS;	  // 第31字节 
	   
       Api_DFdirectory_Write(spd_warn, (u8 *)content, 32); 
	  //----------- debug -----------------------
	  rt_kprintf("\r\n 超速报警  %X-%X-%X %X:%X:%X,MaxSpd=%d\r\n",speed_Exd.ex_endTime[0],speed_Exd.ex_endTime[1],speed_Exd.ex_endTime[2],speed_Exd.ex_endTime[3],speed_Exd.ex_endTime[4],speed_Exd.ex_endTime[5],speed_Exd.current_maxSpd); 
     //--------- clear status ----------------------------
     Spd_ExpInit(); 

  }

void Save_TiredDrive_Record(void) 
{
  u8   content[40];
  u8   wr_add=0,FCS=0,i; 

        wr_add=0; 
	 memset(content,0,70);

	  Time2BCD(TiredConf_struct.Tired_drive.end_time); 
	 
      memcpy(content+wr_add,JT808Conf_struct.Driver_Info.DriveName,18);
	  wr_add+=18;
	  memcpy(content+wr_add,TiredConf_struct.Tired_drive.start_time,6);
	  wr_add+=6;
	  memcpy(content+wr_add,TiredConf_struct.Tired_drive.end_time,6);
	  wr_add+=6; 

	      FCS = 0;
	   for ( i = 0; i < 32; i++ )  
	   {
			 FCS ^=content[i];
	   }			  //求上边数据的异或和
	   content[wr_add++] = FCS;	  // 第31字节 
	   
       Api_DFdirectory_Write(tired_warn, (u8 *)content, 32); 
       //-----  clear status ---------------  
	  TIRED_Drive_Init();      
	  Status_TiredwhRst=0;
	 // DF_WriteFlashSector(DF_TiredStartTime_Page,0,(u8*)&Status_TiredwhRst,1); //清除O后要写入		
	   TIRED_Drive_Init(); //	休息了 
	   Warn_Status[3]&=~0x04;  //BIT(2)	疲劳驾驶  

}

//E:\work_xj\F4_BD\北斗行车记录仪过检(新协议)\2-21 RT-Thread_NewBoard-LCD2-Jiance\bsp\stm32f407vgt6_RecDrv\app_712\lcd\Menu_0_3_Sim.c
void CAN_struct_init(void)
{
      //-------  protocol variables
     CAN_trans.can1_sample_dur=0;  // can1 采集间隔  ms
     CAN_trans.can1_trans_dur=0;  // can1  上报间隔 s
     CAN_trans.can1_enable_get=0; // 500ms 

     CAN_trans.can2_sample_dur=0;  // can2 采集间隔  ms
     CAN_trans.can2_trans_dur=0;  // can2  上报间隔 s      
     
   //   u8      canid_1[8];// 原始设置
     CAN_trans.canid_1_Filter_ID=0;  // 接收判断用
      //u8      canid_1_Rxbuf[400]; // 接收buffer
     CAN_trans.canid_1_RxWr=0; // 写buffer下标
     CAN_trans.canid_1_SdWr=0;
      CAN_trans.canid_1_ext_state=0; // 扩展帧状态
     CAN_trans.canid_1_sample_dur=10;  // 改ID 的采集间隔
     CAN_trans.canid_ID_enableGet=0;   

      //------- system variables  
      CAN_trans.canid_timer=0;  //定时器
      CAN_trans.canid_0705_sdFlag=0;// 发送标志位	   

}

void  CAN_send_timer(void)
{
   u16 i=0,datanum=0;;
       if(CAN_trans.can1_trans_dur>0) 
      {  
            CAN_trans.canid_timer++;
	    // if( CAN_trans.canid_timer>=CAN_trans.can1_trans_dur)		
	      if( CAN_trans.canid_timer>=4)      	        
	      {	  
                   CAN_trans.canid_timer=0; 
		     //------  判断有没有数据项
		     if(CAN_trans.canid_1_RxWr)
		     	{
		     	      datanum=(CAN_trans.canid_1_RxWr>>3); 
		     	      memcpy(CAN_trans.canid_1_Sdbuf,CAN_trans.canid_1_Rxbuf,CAN_trans.canid_1_RxWr);
			      CAN_trans.canid_1_SdWr=CAN_trans.canid_1_RxWr;	
			      for(i=0;i<datanum;i++)
				       CAN_trans.canid_1_ID_SdBUF[i]=CAN_trans.canid_1_ID_RxBUF[i];  
				  
			      CAN_trans.canid_1_RxWr=0; // clear
		             CAN_trans.canid_0705_sdFlag=1;
		     	}
			else
			     rt_kprintf("\r\n rx=%d  dur=%ds   CAN send trigger ,no data \r\n",CAN_trans.canid_1_RxWr,CAN_trans.can1_trans_dur);   	

	     	}	 

       }
	else
       {
                  CAN_trans.canid_0705_sdFlag=0;
		    CAN_trans.canid_timer=0;
	}
}

/*
void  vdrflag_enable(u8 invalue)
{
      switch(invalue)
      	{
              case 8:  VdrData.H_08_saveFlag=1;
			           break;
			  case 9: VdrData.H_09_saveFlag=1;
			           break;
			  case 10:  VdrData.H_10_saveFlag=1;
			           break;
			  case 11: VdrData.H_11_saveFlag=1;
			           break; 	       
			  case 12:	VdrData.H_12_saveFlag=1;
					  break;
			  case 13: VdrData.H_13_saveFlag=1;
					  break;
			  case 14:	VdrData.H_14_saveFlag=1;
					  break;
			  case 15: VdrData.H_15_saveFlag=1;
					  break;		
      	}



}
//FINSH_FUNCTION_EXPORT(vdrflag_enable, vdrflag_enable );     
*/


void  JT808_Related_Save_Process(void)
{
       if(DF_LOCK)
	   	 return ;

	  if(Dev_Voice.CMD_Type!='1')  // 录音时不做以下处理  
      {           
        // 1. VDR  08H  Data  Save          
         if(VdrData.H_08_saveFlag==1)
         {   
             WatchDog_Feed(); 
             //OutPrint_HEX("08H save",VdrData.H_08_BAK,126); 		
             rt_kprintf("\r\n 08H save  %d \r\n",Vdr_Wr_Rd_Offset.V_08H_Write);
             vdr_creat_08h(Vdr_Wr_Rd_Offset.V_08H_Write,VdrData.H_08_BAK,126); 		  
			 Vdr_Wr_Rd_Offset.V_08H_Write++; //  写完之后累加，就不用遍历了
             VdrData.H_08_saveFlag=0;  
			 return;
         }
	    // 2.  VDR  09H  Data  Save 
	     if(VdrData.H_09_saveFlag==1)
         {    
             WatchDog_Feed(); 
            // OutPrint_HEX("09H save",VdrData.H_09,666);  
             rt_kprintf("\r\n 09H save  %d\r\n",Vdr_Wr_Rd_Offset.V_09H_Write);
             vdr_creat_09h(Vdr_Wr_Rd_Offset.V_09H_Write,VdrData.H_09,666); 	
			 Vdr_Wr_Rd_Offset.V_09H_Write++; // //  写完之后累加，就不用遍历了 
             VdrData.H_09_saveFlag=0;    
			 return;
         }
       //  3. VDR  10H  Data  Save
         if(VdrData.H_10_saveFlag==1)
         {   
             WatchDog_Feed(); 
            // OutPrint_HEX("10H save",VdrData.H_10,234);   
             rt_kprintf("\r\n 10H save  %d\r\n",Vdr_Wr_Rd_Offset.V_10H_Write); 
             vdr_creat_10h(Vdr_Wr_Rd_Offset.V_10H_Write,VdrData.H_10,234); 	
			 Vdr_Wr_Rd_Offset.V_10H_Write++; // //  写完之后累加，就不用遍历了 
             VdrData.H_10_saveFlag=0;
			 return;
         }
      //  4.  VDR  11H  Data  Save
          if(VdrData.H_11_saveFlag==1)
         {   
             WatchDog_Feed(); 
            // OutPrint_HEX("11H save",VdrData.H_11,50);   
             rt_kprintf("\r\n 11H save   %d \r\n",Vdr_Wr_Rd_Offset.V_11H_Write);
             vdr_creat_11h(Vdr_Wr_Rd_Offset.V_11H_Write,VdrData.H_11,50); 	
			 Vdr_Wr_Rd_Offset.V_11H_Write++; // //  写完之后累加，就不用遍历了 
             VdrData.H_11_saveFlag=0;
			 return;
         } 
	  //  5.  VDR  12H  Data  Save
          if(VdrData.H_12_saveFlag==1)
         {   
             WatchDog_Feed(); 
             //OutPrint_HEX("12H save",VdrData.H_12,25);  
             rt_kprintf("\r\n 12H save  %d \r\n",Vdr_Wr_Rd_Offset.V_12H_Write);
             vdr_creat_12h(Vdr_Wr_Rd_Offset.V_12H_Write,VdrData.H_12,25); 	
			 Vdr_Wr_Rd_Offset.V_12H_Write++; // //  写完之后累加，就不用遍历了 
             VdrData.H_12_saveFlag=0;
			 
			 rt_kprintf("\r\n 写IC 卡  记录  \r\n");  
			 return;
         }  	 
      //  6.  VDR  13H  Data  Save
          if(VdrData.H_13_saveFlag==1)
         {  
             WatchDog_Feed(); 
            // OutPrint_HEX("13H save",VdrData.H_13,7);  
             rt_kprintf("\r\n 13H save %d \r\n",Vdr_Wr_Rd_Offset.V_13H_Write);
             vdr_creat_13h(Vdr_Wr_Rd_Offset.V_13H_Write,VdrData.H_13,7); 	
			 Vdr_Wr_Rd_Offset.V_13H_Write++; // //  写完之后累加，就不用遍历了 
             VdrData.H_13_saveFlag=0;
			 return;
         } 
      //  7.  VDR  14H  Data  Save
          if(VdrData.H_14_saveFlag==1)
         {    
              WatchDog_Feed(); 
              //OutPrint_HEX("14H save",VdrData.H_14,7);  
             rt_kprintf("\r\n 14H save  %d\r\n",Vdr_Wr_Rd_Offset.V_14H_Write);    
             vdr_creat_14h(Vdr_Wr_Rd_Offset.V_14H_Write,VdrData.H_14,7); 	
			 Vdr_Wr_Rd_Offset.V_14H_Write++; // //  写完之后累加，就不用遍历了 
             VdrData.H_14_saveFlag=0;
			 return; 
         } 		  
 
		 
		//---------------------------------------
	    //  存储每分钟速度信息  
		if(1==Avrgspd_Mint.saveFlag) 
		{
			Save_AvrgSpdPerMin();
			Avrgspd_Mint.saveFlag=0; 
			return;
		}					  	  

		//-----------------  存储疲劳驾驶记录 ---------------			
		if( TiredConf_struct.Tired_drive.Tireddrv_status==2)   
		{
	        Save_TiredDrive_Record();
			return;	 
		}			
		//-----------------  超速报警 ----------------------
		if(speed_Exd.excd_status==2)
		{
		   Spd_Exp_Wr();   
		   return; 
		}   

     }           
	//	 定时存储里程
	if((Vehicle_RunStatus==0x01)&&(DistanceWT_Flag==1))   
	{   //  如果车辆在行驶过程中，每255 秒存储一次里程数据    
	      //rt_kprintf("\r\n distance --------\r\n");        
		  DistanceWT_Flag=0;// clear
		  DF_Write_RecordAdd(Distance_m_u32,DayStartDistance_32,TYPE_DayDistancAdd);
		  JT808Conf_struct.DayStartDistance_32=DayStartDistance_32; 
		  JT808Conf_struct.Distance_m_u32=Distance_m_u32;		
		  return;		
	} 
			
	 //--------------------------------------------------------------

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
void Tired_Check( void )
{
	if( DispContent == 2 )
	{
		rt_kprintf( "\r\n				速度: %d Km/h    Debugspd=%d KM/h \r\n", Spd_Using/10, DebugSpd / 10 );
	}
	if( Spd_Using > 60 )                                                                                                        // Spd_Using 单位为0.1 km/h  速度大于6 km/h  认为是行驶
	// if(DebugSpd>60)
	{
		//-------------疲劳驾驶相关 -----------------------
		TiredConf_struct.Tired_drive.ACC_ONstate_counter++;                                                                     // ACC 累计工作时间
		if( DispContent == 5 )
		{
			rt_kprintf( "\r\n  ACC ON = %d", TiredConf_struct.Tired_drive.ACC_ONstate_counter );
		}
		if( TiredConf_struct.Tired_drive.ACC_ONstate_counter > 3 )                                                              //ACC 开 485 外设开电，防抖处理
		{
			Power_485CH1_ON;                                                                                                    // 第一路485的电        上电工作
		}

		TiredConf_struct.Tired_drive.ACC_Offstate_counter = 0;                                                                  // clear
		//------ 停车超时相关  -----------
		TiredConf_struct.TiredDoor.Parking_currentcnt	= 0;                                                                    // Clear  停车 计数器清 0
		Warn_Status[1]									&= ~0x08;                                                               // 停车超时清除
		//---------------------------------
		if( TiredConf_struct.Tired_drive.ACC_ONstate_counter == ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec- 300 ) )   //提前5分钟蜂鸣器提示注意疲劳驾驶 14100
		{
			TiredConf_struct.Tired_drive.Tgvoice_play = 1;
			TTS_play( "您还有5分钟就疲劳驾驶，请注意行车安全" );
		}

		Status_TiredwhRst = 1;                                                                                                  // 没疲劳时为1
		if( TiredConf_struct.Tired_drive.ACC_ONstate_counter >= ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec) )        //14400 // 连续驾驶超过4小时算疲劳驾驶
		{
			TiredConf_struct.Tired_drive.Tgvoice_play = 0;

			Status_TiredwhRst								= 2;                                                                // 疲劳了就为2了
			TiredConf_struct.Tired_drive.Tireddrv_status	= 1;                                                                // 记录疲劳驾驶的状态
			if( TiredConf_struct.Tired_drive.ACC_ONstate_counter == ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec) )    //14400
			{
				//Tired_drive.Tgvoice_play=1;  // 开始播放疲劳驾驶语音提示
				// Tired_drive.voicePly_counter++;
				Time2BCD( TiredConf_struct.Tired_drive.start_time );
				rt_kprintf( "\r\n   疲劳驾驶触发了! \r\n" );
				TTS_play( "您已经疲劳驾驶，请注意休息" );
				//tts_bro_tired_flag	= 1;
				Warn_Status[3]		|= 0x04; //BIT(2)  疲劳驾驶 
				//---- 触发即时上报数据-------
				PositionSD_Enable( );
				Current_UDP_sd = 1;
				//-------------------------------------
				Rstart_time = 1;
			}
			//--------- 蜂鸣器提示相关	关闭蜂鸣器 记录下疲劳驾驶  ------------
			Warn_Status[3]								|= 0x04;                                                                    //BIT(2)  疲劳驾驶
			TiredConf_struct.Tired_drive.Tgvoice_play	= 0;
		}
		TiredConf_struct.Tired_drive.Flag = 1;
	}else
	{
		//------------ 疲劳驾驶相关  ----------------------
		if( TiredConf_struct.Tired_drive.Flag == 1 )                                                                                //只有从ACCon切换过来时才做以下处理，如果长期关着，没有必要了
		{
			//-- ACC 没有休息前还算AccON 的状态  ---------------
			TiredConf_struct.Tired_drive.ACC_ONstate_counter++;                                                                     // ACC 累计工作时间

			if( DispContent == 5 )
			{
				rt_kprintf( "\r\n	ACC ON 2 = %d, Spd_Using = %d", TiredConf_struct.Tired_drive.ACC_ONstate_counter, Spd_Using );
			}
			if( TiredConf_struct.Tired_drive.ACC_ONstate_counter == ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec - 300 ) )   //提前5分钟蜂鸣器提示注意疲劳驾驶 14100
			{
				TiredConf_struct.Tired_drive.Tgvoice_play = 1;
				TTS_play( "您还有5分钟就疲劳驾驶，请注意行车安全" ); 
			}

			if( TiredConf_struct.Tired_drive.ACC_ONstate_counter >= ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec ) )        //14400 // 连续驾驶超过4小时算疲劳驾驶
			{
				Status_TiredwhRst								= 2;                                                                // 疲劳了就为2了
				TiredConf_struct.Tired_drive.Tireddrv_status	= 1;                                                                // 记录疲劳驾驶的状态
				if( TiredConf_struct.Tired_drive.ACC_ONstate_counter == ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec  ) )    //14400
				{
					Time2BCD( TiredConf_struct.Tired_drive.start_time );
					rt_kprintf( "\r\n	 速度小，但未满足休息门限时间 疲劳驾驶触发了! \r\n");
					TTS_play( "您已经疲劳驾驶，请注意休息" );
					//tts_bro_tired_flag	= 1;
					Warn_Status[3]		|= 0x04;                                                                                    //BIT(2)  疲劳驾驶
					//---- 触发即时上报数据-------
					PositionSD_Enable( );
					Current_UDP_sd = 1;
					//-------------------------------------
					Rstart_time = 1;
				}
				Warn_Status[3] |= 0x04;                                                                                             //BIT(2)  疲劳驾驶
			}
			//------------------------------------------------------------------------------
			TiredConf_struct.Tired_drive.ACC_Offstate_counter++;
			if( DispContent == 5 )
			{
				rt_kprintf( "\r\n  ACC OFF = %d, Spd_Using = %d", TiredConf_struct.Tired_drive.ACC_Offstate_counter, Spd_Using );
			}
			if( TiredConf_struct.Tired_drive.ACC_Offstate_counter >= TiredConf_struct.TiredDoor.Door_MinSleepSec )  //1200	// ACC 关20分钟视为休息
			{
				if( TiredConf_struct.Tired_drive.ACC_Offstate_counter == TiredConf_struct.TiredDoor.Door_MinSleepSec )
				{
					TiredConf_struct.Tired_drive.ACC_Offstate_counter	= 0;
					TiredConf_struct.Tired_drive.ACC_ONstate_counter	= 0;
					Warn_Status[3]										&= ~0x04;                    //BIT(2)	疲劳驾驶
				    /*	
				       if( tts_bro_tired_flag == 1 )
					{
						TTS_play( "您的疲劳驾驶报警已经解除" );
						tts_bro_tired_flag = 0;
					}*/
					// Status_TiredwhRst=0;
					// DF_WriteFlashSector(DF_TiredStartTime_Page,0,(u8*)&Status_TiredwhRst,1); //清除O后要写入
					//---- 触发即时上报数据-------
					PositionSD_Enable( );
					Current_UDP_sd = 1;
					//-------------------------------------	                          rt_kprintf("\r\n   ACC  休息了 \r\n");
					if( Rstart_time == 1 )
					{
						TiredConf_struct.Tired_drive.Flag = 0; ///////
						//Tired_drive.ACC_ONstate_counter=0;

						Rstart_time										= 0;
						TiredConf_struct.Tired_drive.Tireddrv_status	= 2;
						//rt_kprintf("\r\n疲劳结束时间写入");
					}
				}
			}
		}

		//----------  超长停车相关 ------------------------------------------------------
	#if 0	
		if( Spd_Using < 3 )                                                                                         //(Spd_Using==0)  // 停车超时从速度为0时开始
		//if(DebugSpd<3)
		{
			TiredConf_struct.TiredDoor.Parking_currentcnt++;
			//if(DispContent==2)
			//  printf("\r\n @#@\r\n");
			if( TiredConf_struct.TiredDoor.Parking_currentcnt >= TiredConf_struct.TiredDoor.Door_MaxParkingSec )    //TiredDoor.Door_MaxParkingSec)
			{                                                                                                       //  停车大于门限值，就算停车了
				Warn_Status[1] |= 0x08;                                                                             // 停车超时触发
				if( TiredConf_struct.TiredDoor.Parking_currentcnt == TiredConf_struct.TiredDoor.Door_MaxParkingSec )
				{
					//---- 触发即时上报数据-------
					PositionSD_Enable( );
					Current_UDP_sd = 1;
					//-------------------------------------
					TTS_play( " 超时停车触发" );
					rt_kprintf( "\r\n 超时停车触发! \r\n" );
				}
			}
		}else
		{
			TiredConf_struct.TiredDoor.Parking_currentcnt = 0;
		}
	#endif	
		//--------------------------------------------------------------------------------
	}
}



/*
    打印输出 HEX 信息，Descrip : 描述信息 ，instr :打印信息， inlen: 打印长度
*/
void OutPrint_HEX(u8 * Descrip, u8 *instr, u16 inlen )
{
    u32  i=0;
     rt_kprintf("\r\n %s:",Descrip);
     for( i=0;i<inlen;i++)
          rt_kprintf("%2X ",instr[i]); 
     rt_kprintf("\r\n");
}


//------ 天津交通局飞翼判断 ----------  灰线10pin座的 pin6 
void Fly_Swing_judge_TJ(void)
{
    if(DoorLight_StatusGet())  // Pc 1
		{	//		 接高  触发
		//rt_kprintf("\r\n检测到PA1 低");
	    if( (Car_Status[3]&0x10)==0x00)
	      	{
			Car_Status[3]|=0x10; 
			PositionSD_Enable();					
			Current_UDP_sd=1;	
			rt_kprintf("\r\n  飞翼开启\r\n");	  
	      	}	
		  Car_Status[3]|=0x10; //  Bit(4)     Set  1  表示  飞翼开  1 停运状态 
		  
		  //报警状态 bit  15  是渣土车后斗挡板开启 add	on	 13-11-27
		  Warn_Status[2]|=0x80;   
		}
   else
		{  //	常态
		//rt_kprintf("\r\n检测到PA1 高");
	    if( Car_Status[3]&0x10)
	      	{
			Car_Status[3]&=~0x10;
			PositionSD_Enable();					
			Current_UDP_sd=1;	
			rt_kprintf("\r\n  飞翼关闭\r\n");	 	 
	      	}
	       Car_Status[3]&=~0x10; //  Bit(4)     Set  0  表示  飞翼开  0 营运状态 		

		     //报警状态 bit  15  是渣土车后斗挡板开启 add	on	 13-11-27
		  Warn_Status[2]&=~0x80;      
		}	
}  





/*
void gpsspd(u8 *strin)   //  GPS 用假速度
{
   sscanf(strin, "%d", (u32*)&Spd_add_debug);
   rt_kprintf("\r\n set gpsspd= %d s\r\n",Spd_add_debug);  
}
FINSH_FUNCTION_EXPORT(gpsspd, gpsspd);    

void normalspd(void)    // 恢复 GPS 用真速度
{
    Spd_add_debug=0;
    rt_kprintf("\r\n recover normal spd\r\n");   
}
FINSH_FUNCTION_EXPORT(normalspd, normalspd);     
*/
void  dur(u8 *content)
{
  sscanf(content, "%d", (u32*)&Current_SD_Duration);
  rt_kprintf("\r\n 手动设置上报时间间隔 %d s \r\n",Current_SD_Duration); 
  
       JT808Conf_struct.DURATION.Default_Dur=Current_SD_Duration;
        Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));

}
FINSH_FUNCTION_EXPORT(dur, dur);  



void handsms(u8 *instr)
{    
    memset(SMS_Service.SMS_sd_Content,0,sizeof(SMS_Service.SMS_sd_Content)); 
    memcpy(SMS_Service.SMS_sd_Content,instr,strlen((const char*)instr));  
    SMS_Service.SMS_sendFlag=1;	
	SMS_Service.SMS_come=1;
    rt_kprintf("手动发送:  %s",SMS_Service.SMS_sd_Content);    
}
//FINSH_FUNCTION_EXPORT(handsms, handsms);  


void driver_name(u8 *instr)
{   
    memset(JT808Conf_struct.Driver_Info.DriveName,0,sizeof(JT808Conf_struct.Driver_Info.DriveName)); 
	memcpy(JT808Conf_struct.Driver_Info.DriveName,instr,strlen((const char*)instr)); 
    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));		
}
FINSH_FUNCTION_EXPORT(driver_name, set_driver_name );   

void adjust_ok(int in)
{
    if(in>1)
  	 return;
   if(in==1)
    {
       JT808Conf_struct.DF_K_adjustState=1;
        ModuleStatus|=Status_Pcheck;
   	}
   else
   {
      JT808Conf_struct.DF_K_adjustState=0;   
       ModuleStatus&=~Status_Pcheck; 
   }
   rt_kprintf("adjust_ok(%d)\r\n",in); 
   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));	   
}
FINSH_FUNCTION_EXPORT(adjust_ok, set_adjust_ok  1:ok 0: notok); 

void spd_type(int  in)
{
  if(in>1)
  	 return;
  
  if(in==0)
  	{
  	  JT808Conf_struct.Speed_GetType=0;
      JT808Conf_struct.DF_K_adjustState=0; 
	  ModuleStatus&=~Status_Pcheck; 
	  
	  rt_kprintf("spd_type: gps_speed get\r\n"); 
  	}
  else
  if(in==1)	
  	{
       JT808Conf_struct.Speed_GetType=1;
       JT808Conf_struct.DF_K_adjustState=0;
	   ModuleStatus&=~Status_Pcheck; 
	   
	   rt_kprintf("spd_type: sensor_speed get\r\n"); 
  	}  
  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));	  

}
FINSH_FUNCTION_EXPORT(spd_type, spd_type  gps->0  sensor->1); 

void  plus_num(u32 value)
{
  JT808Conf_struct.Vech_Character_Value=value;
  
  rt_kprintf("plus_num(%d)\r\n",value);    
  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));	  
   
}
FINSH_FUNCTION_EXPORT(plus_num, set_plus_num); 


void chepai(u8 *instr)
{    
    memset(Vechicle_Info.Vech_Num,0,sizeof(Vechicle_Info.Vech_Num));
    memcpy(Vechicle_Info.Vech_Num,instr,8); 
   DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));    
   WatchDog_Feed();
   DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
   WatchDog_Feed();
   DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
}
FINSH_FUNCTION_EXPORT(chepai, set_chepai);   

void  vin_set(u8 *instr)
{
	 //车辆VIN
	memset(Vechicle_Info.Vech_VIN,0,sizeof(Vechicle_Info.Vech_VIN));
	memcpy(Vechicle_Info.Vech_VIN,instr,strlen((const char*)instr)); 
	DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));	
	WatchDog_Feed();
	DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
	WatchDog_Feed();
	DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 

       rt_kprintf("\r\n 手动设置vin:%s \r\n",instr); 

}
FINSH_FUNCTION_EXPORT(vin_set, vin_set );    

void  current(void)
{
    PositionSD_Enable();
    Current_UDP_sd=1;
}
FINSH_FUNCTION_EXPORT(current, current );    


void  link_mode(u8 *instr)
{
   if(instr[0]=='1')
   {
      Vechicle_Info.Link_Frist_Mode=1;
	   rt_kprintf("\r\n Mainlink:%s \r\n",instr);   
   }	  
   else
   if(instr[0]=='0')
   {
   	   Vechicle_Info.Link_Frist_Mode=0;
       rt_kprintf("\r\n DNSR :%s \r\n",instr);  
   }	
  
  DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));
  WatchDog_Feed();
  DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
  
  WatchDog_Feed();
  DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 

}
FINSH_FUNCTION_EXPORT(link_mode, link_mode );    

void  redial(void)
{
      DataLink_EndFlag=1; //AT_End();   
        rt_kprintf("\r\n Redial\r\n");      
}
FINSH_FUNCTION_EXPORT(redial, redial); 

void  port_main(u8 *instr)
{
   		   sscanf(instr, "%d", (u32*)&RemotePort_main);  
		   rt_kprintf("\r\n设置主端口=%d!",RemotePort_main);
		   SysConf_struct.Port_main=RemotePort_main;
		   Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
		   
		   DataLink_MainSocket_set(RemoteIP_main,RemotePort_main,1);
			DataLink_EndFlag=1; //AT_End();  

}
FINSH_FUNCTION_EXPORT(port_main, port_main); 

void  port_aux(u8 *instr)
{
   		   sscanf(instr, "%d", (u32*)&RemotePort_aux);  
		   rt_kprintf("\r\n设置辅助端口=%d!",RemotePort_aux);
		   SysConf_struct.Port_Aux=RemotePort_aux;
		   Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
		   
		   DataLink_AuxSocket_set(RemoteIP_main,RemotePort_aux,1);
		//	DataLink_EndFlag=1; //AT_End();  

}
//FINSH_FUNCTION_EXPORT(port_aux, port_aux); 


void dnsr_main(u8*instr)
{
    u16  len=0;

	len=strlen((const char*)instr);

	if(len!=0)
   {
        memset(DomainNameStr,0,sizeof(DomainNameStr));					  
		memset(SysConf_struct.DNSR,0,sizeof(DomainNameStr));  
		memcpy(DomainNameStr,(char*)instr,len);
		memcpy(SysConf_struct.DNSR,(char*)instr,len);
		Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
		//----- 传给 GSM 模块------
		DataLink_DNSR_Set(SysConf_struct.DNSR,1); 
         //--------    清除鉴权码 -------------------
	     idip("clear");		
	}					 

}
FINSH_FUNCTION_EXPORT(dnsr_main, dnsr_main);  

void dnsr_aux(u8*instr)
{
    u16  len=0;

	len=strlen((const char*)instr);

	if(len!=0)
   {
        memset(DomainNameStr_aux,0,sizeof(DomainNameStr_aux));					  
		memset(SysConf_struct.DNSR_Aux,0,sizeof(DomainNameStr_aux));  
		memcpy(DomainNameStr_aux,(char*)instr,len);
		memcpy(SysConf_struct.DNSR_Aux,(char*)instr,len);
		Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
		//----- 传给 GSM 模块------
		DataLink_DNSR2_Set(SysConf_struct.DNSR_Aux,1);  
         //--------    清除鉴权码 -------------------
	  //   idip("clear");		
	}					 

}
//FINSH_FUNCTION_EXPORT(dnsr_aux, dnsr_aux);  

void password(u8 in)
{
  
   Login_Menu_Flag=in;    // clear  first flag 	 Login_Menu_Flag=0;     //  输入界面为0   
   DF_WriteFlashSector(DF_LOGIIN_Flag_offset,0,&Login_Menu_Flag,1); 
   rt_kprintf("\r\n  password=%d \r\n",Login_Menu_Flag);  
}
FINSH_FUNCTION_EXPORT(password, password);   


void  print_power(u8*instr)
{
     if(instr[0]=='0')
    {   
          Print_power_Q5_enable=0;
	  lcd_update_all();
	  //rt_kprintf("\r\n  printer poweroff\r\n");
    }

   if(instr[0]=='1')
    {    
         Print_power_Q5_enable=1;
          lcd_update_all(); 
	  // rt_kprintf("\r\n printer poweron\r\n"); 
  } 
}
//FINSH_FUNCTION_EXPORT(print_power, print_power[1|0]);       

void buzzer_onoff(u8 in) 
{
     if(0==in)
    {  
      if(Module_3020C==GPS_MODULE_TYPE)
      {
         Buzzer_on_Q7_enable=0;
	     lcd_update_all();
      }
	  if(Module_3017A==GPS_MODULE_TYPE)
	  	{
			GPIO_ResetBits(Buzzer_IO_Group,Buzzer_Group_Num); 
	  	}
	  // rt_kprintf("\r\n  buzzer off\r\n");}
    }
	  
   if(1==in)
    {  
      if(Module_3020C==GPS_MODULE_TYPE)
      {
        Buzzer_on_Q7_enable=1;
		lcd_update_all();
      }
	  if(Module_3017A==GPS_MODULE_TYPE)
	  {
           	    GPIO_SetBits(Buzzer_IO_Group,Buzzer_Group_Num);	
	  }
	//	rt_kprintf("\r\n buzzer  on\r\n");
	} 


}
FINSH_FUNCTION_EXPORT(buzzer_onoff, buzzer_onoff[1|0]);    

/*
void jiaspd(u16 in)
{
    Speed_jiade=in;
	rt_kprintf("\r\n jia spd =%d\r\n",Speed_jiade); 
}
FINSH_FUNCTION_EXPORT(jiaspd, jiaspd);     


void bcd(void)
{
   u8 in[6];

	memset(in,0,6); 
    Time2BCD(in); 
	rt_kprintf("\r\n setset %x-%x-%x %x:%x:%x \r\n",in[0],in[1],in[2],in[3],in[4],in[5]);    
}
FINSH_FUNCTION_EXPORT(bcd, bcd);         
*/

void provinceid(u8 *strin)
{
  sscanf(strin, "%d", (u32*)&Vechicle_Info.Dev_ProvinceID);
  
  DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));
  WatchDog_Feed();
  DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
  
  WatchDog_Feed();
  DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
 // rt_kprintf("\r\n 车辆所在省域ID: %d \r\n",Vechicle_Info.Dev_ProvinceID); 
}
//FINSH_FUNCTION_EXPORT(provinceid, provinceid);    

void cityid(u8 *strin) 
{
   sscanf(strin, "%d", (u32*)&Vechicle_Info.Dev_CityID);  
   DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));
   WatchDog_Feed();
   DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
   
   WatchDog_Feed();
   DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
  /// rt_kprintf("\r\n 车辆所在市域ID: %d \r\n",Vechicle_Info.Dev_CityID);   
}
//FINSH_FUNCTION_EXPORT(cityid, cityid);     

void ata_enable(u8 value)
{
  //rt_kprintf("\r\n  ata_enable=%d \r\n",value); 

  JT808Conf_struct.Auto_ATA_flag=value;  
  Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
}
//FINSH_FUNCTION_EXPORT(ata_enable, ata_enable[1|0]);    


void  type_vech(u8 type)   
{
  Vechicle_Info.Vech_Type_Mark=type;   

  if(type==1)
  	{
  	  rt_kprintf("\r\n  车辆类型设置为:  两客一危\r\n");
	  memset(Vechicle_Info.Vech_Type,0,sizeof(Vechicle_Info.Vech_Type));
	  memcpy(Vechicle_Info.Vech_Type,"客车",4);	

  	}
  else
  if(type==2)
     {
         rt_kprintf("\r\n  车辆类型设置为:  货运\r\n");
         memset(Vechicle_Info.Vech_Type,0,sizeof(Vechicle_Info.Vech_Type));
	     memcpy(Vechicle_Info.Vech_Type,"货车",4);	 
  	}	 
  	  
	
  DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));
  WatchDog_Feed();
  DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
  
  WatchDog_Feed();
  DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 

}
FINSH_FUNCTION_EXPORT(type_vech, type_vech[1|0]);      


// C.  Module
