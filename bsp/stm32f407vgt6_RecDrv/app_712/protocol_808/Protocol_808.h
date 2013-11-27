/*
     Protocol 808 .h
*/
#ifndef  _PROTOCOL808
#define   _PROTOCOL808

#include <rthw.h>
#include <rtthread.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>
#include "App_moduleConfig.h" 
#include "gps.h"
#include "DF_Oper.h"


#define  CURREN_LIM_Dur        1  

 // ----   Media  Trans state ---
#define   MEDIA
#define  enable                1
#define  disable               0
#define  other                  2
#define  transfer             3 



//------------  GPS function------
#define  INIT         1
#define  PROCESS      0

#define  K_adjust_Duration       30                //  校验K值所需要时间   


// -------  ISP  Address   -------------
#define    ISP_APP_Addr                        0x6400    //  512*50  Page
#define    ISP_RAM_Addr                       0x1400   // 512*10 Page

//--------ISP Status Byte  ------------
#define    ISP_BYTE_StartValue            0xF1     // 远程下载开始字节
#define    ISP_BYTE_CrcPass               0xE1     //  接收完数据包后，改成这个数值，表示校验通过
#define    ISP_BYTE_TypeNotmatch          0xC1     //   校验通过，但类型不匹配
#define    ISP_BYTE_Rdy2Update            0x01     //   校验通过类型匹配，等待更新


//------- 通话状态---------
#define CallState_Idle 		        0
#define CallState_Ring 		        1
#define CallState_Connected	        2
#define CallState_Hangup		    4
#define CallState_rdytoDialLis      5
#define CallState_Dialing           8   //主叫拨号


//-----------------行车记录仪  附录A 命令字 --------------------
// 1. 上载命令字 
#define  A_Up_DrvInfo          0x01        //  当前驾驶人信息
#define  A_Up_RTC              0x02        //  采集记录仪的实时时钟
#define  A_Up_Dist             0x03        //  采集里程
#define  A_Up_PLUS             0x04        //  采集记录仪特征系数
#define  A_Up_AvrgMin          0x05        //  每分钟平均速度记录
#define  A_Up_VechInfo         0x06        //  车辆信息
#define  A_Up_Doubt            0x07        //  事故疑点数据
#define  A_Up_N2daysDist       0x08        //  采集最近2个日历天的累计行驶里程
#define  A_Up_N2daysSpd        0x09        //  采集最近2个日历天内的行驶速度数据 
#define  A_Up_Tired            0x11        //  疲劳驾驶记录   

// 2. 下传命令 
#define  A_Dn_DrvInfo          0x81        //  设置驾驶员信息
#define  A_Dn_VehicleInfo      0x82        //  设置车辆信息
#define  A_Dn_RTC              0xC2        //  设置记录仪时间
#define  A_Dn_Plus             0xC3        //  设置速度脉冲系数

// 3.  传输方式
#define   Trans_Serial         0           //  串口方式传输
#define   Trans_WireLess       1           //  无线传输


// 4. 808 协议   消息ID
#define   MSG_0x0001           0x0001      // 终端通用应答
#define   MSG_0x8001           0x8001      // 平台通用应答
#define   MSG_0x0002           0x0002      // 终端心跳 
#define   MSG_0x0003           0x0003      // 终端注销
#define   MSG_0x0100           0x0100      // 终端注册
#define   MSG_0x8100           0x8100      // 终端注册应答
#define   MSG_0x0101           0x0101      // 终端注销
#define   MSG_0x0102           0x0102      // 终端鉴权
#define   MSG_0x8103           0x8103      // 设置终端参数 
#define   MSG_0x8104           0x8104      // 查询终端参数
#define   MSG_0x0104           0x0104      // 查询终端参数应答
#define   MSG_0x8105           0x8105      // 终端控制
#define   MSG_0x0200           0x0200      // 位置信息汇报 
#define   MSG_0x8201           0x8201      // 位置信息查询
#define   MSG_0x0201           0x0201      // 位置信息查询应答 
#define   MSG_0x8202           0x8202      // 临时位置跟踪控制
#define   MSG_0x8300           0x8300      // 文本信息下发
#define   MSG_0x8301           0x8301      // ---事件设置
#define   MSG_0x0301           0x0301      // ---事件报告
#define   MSG_0x8302           0x8302      // ---提问下发
#define   MSG_0x0302           0x0302      // ---提问应答
#define   MSG_0x8303           0x8303      // ---信息点播菜单设置
#define   MSG_0x0303           0x0303      // ---信息点播 取消
#define   MSG_0x8304           0x8304      // ---信息服务
#define   MSG_0x8400           0x8400      // ---电话回拨
#define   MSG_0x8401           0x8401      // ---设置电话本
#define   MSG_0x8500           0x8500      // 车辆控制
#define   MSG_0x0500           0x0500      // 车俩控制应答
#define   MSG_0x8600           0x8600      // 设置圆形区域
#define   MSG_0x8601           0x8601      // 删除圆形区域
#define   MSG_0x8602           0x8602      // 设置矩形区域
#define   MSG_0x8603           0x8603      // 删除矩形区域
#define   MSG_0x8604           0x8604      // 设置多边形区域
#define   MSG_0x8605           0x8605      // 删除多边形区域
#define   MSG_0x8606           0x8606      // 设置路线
#define   MSG_0x8607           0x8607      // 删除路线
#define   MSG_0x8700           0x8700      // 行车记录仪数据采集命令
#define   MSG_0x0700           0x0700      // 行驶记录仪数据上传
#define   MSG_0x8701           0x8701      // 行驶记录仪但是下传
#define   MSG_0x0701           0x0701      // ---电子运单上报
#define   MSG_0x0702           0x0702      // 驾驶员身份信息采集上报
#define   MSG_0x0107           0x0107      //  BD --终端属性查询
#define   MSG_0x0108           0x0108      //  BD--终端升级结果通知
#define   MSG_0x0704           0x0704      //  BD--定位数据批量上传
#define   MSG_0x0705           0x0705      //  BD-- CAN 总线数据上传
#define   MSG_0x0805           0x0805      //  BD-- 摄像头立即拍摄命令应答
#define   MSG_0x0800           0x0800      // ---多媒体事件信息上传
#define   MSG_0x0801           0x0801      // ---多媒体数据上传
#define   MSG_0x8800           0x8800      // ---多媒体数据上传应答
#define   MSG_0x8801           0x8801      // 摄像头立即拍照命令
#define   MSG_0x8802           0x8802      // 存储多媒体数据检索
#define   MSG_0x0802           0x0802      // 存储多媒体数据检索应答
#define   MSG_0x0805           0x0805     //  拍照上传立即应答 
#define   MSG_0x8803           0x8803      // 存储多媒体数据上传
#define   MSG_0x8804           0x8804      // 录音开始命令 
#define   MSG_0x8900           0x8900      // 数据下行透传
#define   MSG_0x0900           0x0900      // 数据上行透传
#define   MSG_0x0901           0x0901      // 数据压缩上报
#define   MSG_0x8A00           0x8A00      // 平台RSA公钥
#define   MSG_0x0A00           0x0A00      // 终端RSA公钥

//--------  发送包的方式  ------
#define   Packet_Normal         0        //  正常包     
#define   Packet_Divide         1        //  分包

#define   SpxBuf_Size       512
#define   SpxGet_Size       100



typedef struct _A_AckFlag 
{
	// 1. 上载命令字 
u8  A_Flag__Up_Ver_00H             ;// 0x00   1     //  记录仪执行标准版本号
u8  A_Flag_Up_DrvInfo_01H          ;// 0x01   2     //  当前驾驶人信息
u8  A_Flag_Up_RTC_02H              ;// 0x02   3     //  采集记录仪的实时时钟
u8  A_Flag_Up_Dist_03H             ;//0x03    4    //  采集里程
u8  A_Flag_Up_PLUS_04H             ;//0x04    5    //  采集记录仪速度脉冲系数
u8  A_Flag_Up_VechInfo_06H         ;//0x06    6    //  车辆信息
u8  A_Flag_Up_SetInfo_08H          ;//0x08    7    //  状态配置信息
u8  A_Flag_Up_DevID_16H            ;//0x16    8    //  记录仪唯一ID
u8  A_Flag_Up_AvrgSec_09H          ;//0x09    9    //  每秒钟平均速度记录
u8  A_Flag_Up_AvrgMin_05H          ;//0x05    10    //  每分钟平均速度记录
u8  A_Flag_Up_Posit_13H            ;//0x13    11    //  指定位置信息记录
u8  A_Flag_Up_Doubt_07H            ;//0x07    12    //  事故疑点数据
u8  A_Flag_Up_Tired_11H            ;//0x11    13    //  疲劳驾驶记录
u8  A_Flag_Up_LogIn_10H            ;//0x10    14    //  登入登出记录
u8  A_Flag_Up_Powercut_14H         ;//0x14    15    //  外部供电记录
u8  A_Flag_Up_SetMdfy_15H          ;//0x15    16    //  参数修改记录
	
	// 2. 下传命令 
u8  A_Flag_Dn_DrvInfo_82H          ;//0x82    17    //  设置车辆信息
u8  A_Flag_Dn_SetupDate_83H        ;//0x83    18    //  设置初装日期
u8  A_Flag_Dn_Satus_84H            ;//0x84    19    //  设置状态量信息
u8  A_Flag_Dn_RTC_C2H              ;//0xC2    20    //  设置记录仪时间
u8  A_Flag_Dn_Plus_C3H             ;//0xC3    21    //  设置速度脉冲系数    
}A_AckFlag;

typedef struct _GPRMC_PRO
{
  void (*Time)(u8 *tmpinfo, u8 hour, u8 min , u8 sec);       
  void (*Status)(u8 *tmpinfo);     
  void (*Latitude)(u8 *tmpinfo);
  void (*Latitude_NS)(u8 *tmpinfo);  // S 南纬  N  北纬
  void (*Longitude)(u8 *tmpinfo);
  void (*Longitude_WE)(u8 *tmpinfo); // E 东经  W  西经
  void (*Speed)(u8 *tmpinfo,u8 Invalue,u8 Point);
  void (*Direction)(u8 *tmpinfo,u8 Invalue,u8 Point);
  void (*Date)(u8 *tmpinfo,u8 fDateModify, u8 hour, u8 min , u8 sec); 

}GPRMC_PRO; 

typedef struct _Position
{
  u8  latitude_BgEnd[4];
  u8  longitude_BgEnd[4];
}POSIT;


typedef struct _TCP_ACKFlag
{
u8  f_CentreCMDack_0001H;
u8  f_CentreCMDack_resualt;
u8  f_CurrentPosition_0201H;  
u8  f_CurrentEventACK_0301H; //  事件报告
u8  f_MsgBroadCast_0303H;//  信息点播/ 取消
u8  f_MediaIndexACK_0802H; //  多媒体应答      0  idle  1  查询图片   2  查询音频  3 查询视频
u8  f_QueryEventCode;  //  0  平台下发命令   1 定时动作   2  抢劫报警触发 3  碰撞触发   
u8  f_SettingPram_0104H;
u8  f_DriverInfoSD_0702H; //   驾驶员 身份信息采集上报
u8  f_Worklist_SD_0701H; //   电子运单上传 
     //  -----  北斗扩展----
u8  f_BD_Extend_7F02H; // 北斗信息查询应答
u8  f_BD_Extend_7F00H;// 扩展终端上发指令
u8  f_BD_DeviceAttribute_8107; //  中心查找终端属性
u8  f_BD_BatchTrans_0704H;   //  定位数据批量上传
u8  f_BD_CentreTakeAck_0805H; // 中心拍照命令应答 
u8  f_BD_ISPResualt_0108H;   //    终端升级结果     1  :成功    2: 失败      3:取消

}TCP_ACKFlag;
 

typedef struct _NandSVflag
{
 u8  Doubt_SaveFlag;      // 存储事故疑点 
 u8  MintPosit_SaveFlag;  // 存储每分钟位置  
 u8  PowerCut_SaveFlag;   // 存储断电记录
 u8  Log_SaveFlag;        // 存储登录记录
 u8  Setting_SaveFlag;    // 存储参数修改记录  


}NANDSVFlag;

typedef struct _Stdver
{
   u8 stdverStr[14];   //记录仪执行的国标版本
   u8 MdfyID;          // 修改单号 

}STD_VER; 

typedef struct _DriveInfo
{  
    u8 DriveCode[3]; 
	u8 DriverCard_ID[19];  // 驾驶员驾驶证号码 18位
	u8 DriveName[22];	   // 驾驶员 姓名21
	u8 Driver_ID[21];      // 驾驶员身份证号码20 
	u8 Drv_CareerID[41];  // 驾驶员从业资格证40
	u8 Comfirm_agentID[41]; // 发证机构名称 40
}DRV_INFO; 

typedef struct _VechInfo
{
 u8     Dev_Color;           // 车牌颜色           // JT415    1  蓝 2 黄 3 黑 4 白 9其他
 u8     loginpassword_flag;  //  界面输入标志位    0 :default    1:  longin ok
 u8     Link_Frist_Mode;		 //   首次连接模式		  0  : dnsr first	  1: mainlink  first 
 u8     Vech_Type_Mark;//    1:两客一危    2:货运
 u16    Dev_ProvinceID;      // 车辆所在省ID
 u16    Dev_CityID;          // 车辆所在市ID    
 u8     Vech_VIN[18];        // 车辆VIN号17
 u8     Vech_Num[13];	     // 车牌号12
 u8     Vech_Type[13];       // 车辆类型 12
}VechINFO;

//--------- 终端属性---------
typedef struct  _PRODUCT_ATTRIBUTE
{
   u16  _1_DevType; //   终端类型   
   u8   _2_ProducterID[5];// 制造商ID
   u8   _3_Dev_TYPENUM[20];//终端型号
   u8   _4_Dev_ID[7];//终端ID	
   u8   _5_Sim_ICCID[10];//终端SIM卡ICCID	
   u8   _6_HardwareVer_Len;//硬件版本号长度
   u8   _7_HardwareVer[20]; // 硬件版本号
   u8   _8_SoftwareVer_len;// 软件版本号长度
   u8  _9_SoftwareVer[20];//软件版本号
   u8 _10_FirmWareVer_len;// 固件版本号长度
   u8 _11_FirmWare[20];// 固件版本号
   u8 _12_GNSSAttribute; // 定位模块属性
   u8 _13_ComModuleAttribute;// 通信模块属性
}PRODUCT_ATTRIBUTE;

//-------- 北斗 扩展  -------------
typedef struct _BD_EXTEND
{
	  // 1. ----  车台设置相关 ----
	  u32	  Termi_Type;	 // 终端类型
	  u32	  Software_Ver; //	软件版本
	  u32	  GNSS_Attribute;	// BD  模块属性
	  u32	  GSMmodule_Attribute; // GSM 模块属性
	  u32	  Device_Attribute;   //  终端属性	
	
	 //  2.  北斗设置
		u8	 GNSS_Mode; 	//	北斗模块通信模式
		u8	 GNSS_Baud; 	//	北斗模块 通信波特率  表示
		u32   GNSS_Baud_Value;//   北斗模块数值
		u8	   BD_OutputFreq;  // 北斗模块输出更新率设置 
		u32   BD_SampleFrea; //  北斗模块采集 NMEA	数据频率
				//-----  2013 BD add -------------
		u8	   BD_OriginalDataTransMode;	 //  GNSS 模块详细定位信息上传方式
		u8	   BD_OriginalDataTransSettings; //  GNSS 模块详细数据上传设置01: 单位为秒	02 : 单位为米 03: 单位为秒
			
	
	 // 3.	CAN   相关设置
		u32   CAN_1_Mode;	 //  CAN  模式	01:还回模式  10  : 普通模式   11:  静默模式  
		u32   CAN_1_ID;
		u8	  CAN_1_Type;  
		u8	  CAN_1_SampleDuration;  //  单位秒
		u8	  CAN_1_TransDuration;	//	单位 秒
			
		u32   CAN_2_Mode;	//	 CAN  模式
		u32   CAN_2_ID;
		u8	   CAN_2_Type;		
		u8	   CAN_2_SampleDuration;  //  单位秒
		u8	   CAN_2_TransDuration;  //  单位 秒 
		u32   Collision_Check;	   //  bit 31 : 开启关闭碰撞  b7-b0  碰撞时间门限 b8-b15  碰撞加速度门限
	
		u8	   CloseAllcan;  
	
	 //   位置附加信息
		  // 1. 信号强度
		  u16  FJ_SignalValue;	//	信号强度   高字节 0 ；低字节  高4为 X2 gprs强度 ，低4位 卫星颗数
		 //  2. 自定义状态机模拟量上传	
		  u8	FJ_IO_1;
	   u8	 FJ_IO_2;
	   u16	AD_0;  //  2 Byte
	   u16	AD_1; //   2 Byte
	
		  u8  Extent_IO_status;  // 新北斗协议 IO 状态
	
	
	//	  围栏相关判断
		  u8	Close_CommunicateFlag;	//	进区域 关闭通信   
		  u8	Trans_GNSS_Flag;	//	进区域采集GNSS
	 

  
} BD_EXTEND;



//-------

//-------车台注册相关 ------------
typedef struct _DevRegst
{
  u8  Sd_counter; //  使能发送间隔计数器
  u8  Enable_sd;  //  使能发送 注册信息
  u8  DeRegst_sd; //  注销车台 
  u8  Sd_add;        // 注册重发次数
}DevRegst;

//------- 车台鉴权相关 ----------
typedef struct _DevLOGIN
{
  u8  Operate_enable;  // 使能开始操作鉴权操作   1 :enable  0: idle
  u8  Sd_counter;      // 定时发送计数器
  u8  Sd_times;        // 发送次数计数器  
  u8  Enable_sd;       // 使能发送鉴权信息
  u8  Sd_add;           //  鉴权发送次数
}DevLOGIN;
//------- 超速报警 -------
typedef struct SPDEXP
{
  u8  excd_status;
  u16 current_maxSpd;    //  超速报警下 当前的最大速度
  u32 dur_seconds;       //  超速状态计数器    单位: s
  u8  speed_flag;        //  超速报警触发后先立即上报一包标志位  
  u8  ex_startTime[6];
  u8  ex_endTime[6];     
}SPD_EXP;
//------- 疲劳驾驶门限 ----
typedef struct _TIRED_DOOR
{                                              //  单位:  s
  u32  Door_DrvKeepingSec;       // 连续驾驶时间
  u32  Door_DayAccumlateDrvSec;  // 当天累加驾驶时间
  u32  Door_MinSleepSec;         // 最小休息时间 
  u32  Door_MaxParkingSec;       // 最长停车时间  
  u32  Parking_currentcnt;       // 停车实时计数器 
}TIRED_DOOR;
//-------- 上报间隔相关  ---------
typedef struct _SEND_DUR
{                                       //  单位: s
  u32  Heart_Dur;     //  终端心跳包发送间隔
  u16  Heart_SDCnter; // 心跳包计数器
  u8   Heart_SDFlag;   // 心跳包发送标志位
  //------------------
  u32  TCP_ACK_Dur;   //  TCP 消息应答超时
  u16  TCP_ACK_DurCnter;// TCP 计数器
  u8   TCP_SD_state;    // 发送一包后置1.
  //------------------
  u32  TCP_ReSD_Num;  //  TCP 重传次数
  u16  TCP_ReSD_cnter; // 发送计数器    
  //------------------
  u32  UDP_ACK_Dur;   // UDP消息应答超时
  u32  UDP_ReSD_Num;  // UDP 重传次数 
  
  u32  NoDrvLogin_Dur;  //  驾驶员未登录汇报时间间隔
 // u16  NoLogin_SDcnter;   //  发送计时器
  //------------------	
  u32  Sleep_Dur;     //  休眠时候上报的时间间隔
  //u16  Sleep_SDcnter;   //  发送计数器
  //------------------
  u32  Emegence_Dur;  //  紧急报警时上报的时间间隔 
  //u16  Emegence_SDcnter;
  //------------------
  u32  Default_Dur;   //  缺省时间汇报间隔   
  
  u32  SD_Delta_maxAngle; // 拐点补传角度   < 180

  u16  IllgleMovo_disttance; // 非法移动阈值   -- 补充协议要求添加
}SEND_DUR;
// ----------- 定距离上报相关  ---------
typedef struct _SEND_DIST
{                                     //   单位: m
  u32  Defalut_DistDelta; // 缺省定距离上报
  u32  NoDrvLogin_Dist;   // 驾驶员未登录 上博距离
  u32  Sleep_Dist;        // 休眠情况下 上报距离
  u32  Emergen_Dist;      // 紧急报警时 上报距离
} SEND_DIST;

//----------- 上报模式  ---------------
typedef struct _SEND_MODE
{ 
  
  u8 Send_strategy;  
  u8 DUR_TOTALMODE;     // -----   定时上报方式
  u8 Dur_DefaultMode;  
  u8 Dur_SleepMode;     
  u8 Dur_NologinMode;
  u8 Dur_EmegencMode;

  u8 DIST_TOTALMODE;    // ------   定距上报方式  
  u8 Dist_DefaultMode;
  u8 Dist_NoLoginMode;
  u8 Dist_SleepMode;
  u8 Dist_EmgenceMode;
  
}SEND_MODE;

//------- 实时位置跟踪 ----
typedef struct _REALTIME_LOCK
{
  u8  Lock_state;   //  实时跟踪启动标志位
  u16 Lock_Dur;     //  限定的上报时间间隔
  u32 Lock_KeepDur; //  实时上报持续时间
  u32 Lock_KeepCnter;  
  
}REALTIME_LOCK;

//------- 文本信息 --------
typedef struct _TEXT_INFO
{
  u8  TEXT_FLAG;          //  文本标志
  u8  TEXT_SD_FLAG;       // 发送标志位  
  u8  TEXT_Content[DFBakSize];  // 文本内容
}TEXT_INFO;

//----- 信息 ----
typedef struct _MSG_TEXT
{
  u8   TEXT_mOld;     //  最新的一条信息  写为1代表是最新的一条信息
  u8   TEXT_TYPE;     //  信息类型   1-8  中第几条
  u8   TEXT_LEN;      //  信息长度    
  u8   TEXT_STR[150]; //  信息内容
}MSG_TEXT;

//-----  提问 ------
typedef struct _CENTER_ASK
{
  u8  ASK_SdFlag; //  标志位           发给 TTS  1  ；   TTS 回来  2
  u16 ASK_floatID; // 提问流水号
  u8  ASK_infolen;// 信息长度  
  u8  ASK_answerID;    // 回复ID
  u8  ASK_info[60];//  信息内容
  u8  ASK_answer[140];  // 候选答案  
  u8  ASK_disp_Enable;// 小屏显示使能
}CENTRE_ASK;


//------- 车辆控制  --------
typedef struct _VEHICLE_CONTROL
{
  u8   Control_Flag;  // 控制 标志
  u16  CMD_FloatID;   // 下发应答的消息流水号
  u8   ACK_SD_Flag;   // 回复发送标志位

}VEHICLE_CONTROL;

//--------  行车记录仪相关  -----
typedef struct  _RECODER
{
	u16  Float_ID;	   //  命令流水号
	u8	 CMD;	  //  数据采集 
	u8	 SD_Data_Flag; //  发送返回数返回标志
	u8	 CountStep;  //  发送数据需要一步一步发送 
	u32   timer;
	//--------- add on	5-4 
	u8	 Devide_Flag;//  需要分包上传标志位
	u16  Total_pkt_num;   // 分包总包数
	u16  Current_pkt_num; // 当前发送包数 从 1	开始
	u8	  fcs;
	u8	Error;//	0 idle	 1:   采集错误	 2:  设置错误 

    //---------  记录仪仪采集----------------------
    u8   Get_withDateFlag;  //  根据时间采集 
	u8   Get_startDate[6];  // 采取起始时间
	u8   Get_endDate[6];    //  采集结束时间  
	u16  Get_recNum;      //    获取的条目数   

}RECODER;
//------  Camera  --------
typedef struct _CAMERA
{
   u8 Channel_ID;     //  拍照通道
   u8 Operate_state;  //  拍照后处理状态    0 : 保存   1:  实时上传
   
}CAMERA;
//---------- 多媒体  ----------
typedef struct _MULTIMEDIA
{
  u32  Media_ID;           //   多媒体数据ID
  u8   Media_Type;         //   0:   图像    1 : 音频    2:  视频 
  u8   Media_CodeType;     //   编码格式  0 : JPEG  1:TIF  2:MP3  3:WAV  4: WMV
  u8   Event_Code;         //   事件编码  0: 平台下发指令  1: 定时动作  2 : 抢劫报警触发 3: 碰撞侧翻报警触发 其他保留
  u8   Media_Channel;      //   通道ID
  //----------------------
  u8   SD_Eventstate;          // 发送事件信息上传状态    0 表示空闲   1  表示处于发送状态        
  u8   SD_media_Flag;     // 发送没提事件信息标志位
  u8   SD_Data_Flag;      // 发送数据标志位
  u8   SD_timer;          // 发送定时器
  u8   MaxSd_counter;   // 最大发送次数  
  u8   Media_transmittingFlag;  // 多媒体传输数据状态  1: 多媒体传输前发送1包定位信息    2 :多媒体数据传输中  0:  未进行多媒体数据传输
  u16  Media_totalPacketNum;    // 多媒体总包数 
  u16  Media_currentPacketNum;  // 多媒体当前报数 
  //----------------------
  u8   RSD_State;     //  重传状态   0 : 重传没有启用   1 :  重传开始    2  : 表示顺序传完但是还没收到中心的重传命令
  u8   RSD_Timer;     //  传状态下的计数器   
  u8   RSD_Reader;    //  重传计数器当前数值 
  u8   RSD_total;     //  重传选项数目  
  
   
  u16	Media_ReSdList[125]; //  多媒体重传消息列表 
}MULTIMEDIA;   
//------  Voice Record 录音相关 ----
typedef struct _VOICE_RECODE
{
   u8   Operate_CMD; //  操作命令  0: 停止录音    1: 开始录音 
   u16  Voice_Dur;   //  录音时间， 0 表示一直录
   u8   Save_Flag;   //  0:  实时上传   1  保存
   u8   SampleRate;  //  音频采样率   0:8K   1: 11k  2:23k  3: 32K  , 其他保留
   
}VOICE_RECODE;

//----------   DataTrans  数据透传  -----------
typedef struct _DATATRANS
{
  u8 TYPE;             // 透传消息类型
  u8 Data_RxLen;       // 接收信息长度
  u8 Data_TxLen;       // 发送信息长度 
  u8 DataRx[10];      // 透传信息内容接收
  u8 Data_Tx[20];     // 透传信息内容发送  
  u8 Tx_Wr;  
  
}DATATRANS;   
//------  进出区域报警 -------------------
typedef struct _INOUT
{
  u8  TYPE; //   1 圆形区域 2 矩形区域 3 多边形区域 4 路线
  u32 ID;   //    区域路线ID
  u8  InOutState; //  0 进  1  出   
}INOUT;
//-------  多媒体索引 -------------------
typedef struct _MEDIA_INDEX
{
  u8  Effective_Flag; // 存储数目
  u32  MediaID;  // 多媒体ID 
  u8  Type;     // 类型    0 图像   1  音频   2  视频
  u8  ID  ;      // 通道
  u8  EventCode; // 事件编码    0  平台下发命令   1 定时动作   2  抢劫报警触发 3  碰撞触发
  u8  FileName[12];  // 文件名
  u8  PosInfo[28];//位置信息  

}MEDIA_INDEX;
//----------- 车门开关拍照相关 ------------
typedef struct _DoorCamra
{  
  u8	  currentState;  // 当前状态 	
  u8	  BakState;   // 存储状态
} DOORCamera;


typedef struct   TGPSInfo_GPRS
{
   //---------------  河北天地通协议  ---------------------------
    u8		Date[3];	// 年月日(HEX)
	u8		Time[3];	// 十分秒(HEX)
	u8		Latitude[4];    //纬度	  N/S
	u8		Longitude[4];   //经度	 W/E 
} T_GPS_Info_GPRS;  

typedef struct TIRDdrv
{  
  u32	ACC_ONstate_counter;	 //  ACC 开 计数器
  u32	ACC_Offstate_counter;	  //  ACC 关 计数器
  u8    Tireddrv_status;         // 是否在疲劳驾驶状态下    疲劳的情况下为1 ，疲劳触发结束时为2 ，空闲时为0
  u8    Tgvoice_play;            // 使能语音播报提示
  u8    voicePly_timer;          // 语音播报定时器
  u8    voicePly_counter;        // 语音播报次数  ，只播放10次
  u8    start_time[6];           // 疲劳驾驶起始时间
  u8    end_time[6];             // 疲劳驾驶结束时间
  u8    Flag;                    // ACCon =1  ,ACC_off=0; ACC关保持ACCon=1，直到休息过后清 0    
  u8    Status_TiredwhRst;    //  当复位时 疲劳驾驶的状态   0 :停车  1:停车但没触发 2:触发了还没结束                              

  
  u32   PreWarn_Dur;  //疲劳驾驶预报警
  u32   CurrentDay_DriveTimer_0;         //  上报策略为  0 时 
  u32   CurrentDay_DriveTimer_Driver_1;  // 上报策略为1 时
  u32   CurrentDay_DriveTimer_Driver_2; //  上报策略为1 时 
}TID_DRV; 

typedef struct _DetachPKG
{
    u16  Original_floatID;  //  原始消息流水号
    u8    NumOf_Resend;   //  重传包序号
    u16  List_Resend[50];  //重传包ID 列表   

}DETACH_PKG;

typedef struct  _SET_QRY
{
    u8    Num_pram;//   参数个数
    u32  List_pram[90];//  cansh ID   
}SET_QRY;

typedef struct _HUMAN_CONFIRM_WARN
{
   u16 Warn_FloatID;// 报警消息流水号
   u32 ConfirmWarnType; // 人工确认报警类型

}HUMAN_CONFIRM_WARN;

//  ISP_BD add
typedef struct _ISP_BD
{
   u8     ISP_running;//  远程下载进行中
   u16    ISP_runTimer;// 进行中 计时器
   u16   Total_PacketNum;//  总包数
   u16   CurrentPacket_Num;//  当前报数
   u16   PacketSizeGet; 
   u32   PacketRX_wr;//  接收数据累加长度
   u8     Update_Type;  // 终端数据类型
   u8  ProductID[5];// 制造商编号
   u8  Version_len; // 版本号长度
   u8  VersionStr[20];//  版本号
   u32  Content_len;// 升级包长度
   u8   ContentData[1024]; // 数据包内容
   u8   ISP_resualt;  //升级结果
   u8   ISP_status_Read;  // 上电读取 DF 状态
}ISP_BD;


/*
          APP   级别 的应用    
*/

typedef struct  _SYSConfig           //  name:  config
{
    u16     Version_ID ;   //  系统版本IDE
    u8       APN_str[40];  //   接入点名称
    u8       IP_Main[4];   //   主IP 地址 
    u16      Port_main;   //   主socket  端口
    u8       IP_Aux[4];   //   辅IP 地址 
    u16      Port_Aux;    //   辅socket  端口
    u8       DNSR[50];    //  DNSR 1  域名解析
    u8       DNSR_Aux[50]; //   DNSR2   域名解析     
	//  LINK2  Setting
	u8		Link2_IP[4]; 
	u16 	Link2_Port; 	   	
    u16     AccOn_Dur;   //  ACC 开  上报间隔
    u16     AccOff_Dur;  //   ACC 关  上报间隔
    u8      TriggerSDsatus;  //  传感器触发上报状态	
    //-------- BD add-----------------
    u8      BD_IC_main_IP[4];  //  IC卡认证主服务器IP 地址
    u32     BD_IC_TCP_port;     //  IC 卡认证主服务器TCP 端口
    u32     BD_IC_UDP_port;    //  IC 卡认证主服务器 UDP 端口
    u8      BD_IC_Aux_IP[4];   //  IC 卡备用服务器IP 地址，端口同主的一样   	
    u8      BD_IC_DNSR[50];    //  DNSR 1  域名解析
    u8      BD_IC_DNSR_Aux[50]; //   DNSR2   域名解析     

} SYS_CONF;


typedef struct  _JT808Config   //name:  jt808
{
    u8       LOAD_STATE  ;           //选中车辆的负载状态标志   1:空车   2:半空   3:重车
    u8       ConfirmCode[20];       // 鉴权码
    u8       Regsiter_Status;        //  注册状态
    u8       LISTEN_Num[30];       //  监听号码
    u8       SMS_RXNum[15];        //  中心短息号码
    u32      Vech_Character_Value;   //  特征系数 
    u8       PositionSd_Stratage;    //  位置汇报策略  

    u8       FirstSetupDate[6];           //  初次安装时间
    u8       DeviceOnlyID[35];           //   行车记录仪的唯一ID
    u16      Msg_Float_ID;                 //   消息流水号

    u32      Distance_m_u32;            //  行驶记录仪行驶里程  单位: 米
    u32      DayStartDistance_32;     //  每天的起始里程数目

    u32     Speed_warn_MAX;           //  速度报警门限
    u32     Spd_Exd_LimitSeconds;  //  超速报警持续时间门限
    u8       Speed_GetType;             //  记录仪获取速度的方式  00  gps取速度  01 表示从传感器去速度 
    u8       DF_K_adjustState; // 特征系数自动校准状态说明  1:自动校准过    0:尚未自动校准   

   	
    u8       OutGPS_Flag;     //  0  默认  1  接外部有源天线 
    u8       concuss_step;    //------add by  xijing
    u8       relay_flag;      // 继电器开关状态
    u8       Auto_ATA_flag;   // 自动接听标志位使能    
	
    //-----------2013   BD add  -------------------------------------
    u16     BD_CycleRadius_DoorValue;    //  电子围栏半径(非法移动阈值)，单位米
    u16     BD_MaxSpd_preWarnValue;    //  超速报警预警差值，单位 1/10  Km/h
    u16     BD_TiredDrv_preWarnValue;  // 疲劳驾驶预警差值，单位 秒
    u16     BD_Collision_Setting;     // 碰撞参数报警设置    bit 7-bit 0   碰撞时间 单位 4ms   、 bit15-8 碰撞加速度 0.1g  0-79  默认10
    u16     BD_Laydown_Setting;    // 侧翻报警参数设置     侧翻角度 单位 1 度 ，默认30度

    
    //------  摄像头 相关设置 ------------------------------
    u32  BD_CameraTakeByTime_Settings;   // 摄像头定时拍照开关    0 不允许1 允许 表13
    u32  BD_CameraTakeByDistance_Settings;  //  摄像头定距离拍照控制位
    u8   Close_CommunicateFlag;   // 关闭通信标志位    
    u16     BD_GNSS;
	

    BD_EXTEND     BD_EXT;  //  北斗相关 CAN GNSS 设置	
    SEND_DIST   DISTANCE;	       // 定距离回传相关   16 Bytes
    SEND_DUR    DURATION;		//	发送间隔相关
    SEND_MODE  SD_MODE;         // 信息上报模式    发送方式   
    
     //--------  实时上报 ---------  
    REALTIME_LOCK    RT_LOCK;     // 实时跟踪      

      //----------  Standard Version -----------------------
    STD_VER               StdVersion;   // 标准国家版本   
    DRV_INFO             Driver_Info;        //  驾驶员信息
	
}JT808_CONF;

    //------------- 疲劳驾驶相关----------------

typedef  struct   _TIRED_Config    // name: tired
{
   TID_DRV         Tired_drive; 
   TIRED_DOOR   TiredDoor;  
} TIRED_CONF;

//------ 事件  -------
typedef struct _EVENT               //  name: event
{
  u8 Event_ID;   //  事件ID
  u8 Event_Len;  //  事件长度
  u8 Event_Effective; //  事件是否有效，   1 为要显示  0     
  u8 Event_Str[20];  //  事件内容
}EVENT; 

//----- 信息 ----
typedef struct _MSG_BROADCAST    // name: msg_broadcast
{
  u8   INFO_TYPE;     //  信息类型
  u16  INFO_LEN;      //  信息长度
  u8   INFO_PlyCancel; // 点播/取消标志      0 取消  1  点播
  u8   INFO_SDFlag;    //  发送标志位
  u8   INFO_Effective; //  显示是否有效   1 显示有效    0  显示无效     
  u8   INFO_STR[30];  //  信息内容
}MSG_BRODCAST;


//------ 电话本 -----
typedef struct _PHONE_BOOK            // name: phonebook
{
  u8 CALL_TYPE ;    // 呼入类型  1 呼入 2 呼出 3 呼入/呼出
  u8 NumLen;        // 号码长度   
  u8 UserLen;       // 联系人长度
  u8 Effective_Flag;// 有效标志位   无效 0 ，有效  1
  u8 NumberStr[20]; // 电话号码
  u8 UserStr[10];   // 联系人名称  GBK 编码
}PHONE_BOOK;

//------  圆形围栏  -----------
typedef struct _CIRCLE_RAIL           // name: Rail_cycle
{
  //----------------------- 
  u32  Area_ID;        //  区域ID
  u16  Area_attribute; //  区域属性
  u32  Center_Latitude;//  中心纬度
  u32  Center_Longitude;//  中心经度
  u32  Radius;          //  半径
  u16  MaxSpd;          //  最高速度
  u8   KeepDur;         //  超速持续时间  
  //------------------------
  u8   Rail_Num;        // 当前围栏数目
  u8   Effective_flag;  // 当前围栏是否 停止    0 : 未启用   1 :  启用  
  u8   StartTimeBCD[6]; //  开始时间
  u8   EndTimeBCD[6];   //  结束时间

}CIRCLE_RAIL;
//------  矩形围栏  -----------
typedef struct _RECT_RAIL                // name:  Rail_rect
{
  //-------------------
  u32  Area_ID;        //  区域ID
  u16  Area_attribute; //  区域属性
  u32  LeftUp_Latitude;//  左上纬度
  u32  LeftUp_Longitude;//  左上经度
  u32  RightDown_Latitude;//  右下纬度
  u32  RightDown_Longitude;//  右下经度     
  u16  MaxSpd;          //  最高速度
  u8   KeepDur;         //  超速持续时间  
  //------------------  
  u8   Rail_Num;        // 当前围栏数目
  u8   Effective_flag;  // 当前围栏是否 停止    0 : 未启用   1 :  启用  
  u8   StartTimeBCD[6]; //  开始时间
  u8   EndTimeBCD[6];   //  结束时间

}RECT_RAIL;
//------  多边形围栏  --------
typedef struct _POLYGEN_RAIL             //name: Rail_polygen       
{
   //----------------
  u32  Area_ID;        //  区域ID
  u16  Area_attribute; //  区域属性
  u16  MaxSpd;          //  最高速度
  u8   KeepDur;         //  超速持续时间  
  u16  Acme_Num;        //  顶点数目  至少3个
  u32  Acme1_Latitude; //  顶点1纬度
  u32  Acme1_Longitude;//  顶点1经度
  u32  Acme2_Latitude; //  顶点2纬度
  u32  Acme2_Longitude;//  顶点2经度
  u32  Acme3_Latitude; //  顶点3纬度
  u32  Acme3_Longitude;//  顶点3经度
   //---------------- 
   u8	Rail_Num;		 // 当前围栏数目
   u8	Effective_flag;  // 当前围栏是否 停止	 0 : 未启用   1 :  启用   
   u8	StartTimeBCD[6]; //  开始时间
   u8	EndTimeBCD[6];	 //  结束时间

}POLYGEN_RAIL; 

// -------  路线中拐点  ------
typedef struct _POINT                               //  name:  turn_point      
{
  u32  POINT_ID;        //  区域ID
  u32  Line_ID;        //   路段ID
  u32  POINT_Latitude;//   拐点纬度
  u32  POINT_Longitude;//  拐点经度
  u8   Width;          //  宽度
  u8   Atribute;       //  属性
  u16  TooLongValue;   //  路段行驶过长阈值
  u16  TooLessValue;   //  路段行驶不足阈值	
  u16  MaxSpd;          //  最高速度
  u8   KeepDur;         //  超速持续时间  
}POINT; 
//----------  路线设置  -------
typedef  struct _ROUTE                              // name: route_line
{
 //--------------------------
  u32  Route_ID;        //  线路ID
  u16  Route_attribute; //  线路属性  
  u16  Points_Num;      //  拐点数目  最小3
  u8   Effective_flag;  //  有效状态
  u8   StartTimeBCD[6]; //  开始时间
  u8   EndTimeBCD[6];   //  结束时间
  POINT RoutePoints[3];  //  拐点信息 
 //--------------------------
}ROUTE;

//--------------每分钟的平均速度 
typedef  struct AvrgMintSpeed
{
  u8 datetime[6]; //current
  u8 datetime_Bak[6];//Bak
  u8 avgrspd[60];
  u8 saveFlag;  
}Avrg_MintSpeed;
//----------  事故疑点数据
typedef struct DoubtTYPE
{
  u8  DOUBTspeed;   // 0x00-0xFA  KM/h
  u8  DOUBTstatus;  // status 
} DOUBT_TYPE;



//-------------------- Media_SD_State 
typedef struct
{
  u8  SD_flag;        // 图片数据发送标志
  u8  photo_sending; //  处于照片发送状态
  u8  photo_sendTimeout; //   发送状态超时判断
  u16 SD_packetNum;  //  发送数据的包序号
  u16 Total_packetNum; // 拍照的总包数
  u8  Data_SD_counter; //  数据发送间隔    
  u16  Exeption_timer;   //  异常情况下定时器
  
}_Media_SD_state;


typedef union TIPAddr
{
	u8				ip8[4];	// access the ip as either 4 seperate bytes
	u32				ip32;	// ... or as a single 32-bit dword
} T_IP_Addr;



//---------  CAN ---------------------------
typedef struct _CAN_TRAN
{
    //-------  protocol variables
     u32  can1_sample_dur;  // can1 采集间隔  ms
     u32  can1_trans_dur;  // can1  上报间隔 s
     u32  can1_enable_get;  // 使能接收


     u32  can2_sample_dur;  // can2 采集间隔  ms
     u32  can2_trans_dur;  // can2  上报间隔 s
     
     
      u8      canid_1[8];// 原始设置
      u32    canid_1_Filter_ID;  // 接收判断用
      u32    canid_2_NotGetID;  //   不采集ID   
      u8      canid_1_Rxbuf[512]; // 接收buffer  
      u32    canid_1_ID_RxBUF[128];  // 接收ID  
      u8      canid_1_Sdbuf[512]; // 发送用buffer 
      u32    canid_1_ID_SdBUF[128];  // 接收ID    
      u16    canid_1_SdWr; // 写buffer下标
      u16    canid_1_RxWr; // 写buffer下标
      u8      canid_1_ext_state; // 扩展帧状态
      u32    canid_1_sample_dur;  //该ID 的采集间隔
      u8      canid_ID_enableGet;//   使能该ID 采集

      //------- system variables  
      u16   canid_timer;  //定时器
      u8     canid_0705_sdFlag;// 发送标志位	  

 }CAN_TRAN;

// ------  车辆信息单独了 ---------------
extern  VechINFO             Vechicle_Info;     //  车辆信息  
extern  VechINFO     Vechicle_Info_BAK;  //  车辆信息 BAK
extern  VechINFO     Vechicle_info_BAK2; //  车辆信息BAK2      
extern  u8           Login_Menu_Flag;	   //	登陆界面 标志位    



//------- 北斗扩展协议  ------------
extern BD_EXTEND     BD_EXT;     //  北斗扩展协议
extern DETACH_PKG   Detach_PKG; // 分包重传相关
extern SET_QRY         Setting_Qry; //  终端参数查询
extern u32         CMD_U32ID; 
extern PRODUCT_ATTRIBUTE    ProductAttribute;// 终端属性
extern HUMAN_CONFIRM_WARN   HumanConfirmWarn;// 人工确认报警 

//-----  ISP    远程下载相关 -------
extern ISP_BD  BD_ISP; //  BD   升级包     


//===============================================================================================
extern _Media_SD_state Photo_sdState;   //  图片发送状态
extern _Media_SD_state Sound_sdState;	//声音发送  
extern _Media_SD_state Video_sdState;	//视频发送
//------ Photo -----
extern  u32 PicFileSize; // 图片文件大小  
extern  u8  PictureName[40];       



//------  voice -----



//------  video  --------



//-------  行车记录仪相关extern BD_EXTEND     BD_EXT;     //  北斗扩展协议

extern Avrg_MintSpeed  Avrgspd_Mint; 
extern u32         PerMinSpdTotal; //记录每分钟速度总数  
extern u8          avgspd_Mint_Wr;       // 填写每分钟平均速度记录下标
extern u8          avgspd_Sec_Wr;       // 填写每秒钟平均速度记录下标
extern u8          avgWriteOver;   // 写溢出标志位
extern u8          AspdCounter;    // 每分钟速度有效报数计数器 
extern  u8         Vehicle_sensor; // 车辆传感器状态   0.2s  查询一次  
extern  u8		   Vehicle_sensor_BAK; // 车辆传感器状态	0.2s  查询一次	

extern  DOUBT_TYPE        Sensor_buf[100];// 20s 状态记录   
extern 	u8		   save_sensorCounter,sensor_writeOverFlag;  
extern u16  Delta_1s_Plus;


extern u8 TextInforCounter;//文本信息条数
extern DOORCamera   DoorOpen;    //  开关车门拍照
extern MEDIA_INDEX  MediaIndex;  // 多媒体信息
extern INOUT        InOut_Object;  // 进出围栏状态 
extern DATATRANS    DataTrans;     // 数据信息透传
extern MULTIMEDIA   MediaObj;      // 多媒体信息 
extern VOICE_RECODE VoiceRec_Obj;   //  录音功能  
extern CAMERA       Camera_Obj;     //  中心拍照相关
extern RECODER      Recode_Obj;     // 行车记录仪
extern POINT        POINT_Obj;      // 路线的拐点
extern ROUTE        ROUTE_Obj;      // 路线相关
extern POLYGEN_RAIL Rail_Polygen;   // 多边形围栏
extern RECT_RAIL    Rail_Rectangle; // 矩形围栏
extern RECT_RAIL    Rail_Rectangle_multi[8]; // 矩形围栏
extern CIRCLE_RAIL  Rail_Cycle_multi[8];     // 圆形围栏 
extern CIRCLE_RAIL  Rail_Cycle;  // 圆形围栏
extern VEHICLE_CONTROL Vech_Control; //  车辆控制
extern PHONE_BOOK    PhoneBook;  //  电话本
extern PHONE_BOOK	 PhoneBook_8[8];
extern MSG_BRODCAST  MSG_BroadCast_Obj;    // 信息点播  
extern MSG_BRODCAST   MSG_Obj_8[8];  // 信息点播  
extern CENTRE_ASK     ASK_Centre;  // 中心提问
extern EVENT          EventObj;    // 事件
extern EVENT          EventObj_8[8]; // 事件  
//-------文本信息-------
extern MSG_TEXT       TEXT_Obj;
extern MSG_TEXT       TEXT_Obj_8[8],TEXT_Obj_8bak[8];

extern TEXT_INFO      TextInfo;    // 文本信息下发

extern SPD_EXP    speed_Exd;      // 超速报警
extern DevRegst   DEV_regist;  // 注册
extern DevLOGIN   DEV_Login;   //  鉴权


extern NANDSVFlag   NandsaveFlg;
extern A_AckFlag    Adata_ACKflag;
extern TCP_ACKFlag  SD_ACKflag;  



extern u32           SubCMD_8103H;            //  02 H命令 设置记录仪安装参数回复 子命令 
extern u32         SubCMD_FF01H;            //  FF02 北斗信息扩展
extern u32         SubCMD_FF03H;     //  FF03  设置扩展终端参数设置1

extern u8          SubCMD_10H;            //  10H   设置记录仪定位告警参数
extern u8	       OutGPS_Flag; // 0  默认  1  接外部有源天线
extern u8	       Spd_senor_Null;  // 手动传感器速度为0
extern u32         Centre_DoubtRead;     //  中心读取事故疑点数据的读字段
extern u32         Centre_DoubtTotal;    //  中心读取事故疑点的总字段  


extern u8 		   FCS_GPS_UDP;						//UDP 数据异或和
extern u8          FCS_RX_UDP;                       // UDP 数据接收校验 
extern u8          Centre_IP_modify;            //  中修改IP了 
extern u8          IP_change_counter;           //   中心修改IP 计数器
extern u8		   Down_Elec_Flag;				 //   断油断电使能标志位 


extern ALIGN(RT_ALIGN_SIZE) u8          GPRS_info[900]; 
extern u16         GPRS_infoWr_Tx;


//------ phone
extern u8       CallState; // 通话状态

//   -------  CAN BD new  --------------
extern CAN_TRAN     CAN_trans;



extern  GPRMC_PRO GPRMC_Funs;


//--------  GPS prototcol------------------------------------------------------------------------------------------------
extern u8   UDP_dataPacket_flag;			   /*V	   0X03 	 ;		   A	  0X02*/
extern u8   DispContent;   // 发送时是否显示数据内容  
extern u8  Camera_Number;



extern u8	GPS_getfirst; 		 //  首次有经纬度 
extern u8	HDOP_value;		 //  Hdop 数值	  
extern u8   Satelite_num;   // 卫星颗数

extern u8   CurrentTime[3];
extern u8   BakTime[3];
extern u8   Sdgps_Time[3];  // GPS 发送 时间记录  


extern  ALIGN(RT_ALIGN_SIZE) u8  UDP_HEX_Rx[1024];    // EM310 接收内容hex     

extern u16 UDP_hexRx_len;    // hex 内容 长度
extern u16 UDP_DecodeHex_Len;// UDP接收后808 解码后的数据长度   



extern GPS_RMC  GPRMC;                   // GPMC格式  
//---------- 808 协议 -----------------------------------------------------------------------------------------------
extern 	u16 	        GPS_Hight;			   //	808协议-> 高程	 m 
extern  u16		        Speed_gps;    // 通过GPS计算出来的速度 km/h   
extern 	u16	            Spd_Using;								  //  当前使用的速度 0.1 km/h
extern  u16			    GPS_direction;							  //  GPS方向		 单位2度       

//---------- 用GPS校准特征系数相关 ----------------------------
extern u8		Speed_area; // 校验K值范围
extern u16		Speed_cacu; // 通过K值计算出来的速度 
extern u16 	    Spd_adjust_counter; // 确保匀速状态计数器 
extern u16      Former_DeltaPlus[K_adjust_Duration]; // 前几秒的脉冲数 
extern u8       Former_gpsSpd[K_adjust_Duration];// 前几秒的速度        
 //------- 车辆负载状态 ---------------
extern u8      CarLoadState_Flag;//选中车辆状态的标志   1:空车   2:半空   3:重车
 //------- 多媒体信息类型---------------
extern u8  Multimedia_Flag;//需要上传的多媒体信息类型	1:视频	 2:音频   3:图像
extern u8  SpxBuf[SpxBuf_Size];  
extern u16 Spx_Wr,Spx_Rd; 
extern u8	Duomeiti_sdFlag; 

//------- 录音开始或者结束---------------
extern u8  Recor_Flag; //  1:录音开始   2:录音结束  


// ---- 拐点 -----
extern u16  Inflexion_Current;      
extern u16  Inflexion_Bak;       
extern u16  Inflexion_chgcnter; //变化计数器
extern u16  InflexLarge_or_Small;	    // 判断curent 和 Bak 大小	0 equql  1 large  2 small
extern u16  InflexDelta_Accumulate;	//	差值累计 


// ----休眠状态  ------------
extern u8   SleepState;  //   0  不休眠ACC on        1  休眠Acc Off     
extern u8	SleepConfigFlag; //  休眠时发送鉴权标志位

//---- 固定文件大小 --- 
extern u32 mp3_fsize;   
extern u32 wmv_fsize;    
extern u8  mp3_sendstate;
extern u8  wmv_sendstate;



//---------- 河北天地通协议 ------------------------------------------------------------------------------------------	
extern  u8		SIM_code[6];							   // 要发送的IMSI	号码
extern  u8		IMSI_CODE[15];							//SIM 卡的IMSI 号码
extern  u8		Warn_Status[4]; // 报警状态信息
extern 	u8	    Warn_MaskWord[4];	//	报警屏蔽字	  
extern  u8      Text_MaskWord[4];	 
extern  u8      Key_MaskWord[4]; //   关键字屏蔽字    


extern  u8		Car_Status[4];  // 车辆状态信息   
extern T_GPS_Info_GPRS 	 Gps_Gprs;	 
extern T_GPS_Info_GPRS	 Temp_Gps_Gprs;
extern u8	    A_time[6]; // 定位时刻的时间  
// 定位时刻的时间
extern u8       ReadPhotoPageTotal;
extern u8       SendPHPacketFlag; ////收到中心启动接收下一个block时置位

//-------- 紧急报警 --------
extern u8		warn_flag;		  
extern u8		f_Exigent_warning;//0;     //脚动 紧急报警装置 (INT0 PD0)
extern u8		Send_warn_times;     //   设备向中心上报报警次数，最大3 次
extern u32  	    fTimer3s_warncount;


extern POSIT Posit[60];                //  每分钟位置信息存储
extern u8    PosSaveFlag;             //  存储Pos 状态位

extern u8   Vehicle_RunStatus;    //  bit 0: ACC 开 关             1 开  0关
                                  //  bit 1: 通过速度传感器感知    1 表示行驶  0 表示停止   
                                  //  bit 2: 通过gps速度感知       1 表示行驶  0 表示停止  
extern u8   Status_TiredwhRst;    //  当复位时 疲劳驾驶的状态   0 :停车  1:停车但没触发 2:触发了还没结束                              


extern u8    SpxSrcName[13];
extern u32   SrcFileSize,DestFilesize,SrcFile_read; 
extern u8	 SleepCounter;  

extern u16   DebugSpd;	//调试用GPS速度      
extern u8	 MMedia2_Flag;
extern u8     Send_Rdy4ok;

//-------------    不同北斗模块设置  ----
//#ifdef HC_595_CONTROL
//---------74CH595  Q5   control Power----
extern u8   Print_power_Q5_enable;     
extern u8   Buzzer_on_Q7_enable; 
//#endif




//==================================================================================================
// 第一部分 :   以下是GPS 解析转换相关函数 
//==================================================================================================

extern void Time_pro(u8 *tmpinfo, u8 hour, u8 min , u8 sec);
extern void Status_pro(u8 *tmpinfo);
extern void Latitude_pro(u8 *tmpinfo);
extern void Lat_NS_pro(u8 *tmpinfo);
extern void Longitude_pro(u8 *tmpinfo);
extern void Long_WE_pro(u8 *tmpinfo);
extern void Speed_pro(u8 *tmpinfo,u8 Invalue,u8 Point);
extern void Direction_pro(u8 *tmpinfo,u8 Invalue,u8 Point);
extern void Date_pro(u8 *tmpinfo,u8 fDateModify, u8 hour, u8 min , u8 sec); 
extern void HDop_pro(u8 *tmpinfo); 
extern void   GPS_Delta_DurPro(void);    //告GPS 触发上报处理函数 

//==================================================================================================
// 第二部分 :   以下是外部传感器状态监测
//==================================================================================================
/*    
     -----------------------------
     2.1   和协议相关的功能函数
     ----------------------------- 
*/
extern int IP_Str(char *buf, u32 IP);
extern void strtrim(u8 *s, u8 c);
extern int str2ip(char *buf, u8 *ip);
/*    
     -----------------------------
     2.2 输入管脚状态监测
     ----------------------------- 
*/

/*    
     -----------------------------
    2.3  控制输出
     ----------------------------- 
*/
extern void  Enable_Relay(void);
extern void  Disable_Relay(void);


/*    
     -----------------------------
    2.4  不同协议状态寄存器变化
     ----------------------------- 
*/

extern void StatusReg_WARN_Enable(void);
extern void StatusReg_WARN_Clear(void);
extern void StatusReg_ACC_ON(void);
extern void StatusReg_ACC_OFF(void);
extern void StatusReg_POWER_CUT(void);
extern void StatusReg_POWER_NORMAL(void);
extern void StatusReg_GPS_A(void);
extern void StatusReg_GPS_V(void);
extern void StatusReg_SPD_WARN(void);
extern void StatusReg_SPD_NORMAL(void);
extern void StatusReg_Relay_Cut(void);
extern void StatusReg_Relay_Normal(void);
extern void StatusReg_Default(void);  



	





//==================================================================================================
// 第三部分 :   以下是GPRS无线传输相关协议
//==================================================================================================

extern  u8  Do_SendGPSReport_GPRS(void);   
extern void  Save_GPS(void);    
extern u8    Stuff_DevCommmonACK_0001H(void);        
extern u8    Stuff_RegisterPacket_0100H(u8  LinkNum);  
extern u8    Stuff_DeviceDeregister_0101H(void);      
extern u8    Stuff_DeviceHeartPacket_0002H(void);
extern u8    Stuff_DevLogin_0102H(void);      
extern u8    Stuff_Normal_Data_0200H(void); 
extern u8    Stuff_Current_Data_0200H(void);   //  发送即时数据不存储到存储器中 
extern u8    Stuff_Current_Data_0201H(void);   //   位置信息查询回应
extern u8    Stuff_SettingPram_0104H(void); 
extern u8    Stuff_EventACK_0301H(void);   
extern u8    Stuff_ASKACK_0302H(void);    
extern u8	 Stuff_MSGACK_0303H(void);	  
extern u8    Stuff_ControlACK_0500H(void);    //   车辆控制应答
extern u8    Stuff_RecoderACK_0700H_Error(void);   
extern u8    Stuff_RecoderACK_0700H( u8 PaketType);   //   行车记录仪数据上传
extern u8    Stuff_MultiMedia_InfoSD_0800H(void);      
extern u8    Stuff_MultiMedia_Data_0801H(void);    
extern u8    Stuff_MultiMedia_IndexAck_0802H(void);  
extern u8    Stuff_CentreTakeACK_BD_0805H( void );	
extern u8    Stuff_DataTransTx_0900H(void);     
extern u8    Stuff_DriverInfoSD_0702H(void);  
extern u8    Stuff_Worklist_0701H(void); 
extern u8    Stuff_ISP_Resualt_BD_0108H(void);
extern u8    Stuff_BatchDataTrans_BD_0704H(void);     
extern u8    Stuff_CANDataTrans_BD_0705H(void);
extern u8    Sub_stuff_AppointedPram_0106(void);  

extern void  ISP_file_Check(void);
extern unsigned short int  File_CRC_Get(void);  
extern u16   Instr_2_GBK(u8 *SrcINstr, u16 Inlen, u8* DstOutstr );  
//  河北天地通多媒体事件信息上传报文应答不好，所以单独处理  
extern  void Multimedia_0800H_ACK_process(void);   


extern void delay_us(u16 j);
extern void delay_ms(u16 j);


#if 0
extern void  Stuff_ISP_ack(void);      // ISP  
extern void  Stuff_ISPinfo_ack(void);      // ISP  
#endif

extern  void  Media_Start_Init( u8  MdType , u8  MdCodeType);
extern  void  Media_Clear_State(void);
extern  void  Media_Timer(void); 
extern  void  Media_RSdMode_Timer(void);
extern  void  Media_Timer_Service(void); 
extern  void  Meida_Trans_Exception(void);

extern void   Photo_send_start(u16 Numpic);  
extern  void  DataTrans_Init(void);
extern void  TIRED_Drive_Init(void);

#ifdef REC_VOICE_ENABLE
extern u8     Sound_send_start(void);
#endif

extern void  TCP_RX_Process(u8  LinkNum);  
extern u16    AsciiToGb(u8 *dec,u8 InstrLen,u8 *scr);
extern void  Time2BCD(u8 *dest);     



extern void SpeedWarnJudge(void);
extern void Process_GPRSIN_DeviceData(u8 *instr, u16  infolen);


extern void  Sleep_Mode_ConfigEnter(void); 
extern void  Sleep_Mode_ConfigExit(void); 

//extern u16   WaveFile_EncodeHeader(u32 inFilesize ,u8* DestStr);   
extern void  CycleRail_Judge(u8* LatiStr,u8* LongiStr);
extern void  RectangleRail_Judge(u8* LatiStr,u8* LongiStr);       
extern u8    Save_MediaIndex( u8 type, u8* name, u8 ID,u8 Evencode);  
extern void  vin_set(u8 *instr);
extern void Tired_Check(void);  
extern void OutPrint_HEX(u8 * Descrip, u8 *instr, u16 inlen); 
//extern void  Sound_SaveStart(void);
//extern void  Sound_SaveEnd(void); 
extern void    DoorCameraInit(void);
extern void    MSG_BroadCast_Read(void); 
extern u8      Time_FastJudge(void);   
extern void   CAN_struct_init(void);
extern void   CAN_send_timer(void);
extern void   redial(void);
extern void   dnsr_main(u8*instr);
extern void   dnsr_aux(u8*instr);
extern void   port_main(u8 *instr); 
extern void   port_aux(u8 *instr); 
extern void   buzzer_onoff(u8 in); 

extern void  Photo_send_TimeOut(void); 
extern void  Photo_send_end(void);
#ifdef REC_VOICE_ENABLE
extern void  Sound_send_end(void); 
#endif 
extern void  print_power(u8*instr); 
extern void  dur(u8 *content); 
extern void provinceid(u8 *strin);
extern void cityid(u8 *strin);  
extern void  ata_enable(u8 value);
extern void  plus_num(u32 value);
extern void  spd_type(int  in); 
extern void  adjust_ok(int in); 



//extern u8  RecordSerial_output_Str(const char *fmt,...); 

//==================================================================================================
// 第四部分 :   808  相关存储处理  
//==================================================================================================

extern void  JT808_Related_Save_Process(void);
extern void  Spd_Exp_Wr(void);
extern void Fly_Swing_judge_TJ(void); 




#endif
