/*
     APP_485.H 
*/

#ifndef   _APP_485
#define  _APP_485

#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"

#include  <stdlib.h>
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"


//---------  拍照返回状态  DB  ----------
#define  Photo_transIdle  0
#define  trans_fail       1
#define  trans_succeed    2
#define  takePhoto_fail   3

//----------- MultiTake State ----------
#define Take_idle      0
#define Take_Success   1
#define Take_Fail      2


//------  485  Rx State -------- 
#define    IDLE_485           0
#define    CAMERA_Related     1
#define    LARGE_LCD           3



/*
                  Photo take Related                          
*/
//-------------------  Multi Take ------------------
 //  先各路都拍一遍记录下拍照成功与失败的结果，然后逐个上传
 #define   Max_CameraNum   4    

typedef struct
{
  u8 Taking;      // 多路摄像头在拍照              默认为   0   使能为   1
  u8 Transfering; // 图片传输中                   默认为   0    使能为   1
  u8 CurretCameraNum; //当前拍照的摄像头序号
  u8 TakeResult[Max_CameraNum];   // 每路摄像头拍照结果    默认初始化为  0     拍照成功为  1  拍照失败为 2
  u8 Take_retry;      // 单路拍照重试次数
  u8 Take_success_counter;  // 记录总共拍照成功了多少通道
}_MultiTake;  // 多路摄像头拍照操作寄存器

typedef struct 
{
	 u8     camera_running;     //拍照进行中标志
	 u16    timeoutCounter;     //操作超时计数器
	 u16	block_counter;	    //接收数据包的计数器
	 u8	    status;	  //  拍照过程状态
	 u8     OperateFlag;  // 拍照过程中操作状态标识 ， 给camera 发送则 set 1， camera 反应则清除  0
	 u8     create_Flag;  // 是否创建新图片文件标志 ，     1 : 需要创建新标志     0:   不需要创建
} Camera_state;

typedef struct  _VOICE_DEV
{
 u8  Work_State;  // 语音盒数据传输状态
 u8  CMD_Type;    //  0  停止采集   1: 采集中   2: 语音播报
 u8  Poll_Enble;  // 使能轮询标志
 u8  Sd_Timer;    // 发送定时器   
 u8  info_sdFlag; //  发送语音播报信息       1  播报信息  2 录音控制
 u16 Rec_Dur;     // 录音持续时间 
 u16 Rec_Counter; //  录音定时器
 u8  Rec_runFlag; // 是否一直录音
 u8  SaveOrNotFlag; // 是否保存 
 u8  Centre_RecordFlag; // 中心录音标志位
 u8  Voice_FileOperateFlag;//    语音数据文件操作标志位   0  为未进行文件操作  1 开始对文件进行操作
 u32 Voice_FileSize;//  文件大小
 u32 Voice_PageCounter;
 u8  FileName[20]; // 存储文件名称   
 u8  Play_info[100]; // 语音播报信息 
 u8  Voice_Reg[512]; // 语音数据Buffer
}VOICE_DEV;




extern  VOICE_DEV Dev_Voice;  
/*
                  拍照相关  
*/
 extern  _MultiTake     MultiTake;	  //  多路拍照状态位 
 extern u8  SingleCamera_TakeRetry; // 单路摄像头拍照时，重拍次数计数
 extern Camera_state CameraState; 
 extern   u8  SingleCamra_TakeResualt_BD;   // 单路拍照结果 
 extern  u8  Camera_Take_not_trans;   //  拍照不上传
 extern u8    TX_485const_Enable;   // 使能发送标志位  
 extern u8 	  last_package; // 拍照最后一包标识
 extern  Camera_state CameraState;  










//-----------------------------------------
extern void  Send_const485(u8  flag); 
extern u8     Camera_Take_Enable(void);
extern  u8    Check_MultiTakeResult_b4Trans(void);
extern void   Init_Camera(void);
extern void   End_Camera(void);
extern  u8    Start_Camera(u8  CameraNum);
extern  void  MultiTake_Start(void);
extern  void  MultiTake_End(void); 
extern u8     Camera_Take_Enable(void);
extern  void  Voice_Dev_Init(void);
extern  void  Voice_Dev_Rxprocess(void);
extern  void  Voice_Dev_Txprocess(void);  
extern  void  Camra_Take_Exception(void);
extern int str2ipport(char *buf, u8 *ip, u16 *port); 
extern void takephoto(u8* str);  



#endif 
