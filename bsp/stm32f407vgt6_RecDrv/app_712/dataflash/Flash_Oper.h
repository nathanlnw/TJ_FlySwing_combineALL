#ifndef Flash_Nand
#define  Flash_Nand
#include "App_moduleConfig.h"

//-----------------------------------------------------------------------------------------------------------------------------
//==============================================================================================================================
//   Add Type define
#define   Type_Idle                            0                     // 空闲   
#define   TYPE_CycleAdd                        1                     // 循环
#define   TYPE_PhotoAdd                        2                     // 图片 
#define   TYPE_TiredDrvAdd                     3                     // 疲劳驾驶
#define   TYPE_ExpSpdAdd                       4                     // 超速报警记录偏移地址
#define   TYPE_AvrgSpdAdd                      6                     // 每分钟平均速度
#define   TYPE_ErrorLogAdd                     7                     // 异常Log存储 
#define   TYPE_VechRecordAdd                   8                     // 正常存储记录偏移地址     
#define   TYPE_DoubtAdd                        9                     // 事故疑点偏移地址
#define   TYPE_AvrgSpdSecAdd                   10                    // 车辆单位分钟每秒平均速度记录地址
#define   TYPE_LogInAdd                        11                    // 登录信息记录地址
#define   TYPE_SettingChgAdd                   13                    // 参数修改记录地址   
#define   TYPE_MintPosAdd                      14                    // 平均每小时每分钟位置记录地址
#define   TYPE_DayDistancAdd                   15                    // 每天里程起始数目 
#define   TYPE_ACCONFFcounterAdd               16                    // 异常复位时存储ACCON_Off的计数数值  
//-----------------------------------------------------------------------------------------------------------------------------

//---------  顺序读取发送 相关 define  -----------
#define   RdCycle_Idle                 0     // 空闲
#define   RdCycle_RdytoSD              1     // 准备发送
#define   RdCycle_SdOver               2     // 发送完毕等待中心应答


//--------   顺序读取发送相关  ------------
extern u8       ReadCycle_status;   
extern u8       ReadCycle_timer;   // 超时判断

extern u32     cycle_write, cycle_read;  // 循环存储记录
extern u32    AvrgSpdPerMin_write,AvrgSpdPerMin_Read; // 车辆每分钟平均速度记录
extern u32    AvrgSpdPerSec_write,AvrgSpdPerSec_Read; // 车辆每秒平均速度记录   
extern u32    AvrgMintPosit_write,AvrgMintPosit_Read; // 车辆单位小时内每分钟位置记录 
extern u32    ErrorLog_write,ErrorLog_Read;           // 设备异常记录
extern u32    Recorder_write,Recorder_Read;           // 行车记录仪记录
extern u32    Login_write,Login_Read;				   // 登录记录
extern u32    Powercut_write,Powercut_read;		   // 外部电源断开
extern u32    Settingchg_write,Settingchg_read;	   // 参数修改 
extern u32    TiredDrv_write, TiredDrv_read;  // 疲劳驾驶存储记录
extern u32    ExpSpdRec_write, ExpSpdRec_read;  // 超速报警存储记录
extern u32    pic_write,pic_read,pic_current_page,pic_PageIn_offset,pic_size;       // 图片存储记录 
extern u32    Distance_m_u32;	 // 行车记录仪运行距离	  单位米
extern u32    DayStartDistance_32; //每天起始里程数目    



extern u8  SaveCycleGPS(u32 cycle_wr,u8 *content ,u16 saveLen);   
extern u8  ReadCycleGPS(u32 cycleread,u8 *content ,u16 ReadLen);    
extern u8  Save_DrvRecoder(u32 In_write,u8 *content ,u16 saveLen);
extern u8  Read_DrvRecoder(u32 In_read,u8 *content ,u16 ReadLen);
extern u8  Common_WriteContent(u32 In_write,u8 *content ,u16 saveLen, u8 Type); 
extern u8  Common_ReadContent(u32 In_read,u8 *content ,u16 ReadLen, u8 Type);    
extern u8  Save_PerMinContent(u32 In_wr,u8 *content ,u16 saveLen); 
extern u8  Read_PerMinContent(u32 In_read,u8 *content ,u16 ReadLen);
extern u8  Save_PerSecContent(u32 In_wr,u8 *content ,u16 saveLen);
extern u8  Read_PerSecContent(u32 In_read,u8 *content ,u16 ReadLen);  
extern u8  Save_MintPosition(u32 In_write,u8 *content ,u16 saveLen);
extern u8  Read_MintPosition(u32 In_read,u8 *content ,u16 ReadLen);		
extern void  CHK_ReadCycle_status(void);  
extern void  MediaIndex_Init(void) ; 
extern void Save_Common(u32 In_write,u8 Type);


#endif 

