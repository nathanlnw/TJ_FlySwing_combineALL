/*
     HMI  :  Process  LCD  、Printer 、 KeyChecking
*/

#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>//数字转换成字符串
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"
#include "spi_sd.h"
#include "Usbh_conf.h"
#include <dfs_posix.h>


//#define                                      HMI_MsgQueStack_SIZE  512

 struct rt_mailbox mb_hmi;
 char   mb_hmi_pool[128];
 char   mb_str1[]={"******************"};
 char   mb_str2[]={"abcde"};
 char   hmi_counter=1;
 

unsigned char dayin_15MinSpeed[32]={"00:00 000 km/h\r\n"};
unsigned char dayin_data_time[35]={"\r\n打印时间:                    \r\n"};

unsigned char dayin_chepaihaoma[25]={"\r\n车牌号码:00000000"};
unsigned char dayin_chepaifenlei[25]={"\r\n车辆类型:000000"}; 
unsigned char dayin_cheliangVIN[32]={"\r\n车辆VIN:00000000000000000"};
//unsigned char dayin_driver_NUM[40]={"驾驶员姓名:000000000000000000000"};
unsigned char dayin_driver_NUM[40]={"\r\n驾驶员姓名:000000"};
unsigned char dayin_driver_card[42]={"\r\n驾驶证代码:000000000000000000"};


unsigned char Dayin_TireExpsCard[32]={"1.\r\n"};
unsigned char Dayin_TireExpsStartTime[32]={"起始时间:XX/XX/XX XX:XX:XX\r\n"};//45 78 1011 1314 1617 1920
unsigned char Dayin_TireExpsEndTime[32]=  {"结束时间:XX/XX/XX XX:XX:XX\r\n"};
unsigned char Dayin_TireExpsMaxSpeed[32]=  {"最高时速:XXX km/h\r\n"};//9 10 11


//ALIGN(RT_ALIGN_SIZE)
//static uint8_t					HMI_MsgQueStack[HMI_MsgQueStack_SIZE];
static struct rt_messagequeue	HMI_MsgQue; 

HMI_COM    HMI_Comunicate; 



void  HMI_MsgQue_Post(u16  CMD_ID, u16 len, u8 *content)
{
        HMI_Comunicate.Msg_ID=CMD_ID;
        HMI_Comunicate.Info_Len=len;
	 HMI_Comunicate.Content=content;
		
       rt_mq_send( &HMI_MsgQue, (void*)&HMI_Comunicate,sizeof(HMI_COM) );
}

void Dayin_TireExpsFun(unsigned char type,unsigned char n)
{
unsigned char i=0;
if(type==1)
	{
	if(n==1)
		{
		memcpy(Dayin_TireExpsCard+2,PilaoJilu[0].Drver_Name,strlen((const char*)PilaoJilu[0].Drver_Name));
		strcat((char*)Dayin_TireExpsCard,"\r\n");
		printer((const char *)Dayin_TireExpsCard);
		}
	else if(n==2)
		{
		for(i=0;i<6;i++)
			{
			Dayin_TireExpsStartTime[9+i*3]=PilaoJilu[0].StartTime[i]/10+0x30;
			Dayin_TireExpsStartTime[9+i*3+1]=PilaoJilu[0].StartTime[i]%10+0x30;
			}
		printer((const char *)Dayin_TireExpsStartTime);
		}
	else
		{
		for(i=0;i<6;i++)
			{
			Dayin_TireExpsEndTime[9+i*3]=PilaoJilu[0].EndTime[i]/10+0x30;
			Dayin_TireExpsEndTime[9+i*3+1]=PilaoJilu[0].EndTime[i]%10+0x30;
			}
		printer((const char *)Dayin_TireExpsEndTime);
		}
	}
else
	{
	if(n==1)
		{
		memcpy(Dayin_TireExpsCard+2,ChaosuJilu[0].Drver_Name,strlen((const char*)ChaosuJilu[0].Drver_Name)); 
		strcat((char*)Dayin_TireExpsCard,"\r\n"); 
		printer((const char *)Dayin_TireExpsCard);
		}
	else if(n==2)
		{
		for(i=0;i<6;i++)
			{
			Dayin_TireExpsStartTime[9+i*3]=ChaosuJilu[0].StartTime[i]/10+0x30;
			Dayin_TireExpsStartTime[9+i*3+1]=ChaosuJilu[0].StartTime[i]%10+0x30;
			}
		printer((const char *)Dayin_TireExpsStartTime);
		}
	else if(n==3)
		{
		for(i=0;i<6;i++)
			{
			Dayin_TireExpsEndTime[9+i*3]=ChaosuJilu[0].EndTime[i]/10+0x30;
			Dayin_TireExpsEndTime[9+i*3+1]=ChaosuJilu[0].EndTime[i]%10+0x30;
			}
		printer((const char *)Dayin_TireExpsEndTime);
		}
	else
		{
		Dayin_TireExpsMaxSpeed[9]=ChaosuJilu[0].Speed/100+0x30;
		Dayin_TireExpsMaxSpeed[10]=ChaosuJilu[0].Speed%100/10+0x30;
		Dayin_TireExpsMaxSpeed[11]=ChaosuJilu[0].Speed%10+0x30;
		printer((const char *)Dayin_TireExpsMaxSpeed);
		}
	}
}
	
void Dayin_15MinSpeedFun(unsigned char n)
{   
    dayin_15MinSpeed[0]=speed_time_rec[n][3]/10+0x30;
	dayin_15MinSpeed[1]=speed_time_rec[n][3]%10+0x30;
	
	dayin_15MinSpeed[3]=speed_time_rec[n][4]/10+0x30;
	dayin_15MinSpeed[4]=speed_time_rec[n][4]%10+0x30;
	
	dayin_15MinSpeed[6]=speed_time_rec[n][5]/100+0x30;
	dayin_15MinSpeed[7]=speed_time_rec[n][5]%100/10+0x30;
	dayin_15MinSpeed[8]=speed_time_rec[n][5]%10+0x30;
	printer((const char *)dayin_15MinSpeed);
}
void Dayin_Fun(u8 dayin_par)
{
if(dayin_par==1)
	{
	memcpy(dayin_chepaihaoma+11,Vechicle_Info.Vech_Num,8);    //  2
	memcpy(dayin_chepaifenlei+11,Vechicle_Info.Vech_Type,6);  //  3
	memcpy(dayin_cheliangVIN+10,Vechicle_Info.Vech_VIN,17);   //  4
	memcpy(dayin_driver_NUM+13,JT808Conf_struct.Driver_Info.DriveName,21);    //5
	memcpy(dayin_driver_card+13,JT808Conf_struct.Driver_Info.DriverCard_ID,18);//6
	memcpy((char *)dayin_data_time+11,(char *)Dis_date,20);  //7
	switch(DaYin)
		{
		case 1:
			DaYin++;
			break;
		case 2://车牌号码 9
		    if(License_Not_SetEnable==1)
				printer("\r\n车牌号码:无牌照");
			else
			printer((const char *)dayin_chepaihaoma);
			DaYin++;
			break;
		case 3://车牌分类
			printer((const char *)dayin_chepaifenlei);
			DaYin++;
			break;
		case 4://车辆VIN号码 8
			printer((const char *)dayin_cheliangVIN);
			DaYin++;
			break;
		case 5://驾驶员代码 11   驾驶员姓名
			printer((const char *)dayin_driver_NUM);
			DaYin++;
			break;
		case 6://驾驶证代码
			printer((const char *)dayin_driver_card);
			DaYin++;
			break;
		case 7:
			   if((ModuleStatus&Status_GPS)||((JT808Conf_struct.DF_K_adjustState==1)&&(JT808Conf_struct.Speed_GetType==1)))
				{
				printer((const char *)dayin_data_time);//00/00/00 00:00:00   
				DaYin++;
				}
			else
				{
				printer("\r\n当前不定位，暂不打印停车前速度\r\n");
				DaYin=24;
				}
			break;
		case 8:
			printer("停车前15分钟车速:\r\n"); 
			DaYin++;
			break;
		case 9:
			Dayin_15MinSpeedFun(0);
			DaYin++;
			break;
		case 10:
			Dayin_15MinSpeedFun(1);
			DaYin++;
			break;
		case 11:
			Dayin_15MinSpeedFun(2);
			DaYin++;
			break;
		case 12:
			Dayin_15MinSpeedFun(3);
			DaYin++;
			break;
		case 13:
			Dayin_15MinSpeedFun(4);
			DaYin++;
			break;
		case 14:
			Dayin_15MinSpeedFun(5);
			DaYin++;
			break;
		case 15:
			Dayin_15MinSpeedFun(6);
			DaYin++;
			break;
		case 16:
			Dayin_15MinSpeedFun(7);
			DaYin++;
			break;
		case 17:
			Dayin_15MinSpeedFun(8);
			DaYin++;
			break;
		case 18:
			Dayin_15MinSpeedFun(9);
			DaYin++;
			break;
		case 19:
		    Dayin_15MinSpeedFun(10);
			DaYin++;
			break;
		case 20:
			Dayin_15MinSpeedFun(11);
			DaYin++;
			break;
		case 21:
			Dayin_15MinSpeedFun(12);
			DaYin++;
			break;
		case 22:
			Dayin_15MinSpeedFun(13);
			DaYin++;
			break;
		case 23:
			Dayin_15MinSpeedFun(14);
			DaYin++;
			break;
		case 24:
			printer("最后一次疲劳驾驶记录:\r\n");
			DaYin++;
			break;
		case 25:
			if(TiredDrv_write>0)
				{
				Dayin_TireExpsFun(1,1);
				DaYin++;
				}
			else
				{
				printer("无疲劳驾驶记录\r\n");
				DaYin=28;
				}
			break;
		case 26:
			Dayin_TireExpsFun(1,2);
			DaYin++;
			break;
		case 27:
			Dayin_TireExpsFun(1,3);
			DaYin++;
			break;
		case 28:
			printer("最后一次超速驾驶记录:\r\n");
			DaYin++;
			break;
		case 29:
			if(ExpSpdRec_write>0)
				{
				Dayin_TireExpsFun(2,1);
				DaYin++;
				}
			else
				{
				printer("无超速驾驶记录\r\n");
				DaYin=33;
				}
			break;
		case 30:
			Dayin_TireExpsFun(2,2);
			DaYin++;
			break;
		case 31:
			Dayin_TireExpsFun(2,3);
			DaYin++;
			break;
		case 32:
			Dayin_TireExpsFun(2,4);
			DaYin++;
			break;	
	    case 33:
			step(50,1000);
			DaYin++;
			break;
	   case 34:
			step(50,1000);
			DaYin++;
			break;
	    case 35:
			DaYin=0;
			print_rec_flag=0;
			GPIO_ResetBits(GPIOB,GPIO_Pin_7);//打印关电

			//----------------------------------------------------- 
			gps_onoff(1);  //开启GPS 模块的点
			print_workingFlag=0;  // 打印状态进行中
			Power_485CH1_ON;     // 开启485
			//Speak_ON;      //  开启音频功放     
			//-----------------------------------------------------
			
			rt_kprintf("\r\n----------打印完毕");
			break;
		}
	
	}
}


/* HMI  thread */
ALIGN(RT_ALIGN_SIZE) 
char HMI_thread_stack[2048]; // 4096     
struct rt_thread HMI_thread;

static void HMI_thread_entry(void* parameter)  
{
    u8 counter_printer=0;

     //  finsh_init(&shell->parser);
	rt_kprintf("\r\n ---> HMI thread start !\r\n");

       //------------ lcd  related --------------
       // Init_lcdkey();
	   // delay_ms(400); // 屏rst 拉低    维持一段时间 
        //lcd_init();
	   //-------- IC card related ---------------
	    Init_4442(); 


	rt_kprintf("\r\nVechicle_Info.loginpassword_flag=%d\r\n",Login_Menu_Flag);
    if(Login_Menu_Flag==0)
    	{
    	    JT808Conf_struct.Regsiter_Status=0;   //需要重新注册
            pMenuItem=&Menu_0_0_password;
	        pMenuItem->show();
    	}
	else
		{
	        pMenuItem=&Menu_1_Idle;   
		    pMenuItem->show();  
		}

	while (1)
	{
	       //KeyCheckFun();  
           pMenuItem->timetick( 10 ); 
	 	   pMenuItem->keypress( 10 );  
		   //-------  
		if(print_rec_flag==1)
			{
			counter_printer++;
			if(counter_printer>=5)//加电后1s开始打印，打印间隔必须>300ms
				{
				counter_printer=0; 
				if(TiredDrv_write>0)
					{
					ReadPiLao(1);
					Dis_pilao(data_tirexps);
					}
				if(ExpSpdRec_write>0)
					{
					ReadEXspeed(1);
					Dis_chaosu(data_tirexps);
					}
				
			    Fetch_15minSpeed(15);
				print_rec_flag=2;
				DaYin=1;//开始打印
				WatchDog_Feed();
				rt_kprintf("\r\n----------开始打印");
				}
			}
		else if(print_rec_flag==2)
			{
			counter_printer++;
			if(counter_printer>=2)//打印间隔必须>300ms      7 
				{
				counter_printer=0;
				//if(ModuleStatus&Status_GPS)
					Dayin_Fun(1);
				/*else	 
					Dayin_Fun(0);*/
				}
			} 
		 //---------- IC card  insert --------------------------
		// CheckICInsert();  
		 //------- Buzzer -----------------------------------
		 //KeyBuzzer(IC_CardInsert);
		//--------------------------------------------
		if(ASK_Centre.ASK_disp_Enable==1)
		   {
			ASK_Centre.ASK_disp_Enable=0; 
			pMenuItem=&Menu_3_1_CenterQuesSend;
		       pMenuItem->show();
		 }
		else if(TextInfo.TEXT_SD_FLAG==1)
		{
			TextInfo.TEXT_SD_FLAG=0;
			pMenuItem=&Menu_7_CentreTextDisplay;
		       pMenuItem->show();
		}
	 	//--------------------------------------------	   
        rt_thread_delay(25);                 
     }  
}



/* init HMI  */
void HMI_app_init(void)
{
        rt_err_t result;
		

	result=rt_thread_init(&HMI_thread, "HMI", 
		HMI_thread_entry, RT_NULL,
		&HMI_thread_stack[0], sizeof(HMI_thread_stack),    
		Prio_HMI, 6); 

    if (result == RT_EOK)
    {
           rt_thread_startup(&HMI_thread); 
   	    rt_kprintf("\r\n HMI  thread initial sucess!\r\n");    // nathan add
    	}
    else
	    rt_kprintf("\r\nHMI  thread initial fail!\r\n");    // nathan add	     
}



