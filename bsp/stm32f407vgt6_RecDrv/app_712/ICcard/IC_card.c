/*
       IC  card 
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
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
#include "App_moduleConfig.h" 
#include "Vdr.h"




unsigned char IC_CardInsert=0;//1:IC卡插入正确  2:IC卡插入错误
unsigned char IC_Check_Count=0;
unsigned int      read_counter=0,flag_8024off=1;    
unsigned char   Init8024Flag=0; 
unsigned int      DelayCheckIc=0;
unsigned char   institution[45];
unsigned char administrator_card=0;
u8        powerOn_first=0;   //    首次上电后不判断拔卡  



void KeyBuzzer(unsigned char num)
{

if(num==1)
	{
	BuzzerFlag++;
	if(BuzzerFlag==2)
		    buzzer_onoff(1);
	if(BuzzerFlag==4)
		{
			buzzer_onoff(0);
			BuzzerFlag=0;
			IC_CardInsert=0;
		}
	  
	}
else if(num==2)
	{
	BuzzerFlag++;
	if((BuzzerFlag==12)||(BuzzerFlag==16))
		buzzer_onoff(1);
	if((BuzzerFlag==14)||(BuzzerFlag==18))
		buzzer_onoff(0);
	if(BuzzerFlag==18)
		{BuzzerFlag=0;IC_CardInsert=0;}  
	}
}



void CheckICInsert(void)
{
unsigned char write_flag=0;
u8 result0=0,result1=0,result2=0,result3=0,result4=0,result5=0;//i=0;
u8 FLagx=0;//,len=0;
unsigned char reg_record[32],reg_record_liu[12];
u32 DriveCode32=0;


//===================测试IC卡读写==================================================
if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7))
	{
	IC_Check_Count++;
	if(IC_Check_Count>=10)
		{
		IC_Check_Count=0;
	//带卡上电开8024的电
/*	if(flag_8024off==1)       //  上电不读取IC 卡
		{
			R_Flag|=b_CardEdge;
			Init8024Flag=2;
			flag_8024off=0;
		}
 */
	//8024的off从低变高
	if(Init8024Flag==1)
		{
			Init8024Flag=2;
			R_Flag|=b_CardEdge;
			//rt_kprintf("pc7  为 高，R_Flag＝1\r\n");
		}
	//检测到卡后初始化ic卡
	if((R_Flag&b_CardEdge)&&(Init8024Flag==2))
		{
		Init8024Flag=3;
		_CardCMDVCC_LOW;
		for(DelayCheckIc=0;DelayCheckIc<500;DelayCheckIc++)
		DELAY5us();
		_CardSetPower_HIGH;
		_CardSetRST_LOW;
		for(DelayCheckIc=0;DelayCheckIc<15;DelayCheckIc++)
			{
			_CardSetCLK_LOW;
			DELAY5us();DELAY5us();DELAY5us();
			_CardSetCLK_HIGH;
			DELAY5us();DELAY5us();DELAY5us();
			_CardSetCLK_LOW;
			}			 
		R_Flag&=~b_CardEdge;
		write_flag=1;
	     	rt_kprintf("  插卡  只执行1次\r\n");
			//----------------------------------------------------
            VDR_product_12H(0x01);  //  登录  
			//----------------------------------------------------
		}
		}
	}
else 
	{
	
	IC_Check_Count=0;
	_CardSetRST_HIGH;
	_CardSetPower_LOW;
	_CardCMDVCC_HIGH;
	if(Init8024Flag==0)
		{
			Init8024Flag=1;
			if(powerOn_first==0)
				 powerOn_first=1;
			else
			 {
			    //--------------------------------------------------- 
			     rt_kprintf("   拔卡 pc7  为 低---触发\r\n");  
                 VDR_product_12H(0x02);  //  登出 
				//---------------------------------------------------
				 
			 }		 
		}
	}

if(write_flag==1)
	{
	write_flag=0;
	Rx_4442(241,13,reg_record);	//管理员卡
	memset(reg_record_liu,0,sizeof(reg_record_liu));
	Rx_4442(141,6,reg_record_liu);	//管理员卡
	if(strncmp((char *)reg_record,"administrator",13)==0)
		{
		  rt_kprintf("\r\n管理员卡");
		  administrator_card=1;

		   pMenuItem=&Menu_0_loggingin;  // 管理员卡进入界面
		   pMenuItem->show();
		   BuzzerFlag=1;//响一声提示 
		}
	else if(strncmp((char *)reg_record_liu,"刘旭景",6)==0)
		{
		  rt_kprintf("\r\n生产管理者");
          // 
          Dis_deviceid_flag=1;
		}
	else
		{
	     memset(DriverCardNUM,0,sizeof(DriverCardNUM));
	     memset(DriverName,0,sizeof(DriverName));
	     //memset(JT808Conf_struct.Driver_Info,0,sizeof(JT808Conf_struct.Driver_Info));

           result0=Rx_4442(70,10,(unsigned char *)DriverName);	//读驾驶员姓名
	    rt_kprintf("\r\n驾驶员姓名:%s,result0=%d",DriverName,result0);

		
		result1=Rx_4442(52,18,(unsigned char *)DriverCardNUM);	//读驾驶证号码
		rt_kprintf("\r\n驾驶证代码:%s,result1=%d",DriverCardNUM,result1);

              memset(JT808Conf_struct.Driver_Info.DriveCode,0,sizeof(JT808Conf_struct.Driver_Info.DriveCode));
		result2=Rx_4442(49,3,(unsigned char *)JT808Conf_struct.Driver_Info.DriveCode);	//读驾驶员代码
		DriveCode32=(JT808Conf_struct.Driver_Info.DriveCode[0]<<16)+(JT808Conf_struct.Driver_Info.DriveCode[1]<<8)+JT808Conf_struct.Driver_Info.DriveCode[2];
		rt_kprintf("\r\n驾驶员代码:%d,result2=%d",DriveCode32,result2);   

		memset(JT808Conf_struct.Driver_Info.Driver_ID,0,sizeof(JT808Conf_struct.Driver_Info.Driver_ID));
		result3=Rx_4442(80,20,(unsigned char *)JT808Conf_struct.Driver_Info.Driver_ID);	//身份证号码
		rt_kprintf("\r\n身份证号码:%s,result3=%d",JT808Conf_struct.Driver_Info.Driver_ID,result3);

		memset(JT808Conf_struct.Driver_Info.Drv_CareerID,0,sizeof(JT808Conf_struct.Driver_Info.Drv_CareerID));
		result4=Rx_4442(100,40,(unsigned char *)JT808Conf_struct.Driver_Info.Drv_CareerID);	//从业资格证
		rt_kprintf("\r\n从业资格证:%s,result4=%d",JT808Conf_struct.Driver_Info.Drv_CareerID,result4);

		memset(JT808Conf_struct.Driver_Info.Comfirm_agentID,0,sizeof(JT808Conf_struct.Driver_Info.Comfirm_agentID));
		result5=Rx_4442(140,41,(unsigned char *)institution);	//发证机构
		memcpy(JT808Conf_struct.Driver_Info.Comfirm_agentID,&institution[1],40);
		rt_kprintf("\r\n发证机构:%s,result5=%d",JT808Conf_struct.Driver_Info.Comfirm_agentID,result5);
		
		if((result0==0)&&(result1==0)&&(result2==0)&&(result3==0)&&(result4==0)&&(result5==0))//读结果正确
		{
		IC_CardInsert=1;//IC	卡插入正确
		FLagx=0;
 
		//前18位为驾驶证号码-------------疲劳驾驶相关------看是否更换了卡 --------------
		if(strncmp((char*)DriverCardNUM,(char*)JT808Conf_struct.Driver_Info.DriverCard_ID,18)!=0)
			{	
			memset(JT808Conf_struct.Driver_Info.DriverCard_ID,0,sizeof(JT808Conf_struct.Driver_Info.DriverCard_ID));  
			memcpy(JT808Conf_struct.Driver_Info.DriverCard_ID,DriverCardNUM,18);  
			FLagx=1;
			}
		//后20位为驾驶员姓名-------------疲劳驾驶相关------看是否更换了卡 --------------
		if(strncmp((char*)DriverName,(char*)JT808Conf_struct.Driver_Info.DriveName,20)!=0)
			{							   
			memset(JT808Conf_struct.Driver_Info.DriveName,0,sizeof(JT808Conf_struct.Driver_Info.DriveName));  
			memcpy((u8*)JT808Conf_struct.Driver_Info.DriveName,DriverName,strlen((const char*)DriverName));    		
			FLagx=2;
			}
		if(FLagx)//更换了IC 卡    清除疲劳驾驶相关
			{
			TIRED_Drive_Init();  //清除疲劳驾驶的状态	  
			buzzer_onoff(0);; // 关闭蜂鸣器 		
			Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
			FLagx=0;//clear
			} 
		  BuzzerFlag=1;//响一声提示 
		  pMenuItem=&Menu_2_5_DriverInfor;
	        pMenuItem->show();
		}
	else
		{
		         BuzzerFlag=11;//响一声提示
		         IC_CardInsert=2;//IC	卡插入错误 
		}
		}
	Init8024Flag=0;
	GpsIo_Init();
	} 

//===================测试IC卡读写完成==================================================

}


