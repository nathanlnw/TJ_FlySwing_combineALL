/*
     Device_808.C       和808   协议相关的 I/O 管脚配置     
*/

#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"


u8  HardWareVerion=0;   //   硬件版本检测 
//-----  WachDog related----
u8    wdg_reset_flag=0;    //  Task Idle Hook 相关
u32   TIM3_Timer_Counter=0; //  测试定时器计数器

//--------  电压检测 相关 ---------------------------------
AD_POWER  Power_AD; 



u32  IC2Value=0;   // 
u32  DutyCycle	= 0;





//------------  AD    电压相关  -------------------- 
void AD_PowerInit(void)
{
   Power_AD.ADC_ConvertedValue=0; //电池电压AD数值    
   Power_AD.AD_Volte=0;      // 采集到的实际电压数值
   Power_AD.Classify_Door=160;   //  区分大车小车类型，  >16V  大型车 <16V 小型车 
   Power_AD.LowWarn_Limit_Value=10;  //  欠压报警门限值      
}

u8  HardWareGet(void)   
{  //  获取硬件版本信息   
   // -----    硬件版本状态监测 init  ----------------------
   /*
	PA13	1	复用硬件版本判断
	PA14	1	复用硬件版本判断
	PB3       0	复用硬件版本判断
    */
   u8   Value=0;

     //-----------------------------------------------------------  
     if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13))  // bit 2 
	 	   Value|=0x04;
	 else
	 	   Value&=~0x04;  
     //----------------------------------------------------------
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_14))  // bit 1
	 	   Value|=0x02;
	 else
	 	   Value&=~0x02;   
	//------------------------------------------------------------ 
	 if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)) // bit0 
	 	   Value|=0x01; 
	 else
	 	   Value&=~0x01;  
	 //------------------------------------------------------------
      rt_kprintf("\r\n  硬件版本读取: %2X",Value);      
     return Value;
}
//FINSH_FUNCTION_EXPORT(HardWareGet, HardWareGet); 


void WatchDog_Feed(void)
{
    if(wdg_reset_flag==0)
           IWDG_ReloadCounter();   
}

void  reset(void)
{
   IWDG_SetReload(0);
   IWDG->KR = 0x00001;  //not regular
  wdg_reset_flag=1; 
} 
FINSH_FUNCTION_EXPORT(reset, ststem reset);
void WatchDogInit(void)
{    
  /* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
     dispersion) */
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); 

  /* IWDG counter clock: LSI/32 */
    /*   prescaler            min/ms    max/ms
         4                        0.1             409.6
         8                        0.2             819.2
         16                      0.4             1638.4
         32                      0.8              3276.8
         64                      1.6              6553.5
         128                    3.2              13107.2
         256                    6.4              26214.4   
  */
  IWDG_SetPrescaler(IWDG_Prescaler_16);

  /* Set counter reload value to obtain 250ms IWDG TimeOut.
     Counter Reload Value = 250ms/IWDG counter clock period
                          = 250ms / (LSI/32)
                          = 0.25s / (LsiFreq/32)
                          = LsiFreq/(32 * 4)
                          = LsiFreq/128
   */
  IWDG_SetReload(0X4AAA);//(LsiFreq/128);

  /* Reload IWDG counter */
  IWDG_ReloadCounter();

  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
}

void  APP_IOpinInit(void)   //初始化 和功能相关的IO 管脚
{
  	GPIO_InitTypeDef        gpio_init;

     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);      
     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	 

    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_Speed = GPIO_Speed_100MHz; 
    gpio_init.GPIO_OType = GPIO_OType_PP;  
    gpio_init.GPIO_PuPd  = GPIO_PuPd_NOPULL; 	 
 // 		IN
	//------------------- PE8 -----------------------------
	gpio_init.GPIO_Pin	 = GPIO_Pin_8;	  //紧急报警
	gpio_init.GPIO_Mode  = GPIO_Mode_IN; 
	GPIO_Init(GPIOE, &gpio_init);
	//------------------- PE9 -----------------------------
	//gpio_init.GPIO_Pin	 = GPIO_Pin_9;				//------ACC  状态
	//gpio_init.GPIO_Mode  = GPIO_Mode_IN; 
	//GPIO_Init(GPIOE, &gpio_init);

   //	OUT
   
   //------------------- PB1 -----------------------------
   gpio_init.GPIO_Pin	= GPIO_Pin_1;   //------未定义   输出 常态置0  
   gpio_init.GPIO_Mode	= GPIO_Mode_OUT; 
   GPIO_Init(GPIOB, &gpio_init); 


   if(Module_3017A==GPS_MODULE_TYPE)
   {
	   gpio_init.GPIO_Pin	= GPIO_Pin_6;   //------输出 常态置0   PC13  蜂鸣器
	   gpio_init.GPIO_Mode	= GPIO_Mode_OUT;  
	   GPIO_Init(GPIOB, &gpio_init);  
	   GPIO_ResetBits(GPIOB,GPIO_Pin_6);  // 关闭蜂鸣器	   	 
   }   
   
 //==================================================================== 
 //-----------------------写继电器常态下的情况------------------
 GPIO_ResetBits(GPIOB,GPIO_Pin_1);	 //继电器闭合
 //GPIO_SetBits(GPIOB,GPIO_Pin_1);	 //输出常态 置 0     

// GPIO_ResetBits(GPIOA,GPIO_Pin_13);	 // 关闭蜂鸣器          
 /*
      J1 接口 初始化
 */
	 //---------- PA0--------------------- 
  // gpio_init.GPIO_Pin  =GPIO_Pin_0; 				//-----  PIN 1   速度传感器   // 暂时无用
  // gpio_init.GPIO_Mode = GPIO_Mode_IN;
   //GPIO_Init(GPIOA, &gpio_init);
   //------------- -- --------------
   //gpio_init.GPIO_Pin	 = GPIO_Pin_6;				//------PIN  2   NULL 
  // gpio_init.GPIO_Mode  = GPIO_Mode_IN; 
   //GPIO_Init(GPIOE, &gpio_init);
    //--------------- --------------
   //gpio_init.GPIO_Pin	 = GPIO_Pin_9;				//------PIN 3    NULL 
   //gpio_init.GPIO_Mode  = GPIO_Mode_IN; 
   //GPIO_Init(GPIOD, &gpio_init);
    //------------- PC0 --------------
   gpio_init.GPIO_Pin	 = GPIO_Pin_0;				//------PIN 4    远光灯
   gpio_init.GPIO_Mode  = GPIO_Mode_IN; 
   GPIO_Init(GPIOC, &gpio_init); 
    //------------- ----------------
   gpio_init.GPIO_Pin	 = GPIO_Pin_1;				//------PIN 5   预留  车门灯
   gpio_init.GPIO_Mode  = GPIO_Mode_IN; 
   GPIO_Init(GPIOC, &gpio_init); 
    //------------- PA1 --------------
   gpio_init.GPIO_Pin	 = GPIO_Pin_1;				//------PIN 6   喇叭
   gpio_init.GPIO_Mode  = GPIO_Mode_IN; 
   GPIO_Init(GPIOA, &gpio_init);
    //------------- PC3 --------------
   gpio_init.GPIO_Pin	 = GPIO_Pin_3;				//------PIN 7   左转灯 
   gpio_init.GPIO_Mode  = GPIO_Mode_IN; 
   GPIO_Init(GPIOC, &gpio_init);
    //------------- PC2 --------------   
   gpio_init.GPIO_Pin	 = GPIO_Pin_2;				//------PIN 8   右转灯   
   gpio_init.GPIO_Mode  = GPIO_Mode_IN; 
   GPIO_Init(GPIOC, &gpio_init);  
         
    //------------- PE11 --------------
   gpio_init.GPIO_Pin	 = GPIO_Pin_11;				//------PIN 9   刹车灯
   gpio_init.GPIO_Mode  = GPIO_Mode_IN; 
   GPIO_Init(GPIOE, &gpio_init);
    //------------- PE10 --------------
   gpio_init.GPIO_Pin	 = GPIO_Pin_10;				//------PIN 10  雨刷
   gpio_init.GPIO_Mode  = GPIO_Mode_IN; 
   GPIO_Init(GPIOE, &gpio_init);  
 
   //-----------------------------------------------------------------
   

   //------- 速度信号线 ---------------
    gpio_init.GPIO_Pin = GPIO_Pin_0; 
    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_Speed = GPIO_Speed_100MHz; 
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd  = GPIO_PuPd_NOPULL; 
    GPIO_Init(GPIOA, &gpio_init); 
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2); 


#if 1
   // -----    硬件版本状态监测 init  ---------------------- 
   /*
	PA13	1	复用硬件版本判断
	PA14	1	复用硬件版本判断
	PB3       0	复用硬件版本判断
    */
       //------------- PA13 --------------   
   gpio_init.GPIO_Pin	 = GPIO_Pin_13;				
   gpio_init.GPIO_Mode  = GPIO_Mode_IN; 
   GPIO_Init(GPIOA, &gpio_init);  
         
    //------------- PA14 --------------
   gpio_init.GPIO_Pin	 = GPIO_Pin_14;				
   gpio_init.GPIO_Mode  = GPIO_Mode_IN; 
   GPIO_Init(GPIOA, &gpio_init);
    //------------- PB3 --------------
   gpio_init.GPIO_Pin	 = GPIO_Pin_3;				
   gpio_init.GPIO_Mode  = GPIO_Mode_IN;  
   GPIO_Init(GPIOB, &gpio_init);  
#endif   

}





/*    
     -----------------------------
    1.    输入管脚状态监测
     ----------------------------- 
*/
u8  ACC_StatusGet(void)
{  
    // ACC 状态引脚   
    //返回  非0 为  ACC 开   
    //返回  0  为   ACC 关
   return(!GPIO_ReadInputDataBit(ACC_IO_Group,ACC_Group_NUM)); 
}

u8  WARN_StatusGet(void)
{
    // 紧急报警 状态引脚   
    //返回  非0 为  报警按钮按下  
    //返回  0  为   报警按钮断开
   return (!GPIO_ReadInputDataBit(WARN_IO_Group,WARN_Group_NUM));   
}
u8  MainPower_cut(void)
{ 
    // 断电报警 状态引脚   
    //返回  非0 为  主电断开  
    //返回  0  为   主电工作 
 // return (GPIO_ReadInputDataBit(POWER_IO_Group,POWER_Group_NUM)&0x01) ; 
   return false;
}

u8  FarLight_StatusGet(void)
{
  //  --------------J1pin2		Pc0		远光灯
	   return (!GPIO_ReadInputDataBit(FARLIGHT_IO_Group,FARLIGHT_Group_NUM));	// PE6
	//		 接高  触发
}
u8  NearLight_StatusGet(void)
{
  //  --------------J1pin2		Pc1 		远光灯
	   return (!GPIO_ReadInputDataBit(NEARLIGHT_IO_Group,NEARLIGHT_Group_NUM));	// PE6
	//		 接高  触发
}

u8  FogLight_StatusGet(void)
{
  //  --------------J1pin2		Pc3 		雾灯
	   return (!GPIO_ReadInputDataBit(FOGLIGHT_IO_Group,FOGLIGHT_Group_NUM));	// PE6
	//		 接高  触发
}
u8 WarnLight_StatusGet(void)
{
       //  --------------J1pin3	   PD9			报警灯
	   //return (!(GPIO_ReadInputDataBit(WARNLIGHT_IO_Group,WARNLIGHT_Group_NUM));	// PD9
	   return false;
			//		 接高  触发
}		
u8 Speaker_StatusGet(void)
{
   //  --------------J1pin4 	 PC6		喇叭 ---车门开关拍照
	   return (!GPIO_ReadInputDataBit(SPEAKER_IO_Group,SPEAKER_Group_NUM));	// PC2 
			//		 接高  触发
}			
u8  LeftLight_StatusGet(void)
{
  //  --------------J1pin5	   PC7		   左转灯
	   return (!GPIO_ReadInputDataBit(LEFTLIGHT_IO_Group,LEFTLIGHT_Group_NUM));	//	PC7 
			//		 接高  触发
}	
u8  DoorLight_StatusGet(void)
{
 //  --------------J1pin6	   PC1		  车门灯
	   return (!GPIO_ReadInputDataBit(DOORLIGHT_IO_Group,DOORLIGHT_Group_NUM));	// PC1
			//		 接高  触发
}		
u8  RightLight_StatusGet(void)
{
//	--------------J1pin7   PC9			  右转灯
		 return(!GPIO_ReadInputDataBit(RIGHTLIGHT_IO_Group,RIGHTLIGHT_Group_NUM));  //PC9 
		     //	   接高  触发 
}			
u8  BreakLight_StatusGet(void)
{
	//	--------------J1pin8	PE11	刹车灯
	   return(!GPIO_ReadInputDataBit(BREAK_IO_Group,BREAK_Group_NUM));	//PE11
			//		 接高  触发
}
u8 RainBrush_StatusGet(void)
{
// --------------J1pin9    PD8			 雨刷 
		return(!GPIO_ReadInputDataBit(RAINBRUSH_IO_Group,RAINBRUSH_Group_NUM));  //PD8 
			//		 接高  触发
}	
/*    
     -----------------------------
    2.  控制输出
     ----------------------------- 
*/
void  Enable_Relay(void)
{  // 断开继电器
 
 GPIO_SetBits(RELAY_IO_Group,RELAY_Group_NUM); // 断开	
}
void  Disable_Relay(void)
{ // 接通继电器
   
   GPIO_ResetBits(RELAY_IO_Group,RELAY_Group_NUM); // 通	 
}


u8  Get_SensorStatus(void)    
{        // 查询传感器状态
   u8  Sensorstatus=0;
   
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

   //  --------------J1pin8 		   刹车灯
		  if(BreakLight_StatusGet())  //PA8
		   {   //		接高  触发
			   Sensorstatus|=0x80;
			    BD_EXT.FJ_IO_1 |=0x80;  //  bit7 
			    BD_EXT.Extent_IO_status |= 0x10;  // bit4 ---->刹车
		   }
		  else
		   {  //   常态
			   Sensorstatus&=~0x80;		
			    BD_EXT.FJ_IO_1 &=~0x80;  //  bit7 
			    BD_EXT.Extent_IO_status &= ~0x10; //bit4 ---->刹车
		   } 
	//	-------------- J1pin5			 左转灯
		 if(LeftLight_StatusGet())	//	PC7 
		  {   //	   接高  触发
			  Sensorstatus|=0x40;
			   BD_EXT.FJ_IO_1 |=0x40;  //  bit6
			    BD_EXT.Extent_IO_status |= 0x08;//bit3---->  左转灯
		  }
		 else
		  {  //   常态
			 Sensorstatus&=~0x40; 	
			 BD_EXT.FJ_IO_1 &=~0x40;  //  bit6 
			  BD_EXT.Extent_IO_status &= ~0x08; //bit3---->  左转灯
		  }
   //  -------------- J1pin7				右转灯
	     if(RightLight_StatusGet())	//PC2 
	     {	//		 接高  触发
				Sensorstatus|=0x20;
				 BD_EXT.FJ_IO_1 |=0x20; //bit5
				 BD_EXT.Extent_IO_status |= 0x04;// bit2----> 右转灯
		}
            else
		{  //	常态
			   Sensorstatus&=~0x20;		
		   BD_EXT.FJ_IO_1 &=~0x20; //bit5
		   BD_EXT.Extent_IO_status &= ~0x04;//bit2----> 右转灯
		} 
   
   //  --------------远光灯-----------------------
	   if(FarLight_StatusGet())	// PC0
		{	//		 接高  触发
			Sensorstatus|=0x10;
			BD_EXT.Extent_IO_status |= 0x02; //bit 1  ----->  远光灯
			
		}
	   else
		{  //	常态
		   Sensorstatus&=~0x10;		
		   BD_EXT.Extent_IO_status&= ~0x02;//bit 1  ----->  远光灯     
		}  
    //  --------------近光灯----------------------
   		 if(NearLight_StatusGet())  // Pc1
		  {   //       接高  触发
		      Sensorstatus|=0x08;
		       BD_EXT.FJ_IO_1 |=0x10; //bit4	  
		      BD_EXT.Extent_IO_status |= 0x01; //bit 0  ----->  近光灯
		  }
		 else
		  {  //	  常态
			 Sensorstatus&=~0x08;		
			  BD_EXT.FJ_IO_1 &=~0x10; //bit4	  
			  BD_EXT.Extent_IO_status &=~0x01; //bit 0  ----->  近光灯 
		  } 
  // --------------J1pin9          雾灯/   雨刷     
          if(FogLight_StatusGet())  //PD8  
		  {   //	   接高  触发
			  Sensorstatus|=0x04;
			  BD_EXT.FJ_IO_1 |=0x08; //bit3	    
			  BD_EXT.Extent_IO_status |= 0x40;//  bit6 ----> 雾灯
		  }
		 else
		  {  //   常态
			  Sensorstatus&=~0x04;
			  BD_EXT.FJ_IO_1 &=~0x08; //bit3	  
			   BD_EXT.Extent_IO_status &= ~0x40;//  bit6 ----> 雾灯 
		  } 
  //  --------------J1pin6			 车门/飞翼
	    if(DoorLight_StatusGet())  // PE3     
		{	//		 接高  触发
			Sensorstatus|=0x02;
			 BD_EXT.FJ_IO_2 |=0x01; //bit2       
		}
	   else
		{  //	常态
		   Sensorstatus&=~0x02;		
		   BD_EXT.FJ_IO_2 |=0x01; //bit2	
		}	
			   
   /*				     
   //  --------------J1pin3 			  报警灯
		 if(WarnLight_StatusGet())	// PD9
		  {   //	   接高  触发
			  Sensorstatus|=0x0002;    
		  }
		 else
		  {   //   常态
			 Sensorstatus&=~0x0002; 				
		  }       
  */   

   return Sensorstatus;
}

void  IO_statusCheck(void)
{
      Vehicle_sensor=Get_SensorStatus();
	  //------------ 0.2s    速度状态 -----------------------
	  Sensor_buf[save_sensorCounter].DOUBTspeed=Spd_Using/10;     //   速度  单位是km/h 所以除以10
      Sensor_buf[save_sensorCounter++].DOUBTstatus=Vehicle_sensor;//   状态 
		if(save_sensorCounter>100) 
			{
              save_sensorCounter=0; 
			  sensor_writeOverFlag=1; 
			}  
      //-------------------------------------------------- 
}

void  ACC_status_Check(void)
{
                 //------------车辆运行状态指示 ---------------   
			    if(ACC_StatusGet())           //bit 0
				{	
				   Vehicle_RunStatus|=0x01;
		           //   ACC ON		 打火 
				   StatusReg_ACC_ON();  // ACC  状态寄存器   
				   Sleep_Mode_ConfigExit(); // 休眠相关
			    }
				else
				{
				   Vehicle_RunStatus&=~0x01;
				   //	  ACC OFF	   关火
				   StatusReg_ACC_OFF();  // ACC  状态寄存器 		  
				   Sleep_Mode_ConfigEnter(); // 休眠相关
				}  
}

/*    
     -----------------------------
    2.  应用相关
     ----------------------------- 
*/
//-------------------------------------------------------------------------------------------------

/*采用PA.0 作为外部脉冲计数*/
void pulse_init( void )
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;
	TIM_ICInitTypeDef	TIM_ICInitStructure;

	/* TIM5 clock enable */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5, ENABLE );

	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );

	/* TIM5 chennel1 configuration : PA.0 */
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	/* Connect TIM pin to AF0 */
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource0, GPIO_AF_TIM5 );

	/* Enable the TIM5 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel						= TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	TIM_ICInitStructure.TIM_Channel		= TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity	= TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter	= 0x0;

	TIM_PWMIConfig( TIM5, &TIM_ICInitStructure );

	/* Select the TIM5 Input Trigger: TI1FP1 */
	TIM_SelectInputTrigger( TIM5, TIM_TS_TI1FP1 );

	/* Select the slave Mode: Reset Mode */
	TIM_SelectSlaveMode( TIM5, TIM_SlaveMode_Reset );
	TIM_SelectMasterSlaveMode( TIM5, TIM_MasterSlaveMode_Enable );

	/* TIM enable counter */
	TIM_Cmd( TIM5, ENABLE );

	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig( TIM5, TIM_IT_CC2, ENABLE );
}

/*TIM5_CH1*/
void TIM5_IRQHandler( void )
{
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq( &RCC_Clocks );

	TIM_ClearITPendingBit( TIM5, TIM_IT_CC2 );

	/* Get the Input Capture value */
	IC2Value = TIM_GetCapture2( TIM5 );

	if( IC2Value != 0 )
	{
		/* Duty cycle computation */
		//DutyCycle = ( TIM_GetCapture1( TIM5 ) * 100 ) / IC2Value;
		/* Frequency computation   TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 */
		//Frequency = (RCC_Clocks.HCLK_Frequency)/2 / IC2Value;
/*是不是反向电路?*/
		DutyCycle	= ( IC2Value * 100 ) / TIM_GetCapture1( TIM5 );
		Delta_1s_Plus	= ( RCC_Clocks.HCLK_Frequency ) / 2 / TIM_GetCapture1( TIM5 );
	}else
	{
		DutyCycle	= 0;
		Delta_1s_Plus	= 0;
	}
}

/*定时器配置*/
void TIM3_Config( void )  
{
	NVIC_InitTypeDef		NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );

	/* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel						= TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init( &NVIC_InitStructure );

/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period		= 1000;             /* 1ms */
	TIM_TimeBaseStructure.TIM_Prescaler		= ( 168 / 2 - 1 );  /* 1M*/
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_ARRPreloadConfig( TIM3, ENABLE );

	TIM_TimeBaseInit( TIM3, &TIM_TimeBaseStructure );

	TIM_ClearFlag( TIM3, TIM_FLAG_Update );
/* TIM Interrupts enable */
	TIM_ITConfig( TIM3, TIM_IT_Update, ENABLE );

/* TIM3 enable counter */
	TIM_Cmd( TIM3, ENABLE );
}


void TIM3_IRQHandler(void)
{

   if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET )
  {
                    TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update); 

	        //---------------------Timer  Counter -------------------------- 
            TIM3_Timer_Counter++;

            if(TIM3_Timer_Counter==20)  //  100ms  ----100        
            {
                 //--------  service ------------------                  
				 KeyCheckFun();
				 //---------- IC card  insert --------------------------
				 CheckICInsert();  
				 //------- Buzzer -----------------------------------
				 KeyBuzzer(IC_CardInsert);
	             //-----------service -----------------
		       TIM3_Timer_Counter=0; 		  
            }
			
 //------------------------------------------------------------
   }
}





//---------------------------------------------------------------------------------------------------
void Init_ADC(void)
{
  
  ADC_InitTypeDef   ADC_InitStructure;
  GPIO_InitTypeDef		gpio_init;
ADC_CommonInitTypeDef  ADC_CommonInitStructure;

 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
 

//------Configure PC.5 (ADC Channel15) as analog input -------------------------
gpio_init.GPIO_Pin = GPIO_Pin_5;
gpio_init.GPIO_Mode = GPIO_Mode_AIN;
GPIO_Init(GPIOC, &gpio_init);


  /* ADC Common configuration *************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; /*在独立模式下 每个ADC接口独立工作*/
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);


 /* ADC1 regular channel7 configuration *************************************/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

  /* Enable ADC3 DMA */
//  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC1, ENABLE);

  ADC_SoftwareStartConv(ADC1);

}




//==========================================================
#if 1
void  sys_status(void)
{
     rt_kprintf("\r\n 状态查询: "); 	     
     //-----  报警  ---	 
     if(WARN_StatusGet())
	                 rt_kprintf(" 紧急报警      "); 	      
    //     ACC 	 
     if(ACC_StatusGet())
                        rt_kprintf("ACC 开    "); 
     else
	 	          rt_kprintf("ACC 关    ");  
    //     AD 电压 
    rt_kprintf ("\r\n  获取到的电池AD数值为:	%d	",Power_AD.ADC_ConvertedValue);	  
    rt_kprintf(" AD 电压: %d.%d  V  ",Power_AD.AD_Volte/10,Power_AD.AD_Volte%10);    	        
    //    信号线	 
     rt_kprintf("    %s      ",XinhaoStatus);       
    //   定位模式	
     rt_kprintf("\r\n 定位模式:    ");  
		      switch(GpsStatus.Position_Moule_Status)
		      	{  
		      	         case 1:           rt_kprintf(" BD");  
						 	break;
				  case 2:           rt_kprintf(" GPS ");  
				  	              break;
				  case 3:           rt_kprintf(" BD+GPS");  
				  	              break;
		      	} 
     //   定位状状态
     if(ModuleStatus&Status_GPS)
	 	        rt_kprintf("         定位状态:  定位   卫星颗数: %d 颗 ",Satelite_num);   
    else	 
                      rt_kprintf("         定位状态: 未定位");  	 
    //    GPS 天线状态
    if(GpsStatus.Antenna_Flag==1)
         rt_kprintf("        天线:     断开");  
   else	
   	  rt_kprintf("        天线:     正常");   
    //  欠压报警状态
    if((Warn_Status[3]&0x80)==0x00)  
         rt_kprintf("        电源电压状态:     正常");  
	else
         rt_kprintf("        电源电压状态:      欠压报警");    
     //   GPRS  状态 
    if(ModuleStatus&Status_GPRS)
	 	        rt_kprintf("      GPRS 状态:   Online\r\n");  
    else	 
                      rt_kprintf("      GPRS 状态:   Offline\r\n");  	   
 
}
FINSH_FUNCTION_EXPORT(sys_status, Status);
#endif

void dispdata(char* instr)
{
     if (strlen((const char*)instr)==0)
	{
	     DispContent=1;
	    rt_kprintf("\r\n  默认等于 1\r\n"); 	  
	    return ;
	}
	else 
	{         
	       DispContent=(instr[0]-0x30);    
	  rt_kprintf("\r\n		Dispdata =%d \r\n",DispContent); 
	  return;  
	}
}
FINSH_FUNCTION_EXPORT(dispdata, Debug disp set) ; 


void Socket_main_Set(u8* str)
{
  u8 i=0;
  u8 reg_str[80];
  
	if (strlen((const char*)str)==0){
	    rt_kprintf("\r\n input error\r\n"); 
		return ;
	}
	else 
	{      
	  i = str2ipport((char*)str, RemoteIP_main, &RemotePort_main);
	  if (i <= 4) return ;;
	   
	  memset(reg_str,0,sizeof(reg_str));
	  IP_Str((char*)reg_str, *( u32 * ) RemoteIP_main);		   
	  strcat((char*)reg_str, " :"); 	  
	  sprintf((char*)reg_str+strlen((const char*)reg_str), "%u\r\n", RemotePort_main);  
         memcpy((char*)SysConf_struct.IP_Main,RemoteIP_main,4);
	  SysConf_struct.Port_main=RemotePort_main;
	 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));

        DataLink_MainSocket_set(RemoteIP_main,RemotePort_main,1);
		 DataLink_EndFlag=1; //AT_End();  
			return ;
	}

}
FINSH_FUNCTION_EXPORT(Socket_main_Set,Set Socket main); 

  void Socket_aux_Set(u8* str)
  {
	u8 i=0;
	u8 reg_str[80];
	
	  if (strlen((const char*)str)==0){
		  rt_kprintf("\r\n input error\r\n"); 
		  return ;
	  }
	  else 
	  { 	 
		i = str2ipport((char*)str, RemoteIP_aux, &RemotePort_aux);
		if (i <= 4) return ;;
		 
		memset(reg_str,0,sizeof(reg_str));
		IP_Str((char*)reg_str, *( u32 * ) RemoteIP_aux);		 
		strcat((char*)reg_str, " :");		
		sprintf((char*)reg_str+strlen((const char*)reg_str), "%u\r\n", RemotePort_aux);  
	    memcpy((char*)SysConf_struct.IP_Aux,RemoteIP_aux,4);
		SysConf_struct.Port_Aux=RemotePort_aux;
	    Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
  
		  DataLink_AuxSocket_set(RemoteIP_aux,RemotePort_aux,1);
			  return ;
	  }
  
  }
//  FINSH_FUNCTION_EXPORT(Socket_aux_Set,Set Aux main); 


  void  debug_relay(u8 *str)  
{
 if (strlen((const char*)str)==0)
	{
       rt_kprintf("\r\n继电器(1:断开0:闭合)JT808Conf_struct.relay_flag=%d",JT808Conf_struct.relay_flag);
       }
else 
	{
	       if(str[0]=='1')
		{
		 Car_Status[2]|=0x08;     // 需要控制继电器
		JT808Conf_struct.relay_flag=1;
		Enable_Relay();
		rt_kprintf("\r\n  断开继电器,JT808Conf_struct.relay_flag=%d\r\n",JT808Conf_struct.relay_flag); 
		}
	else if(str[0]=='0')
		{
		Car_Status[2]&=~0x08;    // 需要控制继电器
		JT808Conf_struct.relay_flag=0;
		Disable_Relay();
		rt_kprintf("\r\n  接通继电器,JT808Conf_struct.relay_flag=%d\r\n",JT808Conf_struct.relay_flag); 
		}
	}
 Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
 rt_kprintf("\r\n(debug_relay)状态信息,[0]=%X  [1]=%X  [2]=%X  [3]=%X",Car_Status[0],Car_Status[1],Car_Status[2],Car_Status[3]);	
 }
//FINSH_FUNCTION_EXPORT(debug_relay, Debug relay set) ;

//==========================================================
 
/*    
     -----------------------------
    3.  RT 驱动相关
     ----------------------------- 
*/



/*

       新驱动应用
 
*/



  //  1 .  循环存储 
      u8       Api_cycle_write(u8 *buffer, u16 len) 
      {
       // if(rt_mutex_take(DF_lock_mutex,150)==RT_EOK) 
	   {
	          WatchDog_Feed();
	          if( SaveCycleGPS(cycle_write,buffer,len))
		     { //---- updata pointer   -------------		
				cycle_write++;  	
			       if(cycle_write>=Max_CycleNum)
			  	               cycle_write=0;  
				DF_Write_RecordAdd(cycle_write,cycle_read,TYPE_CycleAdd);   
				DF_delay_us(20);  
		        //-------------------------------	
		       // rt_mutex_release(DF_lock_mutex);  //  释放
		        return true;
	            }  
		    else
		    {		         
				 //-------------------------------	 
				// rt_mutex_release(DF_lock_mutex);  //  释放
			     return  false;  
		    }	 
        }
	//	else
		//	 return false;
  	}

      u8      Api_cycle_read(u8 *buffer, u16 len) 
      {
                return( ReadCycleGPS(cycle_read, buffer, len));
  	}
       u8     Api_cycle_Update(void)
       {
           	 if( ReadCycle_status==RdCycle_SdOver)	 // 发送成功判断 
				 {		 
					 cycle_read++;	 //  收到应答才递增
					 if(cycle_read>=Max_CycleNum)
							cycle_read=0;
					 ReadCycle_status=RdCycle_Idle; 
					  rt_kprintf("\r\n 发送 GPS --saved  OK!\r\n");    
					   ReadCycle_timer=0; 
				 }	
				 return 1;
      }
	u8   Api_CHK_ReadCycle_status(void)
	{
                  CHK_ReadCycle_status();    // 发送状态的检查
		     return true;
	}

 // 2. Config 
        u8    Api_Config_write(u8 *name,u16 ID,u8* buffer, u16 wr_len)  
 	{
                if(strcmp((const char*)name,config)==0)
                {
                     DF_WriteFlashSector(ConfigStart_offset, 0, buffer, wr_len);
			DF_delay_ms(5);
			return true;		 
                }  
		   if(strcmp((const char*)name,tired_config)==0)
                {
                     DF_WriteFlashSector(TiredCondifg_offset, 0, buffer, wr_len);
			return true;		 
                } 		
		 return false;		
 	}
       u8      Api_Config_read(u8 *name,u16 ID,u8* buffer, u16 Rd_len)    //  读取Media area ID 是报数
       {
            if(strcmp((const char*)name,config)==0)
           	{
                     DF_ReadFlash(ConfigStart_offset, 0, buffer, Rd_len); 
					 DF_delay_ms(80); 	// large content delay	
					 return true;		
           	}
              if(strcmp((const char*)name,jt808)==0) 
             	{   
             	     WatchDog_Feed(); 
                     DF_ReadFlash(JT808Start_offset, 0, buffer, Rd_len); 
			         DF_delay_ms(100); // large content delay
			         return true;	
             	}
		   if(strcmp((const char*)name,tired_config)==0)
                {
                     DF_ReadFlash(TiredCondifg_offset, 0, buffer, Rd_len);
			DF_delay_ms(10); 		 
			return true;		 
                } 	  
     		   if(strcmp((const char*)name,BD_ext_config)==0)
     		   {
                     DF_ReadFlash(DF_BD_Extend_Page, 0, buffer, Rd_len); 
			DF_delay_ms(10); 		 
			return true;	
     		   }
         return false;
			
       }
 
       u8    Api_Config_Recwrite_Large(u8 *name,u16 ID,u8* buffer, u16 wr_len)  // 更新最新记录  
 	{
           if(strcmp((const char*)name,jt808)==0)
                {
                     WatchDog_Feed();
                     DF_WriteFlashSector(JT808Start_offset, 0, buffer, wr_len);  // formal  use
                     WatchDog_Feed(); 
					 DF_WriteFlashSector(JT808_BakSetting_offset,0,buffer,wr_len); // bak  setting   					 
					 WatchDog_Feed(); 
					 DF_WriteFlashSector(JT808_Bak2Setting_offset,0,buffer,wr_len); // bak  setting    	
			         return true;		 
                }
	    if(strcmp((const char*)name,BD_ext_config)==0)
	    	{
                    DF_WriteFlashSector(DF_BD_Extend_Page,0, buffer, wr_len);
                    return true;
	    	}				
		 return false;	
 	}
	  
 //  3.  其他 
       u8    Api_DFdirectory_Create(u8* name, u16 sectorNum)  // 
 	{ 
              return true ;  // NO  action here
 	}
       void   Api_MediaIndex_Init(void)
       {
                // Api_DFdirectory_Create(pic_index,pic_index_size);    //  图片索引
	          // Api_DFdirectory_Create(voice_index,voice_index_size);    //  声音索?
                 MediaIndex_Init();
       }

       u32  Api_DFdirectory_Query(u8 *name, u8  returnType)
       {         //  returnType=0  返回记录数目     returnType=1 时返回改目录文件大小
             u8   flag=0;
	      u32  pic_current_page=0;		 
			 
		 if(strcmp((const char*)name,spdpermin)==0)
		 	{return AvrgSpdPerMin_write;}
		  if(strcmp((const char*)name,tired_warn)==0)
		     {return TiredDrv_write ;}
		  if(strcmp((const char*)name,camera_1)==0)
		  {      pic_current_page=PicStart_offset;
		          flag=1;
		   }
		   if(strcmp((const char*)name,camera_2)==0)
		     {      pic_current_page=PicStart_offset2;
		          flag=1;
		   }
		   if(strcmp((const char*)name,camera_3)==0)
		   {      pic_current_page=PicStart_offset3;
		          flag=1;
		   }
		   if(strcmp((const char*)name,camera_4)==0)
			  {      pic_current_page=PicStart_offset4;
		          flag=1;
		   }
		   if(strcmp((const char*)name,voice)==0)
		   {      DF_ReadFlash(SoundStart_offdet,0,(u8*)&SrcFileSize,4);  
		           return SrcFileSize;
		   }
		   if(flag)
		   	{
		   	    DF_delay_ms(5);
		        DF_ReadFlash(pic_current_page, 0,PictureName, 23);
                      memcpy((u8*)&PicFileSize,PictureName+19,4);   
			 return   PicFileSize	;
		   }
		    return  0;
       }
	   
         u8 Api_DFdirectory_Write(u8 *name,u8 *buffer, u16 len) 
         {
		 
          if(strcmp((const char*)name,spdpermin)==0)
		  {
		            Save_PerMinContent(AvrgSpdPerMin_write,buffer, len);
			     //-----  Record update----		
			  AvrgSpdPerMin_write++;
			  if(AvrgSpdPerMin_write>=Max_SPDSperMin)
			     AvrgSpdPerMin_write=0;
			  DF_Write_RecordAdd(AvrgSpdPerMin_write, AvrgSpdPerMin_write, TYPE_AvrgSpdAdd);   
			     //----------------------	 
			return true;	 
               }
		 if(strcmp((const char*)name,spd_warn)==0) 
		{
			   Common_WriteContent( ExpSpdRec_write, buffer, len, TYPE_ExpSpdAdd);    
			   //-----  Record update----	
			  ExpSpdRec_write++;
			  if(ExpSpdRec_write>=Max_SPDSperMin)
			      ExpSpdRec_write=0;
	                DF_Write_RecordAdd(ExpSpdRec_write, ExpSpdRec_write, TYPE_ExpSpdAdd); 	  
			  //-----  Record update----	
			   return true;
               }
                if(strcmp((const char*)name,tired_warn)==0) 
		 {
			   Common_WriteContent( TiredDrv_write, buffer, len,  TYPE_TiredDrvAdd);      
			   //-----  Record update----	
			  TiredDrv_write++;
			  if(TiredDrv_write>=Max_CommonNum)  
			  	TiredDrv_write=0;			  
			  DF_delay_us(10);
			  DF_Write_RecordAdd(TiredDrv_write,TiredDrv_read,TYPE_TiredDrvAdd);   
			  //-------------------------
			  return  true;
                }
				
		 if(strcmp((const char*)name,doubt_data)==0)
		{ 
		       Save_DrvRecoder(Recorder_write, buffer, len );   
			//-----  Record update----	    
			   Recorder_write++; 
			   if(Recorder_write>=Max_RecoderNum)  
			   	 Recorder_write=0;
			   DF_Write_RecordAdd(Recorder_write,Recorder_Read,TYPE_VechRecordAdd); 	
		       //-------------------------		   
			return true;
               }	
		 //------- MultiMedia   RAW  data  ---------
		 if(strcmp((const char*)name,voice)==0)
		{
	            DF_WriteFlashDirect(SoundStart_offdet+Dev_Voice.Voice_PageCounter,0,Dev_Voice.Voice_Reg,500);  
                   return true;
		 }
		if(strcmp((const char*)name,camera_1)==0)
		{
                     DF_WriteFlashDirect(pic_current_page,0,buffer, len);
			return true;		 
               }
		if(strcmp((const char*)name,camera_2)==0)
		{
                     DF_WriteFlashDirect(pic_current_page,0,buffer, len);
			return true;		 
               }
		if(strcmp((const char*)name,camera_3)==0)
		{
                      DF_WriteFlashDirect(pic_current_page,0,buffer, len);
			return true;		  
               }
	       if(strcmp((const char*)name,camera_4)==0)
		{
                      DF_WriteFlashDirect(pic_current_page,0,buffer, len); 
			return true;		  
               }	 
		
              return false;
         }
          u8  Api_DFdirectory_Read(u8 *name,u8 *buffer, u16 len, u8  style ,u16 numPacket)  // style  1. old-->new   0 : new-->old 
         {   /*  连续调用几次 ，依次按style 方式递增*/
               // style  1. old-->new   0 : new-->old 
               //   numPacket    : 安装 style  方式读取开始 第几条数据包  from: 0
               u16   read_addr=0;
			   
			 DF_delay_ms(1); 
              if(strcmp((const char*)name,spdpermin)==0)
		    {
		        if(style==1)
					read_addr=0+numPacket;
			 else
			 { 
			      if(AvrgSpdPerMin_write==0)
				  	    return   false; 
				else  
			      if(AvrgSpdPerMin_write>=(numPacket+1))		
			            read_addr=AvrgSpdPerMin_write-1-numPacket;
				else
					 return false;
			 }  
		            Read_PerMinContent(read_addr,buffer, len);
			     //----------------------	 
			return true;	 
               }
		 if(strcmp((const char*)name,spd_warn)==0) 
		{
			 if(style==1)
					read_addr=0+numPacket;
			 else
			 { 
			      if(ExpSpdRec_write==0)
				  	    return   false; 
				else  
			      if(ExpSpdRec_write>=(numPacket+1))		
			            read_addr=ExpSpdRec_write-1-numPacket;
				else
					 return false;
			 }  	
			   Common_ReadContent( read_addr, buffer, len, TYPE_ExpSpdAdd);    
			   return true;
               }
                if(strcmp((const char*)name,tired_warn)==0) 
		 {
			   if(style==1)
					read_addr=0+numPacket;
			 else
			 { 
			      if(TiredDrv_write==0)
				  	    return   false; 
				else  
			      if(TiredDrv_write>=(numPacket+1))		
			            read_addr=TiredDrv_write-1-numPacket;
				else
					 return false;
			 } 
			   Common_ReadContent( read_addr, buffer, len,  TYPE_TiredDrvAdd);      

			  //-------------------------
			  return  true;
                }
				
		 if(strcmp((const char*)name,doubt_data)==0)
		{ 
		        if(style==1)
					read_addr=0+numPacket;
			 else
			 { 
			      if(Recorder_write==0)
				  	    return   false; 
				else  
			      if(Recorder_write>=(numPacket+1))		
			            read_addr=Recorder_write-1-numPacket;
				else
					 return false;
			 }  
		       Read_DrvRecoder(read_addr, buffer, len );   
	
		       //-------------------------		   
			return true;
               }	
		 //------- MultiMedia   RAW  data  ---------
		 if(strcmp((const char*)name,voice)==0)
		{
		     if(style==0)
					return  false;  //  只允许从old  -> new
	            DF_ReadFlash(SoundStart_offdet+numPacket,0, buffer, len);  
                   return true;
		 }
		if(strcmp((const char*)name,camera_1)==0)
		{
                     DF_ReadFlash(PicStart_offset+numPacket,0,buffer, len);
			return true;		 
               }
		if(strcmp((const char*)name,camera_2)==0)
		{
                     DF_ReadFlash(PicStart_offset2+numPacket,0,buffer, len);
			return true;		 
               }
		if(strcmp((const char*)name,camera_3)==0)
		{
                      DF_ReadFlash(PicStart_offset3+numPacket,0,buffer, len);
			return true;		  
               }
	       if(strcmp((const char*)name,camera_4)==0)
		{
                      DF_ReadFlash(PicStart_offset4+numPacket,0,buffer, len); 
			return true;		  
               }	 
		
              return false;
         }

	u8    Api_DFdirectory_Delete(u8* name)
 	{ 
	   
              if(strcmp((const char*)name,voice)==0)
		{
		                 WatchDog_Feed();    
				   SST25V_BlockErase_32KByte((SoundStart_offdet<<9));
				   DF_delay_ms(300);  
				   WatchDog_Feed(); 
			    return true;
               }
		if(strcmp((const char*)name,camera_1)==0)
		{
		       WatchDog_Feed(); 
                     SST25V_BlockErase_64KByte((PicStart_offset<<9));      
			DF_delay_ms(100);   	
			WatchDog_Feed(); 
			return true;		 
               }
		if(strcmp((const char*)name,camera_2)==0)
		{
		       WatchDog_Feed(); 
                     SST25V_BlockErase_64KByte((PicStart_offset2<<9));   
			DF_delay_ms(100); 	
			WatchDog_Feed(); 
			return true;			 
               }
		if(strcmp((const char*)name,camera_3)==0)
		{
		         WatchDog_Feed(); 
                       SST25V_BlockErase_64KByte((PicStart_offset3<<9));    
			  DF_delay_ms(100); 
			  WatchDog_Feed(); 
			  return true;	 		   
               }
	       if(strcmp((const char*)name,camera_4)==0)
		{
		        WatchDog_Feed(); 
                       SST25V_BlockErase_64KByte((PicStart_offset4<<9));   
		      DF_delay_ms(100); 
			  WatchDog_Feed(); 	 
			  return true;			   
               }  
		  return false; 
 	}
	  
         u8   Api_DFdirectory_DelteAll(void)  
         {
              return 1;  // no action  here
         }  
//-------  固定位置 序号记录  -----------
  u8   Api_RecordNum_Write( u8 *name,u8 Rec_Num,u8 *buffer, u16 len)    //  Rec_Num<128  Len<128
{
          WatchDog_Feed();
           if(strcmp((const char*)name,event_808)==0)
           	{
                   DF_WriteFlash(DF_Event_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}
           if(strcmp((const char*)name,msg_broadcast)==0)
		{
                   DF_WriteFlash(DF_Broadcast_offset+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}   	
	    if(strcmp((const char*)name,phonebook)==0)
		{
                   DF_WriteFlash(DF_PhoneBook_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}	
	    if(strcmp((const char*)name,Rail_cycle)==0)
		{
                   DF_WriteFlash(DF_Event_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}	
           if(strcmp((const char*)name,Rail_rect)==0)
	        {
                   DF_WriteFlash(DF_RectangleRail_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}		   	
	    if(strcmp((const char*)name,Rail_polygen)==0)	
	     {
                   DF_WriteFlash(DF_PolygenRail_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}			
	    if(strcmp((const char*)name,turn_point)==0)
		{
                   DF_WriteFlash(DF_turnPoint_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}		
           if(strcmp((const char*)name,route_line)==0)
		{
                   DF_WriteFlash(DF_Route_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}	   	
	    if(strcmp((const char*)name,ask_quesstion)==0)	
		{
                   DF_WriteFlash(DF_AskQuestion_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}		
	    if(strcmp((const char*)name,text_msg)==0)
	      {
                   DF_WriteFlash(DF_Msg_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}
  //-------------------------------
                if(strcmp((const char*)name,pic_index)==0)
                {
                            DF_WriteFlash(DF_PicIndex_Page+Rec_Num, 0,buffer, len); 
				return true;			
                }
		 if(strcmp((const char*)name,voice_index)==0)
		{
                            DF_WriteFlash(DF_SoundIndex_Page+Rec_Num, 0,buffer, len);
				return  true;			
              }
		 
		 return false;
}

  u8   Api_RecordNum_Read( u8 *name,u8 Rec_Num,u8 *buffer, u16 len)    //  Rec_Num<128  Len<128
  	{

             if(strcmp((const char*)name,event_808)==0)
           	{
                   DF_ReadFlash(DF_Event_Page+Rec_Num, 0,buffer, len);    
		    // DF_delay_ms(10);   		   
		     return true;		   
           	}
           if(strcmp((const char*)name,msg_broadcast)==0)
		{
                   DF_ReadFlash(DF_Broadcast_offset+Rec_Num, 0,buffer, len);    
                   //DF_delay_ms(10);   				   
		     return true;		   
           	}   	
	    if(strcmp((const char*)name,phonebook)==0)
		{
                 DF_ReadFlash(DF_PhoneBook_Page+Rec_Num, 0,buffer, len);   
		   //DF_delay_ms(10);   		   
		     return true;		   
           	}	
	    if(strcmp((const char*)name,Rail_cycle)==0)
		{
                   DF_ReadFlash(DF_Event_Page+Rec_Num, 0,buffer, len);   
		    // DF_delay_ms(10);   		   
		     return true;		   
           	}	
           if(strcmp((const char*)name,Rail_rect)==0)
	        {
                   DF_ReadFlash(DF_RectangleRail_Page+Rec_Num, 0,buffer, len);   
		    // DF_delay_ms(10);   		   
		     return true;		   
           	}		   	
	    if(strcmp((const char*)name,Rail_polygen)==0)	
	     {
                   DF_ReadFlash(DF_PolygenRail_Page+Rec_Num, 0,buffer, len);   
		     //DF_delay_ms(10);   		   
		     return true;		   
           	}			
	    if(strcmp((const char*)name,turn_point)==0)
		{
                   DF_ReadFlash(DF_turnPoint_Page+Rec_Num, 0,buffer, len);   
		    // DF_delay_ms(10);   		   
		     return true;		   
           	}		
           if(strcmp((const char*)name,route_line)==0)
		{
                   DF_ReadFlash(DF_Route_Page+Rec_Num, 0,buffer, len);   
		    DF_delay_ms(10); 		   
		     return true;		   
           	}	   	
	    if(strcmp((const char*)name,ask_quesstion)==0)	
		{
                   DF_ReadFlash(DF_AskQuestion_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}		
	    if(strcmp((const char*)name,text_msg)==0)
	      {
                   DF_ReadFlash(DF_Msg_Page+Rec_Num, 0,buffer, len);   
		     //DF_delay_ms(10); 		   
		     return true;		   
           	}
  	      if(strcmp((const char*)name,pic_index)==0)
                {
                            DF_ReadFlash(DF_PicIndex_Page+Rec_Num, 0,buffer, len);  
				return true;			
                }
		 if(strcmp((const char*)name,voice_index)==0)
		{
                            DF_ReadFlash(DF_SoundIndex_Page+Rec_Num, 0,buffer, len);
				//DF_delay_ms(10); 			
				return  true;			
              }
		 return false;

  	}
    u16  Api_RecordNum_Query(u8 *name) 
       {
       #if 0
             if(strcmp((const char*)name,event_808)==0)
           	{
                       
		     return event;		   
           	}
           if(strcmp((const char*)name,msg_broadcast)==0)
		{
                   DF_ReadFlash(DF_Broadcast_offset+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}   	
	    if(strcmp((const char*)name,phonebook)==0)
		{
                   DF_ReadFlash(DF_PhoneBook_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}	
	    if(strcmp((const char*)name,Rail_cycle)==0)
		{
                   DF_ReadFlash(DF_Event_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}	
           if(strcmp((const char*)name,Rail_rect)==0)
	        {
                   DF_ReadFlash(DF_RectangleRail_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}		   	
	    if(strcmp((const char*)name,Rail_polygen)==0)	
	     {
                   DF_ReadFlash(DF_PolygenRail_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}			
	    if(strcmp((const char*)name,turn_point)==0)
		{
                   DF_ReadFlash(DF_turnPoint_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}		
           if(strcmp((const char*)name,route_line)==0)
		{
                   DF_ReadFlash(DF_Route_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}	   	
	    if(strcmp((const char*)name,ask_quesstion)==0)	
		{
                   DF_ReadFlash(DF_AskQuestion_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}		
	    if(strcmp((const char*)name,text_msg)==0)
	      {
                   DF_ReadFlash(DF_Msg_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}



 // 3.  记录 
                if(strcmp((const char*)name,spd_warn)==0)
	      {
                   DF_ReadFlash(DF_Msg_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}
		    if(strcmp((const char*)name,tired_warn)==0)
	      {
                   DF_ReadFlash(DF_Msg_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}
		    if(strcmp((const char*)name,doubt_data)==0)
	      {
                   DF_ReadFlash(DF_Msg_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}
		    if(strcmp((const char*)name,spdpermin)==0)
	      {
                   DF_ReadFlash(DF_Msg_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}
		    if(strcmp((const char*)name,pospermin)==0)
	      {
                   DF_ReadFlash(DF_Msg_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}
		    if(strcmp((const char*)name,pic_index)==0)
	      {
                   DF_ReadFlash(DF_Msg_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}
		    if(strcmp((const char*)name,voice_index)==0)
	      {
                   DF_ReadFlash(DF_Msg_Page+Rec_Num, 0,buffer, len);   
		     return true;		   
           	}
              return  false;
	  #endif		  
						return 1;
       }
    void  Api_WriteInit_var_rd_wr(void)    //   写初始化话各类型读写记录地址
    	{
		 DF_Write_RecordAdd(cycle_write,cycle_read,TYPE_CycleAdd); 
		 DF_delay_ms(50);   
		 DF_Write_RecordAdd(pic_write,pic_read,TYPE_PhotoAdd);
		 DF_delay_ms(50);
		// DF_Write_RecordAdd(AvrgSpdPerSec_write,AvrgSpdPerSec_Read,TYPE_AvrgSpdSecAdd);
		// DF_delay_ms(50);  
		 //DF_Write_RecordAdd(Login_write,Login_Read,TYPE_LogInAdd);  
		// DF_delay_ms(50);  
		 DF_Write_RecordAdd(Settingchg_write,Settingchg_read,TYPE_SettingChgAdd);
		 DF_delay_ms(50);  						 
		 DF_Write_RecordAdd(AvrgMintPosit_write,AvrgMintPosit_Read,TYPE_MintPosAdd); 
		 DF_delay_ms(50);  
		 
               DF_Write_RecordAdd(Distance_m_u32,DayStartDistance_32,TYPE_DayDistancAdd); 
	 	 DF_Write_RecordAdd(ExpSpdRec_write,ExpSpdRec_read,TYPE_ExpSpdAdd);  
	 	 DF_delay_ms(50); 
		 
	 	 DF_Write_RecordAdd(TiredDrv_write,TiredDrv_read,TYPE_TiredDrvAdd); 
	 	 DF_delay_ms(50); 
	 	 DF_Write_RecordAdd(AvrgSpdPerMin_write,AvrgSpdPerMin_Read,TYPE_AvrgSpdAdd); 
		
		Recorder_write=0;
		Recorder_Read=0;  
		DF_Write_RecordAdd(Recorder_write,Recorder_Read,TYPE_VechRecordAdd); 



    	}
      void  Api_Read_var_rd_wr(void)    //   读初始化话各类型读写记录地址
    	{
             DF_Read_RecordAdd(cycle_write,cycle_read,TYPE_CycleAdd); 
	      DF_delay_ms(50); 
		DF_Read_RecordAdd(pic_write,pic_read,TYPE_PhotoAdd);    
		 DF_delay_ms(50); 
		//DF_Read_RecordAdd(AvrgSpdPerSec_write,AvrgSpdPerSec_Read,TYPE_AvrgSpdSecAdd);
		//DF_Read_RecordAdd(Login_write,Login_Read,TYPE_LogInAdd);  
		DF_Read_RecordAdd(Settingchg_write,Settingchg_read,TYPE_SettingChgAdd);
		DF_Read_RecordAdd(AvrgMintPosit_write,AvrgMintPosit_Read,TYPE_MintPosAdd); 

               DF_Read_RecordAdd(Distance_m_u32,DayStartDistance_32,TYPE_DayDistancAdd); 
	 	 DF_Read_RecordAdd(ExpSpdRec_write,ExpSpdRec_read,TYPE_ExpSpdAdd);  
	 	 DF_delay_ms(50); 
	 	
		 
	 	 DF_Read_RecordAdd(TiredDrv_write,TiredDrv_read,TYPE_TiredDrvAdd); 
	 	 DF_delay_ms(50); 
	 	 DF_Read_RecordAdd(AvrgSpdPerMin_write,AvrgSpdPerMin_Read,TYPE_AvrgSpdAdd); 
 		 DF_Read_RecordAdd(Recorder_write,Recorder_Read,TYPE_VechRecordAdd); 


    	}


//----------   TF 卡检查状态
u8     TF_Card_Status(void)
{                    //    1:  succed             0:  fail
           return 0 ;

}


		 
