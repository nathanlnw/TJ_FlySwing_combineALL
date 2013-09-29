#include  <stdlib.h>//êy×?×a??3é×?·?′?
#include  <stdio.h>
#include  <string.h>
#include <./App_moduleConfig.h>

unsigned char dispstat=0;
unsigned char tickcount=0;
unsigned int  reset_firstset=0;


unsigned char gsm_g[]={
0x1c,					/*[   ***  ]*/
0x22,					/*[  *   * ]*/
0x40,					/*[ *      ]*/
0x40,					/*[ *      ]*/
0x4e,					/*[ *  *** ]*/
0x42,					/*[ *    * ]*/
0x22,					/*[  *   * ]*/
0x1e,					/*[   **** ]*/
};


unsigned char gsm_0[]={
0x00,					/*[        ]*/	
0x00,					/*[        ]*/
0x00,					/*[        ]*/
0x00,					/*[        ]*/
0x00,					/*[        ]*/
0x00,					/*[        ]*/
0x80,					/*[*       ]*/
0x80,					/*[*       ]*/
};

unsigned char gsm_1[]={
0x00,					/*[        ]*/	
0x00,					/*[        ]*/
0x00,					/*[        ]*/
0x00,					/*[        ]*/
0x20,					/*[  *     ]*/
0x20,					/*[  *     ]*/
0xa0,					/*[* *     ]*/
0xa0,					/*[* *     ]*/
};

unsigned char gsm_2[]={
0x00,					/*[        ]*/	
0x00,					/*[        ]*/
0x08,					/*[    *   ]*/
0x08,					/*[    *   ]*/
0x28,					/*[  * *   ]*/
0x28,					/*[  * *   ]*/
0xa8,					/*[* * *   ]*/
0xa8,					/*[* * *   ]*/
};


unsigned char gsm_3[]={
0x02,					/*[      * ]*/	
0x02,					/*[      * ]*/
0x0a,					/*[    * * ]*/
0x0a,					/*[    * * ]*/
0x2a,					/*[  * * * ]*/
0x2a,					/*[  * * * ]*/
0xaa,					/*[* * * * ]*/
0xaa,					/*[* * * * ]*/
};

unsigned char link_on[]={
0x08,					/*[    *   ]*/	
0x04,					/*[     *  ]*/
0xfe,					/*[******* ]*/
0x00,					/*[        ]*/
0xfe,					/*[******* ]*/
0x40,					/*[ *      ]*/
0x20,					/*[  *     ]*/
0x00,					/*[        ]*/
};

unsigned char link_off[]={
0x10,					/*[   *    ]*/	
0x08,					/*[    *   ]*/
0xc6,					/*[**   ** ]*/
0x00,					/*[        ]*/
0xe6,					/*[***  ** ]*/
0x10,					/*[   *    ]*/
0x08,					/*[    *   ]*/
0x00,					/*[        ]*/
};
static unsigned char Battery[]={0x00,0xFC,0xFF,0xFF,0xFC,0x00};//8*6
static unsigned char NOBattery[]={0x04,0x0C,0x98,0xB0,0xE0,0xF8};//6*6
static unsigned char TriangleS[]={0x30,0x78,0xFC,0xFC,0x78,0x30};//6*6
static unsigned char TriangleK[]={0x30,0x48,0x84,0x84,0x48,0x30};//6*6


static unsigned char empty[]={0x84,0x84,0x84,0x84,0x84,0xFC}; /*空车*/
static unsigned char full_0[]={0x84,0x84,0x84,0xFC,0xFC,0xFC};/*半满*/
static unsigned char full_1[]={0xFC,0xFC,0xFC,0xFC,0xFC,0xFC};/*重车*/


//电池 是否校验特征系数的标志
DECL_BMP(8,6,Battery);  DECL_BMP(6,6,NOBattery);
DECL_BMP(6,6,TriangleS);DECL_BMP(6,6,TriangleK);
//信号强度标志
DECL_BMP(7,8,gsm_g);    DECL_BMP(7,8,gsm_0);
DECL_BMP(7,8,gsm_1);    DECL_BMP(7,8,gsm_2);
DECL_BMP(7,8,gsm_3);
//连接或者在线标志
DECL_BMP(7,8,link_on);  DECL_BMP(7,8,link_off);
//空车 半满 重车
DECL_BMP(6,6,empty);    DECL_BMP(6,6,full_0);  DECL_BMP(6,6,full_1);


void GPSGPRS_Status(void)
{
if(GpsStatus.Position_Moule_Status==1)
	lcd_text12(19,0,"BD",2,LCD_MODE_SET);
else if(GpsStatus.Position_Moule_Status==2)
	lcd_text12(19,0,"GPS",3,LCD_MODE_SET);
else if(GpsStatus.Position_Moule_Status==3)
	lcd_text12(19,0,"G/B",3,LCD_MODE_SET);
if(UDP_dataPacket_flag==3)
	lcd_bitmap(37,2,&BMP_link_off, LCD_MODE_SET);
else if(UDP_dataPacket_flag==2)
	lcd_bitmap(37,2,&BMP_link_on, LCD_MODE_SET);

lcd_text12(48,0,"GPRS",4,LCD_MODE_SET);
if(DEV_Login.Operate_enable==2)	
	lcd_bitmap(72,2,&BMP_link_on, LCD_MODE_SET);
else
	lcd_bitmap(72,2,&BMP_link_off, LCD_MODE_SET);

//----- 车辆模式  客  货   ------- add    by   nathan
if(Vechicle_Info.Vech_Type_Mark==1)  
	  lcd_text12(85,0,"K",1,LCD_MODE_SET);
else
if(Vechicle_Info.Vech_Type_Mark==2)	
	  lcd_text12(85,0,"H",1,LCD_MODE_SET);   





//车辆载重标志
if(JT808Conf_struct.LOAD_STATE==1)
	lcd_bitmap(95,2,&BMP_empty, LCD_MODE_SET);
else if(JT808Conf_struct.LOAD_STATE==2)
	lcd_bitmap(95,2,&BMP_full_0, LCD_MODE_SET);
else if(JT808Conf_struct.LOAD_STATE==3)
	lcd_bitmap(95,2,&BMP_full_1, LCD_MODE_SET);

//电源标志
if(ModuleStatus&0x04)
	lcd_bitmap(105,2,&BMP_Battery, LCD_MODE_SET);
else
	lcd_bitmap(105,2,&BMP_NOBattery, LCD_MODE_SET);

//是否校验特征系数
if(DF_K_adjustState)
	lcd_bitmap(115,2,&BMP_TriangleS, LCD_MODE_SET);
else
	lcd_bitmap(115,2,&BMP_TriangleK, LCD_MODE_SET);
}
void  Disp_Idle(void)
{
   u8 i=0;
   u16  disp_spd=0;
   u8  Date[3],Time[3];
   if(UDP_dataPacket_flag==0x02)
   	{
	Date[0]=Temp_Gps_Gprs.Date[0];
	Date[1]=Temp_Gps_Gprs.Date[1];
	Date[2]=Temp_Gps_Gprs.Date[2]; 

	Time[0]=Temp_Gps_Gprs.Time[0]; 
	Time[1]=Temp_Gps_Gprs.Time[1]; 
	Time[2]=Temp_Gps_Gprs.Time[2]; 
   	}
   else
   	{
   	time_now=Get_RTC(); 

	Date[0]= time_now.year;
	Date[1]= time_now.month;
	Date[2]= time_now.day;

	Time[0]= time_now.hour;
	Time[1]= time_now.min;
	Time[2]= time_now.sec;
   	}
	for(i=0;i<3;i++)
		Dis_date[2+i*3]=Date[i]/10+'0';
	for(i=0;i<3;i++)
		Dis_date[3+i*3]=Date[i]%10+'0';

	for(i=0;i<3;i++)
		Dis_date[12+i*3]=Time[i]/10+'0';
	for(i=0;i<3;i++)
		Dis_date[13+i*3]=Time[i]%10+'0'; 

       //----------------速度--------------------------
	 disp_spd=Speed_gps/10;
       if((disp_spd>=100)&&(disp_spd<200))
       	{
                    Dis_speDer[0]=disp_spd/100+'0';
		      Dis_speDer[1]=(disp_spd%100)/10+'0';
		      Dis_speDer[2]=disp_spd%10+'0';	     
					
       	}
	else
       if((disp_spd>=10)&&(disp_spd<100))
       	{
       	      Dis_speDer[0]=' ';
		      Dis_speDer[1]=(disp_spd/10)+'0';
		      Dis_speDer[2]=disp_spd%10+'0';	

       	}
	 else  
	if(disp_spd<10)
		{
		       Dis_speDer[0]=' ';
		      Dis_speDer[1]=' ';
		      Dis_speDer[2]=disp_spd%10+'0'; 
		}

       //---------------方向-----------------------------    
       memset(Dis_speDer+12,' ',8); // 初始化为空格

       // 1. 正北判断
       if((GPS_direction<=23)||(GPS_direction>337))	
	   	 memcpy(Dis_speDer+12,"正北",4);
	   // 2. 东北判断
       if((GPS_direction<=68)&&(GPS_direction>23))	
	   	 memcpy(Dis_speDer+12,"东北",4);
	     // 3. 正东判断
       if((GPS_direction<=113)&&(GPS_direction>=68))	
	   	 memcpy(Dis_speDer+12,"正东",4);
	     // 4.东南判断
       if((GPS_direction<=158)&&(GPS_direction>113))	
	   	 memcpy(Dis_speDer+12,"东南",4);
	     //5. 正南判断
       if((GPS_direction<=203)&&(GPS_direction>158))	
	   	 memcpy(Dis_speDer+12,"正南",4);
	     // 6. 西南判断
       if((GPS_direction<=248)&&(GPS_direction>=203))	
	   	 memcpy(Dis_speDer+12,"西南",4);
	     // 7. 正西判断
       if((GPS_direction<=293)&&(GPS_direction>=248))	
	   	 memcpy(Dis_speDer+12,"正西",4);
	     // 8.西北判断
       if((GPS_direction<=337)&&(GPS_direction>293))	
	   	 memcpy(Dis_speDer+12,"西北",4);  
  /*     
            if((GPS_direction>=100)&&(GPS_direction<=360))
       	{
                    Dis_speDer[12]=GPS_direction/100+'0';
		      Dis_speDer[13]=(GPS_direction%100)/10+'0';
		      Dis_speDer[14]=GPS_direction%10+'0';	     
					
       	}
	else
       if((GPS_direction>=10)&&(GPS_direction<100))
       	{
       	      Dis_speDer[12]=' ';
		      Dis_speDer[13]=(GPS_direction/10)+'0';
		      Dis_speDer[14]=GPS_direction%10+'0';	

       	}
	 else  
	if(GPS_direction<10)
		{
		       Dis_speDer[12]=' ';
		      Dis_speDer[13]=' ';
		      Dis_speDer[14]=GPS_direction%10+'0'; 
		}

*/
	//--------------------------------------------------   
    lcd_fill(0);	
	lcd_text12(0,10,(char *)Dis_date,20,LCD_MODE_SET);
	lcd_text12(0,20,(char *)Dis_speDer,18,LCD_MODE_SET);
	lcd_bitmap(0,3,&BMP_gsm_g, LCD_MODE_SET);

	// ---------- GSM 信号--------
	if(ModuleSQ>26)     //31/4	
	     lcd_bitmap(8,3,&BMP_gsm_3, LCD_MODE_SET);
	else
       if(ModuleSQ>18)	  
	    lcd_bitmap(8,3,&BMP_gsm_2, LCD_MODE_SET);	
	else   
	 if(ModuleSQ>9)	  
	    lcd_bitmap(8,3,&BMP_gsm_1, LCD_MODE_SET);	   
	else 
	     lcd_bitmap(8,3,&BMP_gsm_0, LCD_MODE_SET); 	  

	
	GPSGPRS_Status();
	
	lcd_update_all();
}
static void msg( void *p)
{
}
static void show(void)
{
	Disp_Idle();
	reset_firstset=0;
}


static void keypress(unsigned int key)
{
	switch(KeyValue)
		{
		case KeyValueMenu:
			Dis_deviceid_flag=0;
			CounterBack=0;
		    SetVIN_NUM=1;
			OK_Counter=0;
			
			CounterBack=0;
			UpAndDown=1;//
			

			pMenuItem=&Menu_2_InforCheck;
			pMenuItem->show();
            reset_firstset=0;
			break;
		case KeyValueOk:
			Dis_deviceid_flag=0;
			if(reset_firstset==0)
				reset_firstset=1;
			else if(reset_firstset==3)
				reset_firstset=4;
			else if(reset_firstset==4)
				reset_firstset=5;	
			else    // add later
				 reset_firstset=0;
			break;
		case KeyValueUP:
			Dis_deviceid_flag=0;
			if(reset_firstset==1)
				reset_firstset=2;
			else if(reset_firstset==2)
				reset_firstset=3;
			else if(reset_firstset==5)
				reset_firstset=6;
			else    // add later
				 reset_firstset=0;  
			break;
		case KeyValueDown:
			Dis_deviceid_flag=0;
            reset_firstset=0;
			//打印开电
			GPIO_SetBits(GPIOB,GPIO_Pin_7);

            //------------------------------------------------------
		    //gps_onoff(0);  //关掉GPS 模块的点
			print_workingFlag=1;  // 打印状态进行中
			Power_485CH1_OFF;     // 关闭485
			Speak_OFF;      //  关闭音频功放           
			//----------------------------------------------------
			if(print_rec_flag==0)
				print_rec_flag=1;//打印标志

			break;
		}
	KeyValue=0;	
}

static void timetick(unsigned int systick)  
{
	//u8 Reg_buf[22];  
	
if(reset_firstset==6)
	{
	reset_firstset++;
	//----------------------------------------------------------------------------------	
	 Login_Menu_Flag=0;     //  输入界面为0 
     DF_WriteFlashSector(DF_LOGIIN_Flag_offset,0,&Login_Menu_Flag,1);   
	//----------------------------------------------------------------------------------
	}
else if(reset_firstset>=7)//50ms一次,,60s
	{
	reset_firstset++;
	lcd_fill(0);
	lcd_text12(0,3,"需重新设置车牌号和ID",20,LCD_MODE_SET);
	lcd_text12(24,18,"重新加电查看",12,LCD_MODE_SET); 
	lcd_update_all();
	}
else if(Dis_deviceid_flag>=2)
	{
	Dis_deviceid_flag++;
	if(Dis_deviceid_flag>=50)
		Dis_deviceid_flag=0;
	}
else
	{
	//主电源掉电
	if(Warn_Status[1]&0x01)  
		{
		BuzzerFlag=11;
		lcd_fill(0);
		lcd_text12(30,10,"主电源掉电",10,LCD_MODE_SET); 
		lcd_update_all();
		}
	//循环显示待机界面
	tickcount++;
	if(tickcount>=5) 
		{
		  tickcount=0;
	      Disp_Idle();
		}
	}
    
Cent_To_Disp();

}

ALIGN(RT_ALIGN_SIZE)
MENUITEM	Menu_1_Idle=
{
    "待机界面",
	8,
	&show,
	&keypress,
	&timetick,
	&msg,
	(void*)0
};

