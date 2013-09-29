#include  <string.h>
#include "Menu_Include.h"

#define  Sim_width1  6


u8 Sim_SetFlag=1,Sim_SetFlag_temp=1,Sim_SetCounter=0;
u8 Sim_Modify_Flag=0;//有修改项
unsigned char select_Sim[]={0x0C,0x06,0xFF,0x06,0x0C};

DECL_BMP(8,5,select_Sim);


void Sim_Set(u8 par,u8 invert)
{
	lcd_fill(0);
	
	lcd_text12(0,3,(char *)Menu_sim_Code,Sim_SetFlag-1,LCD_MODE_SET);
	if(invert==1)
		{
		if(Sim_SetFlag_temp>=2)
			lcd_text12((Sim_SetFlag_temp-2)*6,3,(char *)&Menu_sim_Code[Sim_SetFlag_temp-2],1,LCD_MODE_INVERT);
		}
	else
		{
		if(invert==2)
			{
			if(Sim_SetFlag_temp>=2)
				lcd_text12((Sim_SetFlag_temp-2)*6,3,(char *)&Menu_sim_Code[Sim_SetFlag_temp-2],1,LCD_MODE_INVERT);
			}
	
		lcd_bitmap(par*Sim_width1, 14, &BMP_select_Sim, LCD_MODE_SET);
		lcd_text12(0,19,"0123456789",10,LCD_MODE_SET);
		}

	lcd_update_all();
}


static void show(void)
{
Sim_Set(Sim_SetCounter*Sim_width1,0);
}


static void keypress(unsigned int key)
{
	switch(KeyValue)
		{
		case KeyValueMenu:
            if(Sim_Modify_Flag==1)//有字符需要修改
            	{
            	if(Sim_SetFlag==14)
            		{
					memset(Menu_sim_Code,0,sizeof(Menu_sim_Code));
					
					Sim_SetFlag=1;
					Sim_SetFlag_temp=1;
					Sim_SetCounter=0;
	                Sim_Modify_Flag=0;//有修改项

                    pMenuItem=&Menu_0_loggingin;
					pMenuItem->show();
            		}
				else
					{
	            	Sim_Modify_Flag=2;//选择需要修改的字符
					Sim_SetFlag_temp=Sim_SetFlag;

	            	Sim_Set(Sim_SetCounter,1);
					}
				//rt_kprintf("\r\n按  菜单键  修改");
            	}
			else
				{
				pMenuItem=&Menu_0_loggingin;
				pMenuItem->show();
				memset(Menu_sim_Code,0,sizeof(Menu_sim_Code));
				
				Sim_SetFlag=1;
				Sim_SetFlag_temp=1;
				Sim_SetCounter=0;
                Sim_Modify_Flag=0;//有修改项
				}
			break;
			
		case KeyValueOk:
			if(Sim_Modify_Flag==2)
				{
				Sim_Modify_Flag=3;
				Sim_Set(0,2);///////
				//rt_kprintf("\r\n 确定修改位 Sim_SetFlag_temp = %d ",Sim_SetFlag_temp);
				}
			else if(Sim_Modify_Flag==3)
				{
				if(Sim_SetCounter<=9)
					Menu_sim_Code[Sim_SetFlag_temp-2]=Sim_SetCounter+'0';
				Sim_Set(0,0);
				//rt_kprintf("\r\n 单字符修改完成 Sim_SetFlag_temp = %d ");
				Sim_Modify_Flag=1;
				Sim_SetCounter=0;
				}
			else
				{
				if((Sim_SetFlag>=1)&&(Sim_SetFlag<=12))
					{
					Sim_Modify_Flag=1;
					//rt_kprintf("\r\n有修改项，Sim_SetFlag=%d",Sim_SetFlag);
					
					if(Sim_SetCounter<=9)
						Menu_sim_Code[Sim_SetFlag-1]=Sim_SetCounter+'0';
					
					Sim_SetFlag++;
					//rt_kprintf("\r\n  Sim_SetFlag = %d ,已设置字符串=%s",Sim_SetFlag,Menu_sim_Code);
				
					Sim_SetCounter=0;
					Sim_Set(0,0);
					}
				if(Sim_SetFlag==13)
					{
					Sim_SetFlag=14;
					if(MENU_set_carinfor_flag==1)
						{
						//rt_kprintf("\r\nSIM 卡号码设置完成，按菜单键返回，%s",Menu_sim_Code);
						
						memset(SimID_12D,0,sizeof(SimID_12D));
						memcpy(SimID_12D,Menu_sim_Code,12);								 
						DF_WriteFlashSector(DF_SIMID_12D,0,SimID_12D,13); 
						delay_ms(80); 		  
						//rt_kprintf("\r\n 显示屏设备SIM_ID设置为 : %s", SimID_12D);   
						DF_ReadFlash(DF_SIMID_12D,0,SimID_12D,13); 
						SIMID_Convert_SIMCODE();  // 转换

						rt_kprintf("\r\n 存储:%s",SimID_12D);
						}
					lcd_fill(0);
					lcd_text12(0,5,(char *)Menu_sim_Code,12,LCD_MODE_SET);
					lcd_text12(13,19,"SIM卡号设置完成",15,LCD_MODE_SET);
					lcd_update_all();		
					}
				else if(Sim_SetFlag==14)
					{
					CarSet_0_counter=4;
					Sim_SetFlag=1;

					Sim_SetFlag=1,
					Sim_SetFlag_temp=1,
					Sim_SetCounter=0;
					Sim_Modify_Flag=0;//有修改项
					
					pMenuItem=&Menu_0_loggingin;
					pMenuItem->show();
					}
				}
	
			break;
			
		case KeyValueUP:
			//选择需要修改的字符
			if(Sim_Modify_Flag==2)
				{
                if(Sim_SetFlag_temp>2)
					Sim_SetFlag_temp--;
				//rt_kprintf("\r\n Sim_SetFlag_temp=%d",Sim_SetFlag_temp);
				
				Sim_Set(Sim_SetCounter,1);
				}
			else
				{
				if((Sim_SetFlag>=1)&&(Sim_SetFlag<=12))
					{
					if(Sim_SetCounter==0)
						Sim_SetCounter=9;
					else if(Sim_SetCounter>=1)
						Sim_SetCounter--;
					if(Sim_Modify_Flag==3)
						Sim_Set(Sim_SetCounter,2);
					else
						Sim_Set(Sim_SetCounter,0);
					}
				}
			break;
			
		case KeyValueDown:
			//选择需要修改的字符
			if(Sim_Modify_Flag==2)
				{
                if(Sim_SetFlag_temp<Sim_SetFlag)
					Sim_SetFlag_temp++;
				//rt_kprintf("\r\n Sim_SetFlag_temp=%d",Sim_SetFlag_temp);
				
				Sim_Set(Sim_SetCounter,1);
				}
			else
				{
				if((Sim_SetFlag>=1)&&(Sim_SetFlag<=12)) 
					{
					Sim_SetCounter++;
					if(Sim_SetCounter>9)
						Sim_SetCounter=0;
						
					if(Sim_Modify_Flag==3)
						Sim_Set(Sim_SetCounter,2);
					else
						Sim_Set(Sim_SetCounter,0);
					}
				}
			break;
			
		}
	KeyValue=0;
}


static void timetick(unsigned int systick)
{

}


MENUITEM	Menu_0_3_Sim=
{
       "设置手机号码",
	12,
	&show,
	&keypress,
	&timetick,
	(void*)0
};


