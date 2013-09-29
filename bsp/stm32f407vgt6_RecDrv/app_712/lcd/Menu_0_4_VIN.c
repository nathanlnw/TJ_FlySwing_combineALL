#include  <string.h>
#include "Menu_Include.h"

#define  Sim_width1  6


static u8 VIN_SetFlag=1;
static u8 VIN_SetCounter=0;

//逐位修改休要
static u8 VIN_Modify_Flag=0;
static u8 VIN_SetFlag_Temp=1;


unsigned char select_vin[]={0x0C,0x06,0xFF,0x06,0x0C};
unsigned char ABC[36][1]={"0","1","2","3","4","5","6","7","8","9","A","B","C","D","E","F","G","H","I","J","K","L","M","N","O","P","Q","R","S","T","U","V","W","X","Y","Z"};


DECL_BMP(8,5,select_vin);


void Vin_Set(u8 par,u8 type1_2,u8 invert)
{
	lcd_fill(0);
	lcd_text12(0,3,(char *)Menu_Vin_Code,VIN_SetFlag-1,LCD_MODE_SET);
	if(invert==1)
		{
		if(VIN_SetFlag_Temp>=2)
			lcd_text12((VIN_SetFlag_Temp-2)*6,3,(char *)&Menu_Vin_Code[VIN_SetFlag_Temp-2],1,LCD_MODE_INVERT);
		}
	else
		{
		if(invert==2)
			{
			if(VIN_SetFlag_Temp>=2)
				lcd_text12((VIN_SetFlag_Temp-2)*6,3,(char *)&Menu_Vin_Code[VIN_SetFlag_Temp-2],1,LCD_MODE_INVERT);
			}
		if(type1_2==1)
			{
			lcd_bitmap(par*Sim_width1, 14, &BMP_select_vin, LCD_MODE_SET);
			lcd_text12(0,19,"0123456789ABCDEFGHIJ",20,LCD_MODE_SET);
			}
		else
			{
			lcd_bitmap((par-20)*Sim_width1, 14, &BMP_select_vin, LCD_MODE_SET);
			lcd_text12(0,19,"KLMNOPQRSTUVWXYZ",16,LCD_MODE_SET);
			}
		}

	lcd_update_all();
}

static void msg( void *p)
{

}
static void show(void)
{
CounterBack=0;
Vin_Set(VIN_SetCounter,1,0);
}


static void keypress(unsigned int key)
{
	switch(KeyValue)
		{
		case KeyValueMenu:
			 if(VIN_Modify_Flag==1)//有字符需要修改
            	{
            	if(VIN_SetFlag==18)
            		{
            		pMenuItem=&Menu_0_loggingin;
					pMenuItem->show();
					memset(Menu_Vin_Code,0,sizeof(Menu_Vin_Code));
	                
					VIN_SetFlag=1;
					VIN_SetCounter=0;
					VIN_Modify_Flag=0;
					VIN_SetFlag_Temp=1;
            		}
				else
					{
	            	VIN_Modify_Flag=2;//选择需要修改的字符
					VIN_SetFlag_Temp=VIN_SetFlag;

	            	Vin_Set(VIN_SetCounter,1,1);
					}
				//rt_kprintf("\r\n按  菜单键  修改");
            	}
			 else
			 	{
				pMenuItem=&Menu_0_loggingin;
				pMenuItem->show();
				memset(Menu_Vin_Code,0,sizeof(Menu_Vin_Code));
				VIN_SetFlag=1;
				VIN_SetCounter=0;
			 	}

			break;
		case KeyValueOk:
			if(VIN_Modify_Flag==2)
				{
				VIN_Modify_Flag=3;
				Vin_Set(VIN_SetCounter,1,2);
				}
			else if(VIN_Modify_Flag==3)
				{
				//if(VIN_SetCounter<=9)
				Menu_Vin_Code[VIN_SetFlag_Temp-2]=ABC[VIN_SetCounter][0];//VIN_SetCounter+'0';
				Vin_Set(0,1,0);
				//rt_kprintf("\r\n 单字符修改完成 Sim_SetFlag_temp = %d ");
				VIN_Modify_Flag=1;
				VIN_SetCounter=0;
				}
			else
				{
				if((VIN_SetFlag>=1)&&(VIN_SetFlag<=17))
					{
					Menu_Vin_Code[VIN_SetFlag-1]=ABC[VIN_SetCounter][0];
					//rt_kprintf("\r\n VIN_set=%d,%s",VIN_SetFlag,Menu_Vin_Code);
					VIN_SetFlag++;	
					VIN_SetCounter=0;
					Vin_Set(0,1,0);
					VIN_Modify_Flag=1;
					}		
			    if(VIN_SetFlag==18)
					{
					//rt_kprintf("\r\n VIN_set=%d,%s(ok)",VIN_SetFlag,Menu_Vin_Code);
					VIN_SetFlag=19;
					if(MENU_set_carinfor_flag==1)
						{
						 //车辆VIN
						memset(Vechicle_Info.Vech_VIN,0,sizeof(Vechicle_Info.Vech_VIN));
						memcpy(Vechicle_Info.Vech_VIN,Menu_Vin_Code,17);
						DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));    
						delay_ms(20);
						DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
						WatchDog_Feed();
					    DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 

					    //rt_kprintf("\r\nVIN 设置完成，按菜单键返回，%s",Menu_Vin_Code);
					    }
					lcd_fill(0);
					lcd_text12(0,5,(char *)Menu_Vin_Code,17,LCD_MODE_SET);
					lcd_text12(25,19,"VIN设置完成",11,LCD_MODE_SET);
					lcd_update_all();	
					//rt_kprintf("\r\n VIN_set=%d,%s(ok--)",VIN_SetFlag,Menu_Vin_Code);
					
					}
				else if(VIN_SetFlag==19)
					{
					VIN_SetFlag=1;
					CarSet_0_counter=5;

					pMenuItem=&Menu_0_loggingin;
					pMenuItem->show();
					}
				}
			
			break;
		case KeyValueUP:
			if(VIN_Modify_Flag==2)
				{
                if(VIN_SetFlag_Temp>2)
					VIN_SetFlag_Temp--;
				//rt_kprintf("\r\n Sim_SetFlag_temp=%d",Sim_SetFlag_temp);
				
				Vin_Set(VIN_SetCounter,1,1);
				}
			else
				{
				if((VIN_SetFlag>=1)&&(VIN_SetFlag<=17))
					{
					if(VIN_SetCounter==0)
						VIN_SetCounter=35;
					else if(VIN_SetCounter>=1)
						VIN_SetCounter--;
					
					if(VIN_Modify_Flag==3)
						{
						if(VIN_SetCounter<20)
							Vin_Set(VIN_SetCounter,1,2);
						else
							Vin_Set(VIN_SetCounter,2,2);
						}
					else
						{
						if(VIN_SetCounter<20)
							Vin_Set(VIN_SetCounter,1,0);
						else
							Vin_Set(VIN_SetCounter,2,0);
						}
					}
				}
			break;
		case KeyValueDown:
			//选择需要修改的字符
			if(VIN_Modify_Flag==2)
				{
                if(VIN_SetFlag_Temp<VIN_SetFlag)
					VIN_SetFlag_Temp++;
				//rt_kprintf("\r\n Sim_SetFlag_temp=%d",Sim_SetFlag_temp);
				
				Vin_Set(VIN_SetCounter,1,1);
				}
			else
				{
				if((VIN_SetFlag>=1)&&(VIN_SetFlag<=17))
					{
					VIN_SetCounter++;
					if(VIN_SetCounter>35)
						VIN_SetCounter=0;
					
					if(VIN_Modify_Flag==3)
						{
						if(VIN_SetCounter<20)
							Vin_Set(VIN_SetCounter,1,2);
						else
							Vin_Set(VIN_SetCounter,2,2);
						}
					else
						{
						if(VIN_SetCounter<20)
							Vin_Set(VIN_SetCounter,1,0);
						else
							Vin_Set(VIN_SetCounter,2,0);
						}
					}
				}
			break;
		}
	KeyValue=0;
}


static void timetick(unsigned int systick)
{
	/*CounterBack++;
	if(CounterBack!=MaxBankIdleTime*5)
		return;
	CounterBack=0;
	pMenuItem=&Menu_0_loggingin;
	pMenuItem->show();

	VIN_SetFlag=1;
	VIN_SetCounter=0;*/
}

ALIGN(RT_ALIGN_SIZE)
MENUITEM	Menu_0_4_vin=
{
"VIN设置",
	7,
	&show,
	&keypress,
	&timetick,
	&msg,
	(void*)0
};


