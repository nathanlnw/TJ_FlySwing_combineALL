#include  <string.h>
#include "Menu_Include.h"

u8 comfirmation_flag=0;
u8 col_screen=0;
u8 CarBrandCol_Cou=1;

unsigned char car_col[13]={"车牌颜色:黄色"}; 

void car_col_fun(u8 par)
{                                                      
                                           //车牌颜色编码表
if(par==1)
	memcpy(Menu_VecLogoColor,"蓝色",4);     //   1
else if(par==2)
	memcpy(Menu_VecLogoColor,"黄色",4);    //   2
else if(par==3)
	memcpy(Menu_VecLogoColor,"黑色",4);     //   3
else if(par==4)
	memcpy(Menu_VecLogoColor,"白色",4);    //   4
else if(par==5)
   {	memcpy(Menu_VecLogoColor,"其他",4);  par=9; } //   9
   
Menu_color_num=par; 

memcpy(car_col+9,Menu_VecLogoColor,4);  
lcd_fill(0);
lcd_text12(20,10,(char *)car_col,13,LCD_MODE_SET);
lcd_update_all();
}
static void msg( void *p)
{

}
static void show(void)
{
CounterBack=0;
col_screen=1;
car_col_fun(2);// old 1
}


static void keypress(unsigned int key)
{
	switch(KeyValue)
		{
		case KeyValueMenu:
			if(comfirmation_flag==4)
				{
				pMenuItem=&Menu_1_Idle;
				pMenuItem->show();
				}
			else
				{
				pMenuItem=&Menu_0_loggingin;
				pMenuItem->show();
				}
			col_screen=0;
			CarBrandCol_Cou=1;
			comfirmation_flag=0;
			break;
		case KeyValueOk:
             if(col_screen==1)
				{
				if(MENU_set_carinfor_flag==1)
					{
					comfirmation_flag=4;
					// 车牌颜色
					if(License_Not_SetEnable==1) 
						Vechicle_Info.Dev_Color=0;
					else
						Vechicle_Info.Dev_Color=Menu_color_num; 
					DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					
					lcd_fill(0);
					lcd_text12(12,3,"车牌颜色设置完成",16,LCD_MODE_SET);
					lcd_text12(24,18,"按菜单键返回",12,LCD_MODE_SET);
					lcd_update_all();
					rt_kprintf("\r\n车牌颜色设置完成，按菜单键返回，%d",Vechicle_Info.Dev_Color);
					}
				else
					{
					col_screen=2;
					CarSet_0_counter=0;//
					menu_color_flag=1;//车牌颜色设置完成
					lcd_fill(0);
					lcd_text12(20,3,(char *)car_col,13,LCD_MODE_SET);
					lcd_text12(12,18,"按确认键查看信息",16,LCD_MODE_SET);
					lcd_update_all();
					}
				}
			else if(col_screen==2)
				{
				menu_color_flag=0;
				
				col_screen=3;
				comfirmation_flag=1;//保存设置信息标志
				lcd_fill(0);
				if(License_Not_SetEnable==1)
					lcd_text12(0,0,"无牌照",6,LCD_MODE_SET);
				else
					lcd_text12(0,0,(char *)Menu_Car_license,8,LCD_MODE_SET);
				lcd_text12(54,0,(char *)Menu_VechileType,6,LCD_MODE_SET);
				      //====  车牌号未设置=====
                if(License_Not_SetEnable==1)
					 lcd_text12(96,0,(char *)"0",1,LCD_MODE_SET); 
			    else
				     lcd_text12(96,0,(char *)Menu_VecLogoColor,4,LCD_MODE_SET);
				
				lcd_text12(0,12,"SIM卡号",7,LCD_MODE_SET);
				lcd_text12(43,12,(char *)Menu_sim_Code,12,LCD_MODE_SET);
				lcd_text12(24,23,"确定",4,LCD_MODE_INVERT);
				lcd_text12(72,23,"取消",4,LCD_MODE_SET);
				lcd_update_all();
				}
			else if(comfirmation_flag==1)
				{
				col_screen=0;
				comfirmation_flag=4;
				//保存设置的信息
				lcd_fill(0);
				lcd_text12(18,3,"保存已设置信息",14,LCD_MODE_SET);
				lcd_text12(0,18,"按菜单键进入待机界面",20,LCD_MODE_SET);
				lcd_update_all();

                            //车牌号
                //rt_kprintf("\r\n(保存信息)Menu_Car_license=%s",Menu_Car_license);
				memset(Vechicle_Info.Vech_Num,0,sizeof(Vechicle_Info.Vech_Num));
				memcpy(Vechicle_Info.Vech_Num,Menu_Car_license,strlen((const char*)Menu_Car_license));
                //rt_kprintf("\r\n(保存信息)Vechicle_Info.Vech_Num=%s",Vechicle_Info.Vech_Num);
				// 车辆类型
				memset(Vechicle_Info.Vech_Type,0,sizeof(Vechicle_Info.Vech_Type));
				memcpy(Vechicle_Info.Vech_Type,Menu_VechileType,10);
                //rt_kprintf("\r\n(保存1   )Vechicle_Info.Vech_Num=%s",Vechicle_Info.Vech_Num);
                                
///////
				 //车辆VIN
				memset(Vechicle_Info.Vech_VIN,0,sizeof(Vechicle_Info.Vech_VIN));
				memcpy(Vechicle_Info.Vech_VIN,Menu_Vin_Code,17);
				//rt_kprintf("\r\n(保存2   )Vechicle_Info.Vech_Num=%s",Vechicle_Info.Vech_Num);
                 
				// SIM卡号码
				//memset(JT808Conf_struct.Vech_sim,0,sizeof(JT808Conf_struct.Vech_sim));
				//memcpy(JT808Conf_struct.Vech_sim,Menu_sim_Code,11);
				//-----------------------------------------------------------------------------
							  memset(SimID_12D,0,sizeof(SimID_12D));
							  memcpy(SimID_12D,Menu_sim_Code,12);									 
							  DF_WriteFlashSector(DF_SIMID_12D,0,SimID_12D,13); 
							  delay_ms(80); 		  
							  //rt_kprintf("\r\n 显示屏设备SIM_ID设置为 : %s", SimID_12D);   
							  DF_ReadFlash(DF_SIMID_12D,0,SimID_12D,13); 
							  SIMID_Convert_SIMCODE();  // 转换  
		              //----------------------------------------------------------------------------
                //rt_kprintf("\r\n(保存3   )Vechicle_Info.Vech_Num=%s",Vechicle_Info.Vech_Num);
                               
				// 车牌颜色
				if(License_Not_SetEnable==1) 
                     Vechicle_Info.Dev_Color=0;
				else
				     Vechicle_Info.Dev_Color=Menu_color_num;
				//车辆设置完成
				Login_Menu_Flag=1;     //  输入界面为0 
		        DF_WriteFlashSector(DF_LOGIIN_Flag_offset,0,&Login_Menu_Flag,1); 
				//  存储
				//-------------------------------------------------------------------------------------
                //  select mode

				 if(Vechicle_Info.Vech_Type_Mark==1)   //两客一危
				 	{
				 	    Vechicle_Info.Link_Frist_Mode=1;
                        Socket_main_Set("60.28.50.210:9131"); 
					    //--------    清除鉴权码 --------------------
					     idip("clear");		

				 	}
				 else
				 if(Vechicle_Info.Vech_Type_Mark==2)  //  公共货运平台
				 	{
                         Vechicle_Info.Link_Frist_Mode=0;
						 port_main("7008");
						 //--------    清除鉴权码 -------------------
					     idip("clear");		
				 	}
				 //------------------------------------------------------------------------------------				
				//rt_kprintf("\r\n(保存4   )Vechicle_Info.Vech_Num=%s",Vechicle_Info.Vech_Num);
				DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));  
				WatchDog_Feed();
				DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
				WatchDog_Feed();
				DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
				//error=Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
				//rt_kprintf("\r\n write  error=%d",error);
				delay_ms(3);
				DF_ReadFlash(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));  
				
				}
			else if(comfirmation_flag==2)
				{
				col_screen=0;
				comfirmation_flag=3;
				lcd_fill(0);
				lcd_text12(6, 3,"请确认是否重新设置",18,LCD_MODE_SET);
				lcd_text12(12,18,"按确认键重新设置",16,LCD_MODE_SET);
				lcd_update_all();
				}
			else if(comfirmation_flag==3)
				{
				col_screen=0;
				comfirmation_flag=0;
				//重新设置
				pMenuItem=&Menu_0_loggingin;
				pMenuItem->show();
				
				comfirmation_flag=0;
				col_screen=0;
				CarBrandCol_Cou=1;
				}

			break;
		case KeyValueUP:
			if(col_screen==1)
				{
				CarBrandCol_Cou--;
				if(CarBrandCol_Cou<1)
					CarBrandCol_Cou=5;
				car_col_fun(CarBrandCol_Cou);
				}
			else if(col_screen==3)
				{
				comfirmation_flag=1;

				lcd_fill(0);
				if(License_Not_SetEnable==1)
					lcd_text12(0,0,"无牌照",6,LCD_MODE_SET);
				else
					lcd_text12(0,0,(char *)Menu_Car_license,8,LCD_MODE_SET);
				lcd_text12(54,0,(char *)Menu_VechileType,6,LCD_MODE_SET);
				      //====  车牌号未设置=====
                if(License_Not_SetEnable==1)
					 lcd_text12(96,0,(char *)"0",1,LCD_MODE_SET); 
			    else
				     lcd_text12(96,0,(char *)Menu_VecLogoColor,4,LCD_MODE_SET);
				
				lcd_text12(0,12,"SIM卡号",7,LCD_MODE_SET);
				lcd_text12(43,12,(char *)Menu_sim_Code,12,LCD_MODE_SET);
				lcd_text12(24,23,"确定",4,LCD_MODE_INVERT);
				lcd_text12(72,23,"取消",4,LCD_MODE_SET);
				lcd_update_all();
				}

			break;
		case KeyValueDown:
			if(col_screen==1)
				{
				CarBrandCol_Cou++;
				if(CarBrandCol_Cou>5)
					CarBrandCol_Cou=1;
				car_col_fun(CarBrandCol_Cou);
				}
			else if(col_screen==3)
				{
				comfirmation_flag=2;
				lcd_fill(0);
				if(License_Not_SetEnable==1)
					lcd_text12(0,0,"无牌照",6,LCD_MODE_SET);
				else
					lcd_text12(0,0,(char *)Menu_Car_license,8,LCD_MODE_SET);
				lcd_text12(54,0,(char *)Menu_VechileType,6,LCD_MODE_SET);
				      //====  车牌号未设置=====
                if(License_Not_SetEnable==1)
					 lcd_text12(96,0,(char *)"0",1,LCD_MODE_SET); 
			    else
				     lcd_text12(96,0,(char *)Menu_VecLogoColor,4,LCD_MODE_SET);
				
				lcd_text12(0,12,"SIM卡号",7,LCD_MODE_SET);
				lcd_text12(43,12,(char *)Menu_sim_Code,12,LCD_MODE_SET);
				lcd_text12(24,23,"确定",4,LCD_MODE_SET);
				lcd_text12(72,23,"取消",4,LCD_MODE_INVERT);
				lcd_update_all();
				}

			break;
		}
	KeyValue=0;
}


static void timetick(unsigned int systick)
{

}

ALIGN(RT_ALIGN_SIZE)
MENUITEM	Menu_0_5_Colour=
{
"车辆颜色设置",
	12,
	&show,
	&keypress,
	&timetick,
	&msg,
	(void*)0
};


