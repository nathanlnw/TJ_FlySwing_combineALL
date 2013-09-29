/*
     SMS.C 
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
#include  "App_moduleConfig.h"
#include  "App_gsm.h"
#include  "SMS.h"


SMS_Style   SMS_Service;    //  短息相关    


/*********************************************************************************
*函数名称:void SMS_timer(u8 *instr,u16 len)
*功能描述:短信处理函数，这个函数需要在一个1秒的定时器里面调用，用于函数"SMS_Process"的定时处理等
*输    入:none
*输    出:none 
*返 回 值:none
*作    者:白养民
*创建日期:2013-05-29
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
void SMS_timer(void)
{
	if(SMS_Service.SMS_waitCounter)
		SMS_Service.SMS_waitCounter--;
}


/*********************************************************************************
*函数名称:void SMS_protocol(u8 *instr,u16 len)
*功能描述:短信处理函数，这个函数需要在一个线程里面调用，进行相关处理(短信读取，删除和自动发送相关)
*输    入:none
*输    出:none 
*返 回 值:none
*作    者:白养民
*创建日期:2013-05-29
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
void SMS_Process(void) 
{
	u16   	ContentLen=0;
//	u16 		i,j,k;
	char *pstrTemp;
	if(SMS_Service.SMS_waitCounter)
		return;
	//-----------  短信处理相关 -------------------------------------------------------- 
	//---------------------------------
	if(SMS_Service.SMS_read)	   // 读取短信
	{
		memset(SMS_Service.SMSAtSend,0,sizeof(SMS_Service.SMSAtSend));
		/*
		strcpy( ( char * ) SMS_Service.SMSAtSend, "AT+CMGR=" );	  
		if ( SMS_Service.SMIndex > 9 )
		{
			SMS_Service.SMSAtSend[8] = ( SMS_Service.SMIndex >> 4 ) + 0x30;
			SMS_Service.SMSAtSend[9] = ( SMS_Service.SMIndex & 0x0f ) + 0x30;
			SMS_Service.SMSAtSend[10] = 0x0d;
			SMS_Service.SMSAtSend[11] = 0x0a;
		}
		else
		{
			SMS_Service.SMSAtSend[8] = ( SMS_Service.SMIndex & 0x0f ) + 0x30;
			SMS_Service.SMSAtSend[9] = 0x0d;
			SMS_Service.SMSAtSend[10] = 0x0a;
		}
		rt_kprintf("\r\n%s",SMS_Service.SMSAtSend); 
		*/
		///
		sprintf(SMS_Service.SMSAtSend,"AT+CMGR=%d\r\n",SMS_Service.SMIndex);
		rt_kprintf("%s",SMS_Service.SMSAtSend); 
		///
		rt_hw_gsm_output( ( char * ) SMS_Service.SMSAtSend );   		
		SMS_Service.SMS_read--;  
		SMS_Service.SMS_waitCounter=3;
	}
	//-------------------------------
	//       发送短息确认
	else
	if(SMS_Service.SMS_sendFlag==1)
	{
		//#ifdef SMS_TYPE_PDU
		if(SMS_Service.SMS_come==1)
		{
			memset(SMS_Service.SMSAtSend,0,sizeof(SMS_Service.SMSAtSend));
			///申请400字节空间
			pstrTemp=rt_malloc(400);
			memset(pstrTemp,0,400);
			///将字符串格式的目的电话号码设置为PDU格式的号码
			SetPhoneNumToPDU(SMS_Service.Sms_Info.TPA, SMS_Service.SMS_destNum, sizeof(SMS_Service.Sms_Info.TPA));
			///生成PDU格式短信内容
			ContentLen=AnySmsEncode_NoCenter(SMS_Service.Sms_Info.TPA,GSM_UCS2,SMS_Service.SMS_sd_Content,strlen(SMS_Service.SMS_sd_Content),pstrTemp);
			//ContentLen=strlen(pstrTemp);
			///添加短信尾部标记"esc"
			pstrTemp[ContentLen]=0x1A;      // message  end  	
			//////
			sprintf( ( char * ) SMS_Service.SMSAtSend, "AT+CMGS=%d\r\n", (ContentLen-2)/2); 
			rt_kprintf("%s",SMS_Service.SMSAtSend); 
			rt_hw_gsm_output( ( char * ) SMS_Service.SMSAtSend );
			rt_thread_delay(50);
			//////	
			//rt_kprintf("%s",pstrTemp); 
			rt_device_write( &dev_vuart, 0, pstrTemp, strlen(pstrTemp) );
			rt_hw_gsm_output_Data( ( char * ) pstrTemp, ContentLen+1); 
			rt_free( pstrTemp );
			pstrTemp=RT_NULL;
		}
		//#else
		else
		{
			memset(SMS_Service.SMSAtSend,0,sizeof(SMS_Service.SMSAtSend));
			strcpy( ( char * ) SMS_Service.SMSAtSend, "AT+CMGS=\"" ); 
			//strcat(SMS_Service.SMSAtSend,"8613820554863");// Debug
			strcat(SMS_Service.SMSAtSend,SMS_Service.SMS_destNum);
			strcat(SMS_Service.SMSAtSend,"\"\r\n");

			rt_kprintf("\r\n%s",SMS_Service.SMSAtSend); 
			rt_hw_gsm_output( ( char * ) SMS_Service.SMSAtSend );

			rt_thread_delay(50);
			ContentLen=strlen(SMS_Service.SMS_sd_Content);
			SMS_Service.SMS_sd_Content	[ContentLen]=0x1A;      // message  end  		
			rt_kprintf("%s",SMS_Service.SMS_sd_Content); 
			rt_hw_gsm_output_Data( ( char * ) SMS_Service.SMS_sd_Content, ContentLen+1); 
		}
		//#endif
		SMS_Service.SMS_sendFlag=0;  // clear 
		SMS_Service.SMS_waitCounter=3;
	} 
	/*
	else if(SMS_Service.SMS_delALL==1)	  //删除短信
	{	   
		memset(SMS_Service.SMSAtSend,0,sizeof(SMS_Service.SMSAtSend));
		///
		//sprintf(SMS_Service.SMSAtSend,"AT+CMGD=%d\r\n",SMS_Service.SMIndex);
		sprintf(SMS_Service.SMSAtSend,"AT+CMGD=0,4\r\n",SMS_Service.SMIndex);
		rt_kprintf("%s",SMS_Service.SMSAtSend); 
		///
		rt_hw_gsm_output( ( char * )SMS_Service.SMSAtSend ); 
		SMS_Service.SMS_delALL=0; 
		SMS_Service.SMS_waitCounter=3;
	}
	*/
}


///增加发送短信区域的内容，并置位发送短息标记，成功返回true，失败返回false
u8 Add_SMS_Ack_Content(char * instr,u8 ACKflag)
{
    if(ACKflag==0)
		 return false;  
     
	if(strlen(instr)+strlen(SMS_Service.SMS_sd_Content) < sizeof(SMS_Service.SMS_sd_Content))
		{
		 	strcat((char *)SMS_Service.SMS_sd_Content,instr);
			//rt_kprintf(" \r\n  enable  send msg! \r\n"); 
		 	SMS_Service.SMS_sendFlag=1;
			return true;
		}
	return false;
}

/*********************************************************************************
*函数名称:void SMS_protocol(u8 *instr,u16 len)
*功能描述:接收到短信后参数处理函数
*输    入:instr原始短信数据，len长度
*输    出:none 
*返 回 值:none
*作    者:白养民
*创建日期:2013-05-29
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
void   SMS_protocol (u8 *instr,u16 len, u8  ACKstate)   //  ACKstate 
{
	char	sms_content[60];		///短信命令区"()"之间的内容
	char	sms_ack_data[60];		///短信每个命令包括'#'符的完整内容
	u8	u8TempBuf[6];
	u16	i=0,j=0,res=0;
	u16  cmdLen,u16Temp;
	char *p_Instr;
	char *pstrTemp,*pstrTempStart,*pstrTempEnd;

	//SYSID		///修改该值，保存flash
	///应答短信包头部分
	memset(SMS_Service.SMS_sd_Content,0,sizeof(SMS_Service.SMS_sd_Content));
	strcpy(SMS_Service.SMS_sd_Content,Vechicle_Info.Vech_Num);
	strcat(SMS_Service.SMS_sd_Content,"#");// Debug
	strcat(SMS_Service.SMS_sd_Content,SimID_12D);// Debug
	if(Vechicle_Info.Vech_Type_Mark==1)  
	      strcat(SMS_Service.SMS_sd_Content,"#TJK");// 版本信息  
	else
	if(Vechicle_Info.Vech_Type_Mark==2)  
	      strcat(SMS_Service.SMS_sd_Content,"#TJH");// 版本信息   
	/*************************处理信息****************************/
	p_Instr=(char *)instr;
	for(i=0;i<len;i++)
		{
		pstrTemp=strchr(p_Instr,'#');					///查找命令是否存在
		//instr++;
		if(pstrTemp)
			{
			p_Instr=pstrTemp+1;
			pstrTempStart=strchr((char *)pstrTemp,'(');			///查找命令内容开始位置
			pstrTempEnd=strchr((char *)pstrTemp,')');			///查找命令内容结束位置
			if((NULL==pstrTempStart)||(NULL==pstrTempEnd))
				{
				break;
				}
			rt_kprintf("\r\n短信命令格式有效 !");
			///获取命令内容
			memset(sms_ack_data,0,sizeof(sms_ack_data));
			memcpy(sms_ack_data,pstrTemp,pstrTempEnd-pstrTemp+1);

			///获取命令参数区域的内容以及参数长度
			pstrTempStart++;
			pstrTemp++;
			cmdLen=pstrTempEnd-pstrTempStart;
			memset(sms_content,0,sizeof(sms_content));
			rt_memcpy(sms_content,pstrTempStart,cmdLen);

			///进行命令匹配
			if(strncmp(pstrTemp,"DNSR",4)==0)				///  1. 设置域名
				{
				if(cmdLen<=sizeof(DomainNameStr))
					{
					if(pstrTemp[4]=='1')		///主域名
						{
						rt_kprintf("\r\n设置主域名 !");


						
						memset(DomainNameStr,0,sizeof(DomainNameStr));					  
						memset(SysConf_struct.DNSR,0,sizeof(DomainNameStr));  
						memcpy(DomainNameStr,(char*)pstrTempStart,cmdLen);
						memcpy(SysConf_struct.DNSR,(char*)pstrTempStart,cmdLen);
						Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
						//----- 传给 GSM 模块------
						DataLink_DNSR_Set(SysConf_struct.DNSR,1); 

						// ------ 判断域名内容-------
						res=memcmp((u8*)DomainNameStr,"jt1.gghypt.net",14);
						if(res)
							type_vech(2); // 货运     
						
						//
						Add_SMS_Ack_Content(sms_ack_data,ACKstate);

						//------- add on 2013-6-6
						if(ACKstate==SMS_ACK_none)
						     SD_ACKflag.f_CentreCMDack_0001H=2 ;//DataLink_EndFlag=1; //AT_End(); 先返回结果再挂断  
						else
							   DataLink_EndFlag=1; //AT_End();  
							   
                         //--------    清除鉴权码 -------------------
					     idip("clear");		   

						}
					else if(pstrTemp[4]=='2')	///备用域名
						{
						rt_kprintf("\r\n设置备用域名 !");
						memset(DomainNameStr_aux,0,sizeof(DomainNameStr_aux));					  
						memset(SysConf_struct.DNSR_Aux,0,sizeof(DomainNameStr_aux));
						memcpy(DomainNameStr_aux,(char*)pstrTempStart,cmdLen);
						memcpy(SysConf_struct.DNSR_Aux,(char*)pstrTempStart,cmdLen);
						Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
						//----- 传给 GSM 模块------
						DataLink_DNSR2_Set(SysConf_struct.DNSR_Aux,1);
						
						///
						Add_SMS_Ack_Content(sms_ack_data,ACKstate);
						}
					else
						{
						continue;
						}
					}
				}
			else if(strncmp(pstrTemp,"PORT",4)==0)			///2. 设置端口
				{
				j=sscanf(sms_content,"%u",&u16Temp);
				if(j)
					{
					if(pstrTemp[4]=='1')		///主端口
						{
						rt_kprintf("\r\n设置主端口=%d!",u16Temp);
						RemotePort_main=u16Temp;
						SysConf_struct.Port_main=u16Temp;
						Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
						//----- 传给 GSM 模块------
						DataLink_MainSocket_set(RemoteIP_main,RemotePort_main,1);
						///
						Add_SMS_Ack_Content(sms_ack_data,ACKstate);

						//------- add on 2013-6-6
						if(ACKstate==SMS_ACK_none)
						     SD_ACKflag.f_CentreCMDack_0001H=2 ;//DataLink_EndFlag=1; //AT_End(); 先返回结果再挂断  
						else
							  DataLink_EndFlag=1; //AT_End();  

						 //--------    清除鉴权码 -------------------
					     idip("clear");		   	  

						}
					else if(pstrTemp[4]=='2')	///备用端口
						{
						rt_kprintf("\r\n设置备用端口=%d!",u16Temp);
						RemotePort_aux=u16Temp;
						SysConf_struct.Port_Aux=u16Temp;
						Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
						//----- 传给 GSM 模块------
						DataLink_AuxSocket_set(RemoteIP_aux,RemotePort_aux,1);
						///
						Add_SMS_Ack_Content(sms_ack_data,ACKstate);
						}
					else
						{
						continue;
						}
					}
				
				}
			else if(strncmp(pstrTemp,"DUR",3)==0)				///3. 修改发送间隔
				{
				j=sscanf(sms_content,"%u",&u16Temp);
				if(j)
					{
					
						rt_kprintf("\r\n修改发送间隔! %d",u16Temp);
						dur(sms_content);

					///
					Add_SMS_Ack_Content(sms_ack_data,ACKstate);
					}
				}
			else if(strncmp(pstrTemp,"DEVICEID",8)==0)			///4. 修改终端ID
				{
				if(cmdLen<=sizeof(DeviceNumberID))
					{
					rt_kprintf("\r\n修改终端ID  !");
					memset(DeviceNumberID,0,sizeof(DeviceNumberID));
					memcpy(DeviceNumberID,pstrTempStart,cmdLen);
					DF_WriteFlashSector(DF_DeviceID_offset,0,DeviceNumberID,13); 
					///
					Add_SMS_Ack_Content(sms_ack_data,ACKstate);

					 //--------    清除鉴权码 -------------------
					     idip("clear");		     
					
					}
				else
					{
					       continue;
					}
				}    //DeviceNumberID			
				else if(strncmp(pstrTemp,"SIMID",5)==0)			///4. 修改SIMID
				{
				if(cmdLen<=sizeof(SimID_12D))
					{
					rt_kprintf("\r\n修改SIM_ID  !");
					memset(SimID_12D,0,sizeof(SimID_12D));
					memcpy(SimID_12D,pstrTempStart,cmdLen);
					DF_WriteFlashSector(DF_SIMID_12D,0,SimID_12D,13);  
					SIMID_Convert_SIMCODE(); //  转换
					
					///
					Add_SMS_Ack_Content(sms_ack_data,ACKstate);

					 //--------    清除鉴权码 -------------------
					     idip("clear");		     
					
					}
				    else
					{
					       continue;
					}
				}    //DeviceNumberID		
				else if(strncmp(pstrTemp,"PHONENUM",8)==0)			///4. 修改SIMID
				{
				   if(cmdLen==11)
					{
						rt_kprintf("\r\n修改PHONENUM_ID  !"); 
						memset(SimID_12D,0,sizeof(SimID_12D));
						SimID_12D[0]='0'; //    第一位填写 0  
						memcpy(SimID_12D+1,pstrTempStart,cmdLen);
						DF_WriteFlashSector(DF_SIMID_12D,0,SimID_12D,13);  
						SIMID_Convert_SIMCODE(); //  转换
						
						///
						Add_SMS_Ack_Content(sms_ack_data,ACKstate);
						 //--------    清除鉴权码 -------------------
						     idip("clear");		      
					
					}
				else
					{
					       continue;
					}
				}    //DeviceNumberID		
			else if(strncmp(pstrTemp,"IP",2)==0)				///5.设置IP地址
				{
				j = sscanf(sms_content, "%u.%u.%u.%u", (u32*)&u8TempBuf[0], (u32*)&u8TempBuf[1], (u32*)&u8TempBuf[2], (u32*)&u8TempBuf[3]);
				//j=str2ip(sms_content, u8TempBuf);
				if(j==4)
				 	{
				 	rt_kprintf("\r\n设置IP地址!");
					if(pstrTemp[2]=='1')
						{ 
						memcpy(SysConf_struct.IP_Main,u8TempBuf,4);
						memcpy(RemoteIP_main,u8TempBuf,4);
						SysConf_struct.Port_main=RemotePort_main;
						Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
						rt_kprintf("\r\n短信设置主服务器 IP: %d.%d.%d.%d : %d ",RemoteIP_main[0],RemoteIP_main[1],RemoteIP_main[2],RemoteIP_main[3],RemotePort_main);
						//-----------  Below add by Nathan  ----------------------------
						DataLink_MainSocket_set(RemoteIP_main,RemotePort_main,1);

						if((RemoteIP_main[0]==60)&&(RemoteIP_main[1]==28)&&(RemoteIP_main[2]==50)&&(RemoteIP_main[3]==210)) 
							type_vech(1); 
						///
						Add_SMS_Ack_Content(sms_ack_data,ACKstate);

					     //------- add on 2013-6-6
						if(ACKstate==SMS_ACK_none)
						      SD_ACKflag.f_CentreCMDack_0001H=2 ;//DataLink_EndFlag=1; //AT_End(); 先返回结果再挂断  
						else
							   DataLink_EndFlag=1;

						 //--------    清除鉴权码 -------------------
					     idip("clear");		   

						}
					else if(pstrTemp[2]=='2')
						{
						memcpy(SysConf_struct.IP_Aux,u8TempBuf,4);
						memcpy(RemoteIP_aux,u8TempBuf,4);
						SysConf_struct.Port_Aux=RemotePort_aux;
						Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
						rt_kprintf("\r\n短信设置备用服务器 IP: %d.%d.%d.%d : %d ",RemoteIP_aux[0],RemoteIP_aux[1],RemoteIP_aux[2],RemoteIP_aux[3],RemotePort_aux);   
						//-----------  Below add by Nathan  ----------------------------
						DataLink_AuxSocket_set(RemoteIP_aux,RemotePort_aux,1);
						///
						Add_SMS_Ack_Content(sms_ack_data,ACKstate);
						}
					}
				}
			  if(strncmp(pstrTemp,"MODE",4)==0)			///6. 设置定位模式
				{
				        if(strncmp(sms_content,"BD",2)==0)
				        {
                                                 gps_mode("1");
				        }
					  if(strncmp(sms_content,"GP",2)==0)
				        {
                                                 gps_mode("2");
				        }	
					   if(strncmp(sms_content,"GN",2)==0)
				        {
                                                 gps_mode("3");  
				        }
					   Add_SMS_Ack_Content(sms_ack_data,ACKstate);
				}
			else if(strncmp(pstrTemp,"VIN",3)==0)				///7.设置车辆VIN
				{
				     vin_set(sms_content);
				    Add_SMS_Ack_Content(sms_ack_data,ACKstate);
				}
			else if(strncmp(pstrTemp,"TIREDCLEAR",10)==0)		///8.清除疲劳驾驶记录
				{
				      TiredDrv_write=0;
				      TiredDrv_read=0;	   
				      DF_Write_RecordAdd(TiredDrv_write,TiredDrv_read,TYPE_TiredDrvAdd);      
				      Add_SMS_Ack_Content(sms_ack_data,ACKstate);	   
				}
			else if(strncmp(pstrTemp,"DISCLEAR",8)==0)			///9清除里程
				{
				   	  JT808Conf_struct.DayStartDistance_32=0;
					  JT808Conf_struct.Distance_m_u32=0;
                                     Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
					   Add_SMS_Ack_Content(sms_ack_data,ACKstate);				 
				}
            else if(strncmp(pstrTemp,"SPDWARNCLEAR",12)==0)    //  超速报警清除
            	{
            	  
					 //-----  Record update---- 	
				     AvrgSpdPerMin_write=0;
					 AvrgSpdPerMin_write=0;
				     DF_Write_RecordAdd(AvrgSpdPerMin_write, AvrgSpdPerMin_write, TYPE_AvrgSpdAdd);  
					 Add_SMS_Ack_Content(sms_ack_data,ACKstate);	

            	}
			else if(strncmp(pstrTemp,"RESET",5)==0)			///10.终端重启
				{
				      reset();
				}
			else if(strncmp(pstrTemp,"RELAY",5)==0)			///11.继电器控制
				{
				       if(sms_content[0]=='0')
				              debug_relay("0");
				       if(sms_content[0]=='1')
					   	debug_relay("1"); 

					Add_SMS_Ack_Content(sms_ack_data,ACKstate);	    
				}
			else if(strncmp(pstrTemp,"TAKE",4)==0)				//12./拍照
				{
				    switch(sms_content[0])
				   {
                                    case '1':
                                                 takephoto("1");
							break;		
					case '2':
                                                  takephoto("2"); 
							break;		
				       case '3':
                                                takephoto("3");
							break;	
				       case '4':
                                                  takephoto("4");
							break;
				   }
				    Add_SMS_Ack_Content(sms_ack_data,ACKstate);	
				}
			else if(strncmp(pstrTemp,"PLAY",4)==0)				///13.语音播报
				{
				     TTS_Get_Data(sms_content,strlen(sms_content));
				     Add_SMS_Ack_Content(sms_ack_data,ACKstate);		  
				}
			else if(strncmp(pstrTemp,"QUERY",5)==0)			///14.车辆状态查询
				{
				       switch(sms_content[0])
				     	{
				     	    case '0':  // 终端号  主域名  主端口
								     
									 strcat((char *)SMS_Service.SMS_sd_Content,sms_ack_data);
									 strcat(SMS_Service.SMS_sd_Content,"#");// 分隔符号
                                     strcat(SMS_Service.SMS_sd_Content,DeviceNumberID);// 终端ID
                                     strcat(SMS_Service.SMS_sd_Content,"#");// 分隔符号
                                     strcat(SMS_Service.SMS_sd_Content,DomainNameStr);// 分隔符号
                                     strcat(SMS_Service.SMS_sd_Content,"#");// 分隔符号
                                     sprintf(SMS_Service.SMS_sd_Content+strlen(SMS_Service.SMS_sd_Content),"%d",RemotePort_main); 
								
								   break;
							case '1':  // MODE    主IP   主端口
								    strcat((char *)SMS_Service.SMS_sd_Content,sms_ack_data);
									strcat(SMS_Service.SMS_sd_Content,"#");// 分隔符号
									if(Vechicle_Info.Link_Frist_Mode==1)  
                                         strcat(SMS_Service.SMS_sd_Content,"MODE(mainlink)");
									else
                                         strcat(SMS_Service.SMS_sd_Content,"MODE(dnsr)");
									strcat(SMS_Service.SMS_sd_Content,"#");// 分隔符号
									IP_Str(SMS_Service.SMS_sd_Content+strlen(SMS_Service.SMS_sd_Content), *( u32 * ) RemoteIP_main);
								    strcat(SMS_Service.SMS_sd_Content,"#");// 分隔符号
                                    sprintf(SMS_Service.SMS_sd_Content+strlen(SMS_Service.SMS_sd_Content),"%d",RemotePort_main);  
												
								    break;
                            case '2':
								    strcat(SMS_Service.SMS_sd_Content,"#");// 分隔符号
                                    strcat(SMS_Service.SMS_sd_Content,Posit_ASCII.Lat_ASCII);// 分隔符号
                                    strcat(SMS_Service.SMS_sd_Content,"#");// 分隔符号
                                    strcat(SMS_Service.SMS_sd_Content,Posit_ASCII.Longi_ASCII);// 分隔符号
                                    strcat(SMS_Service.SMS_sd_Content,"#");// 分隔符号
                                    strcat(SMS_Service.SMS_sd_Content,&Posit_ASCII.AV_ASCII);// 分隔符号
									strcat(SMS_Service.SMS_sd_Content,"#");// 分隔符号
								    if(GpsStatus.Position_Moule_Status==1)
									{
									  strcat(SMS_Service.SMS_sd_Content,"BD");
									}
								else if(GpsStatus.Position_Moule_Status==2)
									{
									  strcat(SMS_Service.SMS_sd_Content,"GPS");
									}
								else if(GpsStatus.Position_Moule_Status==3)
									{
									  strcat(SMS_Service.SMS_sd_Content,"BD+GPS");
									}
                                     strcat(SMS_Service.SMS_sd_Content,"#");// 分隔符号
                                     sprintf(SMS_Service.SMS_sd_Content+strlen(SMS_Service.SMS_sd_Content),"%d",ModuleSQ); 
								
								    break; 
						   case '3':  // 省域ID  市域ID
						   	       strcat(SMS_Service.SMS_sd_Content,"provinceid(");// 分隔符号						   	       
							       sprintf(SMS_Service.SMS_sd_Content+strlen(SMS_Service.SMS_sd_Content),"%d",Vechicle_Info.Dev_ProvinceID); 
							       strcat(SMS_Service.SMS_sd_Content,")#");// 分隔符号							       
						   	       strcat(SMS_Service.SMS_sd_Content,"cityid(");// 分隔符号						   	       
							       sprintf(SMS_Service.SMS_sd_Content+strlen(SMS_Service.SMS_sd_Content),"%d)",Vechicle_Info.Dev_CityID); 
           					        break;
				     	}	
					   
					    if(ACKstate)
							SMS_Service.SMS_sendFlag=1;  // 发送返回短息标志位
				}
			else if(strncmp(pstrTemp,"ISP",3)==0)				///15.远程下载IP 端口
				{
				     ;
				    Add_SMS_Ack_Content(sms_ack_data,ACKstate);		 
				}
			else if(strncmp(pstrTemp,"PLATENUM",8)==0)
				{
				    rt_kprintf("车牌号:%s", sms_content); 
					memset((u8*)&Vechicle_Info.Vech_Num,0,sizeof(Vechicle_Info.Vech_Num));	//clear	
				    rt_memcpy(Vechicle_Info.Vech_Num,sms_content,strlen(sms_content));
					DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 					
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					Add_SMS_Ack_Content(sms_ack_data,ACKstate);

					 //--------    清除鉴权码 -------------------
					     idip("clear");		   
				}
			else if(strncmp(pstrTemp,"COLOUR",6)==0)
				{
				     j=sscanf(sms_content,"%d",&u16Temp);
					if(j)
					{
						
					Vechicle_Info.Dev_Color=u16Temp; 
	        		rt_kprintf("\r\n 车辆颜色: %s ,%d \r\n",sms_content,Vechicle_Info.Dev_Color);          
	        		DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));  
					
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					
					WatchDog_Feed();
					DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
				  	Add_SMS_Ack_Content(sms_ack_data,ACKstate);
					}
				}
			else if(strncmp(pstrTemp,"CONNECT",6)==0)
				{
				    j=sscanf(sms_content,"%d",&u16Temp);
					if(j)
					{
						
						Vechicle_Info.Link_Frist_Mode=u16Temp;  
		        		rt_kprintf("\r\n 首次连接方式 %s ,%d \r\n",sms_content,Vechicle_Info.Link_Frist_Mode);          
						DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));		
						
						WatchDog_Feed();
						DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
						
						WatchDog_Feed();
						DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
					  	Add_SMS_Ack_Content(sms_ack_data,ACKstate);  
						 //--------    清除鉴权码 -------------------
					     idip("clear");		   
					} 
				}
           else if(strncmp(pstrTemp,"CLEARREGIST",11)==0)
           	    {
                     //--------    清除鉴权码 -------------------
					     idip("clear");		
                     DEV_regist.Enable_sd=1; // set 发送注册标志位
			         DataLink_EndFlag=1; 
					 Add_SMS_Ack_Content(sms_ack_data,ACKstate);  //  ack  state
           	    }
		   //  20
	        else if(strncmp(pstrTemp,"PASSWORD",8)==0)  //设置密码通过
       	    {
                j=sscanf(sms_content,"%d",&u16Temp);
				if(j)
				{
				  Login_Menu_Flag=(u8)u16Temp;	   // clear  first flag 	 			
			      DF_WriteFlashSector(DF_LOGIIN_Flag_offset,0,&Login_Menu_Flag,1); 
			
				  Add_SMS_Ack_Content(sms_ack_data,ACKstate);  //  ack  state
				    if(Login_Menu_Flag)
				    	{
				    	  //--------    清除鉴权码 -------------------
					      idip("clear");
						  DEV_regist.Enable_sd=1; // set 发送注册标志位
			              DataLink_EndFlag=1; 
				    	}
				} 
       	    } 
			else if(strncmp(pstrTemp,"RECOVER",7)==0)
				{
                     //------  界面回复出厂设置
					 Login_Menu_Flag=0;	  // clear	first flag		 
					 DF_WriteFlashSector(DF_LOGIIN_Flag_offset,0,&Login_Menu_Flag,1); 

					 Add_SMS_Ack_Content(sms_ack_data,ACKstate);  //  ack  state                  
                     Systerm_Reset_counter=Max_SystemCounter-30; //返回短信后恢复出厂设置
					 
				} //provinceid
			else if(strncmp(pstrTemp,"PROVINCEID",10)==0)
			{
                  provinceid(sms_content);
				  Add_SMS_Ack_Content(sms_ack_data,ACKstate);
				 
			}	
			else if(strncmp(pstrTemp,"CITYID",6)==0)
			{
                  cityid(sms_content); 
				  Add_SMS_Ack_Content(sms_ack_data,ACKstate); 
				 
			}
		    else if(strncmp(pstrTemp,"ATAENABLE",9)==0)
			{
			    j=sscanf(sms_content,"%d",&u16Temp);
				if(j)
				{
                  ata_enable(u16Temp); 
				  Add_SMS_Ack_Content(sms_ack_data,ACKstate);  
				}
				 
			}
			else if(strncmp(pstrTemp,"TYPEVECH",8)==0)			///14.车辆状态查询
			{
			     if(sms_content[0]=='1')
				 {
				   type_vech(1);
				   Add_SMS_Ack_Content(sms_ack_data,ACKstate);
				 }
				 if(sms_content[0]=='2')
				 {
				   type_vech(2);
				   Add_SMS_Ack_Content(sms_ack_data,ACKstate);
				 }
			}	 	   
			else												
				{
				;
				}
			}
		else
			{
			break;
			}
		}
}



/*********************************************************************************
*函数名称:u8 SMS_Rx_Text(char *instr,char *strDestNum)
*功能描述:接收到TEXT格式的短信处理函数
*输    入:instr 原始短信数据，strDestNum接收到得信息的发送方号码
*输    出:none 
*返 回 值:	1:正常完成，
			0:表示失败
*作    者:白养民
*创建日期:2013-05-29
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u8 SMS_Rx_Text(char *instr,char *strDestNum)
{
	u16 len;
	u8 ret=0;
	len=strlen(strDestNum);
	memset( SMS_Service.SMS_destNum, 0, sizeof( SMS_Service.SMS_destNum ) );
	if(len>sizeof( SMS_Service.SMS_destNum ))
		{
		len=sizeof( SMS_Service.SMS_destNum );
		}
	memcpy(SMS_Service.SMS_destNum,strDestNum,len);
	rt_kprintf( "\r\n  短息来源号码:%s", SMS_Service.SMS_destNum );
	
	len=strlen(instr);
	rt_kprintf( "\r\n 短信收到消息: " );
	rt_device_write( &dev_vuart, 0, instr, len);
	
	if( strncmp( (char*)instr, "TW703#", 6 ) == 0 )                                                //短信修改UDP的IP和端口
	{
		//-----------  自定义 短息设置修改 协议 ----------------------------------
		SMS_protocol( instr + 5, len - 5 ,SMS_ACK_msg);
		ret=1;
	}
	SMS_Service.SMS_read		= 0;
	SMS_Service.SMS_waitCounter = 0;
	SMS_Service.SMS_come		= 0;
	//SMS_Service.SMS_delALL		= 1;
	return ret;
}


/*********************************************************************************
*函数名称:u8 SMS_Rx_PDU(char *instr,u16 len)
*功能描述:接收到PDU格式的短信处理函数
*输    入:instr 原始短信数据，len接收到得信息长度，单位为字节
*输    出:none 
*返 回 值:	1:正常完成，
			0:表示失败
*作    者:白养民
*创建日期:2013-05-29
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u8 SMS_Rx_PDU(char *instr,u16 len)
{
	char *pstrTemp;
	u8 ret=0;
	
	//////
	memset( SMS_Service.SMS_destNum, 0, sizeof( SMS_Service.SMS_destNum ) );
	pstrTemp=(char *)rt_malloc(200);	///短信解码后的完整内容，解码后汉子为GB码
	memset(pstrTemp,0,200);
	rt_kprintf( "\r\n 短信原始消息: " );
	rt_device_write( &dev_vuart, 0, GSM_rx, len );
	
	len=GsmDecodePdu(GSM_rx,len,&SMS_Service.Sms_Info,pstrTemp);
	GetPhoneNumFromPDU( SMS_Service.SMS_destNum,  SMS_Service.Sms_Info.TPA, sizeof(SMS_Service.Sms_Info.TPA));

	//memcpy( SMS_Service.SMS_destNum, SMS_Service.Sms_Info.TPA,sizeof( SMS_Service.SMS_destNum ) );
	rt_kprintf( "\r\n  短息来源号码:%s \r\n", SMS_Service.SMS_destNum );
	rt_kprintf( "\r\n 短信消息: " ); 
	rt_device_write( &dev_vuart, 0, pstrTemp, len );
	//rt_hw_console_output(GSM_rx);
	if( strncmp( (char*)pstrTemp, "TW703#", 6 ) == 0 )                                                //短信修改UDP的IP和端口
	{
		//-----------  自定义 短息设置修改 协议 ----------------------------------
		SMS_protocol( pstrTemp + 5, len - 5 ,SMS_ACK_msg);
		ret=1;
	}
	SMS_Service.SMS_read		= 0;
	SMS_Service.SMS_waitCounter = 3;
	SMS_Service.SMS_come		= 1;
	//SMS_Service.SMS_delALL		= 1;
	rt_free( pstrTemp );
	pstrTemp = RT_NULL;
	//////
	return ret;
}




/*********************************************************************************
*函数名称:u8 SMS_Tx_Text(char *strDestNum,char *s)
*功能描述:发送TEXT格式的短信函数
*输    入:s 原始短信数据，strDestNum接收方号码
*输    出:none 
*返 回 值:	1:正常完成，
			0:表示失败
*作    者:白养民
*创建日期:2013-05-29
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u8 SMS_Tx_Text(char *strDestNum,char *s) 
{
	u16 len;
	char *pstrTemp;
	
	memset(SMS_Service.SMSAtSend,0,sizeof(SMS_Service.SMSAtSend));
	strcpy( ( char * ) SMS_Service.SMSAtSend, "AT+CMGS=\"" ); 
	//strcat(SMS_Service.SMSAtSend,"8613820554863");// Debug
	strcat(SMS_Service.SMSAtSend,strDestNum);
	strcat(SMS_Service.SMSAtSend,"\"\r\n");

	rt_kprintf("\r\n%s",SMS_Service.SMSAtSend); 
	rt_hw_gsm_output( ( char * ) SMS_Service.SMSAtSend );

	rt_thread_delay(50);
	
	pstrTemp=rt_malloc(150);
	memset(pstrTemp,0,150);
	len=strlen(s);
	memcpy(pstrTemp,s,len);
	pstrTemp[len++]=0x1A;      // message  end  

	///发送调试信息
	rt_device_write( &dev_vuart, 0, pstrTemp, len);
	///发送到GSM模块
	rt_hw_gsm_output_Data(pstrTemp, len); 
	rt_free( pstrTemp );
	pstrTemp=RT_NULL; 
	return 1;
}
//FINSH_FUNCTION_EXPORT(SMS_Tx_Text, SMS_Tx_Text);



/*********************************************************************************
*函数名称:u8 SMS_Tx_PDU(char *strDestNum,char *s)
*功能描述:发送PDU格式的短信函数
*输    入:s 原始短信数据，strDestNum接收方号码
*输    出:none 
*返 回 值:	1:正常完成，
			0:表示失败
*作    者:白养民
*创建日期:2013-05-29
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u8 SMS_Tx_PDU(char *strDestNum,char *s)
{
	u16 len;
	u16 i;
	char *pstrTemp;
	memset(SMS_Service.SMSAtSend,0,sizeof(SMS_Service.SMSAtSend));
	pstrTemp=rt_malloc(400);
	memset(pstrTemp,0,400);
	i=0;
	SetPhoneNumToPDU(SMS_Service.Sms_Info.TPA, strDestNum, sizeof(SMS_Service.Sms_Info.TPA));
	len=AnySmsEncode_NoCenter(SMS_Service.Sms_Info.TPA,GSM_UCS2,s,strlen(s),pstrTemp);
	//len=strlen(pstrTemp);
	pstrTemp[len++]=0x1A;      // message  end  	
	//////
	sprintf( ( char * ) SMS_Service.SMSAtSend, "AT+CMGS=%d\r\n", (len-2)/2); 
	rt_kprintf("%s",SMS_Service.SMSAtSend); 
	rt_hw_gsm_output( ( char * ) SMS_Service.SMSAtSend );
	rt_thread_delay(50);
	//////	
	rt_device_write( &dev_vuart, 0, pstrTemp, strlen(pstrTemp) );
	//rt_hw_console_output(pstrTemp);
	rt_hw_gsm_output_Data(pstrTemp, len); 
	rt_free( pstrTemp );
	pstrTemp=RT_NULL;
	return 1;
}
//FINSH_FUNCTION_EXPORT(SMS_Tx_PDU, SMS_Tx_PDU);


/*********************************************************************************
*函数名称:u8 SMS_Rx_Notice(u16 indexNum)
*功能描述:模块收到新短信通知
*输    入:新短信的索引号
*输    出:none 
*返 回 值:	1:正常完成，
			0:表示失败
*作    者:白养民
*创建日期:2013-05-29
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u8 SMS_Rx_Notice(u16 indexNum)
{
	SMS_Service.SMIndex=indexNum;
	rt_kprintf( " index=%d", SMS_Service.SMIndex );
	SMS_Service.SMS_read		= 3;
	SMS_Service.SMS_waitCounter = 1;
	return 1;
}


///测试函数，测试SMS短信接收处理命令解析的正确性
void SMS_Test(char * s)
{
	SMS_protocol(s,strlen(s),SMS_ACK_none);
}
//FINSH_FUNCTION_EXPORT(SMS_Test, SMS_Test);

///测试函数，测试PDU数据包的接收解析功能
void SMS_PDU(char *s)
{
	u16 len;
	char *pstrTemp;
	pstrTemp=(char *)rt_malloc(160);	///短信解码后的完整内容，解码后汉子为GB码
	len=GsmDecodePdu(s,strlen(s),&SMS_Service.Sms_Info,pstrTemp);
	GetPhoneNumFromPDU( SMS_Service.SMS_destNum,  SMS_Service.Sms_Info.SCA, sizeof(SMS_Service.Sms_Info.SCA));
	rt_kprintf( "\r\n  短息中心号码:%s \r\n", SMS_Service.SMS_destNum );
	GetPhoneNumFromPDU( SMS_Service.SMS_destNum,  SMS_Service.Sms_Info.TPA, sizeof(SMS_Service.Sms_Info.TPA));
	rt_kprintf( "\r\n  短息来源号码:%s \r\n", SMS_Service.SMS_destNum );
	rt_kprintf( "\r\n 短信消息:\"%s\"\r\n",pstrTemp);
	rt_free( pstrTemp );
	pstrTemp = RT_NULL;
}
//FINSH_FUNCTION_EXPORT(SMS_PDU, SMS_PDU);
