#include "App_moduleConfig.h"




//--------   顺序读取发送相关  ------------
u8    ReadCycle_status=RdCycle_Idle;  
u8    ReadCycle_timer=0;   // 超时判断


u32     cycle_write=0, cycle_read=0;  // 循环存储记录
u32    AvrgSpdPerMin_write=0,AvrgSpdPerMin_Read=0; // 车辆每分钟平均速度记录
u32    AvrgSpdPerSec_write=0,AvrgSpdPerSec_Read=0; // 车辆每秒平均速度记录
u32    AvrgMintPosit_write=0,AvrgMintPosit_Read=0; // 车辆单位小时内每分钟位置记录
u32    ErrorLog_write=0,ErrorLog_Read=0;           // 设备异常记录
u32    Recorder_write=0,Recorder_Read=0;           // 事故疑点
u32    Login_write=0,Login_Read=0;                 // 登录记录
u32    Powercut_write=0,Powercut_read=0;           // 外部电源断开
u32    Settingchg_write=0,Settingchg_read=0;       // 参数修改
u32    TiredDrv_write=0, TiredDrv_read=0;  // 疲劳驾驶存储记录
u32    ExpSpdRec_write=0, ExpSpdRec_read=0;  // 超速报警存储记录
u32    pic_write=0,pic_read=0,pic_current_page=0,pic_PageIn_offset=0,pic_size=0;;       // 图片存储记录 
u32   	Distance_m_u32=0;	 // 行车记录仪运行距离	  单位米
u32     DayStartDistance_32=0; //每天起始里程数目


//-----------------------------------------------------------------------------------------------------------------------------
u8 SaveCycleGPS(u32 cyclewr,u8 *content ,u16 saveLen) 
{
  /*
     NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
  */
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
//	u8   reg[1]={0};
	u8   rd_back[40];
	u16  i=0,retry=0;

  //----------------------------------------------------------------------------------------------
  //   1. Judge  Whether  needs to Erase page 
  
     pageoffset=(u32)(cycle_write>>4);                // 计算出 Page 偏移  除以16  
     InPageoffset=cycle_write-(u32)(pageoffset<<4);   // 计算出 页内偏移地址 
     InPageAddr=(u16)(InPageoffset<<5);           // 计算出页内 字节   乘以 32 (每个记录32个字节)
     if(((pageoffset%8)==0)&&(InPageoffset==0))  // 判断是否需要擦除Sector  被移除到下一个Sector  1Sector=8Page  
     {
        WatchDog_Feed();
		SST25V_SectorErase_4KByte((pageoffset+CycleStart_offset)*PageSIZE);      // erase Sector		
		DF_delay_ms(20); 
	    rt_kprintf("\r\n Erase Cycle Sector : %d\r\n",(pageoffset>>3));       
	 }
  //	   2. write  and read back    
  SV_RTRY:
      if(retry>=2)
	  	  return false;
	   delay_ms(5);
	   WatchDog_Feed(); 
	   DF_WriteFlashDirect(pageoffset+CycleStart_offset,InPageAddr,content,saveLen);  //   写入信息
	   DF_delay_us(30);   
       DF_ReadFlash(pageoffset+CycleStart_offset,InPageAddr,rd_back,saveLen);  //   读取信息
  //  compare 
       for(i=0;i<saveLen;i++)
       	{
             if(content[i]!=rd_back[i])
			 {
			     cycle_write++;		  
		         if(cycle_write>=Max_CycleNum)
			       cycle_write=0; 
				 if(retry==0)
				 {
				     retry++;
			         goto SV_RTRY;
				 }
				 else
				 {
				   //---------------------------
				  PositionSD_Enable(); 
				  Current_State=1; // 使能即时上报 		
				  Current_UDP_sd=1;
				  rt_kprintf("\r\n wrte error current\r\n");
				    return false;
				 }
             }
       	}
		return true;  
  //-------------------------------------------------------------------------------------------- 
}  


u8 ReadCycleGPS(u32 cycleread,u8 *content ,u16 ReadLen)
{
  /*
     NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
  */
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u8  i=0,FCS=0;;

  /*
      上报的每一包数据位31个字节， 每条记录为32个字节，用每条记录的最后一个字节来作为是否上报过的标志位。没有上报该标志为0xFF
      上报过后该标志为被写为0x01
  */
  //----------------------------------------------------------------------------------------------
  //   1. caculate address 
     pageoffset=(u32)(cycle_read>>4);                // 计算出 Page 偏移  除以16 
     InPageoffset=cycle_read-(u32)(pageoffset<<4);   // 计算出 页内偏移地址 
     InPageAddr=(u16)(InPageoffset<<5);            // 计算出页内 字节   乘以 32 (每个记录32个字节)
  //   2. Write Record Content 
     DF_ReadFlash(pageoffset+CycleStart_offset,InPageAddr,content,ReadLen); 
     DF_delay_us(20);
  if(DispContent==2)
  {
     rt_kprintf("\r\n  读取CycleGPS 内容为 :\r\n ");   
	  for(i=0;i<ReadLen;i++)
	  	rt_kprintf("%2X ",content[i]);  
	 rt_kprintf("\r\n"); 
  }	 
  //  3. Judge FCS	
	//--------------- 过滤已经发送过的信息 -------
	  FCS = 0;
	   for ( i = 0; i < ReadLen-1; i++ )   //计算读取信息的异或和
	   {
			   FCS ^= *( content + i );  
	   }			  
	  if(((content[ReadLen-1]!=FCS)&&(content[0]!=0xFF))||(content[0]==0xFF))  // 判断异或和   
	    { 	      
		  if(content[0]==0xFF)
		  {
			 rt_kprintf("\r\n  content[0]==0xFF   read=%d,  write=%d  \r\n",cycle_read,cycle_write);   
			 //cycle_read=cycle_write;//如果是内容是0xFF ，读指针和写指针相等，不再触发上报。
                       
			  cycle_read++;	
		        if(cycle_read>=Max_CycleNum)
		  	      cycle_read=0; 
			 ReadCycle_status=RdCycle_Idle;  
			 return false;	
		  }  
		  //------------------------------------------------
          cycle_read++;	
		  if(cycle_read>=Max_CycleNum)
		  	cycle_read=0;
		  ReadCycle_status=RdCycle_Idle; 
		  return false; 
     	}	  
	//------------------------------------------------------------------------------------------   	 
	    return true;     
  //-------------------------------------------------------------------------------------------- 
} 

//-----------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------
u8 Save_DrvRecoder(u32 In_write,u8 *content ,u16 saveLen) 
{
	/*
	   NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
	*/
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移 
    u16  InPageAddr=0;   //页内 地址偏移 
	u8   reg[1]={0};
  //----------------------------------------------------------------------------------------------
  //   1. Judge  Whether  needs to Erase page 
  
     pageoffset=(u32)(In_write>>1);                // 计算出 Page 偏移  除以2  
     InPageoffset=In_write-(u32)(pageoffset<<1);   // 计算出 页内偏移地址 
     InPageAddr=(u16)(InPageoffset<<8);           // 计算出页内 字节   乘以 256 (每个记录256个字节)  206(content)+1(FCS)
     if(((pageoffset%8)==0)&&(InPageoffset==0))  // 判断是否需要擦除Block  被移除到下一个Sector  1Sector=8 Page  
     {
        SST25V_SectorErase_4KByte((pageoffset+VehicleRecStart_offset)*PageSIZE);      // erase Sector	
        DF_delay_ms(70);
		// rt_kprintf("\r\n Erase Cycle Block : %d\r\n",(pageoffset>>6));   
	 }
  //	   2. Filter write  area    
        DF_ReadFlash(pageoffset+VehicleRecStart_offset,InPageAddr,reg,1); 
	   if(reg[0]!=0xff)  // 如果要写入的区域 dirty  ，则地址跳到下一个
	   	{
          In_write++;
		  Recorder_write++;		  
		  if(Recorder_write>=Max_RecoderNum)  
			   Recorder_write=0;  
		 // rt_kprintf("\r\n    *******   行车记录仪写区域 Write area : %d   is   Dirity!  \r\n",In_write);  
		  return false;
	   	}       
  //   3. Write Record Content 
     DF_WriteFlashDirect(pageoffset+VehicleRecStart_offset,InPageAddr,content,saveLen);  //   写入信息
     DF_delay_ms(8); 
	// rt_kprintf("\r\n DrvRec Starpageoffset=%d  PageOffset= %d ,  InPageAddr= %d \r\n",VehicleRecStart_offset,pageoffset,InPageoffset); 
		return true; 
  //-------------------------------------------------------------------------------------------- 
}

u8 Read_DrvRecoder(u32 In_read,u8 *content ,u16 ReadLen)
{
	/*
	   NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
	*/
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u8   FCS=0;
	u16  i=0;    

  /*
      上报的每一包数据位31个字节， 每条记录为32个字节，用每条记录的最后一个字节来作为是否上报过的标志位。没有上报该标志为0xFF
      上报过后该标志为被写为0x01
  */
  //----------------------------------------------------------------------------------------------
  //   1. caculate address 
  pageoffset=(u32)(In_read>>1);				// 计算出 Page 偏移  除以2	
  InPageoffset=In_read-(u32)(pageoffset<<1);	// 计算出 页内偏移地址 
  InPageAddr=(u16)(InPageoffset<<8);		   // 计算出页内 字节	乘以 256 (每个记录256个字节)  238(content)+1(FCS)
  //   2. Write Record Content 
     DF_ReadFlash(pageoffset+VehicleRecStart_offset,InPageAddr,content,ReadLen); 
    DF_delay_us(10);
	// rt_kprintf("\r\n  ReadRecoder Starpageoffset=%d  PageOffset= %d ,  InPageAddr= %d  \r\n",VehicleRecStart_offset,pageoffset,InPageoffset); 
    // rt_kprintf("\r\n  读取  Records 内容为 :\r\n"); 
	// for(i=0;i<ReadLen;i++)
	//  	rt_kprintf("%2X ",content[i]);   
	// rt_kprintf("\r\n");             
     
  //  3.  Judge FCS	
	//--------------- 过滤已经发送过的信息 -------
	  FCS = 0;
	   for ( i = 0; i < ReadLen-1; i++ )   //计算读取信息的异或和
	   {
			   FCS ^= *( content + i );  
	   }			  
	   if(((content[ReadLen-1]!=FCS)&&(content[0]!=0xFF))||(content[0]==0xFF))	// 判断异或和 
	    { 	      
		  if(content[0]==0xFF)
		  {
			 Recorder_Read=Recorder_write;//如果是内容是0xFF ，读指针和写指针相等，不再触发上报。			
			 return false;
		  }
          Recorder_Read++;	
		  if(Recorder_Read>=Max_RecoderNum) 
		  	Recorder_Read=0; 
		//  rt_kprintf("\r\n   *******  DrvRecoder 该条记录内容不对 *******  \r\n"); 
		  return false;
     	} 
	   	if(content[0]==0xFF)
	   	{
           Recorder_Read=Recorder_write;//如果是内容是0xFF ，读指针和写指针相等，不再触发上报。           
		   return false;
	   	}
	//--------------------------------------------------------------  	
		 return true; 
  //-------------------------------------------------------------------------------------------- 
}


u8  Common_WriteContent(u32 In_write,u8 *content ,u16 saveLen, u8 Type) 
{
  //-----------------------------------------------------
  u8     reg[1];
  //u8   regStr[25];   
  //-----------------------------------------------------
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u32  Start_offset=0; 

   //--------------------------------------------------	
    //memset(regStr,0,sizeof(regStr)); 
  //  1.   Classify
    switch(Type)
    {
	
		case TYPE_TiredDrvAdd:
							 Start_offset=TiredDrvStart_offset;
							// memcpy(regStr,"疲劳驾驶",25);
							 break;
		case TYPE_ExpSpdAdd:
							 Start_offset=ExpSpdStart_offset;
							// memcpy(regStr,"超速报警",25);
							 break; 					 
		case TYPE_ErrorLogAdd:
							 Start_offset=AbNormalStart_offset;
							 //memcpy(regStr,"异常LOG",25); 
							 break; 
		case TYPE_LogInAdd:
							 Start_offset=LogIn_offset; 
							// memcpy(regStr,"登录记录",25);   
							 break; 
		case TYPE_SettingChgAdd:
							 Start_offset=SettingChg_offset;    
							 //memcpy(regStr,"参数修改",25);       
							 break; 					 
		default :
							 return false;		  					 
    }
	//----------------------------------------------------------------------------------------------
	//	 2. caculate address 
	 
		pageoffset=(u32)(In_write>>4);				 // 计算出 Page 偏移  除以16  
		InPageoffset=In_write-(u32)(pageoffset<<4);	 // 计算出 页内偏移地址 
		InPageAddr=(u16)(InPageoffset<<5);			 // 计算出页内 字节   乘以 32 (每个记录32个字节)
		if(((pageoffset%8)==0)&&(InPageoffset==0))  // 判断是否需要擦除Block  被移除到下一个Block	1Block=8Page  
		{
        SST25V_SectorErase_4KByte((pageoffset+Start_offset)*PageSIZE);      // erase Sector	
        DF_delay_ms(70);
		//  rt_kprintf("\r\n Common --- Erase Cycle Block : %d\r\n",(pageoffset>>6));    
		}
	 // 	  2. Filter write  area    
		DF_ReadFlash(pageoffset+Start_offset,InPageAddr,reg,1); 
		  if(reg[0]!=0xff)	// 如果要写入的区域 dirty  ，则地址增然后从新开始寻找知道找到为止
		   {
			 In_write++;
			 if(In_write>=Max_CommonNum)
				  In_write=0;  
			         // rt_kprintf("\r\n	 *******   Common 写区域 Write area : %d	is	 Dirity!  \r\n",In_write);  
				 return false;
		   }	   
	 //   3. Write Record Content 
        DF_WriteFlashDirect(pageoffset+Start_offset,InPageAddr,content,saveLen);  //   写入信息
		DF_delay_ms(10);
		//rt_kprintf("\r\n Common Starpageoffset=%d	PageOffset= %d ,  InPageAddr= %d  TYPE= %s \r\n",CycleStart_offset,pageoffset,InPageoffset,regStr);  

     //   4. end  
        switch(Type) 
    {
	
		case TYPE_TiredDrvAdd:							 
							 TiredDrv_write=In_write;
							 break;
		case TYPE_ExpSpdAdd:
							 ExpSpdRec_write=In_write;
							 break; 					 
		case TYPE_AvrgSpdAdd:
							 AvrgSpdPerMin_write=In_write;
							 break;
		case TYPE_ErrorLogAdd:
							 ErrorLog_write=In_write;  
							 break; 
		case TYPE_LogInAdd:
							 Login_write=In_write;
							 break; 
		case TYPE_SettingChgAdd:
							 Settingchg_write=In_write;     
							 break; 					 
		default :
							 return false;	 						 
    }

	return true; 
	     	     
}


u8  Common_ReadContent(u32 In_read,u8 *content ,u16 ReadLen, u8 Type) 
{
  //-----------------------------------------------------
  //u8    regStr[25];   
  //-----------------------------------------------------
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u32  Start_offset=0;     
	u8   i=0,FCS=0;;
   //--------------------------------------------------	
 
	// memset(regStr,0,sizeof(regStr)); 
   //  1.	Classify
	 switch(Type)
	 {
	 
		 case TYPE_TiredDrvAdd:
							  Start_offset=TiredDrvStart_offset;
							//  memcpy(regStr,"疲劳驾驶",25);
							  break;
		 case TYPE_ExpSpdAdd:
							  Start_offset=ExpSpdStart_offset;
							//  memcpy(regStr,"超速报警",25);
							  break;					  
		 case TYPE_ErrorLogAdd:
							  Start_offset=AbNormalStart_offset;
							//  memcpy(regStr,"异常LOG",25); 
							  break; 
		  case TYPE_LogInAdd:
							   Start_offset=LogIn_offset; 
							//   memcpy(regStr,"登录记录",25);   
							   break; 
		  case TYPE_SettingChgAdd:
							   Start_offset=SettingChg_offset;	  
							 //  memcpy(regStr,"参数修改",25);	   
							   break;					   
		 					  
		 default :
							  return false; 						  
	 } 
 
	//----------------------------------------------------------------------------------------------
	//	 2. caculate address 
	 
		   pageoffset=(u32)(In_read>>4);				  // 计算出 Page 偏移  除以64  
		   InPageoffset=In_read-(u32)(pageoffset<<4);   // 计算出 页内偏移地址 
		   InPageAddr=(u16)(InPageoffset<<5);			 // 计算出页内 字节   乘以 32 (每个记录32个字节)
		  //	 2. Write Record Content 
		   DF_ReadFlash(pageoffset+Start_offset,InPageAddr,content,ReadLen);    
		   DF_delay_us(10);
		 //  rt_kprintf("\r\n  Common Starpageoffset=%d	PageOffset= %d ,  InPageAddr= %d  TYPE= %s\r\n",Start_offset,pageoffset,InPageoffset,regStr);  
		 if(DispContent)
		 {
			   rt_kprintf("\r\n  读取Common 内容为 :\r\n");  
				for(i=0;i<ReadLen;i++)
				  rt_kprintf("%2X ",content[i]);  
			   rt_kprintf("\r\n");   
		 }   
		//	3. Judge FCS  
		  //--------------- 过滤已经发送过的信息 -------
			FCS = 0;
			 for ( i = 0; i < ReadLen-1; i++ )	 //计算读取信息的异或和
			 {
					 FCS ^= *( content + i );  
			 }				
			 if(((content[ReadLen-1]!=FCS)&&(content[0]!=0xFF))||(content[0]==0xFF))  // 判断异或和 
			  { 
			   
			   if(content[0]==0xFF)
			   {
				  //如果是内容是0xFF ，读指针和写指针相等，不再触发上报。	
				         switch(Type)
					    {
						
							case TYPE_TiredDrvAdd:							 
												 TiredDrv_read=TiredDrv_write;
												 break;
							case TYPE_ExpSpdAdd:
												 ExpSpdRec_read=ExpSpdRec_write;
												 break; 					 
							case TYPE_AvrgSpdAdd:
												 AvrgSpdPerMin_Read=AvrgSpdPerMin_write;
												 break;
							case TYPE_ErrorLogAdd:
												 ErrorLog_Read=ErrorLog_write;        
												 break; 
							 case TYPE_LogInAdd:
												 Login_Read=Login_write;
												  break; 
							 case TYPE_SettingChgAdd:
							 	                 Settingchg_read=Settingchg_write;
												  break;					  
												 
							default :
												 return false;							 
					    
				         	}	
						return false; 
			   }
				In_read++; 
				if(In_read>=Max_CycleNum)
				  In_read=0;
				rt_kprintf("\r\n   *******	该条记录内容不对 *******  \r\n");  
				return false;
			  }
			 if(content[0]==0xFF)
			 {
               rt_kprintf("\r\n  读取内容为 0xFF \r\n");
			        switch(Type)
					    {
						
							case TYPE_TiredDrvAdd:							 
												 TiredDrv_read=TiredDrv_write;
												 break;
							case TYPE_ExpSpdAdd:
												 ExpSpdRec_read=ExpSpdRec_write;
												 break; 					 
							case TYPE_AvrgSpdAdd:
												 AvrgSpdPerMin_Read=AvrgSpdPerMin_write;
												 break;
							case TYPE_ErrorLogAdd:
												 ErrorLog_Read=ErrorLog_write;        
												 break; 
							 case TYPE_LogInAdd:
												 Login_Read=Login_write;
												  break; 
							 case TYPE_SettingChgAdd:
												 Settingchg_read=Settingchg_write;
												  break;					  
												 
							default :
												 return false;							 
					    
				         	}	
               return false;
			 } 
			 
			 	
     //   4. end  
        switch(Type)
    {
	
		case TYPE_TiredDrvAdd:							 
							 TiredDrv_read=In_read;
							 break;
		case TYPE_ExpSpdAdd:
							 ExpSpdRec_read=In_read;
							 break; 					 
		case TYPE_AvrgSpdAdd:
							 AvrgSpdPerMin_Read=In_read;
							 break;
		case TYPE_ErrorLogAdd:
							 ErrorLog_Read=In_read;        
							 break; 
		 case TYPE_LogInAdd:
							 Login_Read=In_read;
							  break; 
		 case TYPE_SettingChgAdd:
		 	                 Settingchg_read=In_read; 
							  break;					  
							 
		default :
							 return false;							 
    }

	return true; 
	     	     
}


u8 Save_PerMinContent(u32 In_wr,u8 *content ,u16 saveLen)
{
	/*
	   NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
	*/
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u8   reg[1]={0};
  //----------------------------------------------------------------------------------------------
  //   1. Judge  Whether  needs to Erase page 
  
     pageoffset=(u32)(In_wr/7);                // 计算出 Page 偏移  除以7  
     InPageoffset=In_wr-(u32)(pageoffset*7);   // 计算出 页内偏移地址 
     InPageAddr=(u16)(InPageoffset*70);           // 计算出页内 字节   乘以 70 (每个记录70个字节)    
     if(((pageoffset%8)==0)&&(InPageoffset==0))  // 判断是否需要擦除Block  被移除到下一个Block  1Block=8Page  
     {
	  SST25V_SectorErase_4KByte((pageoffset+AverageSpdStart_offset)*PageSIZE);	  // erase Sector 
	  DF_delay_ms(80);
	  // rt_kprintf("\r\n Erase Cycle Block : %d\r\n",(pageoffset>>6));    
	 }
  //	   2. Filter write  area    
      DF_ReadFlash(pageoffset+AverageSpdStart_offset,InPageAddr,reg,1);
	   if(reg[0]!=0xff)  // 如果要写入的区域 dirty  ，则地址增然后从新开始寻找知道找到为止
	   	{
	   	  In_wr++;
		  AvrgSpdPerMin_write++;		  
		  if(AvrgSpdPerMin_write>=Max_SPDSperMin)
			   AvrgSpdPerMin_write=0;  
		 // rt_kprintf("\r\n    *******  每分钟速度 写区域 Write area : %d   is   Dirity!  \r\n",AvrgSpdPerMin_write);  
          //--------------------------------
		  	return false;
	   	}       
  //   3. Write Record Content 
     DF_WriteFlashDirect(pageoffset+AverageSpdStart_offset,InPageAddr,content,saveLen);  //   写入信息
     DF_delay_ms(5);
	// rt_kprintf("\r\n Write PerMit Starpageoffset=%d  PageOffset= %d ,  InPageAddr= %d \r\n",AverageSpdStart_offset,pageoffset,InPageoffset); 
		return true; 
  //-------------------------------------------------------------------------------------------- 
}

u8 Read_PerMinContent(u32 In_read,u8 *content ,u16 ReadLen) 
{
	/*
	   NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
	*/
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u8   FCS=0;
	u16  i=0;  

  /*
      上报的每一包数据位31个字节， 每条记录为32个字节，用每条记录的最后一个字节来作为是否上报过的标志位。没有上报该标志为0xFF
      上报过后该标志为被写为0x01
  */
  //----------------------------------------------------------------------------------------------
  //   1. caculate address 
  pageoffset=(u32)(In_read/7);				 // 计算出 Page 偏移  除以7  
  InPageoffset=In_read-(u32)(pageoffset*7);	 // 计算出 页内偏移地址  
  InPageAddr=(u16)(InPageoffset*70);		   // 计算出页内 字节	乘以 70 (每个记录70个字节)	  
  //   2. Write Record Content 
    DF_ReadFlash(pageoffset+AverageSpdStart_offset,InPageAddr,content,ReadLen);     
    DF_delay_us(10);
	// rt_kprintf("\r\n   Read PerMintSPD  Starpageoffset=%d  PageOffset= %d ,  InPageAddr= %d   每分钟平均速度 \r\n",AverageSpdStart_offset,pageoffset,InPageoffset);  
   if(DispContent==2)
   {
	rt_kprintf("\r\n  >>>>>>>>>>读取每分钟平均速度   PerMintSPD 内容为 :\r\n");       
   	  for(i=0;i<ReadLen;i++)
   	  	rt_kprintf("%2X ",content[i]);   
   	 rt_kprintf("\r\n");    
   }	 
  //  3. Judge FCS	
	//--------------- 过滤已经发送过的信息 -------
	  FCS = 0;
	   for ( i = 0; i < ReadLen-1; i++ )   //计算读取信息的异或和
	   {
			   FCS ^= *(content + i );  
	   }			  
	   if(((content[ReadLen-1]!=FCS)&&(content[0]!=0xFF))||(content[0]==0xFF))	// 判断异或和 
	    { 
	    
		    if(content[0]==0xFF)
	 		{             
	 		       rt_kprintf("\r\n  读取内容为0xFF --1\r\n"); 	 		    
				   AvrgSpdPerMin_Read=AvrgSpdPerMin_write;//如果是内容是0xFF ，读指针和写指针相等，不再触发上报。   		  
	               return false; 
			} 
          AvrgSpdPerMin_Read++;	
		  if(AvrgSpdPerMin_Read>=Max_SPDSperMin) 
		  	AvrgSpdPerMin_Read=0; 
		  //rt_kprintf("\r\n   ******* PerMit  该条记录内容不对 *******  \r\n");    
		  return false; 
     	}  
	    if(content[0]==0xFF)
 		{             
 		       rt_kprintf("\r\n  读取内容为0xFF \r\n"); 	 		    
			   AvrgSpdPerMin_Read=AvrgSpdPerMin_write;//如果是内容是0xFF ，读指针和写指针相等，不再触发上报。   		  
               return false; 
		} 
	//------------------------------------------------------------------------------------------  
    return true; 
    //-------------------------------------------------------------------------------------------- 
}

u8 Save_PerSecContent(u32 In_wr,u8 *content ,u16 saveLen)
{
	/*
	   NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
	*/
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u8   reg[1]={0};
  //----------------------------------------------------------------------------------------------
  //   1. Judge  Whether  needs to Erase page 
  
     pageoffset=(u32)(In_wr/7);                // 计算出 Page 偏移  除以7 
     InPageoffset=In_wr-(u32)(pageoffset*7);   // 计算出 页内偏移地址 
     InPageAddr=(u16)(InPageoffset*70);           // 计算出页内 字节   乘以 70 (每个记录70个字节)    
     if(((pageoffset%8)==0)&&(InPageoffset==0))  // 判断是否需要擦除Block  被移除到下一个Block  1Block=8Page  
     {
	  SST25V_SectorErase_4KByte((pageoffset+AvrgSpdSec_offset)*PageSIZE);	  // erase Sector 
	  mDelaymS(50);   
	  // rt_kprintf("\r\n Erase Cycle Block : %d\r\n",(pageoffset>>6));   
	 }
  //	   2. Filter write  area    
      DF_ReadFlash(pageoffset+AvrgSpdSec_offset,InPageAddr,reg,1);
      DF_delay_us(10);
	   if(reg[0]!=0xff)  // 如果要写入的区域 dirty  ，则地址增然后从新开始寻找知道找到为止
	   	{
	   	  In_wr++; 
		  AvrgSpdPerSec_write++;		  
		  if(AvrgSpdPerSec_write>=Max_SPDerSec) 
			   AvrgSpdPerSec_write=0;   
//		  rt_kprintf("\r\n    *******  每秒钟速度 写区域 Write area : %d  In_wr=%d  is   Dirity!  \r\n",AvrgSpdPerSec_write,In_wr);  
          //--------------------------
		  	 return false;
	   	}       
  //   3. Write Record Content 
     DF_WriteFlashDirect(pageoffset+AvrgSpdSec_offset,InPageAddr,content,saveLen);  //   写入信息
     DF_delay_ms(10);
	// rt_kprintf("\r\n Write PerMit Starpageoffset=%d  PageOffset= %d ,  InPageAddr= %d \r\n",AverageSpdStart_offset,pageoffset,InPageoffset); 
     return true; 
  //-------------------------------------------------------------------------------------------- 
}


u8 Read_PerSecContent(u32 In_read,u8 *content ,u16 ReadLen)   
{
	/*
	   NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
	*/
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u8   FCS=0;
	u16  i=0;   

  /* 
      上报的每一包数据位31个字节， 每条记录为32个字节，用每条记录的最后一个字节来作为是否上报过的标志位。没有上报该标志为0xFF
      上报过后该标志为被写为0x01     
   */
  //----------------------------------------------------------------------------------------------
  //   1. caculate address 
  pageoffset=(u32)(In_read/7);				 // 计算出 Page 偏移  除以7  
  InPageoffset=In_read-(u32)(pageoffset*7);	 // 计算出 页内偏移地址 
  InPageAddr=(u16)(InPageoffset*70);		   // 计算出页内 字节	乘以 70 (每个记录70个字节)	  
  //   2. Write Record Content 
 
  DF_ReadFlash(pageoffset+AvrgSpdSec_offset,InPageAddr,content,ReadLen);	  
  DF_delay_us(10);
	// rt_kprintf("\r\n   Read PerMintSPD  Starpageoffset=%d  PageOffset= %d ,  InPageAddr= %d   每分钟平均速度 \r\n",AverageSpdStart_offset,pageoffset,InPageoffset);  
	if(DispContent==2)  
		{
			  rt_kprintf("\r\n  读取每秒PerSecSPD 内容为 :\r\n");    
			  for(i=0;i<ReadLen;i++) 
				  	rt_kprintf("%2X ",content[i]);          
			  rt_kprintf("\r\n");    
		}
  //  3. Judge FCS	
	//--------------- 过滤已经发送过的信息 -------
	  FCS = 0;
	   for ( i = 0; i < ReadLen-1; i++ )   //计算读取信息的异或和 
	   {
			   FCS ^= *(content + i );  
	   }			  
	   if(((content[ReadLen-1]!=FCS)&&(content[0]!=0xFF))||(content[0]==0xFF))	// 判断异或和 
	    { 	    
		    if(content[0]==0xFF)
	 		{             
	 		       rt_kprintf("\r\n  读取内容为0xFF \r\n"); 	 		    
				   AvrgSpdPerSec_Read=AvrgSpdPerSec_write;//如果是内容是0xFF ，读指针和写指针相等，不再触发上报。   		  
	               return false;     
			}     
          AvrgSpdPerSec_Read++;	 
		  if(AvrgSpdPerSec_Read>=Max_SPDerSec)      
		  	AvrgSpdPerSec_Read=0;  
		  //rt_kprintf("\r\n   ******* PerMit  该条记录内容不对 *******  \r\n");    
		  return false; 
     	}  
	    if(content[0]==0xFF)
 		{             
 		       rt_kprintf("\r\n  读取内容为0xFF \r\n"); 	 		    
			   AvrgSpdPerSec_Read=AvrgSpdPerSec_write;//如果是内容是0xFF ，读指针和写指针相等，不再触发上报。			  
               return false;      
		}     
	//----------------------------------------------------------------------------------------------------- 	
		return true; 
    //----------------------------------------------------------------------------------------------------- 
}

u8 Save_MintPosition(u32 In_write,u8 *content ,u16 saveLen)
{    // 记录单位小时内每分钟的位置信息
	/*
	   NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
	*/
    u32  pageoffset=0;   //Page 偏移
	u8   reg[1]={0};
  //----------------------------------------------------------------------------------------------
  //   1. Judge  Whether  needs to Erase page     每条记录512个字节
  
     pageoffset=In_write;                // 计算出 Page 偏移  除以4  
        // 计算出 页内偏移地址 0
        // 计算出页内 字节   乘以 512 (每个记录512个字节)  485(content)+1(FCS)
     if((pageoffset%8)==0)  // 判断是否需要擦除Block  被移除到下一个Block  1Block=8 Page  
     {
	  SST25V_SectorErase_4KByte((pageoffset+AvrgMintPosit_offset)*PageSIZE);	  // erase Sector 
	  DF_delay_ms(50);
	  // rt_kprintf("\r\n Erase Cycle Block : %d\r\n",(pageoffset>>6));   
	 }
  //	   2. Filter write  area    
       DF_ReadFlash(pageoffset+AvrgMintPosit_offset,0,reg,1); 
	   if(reg[0]!=0xff)  // 如果要写入的区域 dirty  ，则地址增然后从新开始寻找知道找到为止
	   	{
          In_write++;
		  AvrgMintPosit_write++;		  
		  if(AvrgMintPosit_write>=Max_MintPos) 
			   AvrgMintPosit_write=0;  
		 // rt_kprintf("\r\n    *******   行车记录仪写区域 Write area : %d   is   Dirity!  \r\n",In_write);  
		  //----------------------------
		  	return  false;
	   	}       
  //   3. Write Record Content 
       DF_WriteFlashDirect(pageoffset+AvrgMintPosit_offset,0,content,saveLen);  //   写入信息
       DF_delay_ms(10);
	// rt_kprintf("\r\n DrvRec Starpageoffset=%d  PageOffset= %d ,  InPageAddr= %d \r\n",VehicleRecStart_offset,pageoffset,InPageoffset); 
     return true; 
  //-------------------------------------------------------------------------------------------- 
}

u8 Read_MintPosition(u32 In_read,u8 *content ,u16 ReadLen)
{  // 读取单位小时内每分钟的位置信息
	/*
	   NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
	*/
    u32  pageoffset=0;   //Page 偏移 
	u8   FCS=0;
	u16  i=0;    

  /*
      上报的每一包数据位31个字节， 每条记录为32个字节，用每条记录的最后一个字节来作为是否上报过的标志位。没有上报该标志为0xFF
      上报过后该标志为被写为0x01
  */
  //----------------------------------------------------------------------------------------------
  //   1. caculate address 
  pageoffset=In_read;				// 计算出 Page 偏移  除以1	
  	// 计算出 页内偏移地址 
    // 计算出页内 字节	乘以 512 (每个记录512个字节)  485(content)+1(FCS)
  //   2. Write Record Content 
    DF_ReadFlash(pageoffset+AvrgMintPosit_offset,0,content,ReadLen);	  
    DF_delay_us(10);
	// rt_kprintf("\r\n  ReadRecoder Starpageoffset=%d  PageOffset= %d  \r\n",VehicleRecStart_offset,pageoffset);   
    // rt_kprintf("\r\n  单位小时每分钟位置读取  Position 内容为 :\r\n");   
	//  for(i=0;i<ReadLen;i++)
	//	rt_kprintf("%2X ",content[i]);   
	//  rt_kprintf("\r\n");         
    
  //  3.  Judge FCS	
	//--------------- 过滤已经发送过的信息 -------
	  FCS = 0;
	   for ( i = 0; i < ReadLen-1; i++ )   //计算读取信息的异或和
	   {
			   FCS ^= *( content + i );  
	   }			  
	   if(((content[ReadLen-1]!=FCS)&&(content[0]!=0xFF))||(content[0]==0xFF))	// 判断异或和 
	    { 	      
		  if(content[0]==0xFF)
		  {
			 AvrgMintPosit_Read=AvrgMintPosit_write;//如果是内容是0xFF ，读指针和写指针相等，不再触发上报。			
			 return false;
		  }
          AvrgMintPosit_Read++;	
		  if(AvrgMintPosit_Read>=Max_MintPos)    
		  	AvrgMintPosit_Read=0; 
		//  rt_kprintf("\r\n   *******  DrvRecoder 该条记录内容不对 *******  \r\n"); 
		  return false;
     	} 
	   	if(content[0]==0xFF)
	   	{
           AvrgMintPosit_Read=AvrgMintPosit_write;//如果是内容是0xFF ，读指针和写指针相等，不再触发上报。           
		   return false;
	   	}
	//--------------------------------------------------------------  	
		 return true; 		    
  //-------------------------------------------------------------------------------------------- 
}

//----------------------------------------------------------------------  
void  CHK_ReadCycle_status(void) 
{  
  		if(RdCycle_Idle==ReadCycle_status)
			  {
			       
				   if(cycle_write!=cycle_read)
					 {
						ReadCycle_status=RdCycle_RdytoSD; 
					 }
				    else
						ReadCycle_status=RdCycle_Idle; 
				
			  } 
}


void MediaIndex_Init(void)  
{
   u8 i=0;
   
   memset((u8*)&MediaIndex,0,sizeof(MediaIndex)); 
   
   for(i=0;i<8;i++)
   {
     MediaIndex.ID=i+1;
	if(i==0)
     {
       MediaIndex.Type=0;
	   memset(MediaIndex.FileName,0,sizeof(MediaIndex.FileName));
	   memcpy(MediaIndex.FileName,"pic.jpg",7);
       DF_WriteFlashSector(DF_PicIndex_Page,0,(u8*)&MediaIndex,sizeof(MediaIndex));  
	   DF_delay_ms(50);
	   MediaIndex.Type=1;
	   memset(MediaIndex.FileName,0,sizeof(MediaIndex.FileName));
	   memcpy(MediaIndex.FileName,"sound.wav",9);
       DF_WriteFlashSector(DF_SoundIndex_Page,0,(u8*)&MediaIndex,sizeof(MediaIndex));  
	   DF_delay_ms(50);
	 }  
	else 
     {
       MediaIndex.Type=0;
	   memset(MediaIndex.FileName,0,sizeof(MediaIndex.FileName));
	   memcpy(MediaIndex.FileName,"pic.jpg",7);
       DF_WriteFlashDirect(DF_PicIndex_Page+i, 0,(u8*)&MediaIndex, sizeof(MediaIndex));    
       DF_delay_ms(10);    
	   MediaIndex.Type=1;
	   memset(MediaIndex.FileName,0,sizeof(MediaIndex.FileName));
	   memcpy(MediaIndex.FileName,"sound.wav",9);
       DF_WriteFlashDirect(DF_SoundIndex_Page+i,0,(u8*)&MediaIndex,sizeof(MediaIndex));      
	   DF_delay_ms(10);
	 }  
   }
}

void Save_Common(u32 In_write,u8 Type)
{
  u8 content[32],regDateTime[6];
  u8 wr_add=0,FCS=0,i;

  
 
   //-----------------
    memset(content,0,sizeof(content));
    wr_add=0; 
   //-----------------
   switch(Type)
   	{
   	   
	   case	 TYPE_ExpSpdAdd:  // 附录A 中没有超速记录报警，但是我们有
	                           break;
	   case  TYPE_TiredDrvAdd:
		                      break;
    
						  
	  case  TYPE_LogInAdd:
	  case	TYPE_SettingChgAdd:
	  	                        Time2BCD(regDateTime);  
	                            memcpy(content+wr_add,regDateTime,6);
							    wr_add+=6; 
								memcpy(content+wr_add,JT808Conf_struct.Driver_Info.DriverCard_ID,18);
							    wr_add+=18;
								switch(Type)  
									{
                                       case  TYPE_LogInAdd:  
									   	                     content[wr_add++]=Login_Status;
															 break;
									   case	TYPE_SettingChgAdd:
									   	                     content[wr_add++]=Settingchg_Status;
															 break;
									}
								break;
     default:
	 	         rt_kprintf("\r\n  Save common Type Error!\r\n");
	 	         return;
   	}
   //-----------------------
   FCS = 0;
   for ( i = 0; i < 32; i++ )  
   {
		 FCS ^=content[i];
   }			  //求上边数据的异或和
   content[wr_add++] = FCS;	  // 第31字节    
    if(DispContent)
   rt_kprintf("\r\n Common  WriteLen:  %d  \r\n ",wr_add);   
   Common_WriteContent( In_write, content, wr_add, Type);       	
}

