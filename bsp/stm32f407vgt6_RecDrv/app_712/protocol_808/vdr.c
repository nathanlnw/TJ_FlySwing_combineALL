/*
   vdr Vichle Driver Record 车辆行驶记录
 */

#include <rtthread.h>
#include <finsh.h>
#include "stm32f4xx.h"

#include <rtdevice.h>
#include <dfs_posix.h>

#include <time.h>

#include "SST25VF.h"
#include "Vdr.h"
#include "App_moduleConfig.h"


//  =================  行车记录仪 相关 ============================
/*
    StartPage :    6320          Start Address offset :   0x316000       

    Area Size :
                          213  Sector       = 1704 pages
                           ----------------------
                           
				扇区               
				1                                     00-07H
				90                                    08H               
				85                                    09H
				7                                      10H
				2                                      11H 
				2                                      12H
				1                                      13H
				1                                      14H  
				1                                      15H   

          ----------  只是在这里做了---  注释 ，具体操作在 Vdr.C  				
*/

#define       SECTORSIZE                  4096
#define       VDR_START_ADDRESS           0x316000
#define       MarkByte                    0x0F          // 表示写入过相关信息

//-------------------------- Sectors-----------  total   195 Sectors --------------
#define  VDR_00_07_SIZE      1
#define  VDR_08_SIZE         90    //   128 rec_size               1sector=32 recrods    2880/32=90
#define  VDR_09_SIZE         72    //  666=>>768 rec_size      1sector=5 recrods     360/5=72
#define  VDR_10_SIZE         7     //   234=>> 256 rec_size    1sector=16              100/16 =7
#define  VDR_11_SIZE         2     //   50 =>> 64                  1 sector=64             100/64 =2 
#define  VDR_12_SIZE         2     //    25 =>> 32                 1 sector=128            200/128=2
#define  VDR_13_SIZE         1     //     7==>>  8                 1 sector= 512             100/512=1
#define  VDR_14_SIZE         1     //     7==>>  8                 1 sector= 512             100/512=1
#define  VDR_15_SIZE         1     //  133==>256                  1 sector =16               10/16 =1

 //   存储区域最大条数
#define  VDR_08_MAXindex         2880
#define  VDR_09_MAXindex         360   
#define  VDR_10_MAXindex         100
#define  VDR_11_MAXindex         10
#define  VDR_12_MAXindex         200
#define  VDR_13_MAXindex         100
#define  VDR_14_MAXindex         100
#define  VDR_15_MAXindex         10   



//----------------------absolutely  address   --------------  
#define VDR_00_to_07H    VDR_START_ADDRESS 

#define VDR_08H_START	 VDR_START_ADDRESS+VDR_00_07_SIZE*SECTORSIZE
#define VDR_08H_END		 VDR_START_ADDRESS+VDR_00_07_SIZE*SECTORSIZE-1

#define VDR_09H_START	 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE)*SECTORSIZE
#define VDR_09H_END		 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE)*SECTORSIZE-1


#define VDR_10H_START	 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE)*SECTORSIZE
#define VDR_10H_END		 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE)*SECTORSIZE-1

#define VDR_11H_START	 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE)*SECTORSIZE
#define VDR_11H_END		 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE)*SECTORSIZE-1

#define VDR_12H_START	 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE)*SECTORSIZE
#define VDR_12H_END		 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE)*SECTORSIZE-1

#define VDR_13H_START	 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE)*SECTORSIZE
#define VDR_13H_END		 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE)*SECTORSIZE-1

#define VDR_14H_START	 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE+VDR_13_SIZE)*SECTORSIZE
#define VDR_14H_END		 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE+VDR_13_SIZE)*SECTORSIZE-1

#define VDR_15H_START	 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE+VDR_13_SIZE+VDR_14_SIZE)*SECTORSIZE
#define VDR_15H_END		 VDR_START_ADDRESS+(VDR_00_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE+VDR_13_SIZE+VDR_14_SIZE)*SECTORSIZE-1 






//  -   no  use  actually----
#define VDR_08H_09H_START	 VDR_START_ADDRESS+VDR_00_07_SIZE*SECTORSIZE
#define VDR_08H_09H_END		 VDR_START_ADDRESS+VDR_00_07_SIZE*SECTORSIZE-1




/*
   实现每条记录128字节，便于定位。要做一定的处理。没有使用delta编码

   记录格式

   000--003  yy-mm-dd hh:mm   可以把时间转成UTC格式,便于比较查找
      又可以节省一个字节，无效的时间格式FFFFFFFFFF
   004--056  秒速度记录 速度要求+/- 1kmh
          原先60*8=480bit 现在用7bit保存需要 60*7=420bit=53byte
          7bit的 0b1111111 表示速度无效(gps未定位)
   057--116  状态信息
   117--126  单位分钟位置,参见行车记录仪 GBT19065 首个有效位置

 */


/*
   4MB serial flash 0x400000
 */

/*转换hex到bcd的编码*/
#define HEX_TO_BCD( A ) ( ( ( ( A ) / 10 ) << 4 ) | ( ( A ) % 10 ) )



/*基于小时的时间戳,主要是为了比较大小使用
   byte0 year
   byte1 month
   byte2 day
   byte3 hour
 */
typedef unsigned int YMDH_TIME;

typedef struct
{
	uint8_t cmd;

	uint32_t	ymdh_start;
	uint8_t		minute_start;

	uint32_t	ymdh_end;
	uint8_t		minute_end;

	uint32_t	ymdh_curr;
	uint8_t		minute_curr;
	uint32_t	addr;

	uint16_t	blocks;         /*定义每次上传多少个数据块*/
	uint16_t	blocks_remain;  /*当前组织上传包是还需要的的blocks*/
}VDR_CMD;

VDR_CMD		vdr_cmd;

uint8_t		vdr_tx_info[1024];
uint16_t	vtr_tx_len = 0;


/*传递写入文件的信息
   0...3  写入SerialFlash的地址
   4...31 文件名
 */

//=====================================================
VDR_INDEX         Vdr_Wr_Rd_Offset;      //  记录仪写索引 位置
VDR_DATA          VdrData;               //  行驶记录仪的databuf
VDR_TRIG_STATUS   VDR_TrigStatus;









//======================================================
/*
      遍历获取当前行驶记录的操作位置

      type :    0  代表遍历所有      08  09  10  11 12 13  14 
      u8    :     0   :遍历所有  
      
*/
u8  Vdr_PowerOn_getWriteIndex(u8 type)   
{
   
       u16    i;
	   u8	  c;  
      
	  //	 1.  get  address	


     switch(type)
     	{
		   case 0x08:
		   	           // 1.1  先读取第一个字符做判断
		   	           //rt_kprintf("\r\n 08H write=0x%X  \r\n",VDR_08H_START); 
		   	           for(i=0;i<VDR_08_MAXindex;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_08H_START+i*128);
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_08H_Write=i; 
							  rt_kprintf("\r\n 08H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_08H_Write); 
                              break;
						    }
		   	           	}
					   // 1.2. 没有找到2880 那么 擦除第一个  从0 开始
					   if(i==VDR_08_MAXindex)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_08H_START );  
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_08H_Write=0;
					   	}							   	
		   	          
		   	           break;
		   case 0x09:
		   	           //  2.1 
		   	           // rt_kprintf("\r\n 09H write=0x%X  \r\n",VDR_09H_START); 
		   	          for(i=0;i<360;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_09H_START+i*768);
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_09H_Write=i; 
							  rt_kprintf("\r\n 09H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_09H_Write); 
                              break;
						    }
		   	           	}
					   // 2. 没有找到360 那么 擦除第一个  从0 开始
					   if(i==360)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_09H_START );   
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_09H_Write=0;  
					   	} 
					   break;
		   
		   case 0x10:
		   	          //  rt_kprintf("\r\n 10H write=0x%X  \r\n",VDR_10H_START);  
		   	           for(i=0;i<100;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_10H_START+i*256);
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_10H_Write=i; 
							  rt_kprintf("\r\n 10H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_10H_Write);  
                              break;
						    }
		   	           	}
					   // 2. 没有找到100 那么 擦除第一个  从0 开始
					   if(i==100)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_10H_START );    
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_10H_Write=0;   
					   	}  
					   break;
		   
		   case 0x11:
		   	            for(i=0;i<100;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_11H_START+i*64);
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_11H_Write=i; 
							  rt_kprintf("\r\n 11H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_11H_Write);   
                              break;
						    }
		   	           	}
					   // 2. 没有找到100 那么 擦除第一个  从0 开始
					   if(i==100)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_11H_START );    
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_11H_Write=0;       
					   	}  
					   break;

		   case 0x12: 
		   	            for(i=0;i<200;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_12H_START+i*32);
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_12H_Write=i; 
							  rt_kprintf("\r\n 12H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_12H_Write);   
                              break;
						    }
		   	           	}
					    // 2. 没有找到200 那么 擦除第一个  从0 开始 
					    if(i==200)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_12H_START );   
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_12H_Write=0;           
					   	}  
						break;

		   case 0x13:   for(i=0;i<100;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_13H_START+i*8); 
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_11H_Write=i; 
							  rt_kprintf("\r\n 13H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_13H_Write);   
                              break;
						    }
		   	           	}
					   // 2. 没有找到100 那么 擦除第一个  从0 开始
					   if(i==100)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_13H_START );     
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_13H_Write=0;       
					   	}  
						break;

		   case 0x14:   
		   	             for(i=0;i<100;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_14H_START+i*8); 
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_14H_Write=i; 
							  rt_kprintf("\r\n 14H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_14H_Write);    
                              break;
						    }
		   	           	}
					   // 2. 没有找到100 那么 擦除第一个  从0 开始
					   if(i==100)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_14H_START );        
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_14H_Write=0;       
					   	}  
						break;

		   case 0x15:
		   	            for(i=0;i<10;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_15H_START+i*256);  
						    if(c==0xFF)
						    {
						      Vdr_Wr_Rd_Offset.V_15H_Write=i; 
							  rt_kprintf("\r\n 15H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_15H_Write);    
                              break;
						    }
		   	           	}
					   // 2. 没有找到10 那么 擦除第一个  从0 开始
					   if(i==10)
					   	{
					   	   WatchDog_Feed();
				           SST25V_SectorErase_4KByte( VDR_15H_START );     
						   DF_delay_ms(80);
						   Vdr_Wr_Rd_Offset.V_15H_Write=0;          
					   	}  
						break;

     	}
  return true;
}


//    遍历所有行车记录写入的起始地址   from  08H
void total_ergotic(void)    
{
    Vdr_PowerOn_getWriteIndex(0x08);
    Vdr_PowerOn_getWriteIndex(0x09);
	Vdr_PowerOn_getWriteIndex(0x10);
    Vdr_PowerOn_getWriteIndex(0x11);
	Vdr_PowerOn_getWriteIndex(0x12);
    Vdr_PowerOn_getWriteIndex(0x13);
	Vdr_PowerOn_getWriteIndex(0x14);
	Vdr_PowerOn_getWriteIndex(0x15);     

	Vdr_Wr_Rd_Offset.V_07H_Read=0;
	Vdr_Wr_Rd_Offset.V_08H_Read=0;
	Vdr_Wr_Rd_Offset.V_09H_Read=0;
	Vdr_Wr_Rd_Offset.V_10H_Read=0;
	Vdr_Wr_Rd_Offset.V_11H_Read=0;
	Vdr_Wr_Rd_Offset.V_12H_Read=0;
	Vdr_Wr_Rd_Offset.V_13H_Read=0;
	Vdr_Wr_Rd_Offset.V_14H_Read=0;
	Vdr_Wr_Rd_Offset.V_15H_Read=0; 
}
FINSH_FUNCTION_EXPORT( total_ergotic, total_ergotic );
//   只能在初始化是用到
void vdr_erase(void) 
{
   u16  i=0;
    // start address        0x316000        start sector :790th        total: 195 sector 

    //  1. erase  to  839          840-790=50 
// if(rt_mutex_take(DF_lock_mutex,150)==RT_EOK) 
{
   DF_LOCK=1;
	//  32K
	 rt_kprintf("\r\n   <------------  vdr  area  erase  0 ------------------>\r\n");   
   for(i=0;i<10;i++)
   {
	 WatchDog_Feed();
	 SST25V_SectorErase_4KByte(VDR_START_ADDRESS+i*SECTORSIZE);  	 
	// rt_kprintf("\r\n  addr=0x%X \r\n",VDR_START_ADDRESS+i*SECTORSIZE);   
     delay_ms(150);     
	 
   }	 
   
   rt_kprintf("\r\n   <------------  vdr  area  erase  1 ------------------>\r\n");   
   
   #if 1
   // 195-50=145         145-64*2=17             0x320000
   for(i=0;i<22;i++) 
   {
	 WatchDog_Feed();
	// SST25V_BlockErase_64KByte(VDR_START_ADDRESS+0xA000+i*0x10000);   // 32k  <=> 0x8000   
      
	 SST25V_BlockErase_32KByte(VDR_START_ADDRESS+0xA000+i*0x8000);   // 32k  <=> 0x8000   8sector
     DF_delay_ms(700); 
	 WatchDog_Feed();
	 
	// rt_kprintf("\r\n  addr=0x%X \r\n",VDR_START_ADDRESS+0xA000+i*0x8000);   
	//  rt_kprintf("\r\n   <------------ erase 32K : %d\r\n",i+1);    
   }
  #endif

  
   DF_LOCK=0;
 //  rt_mutex_release(DF_lock_mutex);  
 }   

   rt_kprintf("\r\n   <------------  vdr  area  erase   over  ------------------>\r\n");  
}
FINSH_FUNCTION_EXPORT( vdr_erase, vdr_erase ); 




/*
         根据输入的类型，和起始时间查找到符合条件的起始地址和条数
*/
void query_vdr(u8 type,u8 *intime, u16 current_start,u16 num)
{
  u16 i=0;
	u8  c;

   switch(type)
   {
		  case 0x08:
					  // 1.1  先读取当前写入过的时间
                      if(Vdr_Wr_Rd_Offset.V_08H_Write==0)
                      	{
                             current_start=0;
							 num=0;
							 break;
                      	}
                      
                      //  1.2    读取当前最近的记录的时间
					  
					  for(i=0;i<VDR_08_MAXindex;i++)
					   {
						   c=SST25V_ByteRead(VDR_08H_START+i*128);
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_08H_Write=i; 
							 rt_kprintf("\r\n 08H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_08H_Write); 
							 break;
						   }
					   }
					  // 1.2. 没有找到2880 那么 擦除第一个	从0 开始
					  if(i==VDR_08_MAXindex) 
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_08H_START );  
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_08H_Write=0;
					   }							   
					 
					  break;
		  case 0x09:
					  //  2.1 
					 for(i=0;i<360;i++)
					   {
						   c=SST25V_ByteRead(VDR_09H_START+i*768);
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_09H_Write=i; 
							 rt_kprintf("\r\n 09H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_09H_Write); 
							 break;
						   }
					   }
					  // 2. 没有找到360 那么 擦除第一个  从0 开始
					  if(i==360)
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_09H_START );	
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_09H_Write=0;  
					   } 
					  break;
		  
		  case 0x10:
					  for(i=0;i<100;i++)
					   {
						   c=SST25V_ByteRead(VDR_10H_START+i*256);
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_10H_Write=i; 
							 rt_kprintf("\r\n 10H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_10H_Write);  
							 break;
						   }
					   }
					  // 2. 没有找到100 那么 擦除第一个  从0 开始
					  if(i==100)
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_10H_START );	
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_10H_Write=0;	
					   }  
					  break;
		  
		  case 0x11:
					   for(i=0;i<100;i++)
					   {
						   c=SST25V_ByteRead(VDR_11H_START+i*64);
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_11H_Write=i; 
							 rt_kprintf("\r\n 11H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_11H_Write);   
							 break;
						   }
					   }
					  // 2. 没有找到100 那么 擦除第一个  从0 开始
					  if(i==100)
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_11H_START );	
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_11H_Write=0;		
					   }  
					  break;
   
		  case 0x12: 
					   for(i=0;i<200;i++)
					   {
						   c=SST25V_ByteRead(VDR_12H_START+i*32);
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_12H_Write=i; 
							 rt_kprintf("\r\n 12H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_12H_Write);   
							 break;
						   }
					   }
					   // 2. 没有找到200 那么 擦除第一个  从0 开始 
					   if(i==200)
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_12H_START );		
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_12H_Write=0;			
					   }  
					   break;
   
		  case 0x13:   for(i=0;i<100;i++)
					   {
						   c=SST25V_ByteRead(VDR_13H_START+i*8); 
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_11H_Write=i; 
							 rt_kprintf("\r\n 13H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_13H_Write);   
							 break;
						   }
					   }
					  // 2. 没有找到100 那么 擦除第一个  从0 开始
					  if(i==100)
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_13H_START );		
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_13H_Write=0;		
					   }  
					   break;
   
		  case 0x14:   
						for(i=0;i<100;i++)
					   {
						   c=SST25V_ByteRead(VDR_14H_START+i*8); 
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_14H_Write=i; 
							 rt_kprintf("\r\n 14H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_14H_Write);	
							 break;
						   }
					   }
					  // 2. 没有找到100 那么 擦除第一个  从0 开始
					  if(i==100)
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_14H_START );		 
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_14H_Write=0;		
					   }  
					   break;
   
		  case 0x15:
					   for(i=0;i<10;i++)
					   {
						   c=SST25V_ByteRead(VDR_15H_START+i*256);	
						   if(c==0xFF)
						   {
							 Vdr_Wr_Rd_Offset.V_15H_Write=i; 
							 rt_kprintf("\r\n 15H write=%d	\r\n",Vdr_Wr_Rd_Offset.V_15H_Write);	
							 break;
						   }
					   }
					  // 2. 没有找到10 那么 擦除第一个	从0 开始
					  if(i==10)
					   {
						  WatchDog_Feed();
						  SST25V_SectorErase_4KByte( VDR_15H_START );		
						  DF_delay_ms(80);
						  Vdr_Wr_Rd_Offset.V_15H_Write=0;		   
					   }  
					   break;
   
	   }

 
}

//    创建行驶记录仪数据  

void  VDR_product_08H_09H_10H(void)
{
  
  u16 j=0,cur=0;	 
  
  //  行驶记录相关数据产生 触发,  且定位的情况下   
  if((VDR_TrigStatus.Running_state_enable==1)&&(UDP_dataPacket_flag==0X02)) 	  
  { 	//	  必须在行驶状态下
			
			//-------------------  08H	 --------------------------------------------------
			if((Temp_Gps_Gprs.Time[2]==0)&&(VDR_TrigStatus.Running_state_enable==1)) 
			{  // 秒为0 
				// 0.  判断是否是第一次上电
				if((VdrData.H_08[0]==0x00)&&(VdrData.H_08[1]==0x00)) 
				{
					  rt_kprintf("\r\n first  get  08H	  \r\n");
				}
				else
				{ 
					  memcpy(VdrData.H_08_BAK,VdrData.H_08,126);
					  VdrData.H_08_saveFlag=1;	//	 保存  当前分钟的数据记录					 
				}	 
			   //  1.  年月日 时分秒	 秒是0
				 VdrData.H_08[0]=((Temp_Gps_Gprs.Date[0]/10)<<4)+(Temp_Gps_Gprs.Date[0]%10);	   
				 VdrData.H_08[1]=((Temp_Gps_Gprs.Date[1]/10)<<4)+(Temp_Gps_Gprs.Date[1]%10); 
				 VdrData.H_08[2]=((Temp_Gps_Gprs.Date[2]/10)<<4)+(Temp_Gps_Gprs.Date[2]%10);
				 VdrData.H_08[3]=((Temp_Gps_Gprs.Time[0]/10)<<4)+(Temp_Gps_Gprs.Time[0]%10);
				 VdrData.H_08[4]=((Temp_Gps_Gprs.Time[1]/10)<<4)+(Temp_Gps_Gprs.Time[1]%10);
				 VdrData.H_08[5]=0; 	
  
			   // 2.  initial 
			   memset(VdrData.H_08+6,0xFF,120);    // 默认是 0xFF
			}  
			   // 3.   save 	Minute		   1-59   s 		
			   VdrData.H_08[6+2*Temp_Gps_Gprs.Time[2]]=Speed_gps/10; //km/h
			   VdrData.H_08[7+2*Temp_Gps_Gprs.Time[2]]=Vehicle_sensor;	
  
		  //------------------------09H ---------  min=0;	sec=0	----------------------------
		   if((Temp_Gps_Gprs.Time[1]==0)&&(Temp_Gps_Gprs.Time[2]==0)) 
		   {
			   
				// 1.  判断是否是需要存储
					VdrData.H_09_saveFlag=1;  //   保存  当前分钟的数据记录 
			   //  2.  年月日 时分秒	 |	 分  秒是0
				 VdrData.H_09[0]=((Temp_Gps_Gprs.Date[0]/10)<<4)+(Temp_Gps_Gprs.Date[0]%10);	   
				 VdrData.H_09[1]=((Temp_Gps_Gprs.Date[1]/10)<<4)+(Temp_Gps_Gprs.Date[1]%10); 
				 VdrData.H_09[2]=((Temp_Gps_Gprs.Date[2]/10)<<4)+(Temp_Gps_Gprs.Date[2]%10);
				 VdrData.H_09[3]=((Temp_Gps_Gprs.Time[0]/10)<<4)+(Temp_Gps_Gprs.Time[0]%10);
				 VdrData.H_09[4]=0;
				 VdrData.H_09[5]=0;
  
				 //  initial  0xFF
				 memset(VdrData.H_09+6,0xFF,660);	 // 默认是 0xFF
  
		   }   
		   else
			// 3.  判断是否是第一次上电 ,  如果是那么给他赋值
		   if((VdrData.H_09[0]==0x00)&&(VdrData.H_09[1]==0x00))  // 年月为 0
		   {
				 VdrData.H_09[0]=((Temp_Gps_Gprs.Date[0]/10)<<4)+(Temp_Gps_Gprs.Date[0]%10);	   
				 VdrData.H_09[1]=((Temp_Gps_Gprs.Date[1]/10)<<4)+(Temp_Gps_Gprs.Date[1]%10); 
				 VdrData.H_09[2]=((Temp_Gps_Gprs.Date[2]/10)<<4)+(Temp_Gps_Gprs.Date[2]%10);
				 VdrData.H_09[3]=((Temp_Gps_Gprs.Time[0]/10)<<4)+(Temp_Gps_Gprs.Time[0]%10);
				 VdrData.H_09[4]=0; 
				 VdrData.H_09[5]=0;
		   }   
  
		   //  4 .	填写常规位置	1-59 min
		   if(Temp_Gps_Gprs.Time[2]==59)  // 59 秒时候做处理，如果是0 那么分钟就变了
		   {  
			  //   表 A.20		  (当前分钟的位置  +	当前分钟的平均速度) 
			  memcpy( VdrData.H_09+6+Temp_Gps_Gprs.Time[1]*11,VdrData.Longi,4);  // 经度
			  memcpy( VdrData.H_09+6+Temp_Gps_Gprs.Time[1]*11+4,VdrData.Lati,4); //纬度
			  VdrData.H_09[6+Temp_Gps_Gprs.Time[1]*11+8]=(GPS_Hight>>8);
			  VdrData.H_09[6+Temp_Gps_Gprs.Time[1]*11+9]=GPS_Hight; 		   
			  //  当前分钟的平均速度	 从AvrgSpd_MintProcess()  引用的变量
			  VdrData.H_09[6+Temp_Gps_Gprs.Time[1]*11+10]= PerMinSpdTotal/AspdCounter;			  
		   }
		 //-------------------- 10H    事故疑点数据  ---------------------------------
		 if((VdrData.H_10_saveFlag==0)&&(XinhaoStatus[11]=='1')&&(XinhaoStatusBAK[11]=='0')) 
		 {	//	刹车状态发生变化
			  //  1.  行驶结束时间
			 time_now=Get_RTC();	 //  RTC  相关 
			 Time2BCD(VdrData.H_10); 
			 //   2.   机动车驾驶证号码
			 memcpy(VdrData.H_10+6,JT808Conf_struct.Driver_Info.DriverCard_ID,18);
			 //   3.  速度和状态信息   
					  //-----------------------  Status Register   --------------------------------
					cur=save_sensorCounter;  //20s的事故疑点
					for(j=0;j<100;j++)
					{
						VdrData.H_10[24+j*2]=Sensor_buf[cur].DOUBTspeed;	//速度
						VdrData.H_10[24+j*2+1]=Sensor_buf[cur].DOUBTstatus;   //状态	
						
						if(cur>0)	  // 从当前往前读取
						   cur--;
						else
						   cur=100; 						   
					}  
			 // 	4.	位置信息
			  memcpy( VdrData.H_10+224,VdrData.Longi,4);  // 经度
			  memcpy( VdrData.H_10+224+4,VdrData.Lati,4); //纬度
			  VdrData.H_10[224+8]=(GPS_Hight>>8);
					VdrData.H_10[224+9]=GPS_Hight;	
  
			 //--------------------------------------------------------------------------			  
			 VdrData.H_10_saveFlag=1;
		 }		
		 memcpy(XinhaoStatusBAK,XinhaoStatus,20);
		 // Vehicle_sensor_BAK=Vehicle_sensor;
  
   }
  
  
}


void VDR_product_11H_Start(void)
{
           //    11 H   连续驾驶开始时间   和起始位置     从60s 开始算驾驶起始
                   if(VDR_TrigStatus.Run_baseon_spd_10s_couter==60)
                   	{ 
                   	    //  1.   机动车驾驶证号
                   	    memcpy(VdrData.H_11,JT808Conf_struct.Driver_Info.DriverCard_ID,18);
					    //   2.   起始时间
						time_now=Get_RTC();     //  RTC  相关 
			            Time2BCD(VdrData.H_11+18); 
                        //   3.  起始位置                        
						memcpy( VdrData.H_11+30,VdrData.Longi,4);  // 经度
						memcpy( VdrData.H_11+30+4,VdrData.Lati,4); //纬度
						VdrData.H_11[30+8]=(GPS_Hight>>8);
						VdrData.H_11[30+9]=GPS_Hight;	

                   	}
}

void VDR_product_11H_End(void)
{
         if(TiredConf_struct.TiredDoor.Door_DrvKeepingSec<=0) 
		 	 return;
            //         11 H     相关
        if(VDR_TrigStatus.Run_baseon_spd_10s_couter>TiredConf_struct.TiredDoor.Door_DrvKeepingSec)
        	{
                   
			
                    //   2.   结束时间
						time_now=Get_RTC();     //  RTC  相关 
			       Time2BCD((VdrData.H_11+25)); 
                        //   3.  起始位置                        
						memcpy( VdrData.H_11+40,VdrData.Longi,4);  // 经度
						memcpy( VdrData.H_11+40+4,VdrData.Lati,4); //纬度
						VdrData.H_11[40+8]=(GPS_Hight>>8);
						VdrData.H_11[40+9]=GPS_Hight;	
                    //    4.   save 
                    VdrData.H_11_saveFlag=1; 
        	}
}




void VDR_product_12H(u8  value)    // 插拔卡记录
{
     //   1. 时间发生的时间
		time_now=Get_RTC();     //  RTC  相关 
        Time2BCD(VdrData.H_12); 

     //  2.   机动车驾驶证号
         memcpy(VdrData.H_12+6,JT808Conf_struct.Driver_Info.DriverCard_ID,18);
     //  3.  结果 
         VdrData.H_12[24]=value;

	 // 4.  save 
	     VdrData.H_12_saveFlag=1; 
}


//  Note :  定义了但是没有用，不带电池情况下，断电操作SPI DF  危害会很大
void VDR_product_13H(u8  value)    //  外部供电记录
{
       //   1. 时间发生的时间
		time_now=Get_RTC();     //  RTC  相关 
        Time2BCD(VdrData.H_13); 
       //  2.  value
         VdrData.H_13[6]=value;
       //  3. save
       VdrData.H_13_saveFlag=1; 
}


void VDR_product_14H(u8 cmdID)    // 记录仪参数修改记录
{

    //   1. 时间发生的时间
		time_now=Get_RTC();     //  RTC  相关 
        Time2BCD(VdrData.H_14); 
       //  2.  value
         VdrData.H_14[6]=cmdID;
       //  3. save
       VdrData.H_14_saveFlag=1;
}


 
// note  :    需要索引检索   08 H 的数据记录 ，这里不需要填充。
void VDR_product_15H(u8  value)    //  采集指定速度记录 
{
    




}

void  VDR_get_15H_StartEnd_Time(u8  *Start, u8 *End)  
{
    u8  i=0;

	for(i=0;i<6;i++)
	{
      VdrData.H_15_Set_StartDateTime[i]=Start[i];
	  VdrData.H_15_Set_EndDateTime[i]=End[i];
	}
	
}

//------- stuff     user data       ---------------
u16  stuff_drvData(u8 type,u16 Start_recNum,u16 REC_nums,u8 *dest)
{
    u16  res_len=0,get_len=0;
    u16  i=0;

	for(i=0;i<REC_nums;i++)
	{
	  switch(type)
	  	{
	  	   case 0x08:
                     get_len=get_08h(Start_recNum+i,dest+res_len);
					 break;
		   case 0x09:
                     get_len=get_09h(Start_recNum+i,dest+res_len);    
					 break;
		   case 0x10:
                     get_len=get_10h(Start_recNum+i,dest+res_len);
					 break;
		   case 0x11:
                     get_len=get_11h(Start_recNum+i,dest+res_len);
					 break;
		   case 0x12:
                     get_len=get_12h(Start_recNum+i,dest+res_len);
					 break;	
		   case 0x13:
                     get_len=get_13h(Start_recNum+i,dest+res_len);
					 break;		
		   case 0x14:
                     get_len=get_14h(Start_recNum+i,dest+res_len);
					 break;		
		   case 0x15:
                     get_len=get_15h(Start_recNum+i,dest+res_len); 
					 break;	
		   default:
		   	         return 0;

	  	}
	   
	   res_len+=get_len;
	}    
    return   res_len;
}









// 00H  采集行车记录仪执行标准版本
u16   get_00h(u8  *p_in ,u16 inLen, u8 *p_out)   
{

 return 0;
}

// 01H  采集当前驾驶员人信息
u16   get_01h(u8  *p_in ,u16 inLen, u8 *p_out)  
{

 return 0;
}

//  02H   采集记录仪的实时时间
u16   get_02h(u8  *p_in ,u16 inLen, u8 *p_out)  
{

 return 0;
}

// 03H  采集累计行驶里程
u16   get_03h(u8  *p_in ,u16 inLen, u8 *p_out)  
{

 return 0;
}

u16   get_04h(u8  *p_in ,u16 inLen, u8 *p_out)  
{

 return 0;
}

u16   get_05h(u8  *p_in ,u16 inLen, u8 *p_out)  
{

 return 0;
}

u16   get_06h(u8  *p_in ,u16 inLen, u8 *p_out)  
{

 return 0;
}

u16   get_07h(u8  *p_in ,u16 inLen, u8 *p_out)  
{

 return 0;
}

/*
   行驶速度记录
   不管他，自己填数据，自己报
   单条126字节每分钟
   48*60=2880包

    每一条   128 字节

 */

u16 get_08h( u16 indexnum,u8 *p)    
{
	int			i;
	uint8_t		buf[128];
    u32  addr=0;
	u8  FCS=0;


    
	//	  1.  get  address	 
	 addr=VDR_08H_START+indexnum*128;

   //    2. read   
      SST25V_BufferRead(buf, addr, 128 );
      DF_delay_ms(60);
       OutPrint_HEX("08H content",buf,128); // debug   
   //     3.  FCS  check
           FCS=0;
       for(i=0;i<127;i++)
	 	   FCS^=buf[i];
	   if(buf[127]!=FCS)
	   	{
	   	     rt_kprintf("\r\n  08H read fcs error  save:%X  cacu: %X \r\n",buf[127],FCS);
             return  0;
	   	}
     if(buf[0]==MarkByte)
      {  
         memcpy(p,buf+1,126);
	     return  126;
      }	
	 else 
	 	 return 0; 
}



//FINSH_FUNCTION_EXPORT( get_08h, get_08 );


/*
   位置信息
   360小时   ，每小时666字节
 */

u16  get_09h( u16 indexnum,u8 *p)  
{
	 int		 i;
	 uint8_t	 buf[669];
	 u32  addr=0;
	 u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_09H_START+indexnum*768;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 669 ); //2+content 
	   DF_delay_ms(60);
	   OutPrint_HEX("09H content",buf,669); // debug 
	//	   3.  FCS	check
	          FCS=0;
		for(i=0;i<667;i++)
			FCS^=buf[i];
		if(buf[667]!=FCS)
		 {
			  rt_kprintf("\r\n	09H read fcs error	save:%X  cacu: %X \r\n",buf[667],FCS);
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,666); 
		  return  666;
	   } 
	  else 
		  return 0; 

}

//FINSH_FUNCTION_EXPORT( get_09h, get_09 );


/*
   事故疑点
   234Byte
   共100个
 */
u16 get_10h( u16 indexnum,u8 *p)  
{
	 int		 i;
	 uint8_t	 buf[237];
	 u32  addr=0;
	 u8   FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_10H_START+indexnum*256;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 236 );	
	   DF_delay_ms(60); 
	   OutPrint_HEX("10H content",buf,236); // debug 
	//	   3.  FCS	check
	         FCS=0;
		for(i=0;i<235;i++)
			FCS^=buf[i];
		if(buf[235]!=FCS)
		 {
			  rt_kprintf("\r\n	10H read fcs error	save:%X  cacu: %X \r\n",buf[235],FCS);
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,234); 
		  return  234;
	   } 
	  else 
		  return 0; 

}

//FINSH_FUNCTION_EXPORT( get_10h, get_10 );


/*超时驾驶记录
   50Bytes   100条

      每个记录   64   个字节
 */

u16 get_11h( u16 indexnum,u8 *p)   
{
	
	 int		 i;
	 uint8_t	 buf[60];
	 u32  addr=0;
	 u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_11H_START+indexnum*64;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 52 );	
	   DF_delay_ms(10);
	   OutPrint_HEX("11H content",buf,52); // debug 
	//	   3.  FCS	check
	        FCS=0;
		for(i=0;i<51;i++)
			FCS^=buf[i];
		if(buf[51]!=FCS)
		 {
			  rt_kprintf("\r\n	11H read fcs error	save:%X  cacu: %X \r\n",buf[51],FCS);
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,50); 
		  return  50;
	   } 
	  else 
		  return 0; 

	
}

//FINSH_FUNCTION_EXPORT( get_11h, get_11 );


/*驾驶员身份登录
   25Bytes   200条
 */

u16 get_12h( u16 indexnum,u8 *p)  
{
     int		 i;
	 uint8_t	 buf[30];
	 u32  addr=0;
	 u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_12H_START+indexnum*32;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 27 ); 
	   DF_delay_ms(10);
	   OutPrint_HEX("12H content",buf,27); // debug 
	//	   3.  FCS	check
	       FCS=0;
		for(i=0;i<26;i++)
			FCS^=buf[i];
		if(buf[26]!=FCS)
		 {
			  rt_kprintf("\r\n	11H read fcs error	save:%X  cacu: %X \r\n",buf[26],FCS); 
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,25); 
		  return  25;
	   } 
	  else 
		  return 0; 
}

//FINSH_FUNCTION_EXPORT( get_12h, get_12 );


/*外部供电记录
   7字节 100条**/
u16 get_13h( u16 indexnum,u8 *p)  
{
    // int		 i;
	 uint8_t	 buf[10];
	 u32  addr=0;
	// u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_13H_START+indexnum*8;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 8 );	
	   OutPrint_HEX("13H content",buf,8); // debug 
	//	   3.  FCS	check
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,7); 
		  return  7;
	   } 
	  else 
		  return 0; 

}

//FINSH_FUNCTION_EXPORT( get_13h, get_13 );


/*记录仪参数
   7字节 100条    8 */
u16 get_14h( u16 indexnum,u8 *p)  
{
  //   int		 i;
	 uint8_t	 buf[10];
	 u32  addr=0;
	// u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_14H_START+indexnum*8;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 8 );	
	   OutPrint_HEX("14H content",buf,8); // debug 
	//	   3.  FCS	check
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,7); 
		  return  7;
	   } 
	  else 
		  return 0; 

}

//FINSH_FUNCTION_EXPORT( get_14h, get_14 );


/*
   速度状态日志
   133Byte    256
   共10个
 */
u16 get_15h( u16 indexnum,u8 *p)  
{
	
     int		 i;
	 uint8_t	 buf[150];
	 u32  addr=0;
	 u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_15H_START+indexnum*256;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 135 );
	   OutPrint_HEX("15H content",buf,135); // debug 
	//	   3.  FCS	check
		for(i=0;i<134;i++)
			FCS^=buf[i];
		if(buf[134]!=FCS)
		 {
			  rt_kprintf("\r\n	15H read fcs error	save:%X  cacu: %X \r\n",buf[134],FCS); 
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,133); 
		  return  133;
	   } 
	  else 
		  return 0;

}

//FINSH_FUNCTION_EXPORT( get_15h, get_15 );








//==========  write   ------------------------------------------------------------------------------
u16  vdr_creat_08h( u16 indexnum,u8 *p, u16 inLen) 
{
    u8  inbuf[128];
	u8  FCS=0,i=0;
	u32 inaddress=0;//  写入地址

    //  1.  Stuff  Head 
	inbuf[0]=MarkByte;           //
    //  2.   Index  get  address  
    if(indexnum>VDR_08_MAXindex)
    	{
             Vdr_Wr_Rd_Offset.V_08H_Write=0; 
			 indexnum=0;
    	}
	memcpy(inbuf+1,p,inLen);   //  126 bytes
   //   3.   caculate  fcs
     for(i=0;i<127;i++)
	 	FCS^=inbuf[i];
	 inbuf[127]=FCS;   
   //    4.  get  address 	
	inaddress=VDR_08H_START+indexnum*128;

	//  判断擦除相关区域
	if( ( inaddress & 0x0FFF ) == 0 )
			{
			    WatchDog_Feed();
				SST25V_SectorErase_4KByte( inaddress );
				delay_ms(150);
			}
   //     5 .  write
     WatchDog_Feed(); 
    SST25V_BufferWrite(inbuf, inaddress, 128 );  
    delay_ms(50);

	return  indexnum;
}


//	666=>>768 rec_size   
u16  vdr_creat_09h( u16 indexnum,u8 *p, u16 inLen) 
{
    u8  inbuf[669];     
	u8  FCS=0;
	u16 i=0;
	u32 inaddress=0;//  写入地址
   
    //  1.  Stuff  Head 
	inbuf[0]=MarkByte;           //
    //  2.   Index  get  address  
    if(indexnum>VDR_09_MAXindex)
    	{
             Vdr_Wr_Rd_Offset.V_09H_Write=0; 
			 indexnum=0;
    	}
	memcpy(inbuf+1,p,inLen);   // 666 bytes   
   //   3.   caculate  fcs
       for(i=0;i<667;i++)
	    	FCS^=inbuf[i];
	   
	   inbuf[667]=FCS;   
   //    4.  get  address 	
	inaddress=VDR_09H_START+indexnum*768;

	//  判断擦除相关区域
	if( ( inaddress&0x0FFF)==0 )
			{
			    WatchDog_Feed();
				SST25V_SectorErase_4KByte( inaddress );   
				delay_ms(150); 
			}
   //     5 .  write
    WatchDog_Feed(); 
    SST25V_BufferWrite(inbuf, inaddress, 668 ); // 2+content
    delay_ms(80); 
   
     return  indexnum; 

} 

/*
   事故疑点 Accident Point    234
 */
 
u16  vdr_creat_10h( u16 indexnum,u8 *p, u16 inLen) 
{
   
      u8  inbuf[256];
	  u8  FCS=0,i=0;
	  u32 inaddress=0;//  写入地址
	 
	  //  1.  Stuff  Head 
	  inbuf[0]=MarkByte	;	   //
	  //  2.   Index  get  address	
	  if(indexnum>VDR_10_MAXindex)
		  {
			   Vdr_Wr_Rd_Offset.V_10H_Write=0; 
			   indexnum=0;
		  }
	  memcpy(inbuf+1,p,inLen);	 // 666 bytes
	 //   3.   caculate  fcs
	   for(i=0;i<235;i++)
		  FCS^=inbuf[i];
	   inbuf[235]=FCS;	 
	 //    4.  get	address   
	  inaddress=VDR_10H_START+indexnum*256;
   
	  //  判断擦除相关区域
	  if( ( inaddress & 0x0FFF ) == 0 )
			  {
				  WatchDog_Feed();
				  SST25V_SectorErase_4KByte( inaddress );
				  delay_ms(150);
			  }
	 // 	5 .  write
	  WatchDog_Feed(); 
	  SST25V_BufferWrite(inbuf, inaddress, 236 ); // 2+content 
	  delay_ms(10);
   
	   return  indexnum;

}

/***********************************************************
* Function:
* Description:       recordsize 5==>> 64 size 
* Input:             100   条
* Input:
* Output:
* Return:
* Others:
***********************************************************/

u16  vdr_creat_11h( u16 indexnum,u8 *p, u16 inLen) 
{

   
	u8	inbuf[60];
	u8	FCS=0,i=0;
	u32 inaddress=0;//	写入地址
   
	//	1.	Stuff  Head 
	inbuf[0]=MarkByte;		 //
	//	2.	 Index	get  address  
	if(indexnum>VDR_11_MAXindex)
		{
			 Vdr_Wr_Rd_Offset.V_11H_Write=0; 
			 indexnum=0;
		}
	memcpy(inbuf+1,p,inLen);   // 666 bytes
   //	3.	 caculate  fcs
	 for(i=0;i<51;i++)
		FCS^=inbuf[i];
	 inbuf[51]=FCS;   
   //	 4.  get  address	
	inaddress=VDR_11H_START+indexnum*64;
   
   //	  5 .  write   
	SST25V_OneSector_Write(inbuf, inaddress, 52 ); // 2+content     读擦写  
     WatchDog_Feed(); 
	 return  indexnum;

}
/*
   外部供电记录
   都存在4k的记录里，1次读出

      25 =>>   32
 */

u16  vdr_creat_12h( u16 indexnum,u8 *p, u16 inLen) 
{
  
   u8  inbuf[30];
   u8  FCS=0,i=0;
   u32 inaddress=0;//  写入地址
  
   //  1.  Stuff  Head 
   inbuf[0]=MarkByte;		//
   //  2.	Index  get	address  
   if(indexnum>VDR_12_MAXindex)
	   {
			Vdr_Wr_Rd_Offset.V_12H_Write=0; 
			indexnum=0;
	   }
   memcpy(inbuf+1,p,inLen);   // 666 bytes
  //   3.	caculate  fcs
	for(i=0;i<26;i++)
	   FCS^=inbuf[i];
	inbuf[26]=FCS;	 
  //	4.	get  address   
   inaddress=VDR_12H_START+indexnum*32; 
  
  //	 5 .  write   
   SST25V_OneSector_Write(inbuf, inaddress, 27 ); // 2+content	   读擦写   
    WatchDog_Feed(); 
   return	indexnum;

}


/*都存在4k的记录里，1次读出
     7 bytes    >> 8       record size     

*/
u16  vdr_creat_13h( u16 indexnum,u8 *p, u16 inLen) 
{
   u8  inbuf[10];
  // u8  FCS=0,i=0;
   u32 inaddress=0;//  写入地址
  
   //  1.  Stuff  Head 
   inbuf[0]=MarkByte;		//
   //  2.	Index  get	address  
   if(indexnum>VDR_13_MAXindex)
	   {
			Vdr_Wr_Rd_Offset.V_13H_Write=0; 
			indexnum=0;
	   }
   memcpy(inbuf+1,p,inLen);   // 666 bytes
/*  //   3.	caculate  fcs
	for(i=0;i<8;i++)
	   FCS^=inbuf[i];
	inbuf[8]=FCS;	 */
  //	4.	get  address   
   inaddress=VDR_13H_START+indexnum*8; 
  
  //	 5 .  write   
   SST25V_OneSector_Write(inbuf, inaddress, 8 ); // 2+content	   读擦写   
   WatchDog_Feed();   
   return	indexnum;
    

}

/*都存在4k的记录里，1次读出
     7 bytes    >> 8 	  record size 

*/
u16  vdr_creat_14h( u16 indexnum,u8 *p, u16 inLen) 
{
   u8  inbuf[10];
   //u8  FCS=0,i=0;
   u32 inaddress=0;//  写入地址
  
   //  1.  Stuff  Head 
   inbuf[0]=MarkByte;		//
   //  2.	Index  get	address  
   if(indexnum>VDR_14_MAXindex)
	   {
			Vdr_Wr_Rd_Offset.V_14H_Write=0; 
			indexnum=0;
	   }
   memcpy(inbuf+1,p,inLen);   // 666 bytes
/*  //   3.	caculate  fcs
	for(i=0;i<8;i++)
	   FCS^=inbuf[i];
	inbuf[8]=FCS;	 */
  //	4.	get  address   
   inaddress=VDR_14H_START+indexnum*8; 
  
  //	 5 .  write   
   SST25V_OneSector_Write(inbuf, inaddress, 8 ); // 2+content	   读擦写   
   WatchDog_Feed(); 
   return	indexnum;
  
}

/*都存在4k的记录里，1次读出
       133   >>  256 
*/
u16  vdr_creat_15h( u16 indexnum,u8 *p, u16 inLen) 
{
	
	 u8  inbuf[260];
	 u8  FCS=0,i=0;
	 u32 inaddress=0;//  写入地址
	
	 //  1.  Stuff	Head 
	 inbuf[0]=MarkByte; 	  //
	 //  2.   Index  get  address  
	 if(indexnum>VDR_15_MAXindex)
		 {
			  Vdr_Wr_Rd_Offset.V_15H_Write=0; 
			  indexnum=0;
		 }
	 memcpy(inbuf+1,p,inLen);	// 666 bytes
	//	 3.   caculate	fcs
	  for(i=0;i<134;i++)
		 FCS^=inbuf[i];
	  inbuf[134]=FCS;   
	//	  4.  get  address	 
	 inaddress=VDR_15H_START+indexnum*256; 
	
	//	   5 .	write	
	 SST25V_OneSector_Write(inbuf, inaddress, 135); // 2+content	 读擦写   
	 WatchDog_Feed(); 
	 return   indexnum;

}

//---------------------------------------------------------------------------------

/************************************** The End Of File **************************************/
