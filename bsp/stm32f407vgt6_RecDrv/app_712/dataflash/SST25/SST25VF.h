
//#include <includes.h>
#ifndef __SST25V_H
#define __SST25V_H
#include <stm32f4xx.h>

//  1.   Total  Define 
#define   DF_SPI           SPI2
//------------------------------------------------------------------
#define Read_Data                     0x03       //读取存储器数据
#define HighSpeedReadData             0x0B       //快速读取存储器数据
#define SectorErace_4KB               0x20       //扇区擦除
#define BlockErace_32KB               0x52       //32KB块擦除
#define BlockErace_64KB               0xD8       //64KB块擦除
#define ChipErace                     0xC7       //片擦除
 
#define Byte_Program                  0x02       //页面编程--写数据  
#define AAI_WordProgram               0xAD
#define ReadStatusRegister            0x05       //读状态寄存器
#define EnableWriteStatusRegister     0x50
#define WriteStatusRegister           0x01       //写状态寄存器

#define WriteEnable                   0x06       //写使能，设置状态寄存器
#define WriteDisable                  0x04       //写禁止
#define SST25_WRDI		              0x04       //  退出AAI 模式
#define ReadDeviceID                  0xAB       //获取设备ID信息

#define ReadJedec_ID                  0x9F       //JEDEC的ID信息

#define EBSY                          0X70
#define DBSY                          0X80

#define Dummy_Byte                    0xA5 //0xFF

//  3.   PIN   
#define SST25V_CS_LOW()      GPIO_ResetBits(GPIOD,GPIO_Pin_14)  
#define SST25V_CS_HIGH()     GPIO_SetBits(GPIOD,GPIO_Pin_14)

#define SST25V_WP_HIGH()   GPIO_SetBits(GPIOD,GPIO_Pin_15)
#define SST25V_WP_LOW()    GPIO_SetBits(GPIOD,GPIO_Pin_15) 







extern void SST25V_Init(void);
extern u8   SST25V_ByteRead(u32 ReadAddr);
extern void SST25V_strWrite(u8 *p, u32 WriteAddr,u16 length);    
extern u8  SST25V_OneSector_Write(u8 *p,  u32  addr,  u32 len);

void SST25V_BufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead);

//extern u8 SPI_Flash_SendByte(u8 byte);
//extern u8 SPI_Flash_ReceiveByte(void);
extern void SST25V_ByteWrite(u8 Byte, u32 WriteAddr);
extern void SST25V_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToRead); 

extern void SST25V_SectorErase_4KByte(u32 Addr);
void SST25V_BlockErase_32KByte(u32 Addr);
void SST25V_BlockErase_64KByte(u32 Addr);

u8 SST25V_ReadStatusRegister(void);
void SST25V_WriteEnable(void);
void SST25V_WriteDisable(void);

void SST25V_EnableWriteStatusRegister(void);
void SST25V_WriteStatusRegister(u8 Byte);
void SST25V_WaitForWriteEnd(void);


u16 SST25V_ReadManuID_DeviceID(u32 ReadManu_DeviceID_Addr);

extern u8 SPI_Flash_SendByte(u8 data);
extern u8 SPI_Flash_ReceiveByte(void);   



#endif 

