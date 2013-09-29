#ifndef __RTC_H__
#define __RTC_H__

//#include <time.h>
#include "stm32f4xx_rtc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include <time.h>


#define   RTC_First        0xA5A5


typedef struct{
	uint8_t year;    // modify by nathan   Orginal: uint16_t  20xx
	uint8_t month;
	uint8_t day;
	uint8_t week;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;	
}TDateTime;

extern TDateTime time_now;
extern RTC_TimeTypeDef RTC_TimeStructure;
extern RTC_DateTypeDef RTC_DateStructure;  


void RTC_TimeShow(void);
u8  rt_hw_rtc_init(void);
void set_time(rt_uint32_t hour, rt_uint32_t minute, rt_uint32_t second);
void set_date(rt_uint32_t year, rt_uint32_t month, rt_uint32_t date);
extern TDateTime  Get_RTC(void);
extern u8    Set_RTC( TDateTime now);
extern void  RTC_Demo_init(void);
extern u8    Device_RTC_set(TDateTime now);
extern int   RTC_Config(void); 
extern u8 RT_Total_Config(void);


/*time_t time(time_t* t);
 */
#endif
