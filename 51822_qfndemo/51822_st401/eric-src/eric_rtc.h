#ifndef __ERIC_RTC_H
#define __ERIC_RTC_H

#include <stdint.h>
#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF


void eric_rtc_init();
void RTC_CalendarShow(uint8_t* showtime, uint8_t* showdate);
void RTC_Read_datetime(uint8_t * data,uint8_t flag);
//void RTC_Set_datetime(uint8_t * data,uint8_t flag);
void RTC_Set_datetime(uint8_t * data);
#endif