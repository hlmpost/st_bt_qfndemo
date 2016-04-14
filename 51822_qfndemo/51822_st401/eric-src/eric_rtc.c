#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal_rtc.h"
#include "eric_rtc.h"


RTC_HandleTypeDef RtcHandle;
//extern RTC_HandleTypeDef hrtc;

void RTC_AlarmConfig(uint8_t hour,uint8_t min);
uint8_t alarm_flag=0;

//rtc alarm 同步
osSemaphoreId osSemaphore;

osMutexId  rtc_mutex;
extern osMessageQId myQueue01Handle;

#if 0
//---------------------------------------------------------
uint8_t RTC_get_state()
{
	return HAL_RTC_GetState(&RtcHandle);

}

//-----------------------------------------------------------
//flag=1-时间，2-日期
void RTC_Read_datetime(uint8_t * data,uint8_t flag)
{
	RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
	HAL_RTCStateTypeDef status;

osMutexWait(rtc_mutex, osWaitForever);
	if(data!=NULL)
	{	
		if(flag==1)
		{
			/* Get the RTC current Time */
			status=HAL_RTC_GetState(&RtcHandle);
			HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);
			data[0]=stimestructureget.Hours;
			data[1]=stimestructureget.Minutes;
			data[2]=stimestructureget.Seconds;
		}
		else if(flag==2)
		{
			/* Get the RTC current Date */
			HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
			data[0]=sdatestructureget.Year;
			data[1]=sdatestructureget.Month;
			data[2]=sdatestructureget.Date;
		}
	}
	osMutexRelease(rtc_mutex);
}
//-----------------------------------------------------------
//传入BCD码
void RTC_Set_datetime(uint8_t * data)
{
	RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;
	if(data!=NULL)
	{
		
			sdatestructure.Year =data[0];
			sdatestructure.Month = data[1];
			sdatestructure.Date = data[2];

			if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BIN) != HAL_OK)
			{
				/* Initialization Error */
				Error_Handler("set time error"); 
			} 
			stimestructure.Hours = data[3];
			stimestructure.Minutes = data[4];
			stimestructure.Seconds = data[5];
			stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
			stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
			stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
			if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BIN) != HAL_OK)
				{
					/* Initialization Error */
					Error_Handler("set date error"); 
				}		

	}//data=null
}
//void RTC_Set_datetime(uint8_t * data,uint8_t flag)
//{
//	RTC_DateTypeDef sdatestructure;
//  RTC_TimeTypeDef stimestructure;
//	if(data!=NULL)
//	{
//		if(flag==1)
//		{
//			SEGGER_RTT_printf(0,"RTC_Set_datetime:time=%02d-%02d-%02d;\r\n",data[0],data[1],data[2]);
//			 /* Set Time: 02:00:00 */
//			stimestructure.Hours = data[0];
//			stimestructure.Minutes = data[1];
//			stimestructure.Seconds = data[2];
//			stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
//			stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//			stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
//			if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BIN) != HAL_OK)
//				{
//					/* Initialization Error */
//					Error_Handler("set date error"); 
//				}		
//		}
//		else if(flag==2)
//		{
//			SEGGER_RTT_printf(0,"RTC_Set_datetime:date=%02d-%02d-%02d;\r\n",data[0],data[1],data[2]);
//			sdatestructure.Year =data[0];
//			sdatestructure.Month = data[1];
//			sdatestructure.Date = data[2];

//			if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BIN) != HAL_OK)
//			{
//				/* Initialization Error */
//				Error_Handler("set time error"); 
//			} 
//		}
//	}//data=null

//}
//---------------------------------------------------------
//设置alarm
 void RTC_AlarmConfig(uint8_t hour,uint8_t min)
{
  static RTC_AlarmTypeDef salarmstructure;

 
  /*##-1- Configure the RTC Alarm peripheral #################################*/
  /* Set Alarm to 02:20:30 
     RTC Alarm Generation: Alarm on Hours, Minutes and Seconds */
  salarmstructure.Alarm = RTC_ALARM_A;
  salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
  salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
  salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
  salarmstructure.AlarmTime.Hours = hour;
  salarmstructure.AlarmTime.Minutes = min;
	//if(flag==1)
		salarmstructure.AlarmTime.Seconds = 1;
	//salarmstructure.AlarmTime.Seconds+=10;
	//else if(flag==2)
	//	salarmstructure.AlarmTime.Seconds = 10;
	//else
	//	salarmstructure.AlarmTime.Seconds+=5;
  //salarmstructure.AlarmTime.SubSeconds = 0x56;
	
	//7SEGGER_RTT_printf(0,"RTC_AlarmConfig:%02d-%02d;\r\n",hour,min);

  
  if(HAL_RTC_SetAlarm_IT(&RtcHandle,&salarmstructure,RTC_FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler("RTC_AlarmConfig:error"); 
  }  
}

//---------------------------------------------------
static void RTC_CalendarConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /*##-1- Configure the Date #################################################*/
  /* Set Date: Tuesday February 18th 2014 */
  sdatestructure.Year = 0x16;
  sdatestructure.Month = RTC_MONTH_FEBRUARY;
  sdatestructure.Date = 0x04;
  sdatestructure.WeekDay = RTC_WEEKDAY_TUESDAY;
  
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler("set date error-config"); 
  } 
  
  /*##-2- Configure the Time #################################################*/
  /* Set Time: 02:00:00 */
  stimestructure.Hours = 0x05;
  stimestructure.Minutes = 0x33;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler("set time error-config"); 
  }
  
  /*##-3- Writes a data in a RTC Backup data Register0 #######################*/
  HAL_RTCEx_BKUPWrite(&RtcHandle,RTC_BKP_DR0,0x32F4);  
}
//--------------------------------------------------------
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  		SEGGER_RTT_printf(0,"rtc alarm\r\n");
			osSemaphoreRelease(osSemaphore);
			//osMessagePut(myQueue01Handle, 1066, 0);
}
#endif
//------------------------------------------
void eric_rtc_init()
{
	uint16_t temp=0;
	//rtc
	RtcHandle.Instance = RTC;
	RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  
  if(HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler("rtc init fail");
  }
	
	//HAL_RTCEx_BKUPWrite(&RtcHandle,RTC_BKP_DR0,0x32F4);  
//	temp=HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR0);
//	if(temp != 0x32F4)
//  {  
//    /* Configure RTC Calendar */
//    RTC_CalendarConfig();
//  }
//  else
//  {
//    /* Clear Reset Flag */
//    __HAL_RCC_CLEAR_RESET_FLAGS();
//  }
	
	//初始化时间
	{
		//uint8_t time[3]={0x10,0x00,0x10};
		//uint8_t data[6]={0x00,0x03,0x09,0x10,0x00,0x10};
		//RTC_Set_datetime(date,2);
		//RTC_Set_datetime(time,1);
		//SEGGER_RTT_printf(0,"RTC_Set_datetime:date=%02d-%02d-%02d;time=%02d-%02d-%02d;\r\n",data[0],data[1],data[2],data[3],data[4],data[5]);

		//RTC_Set_datetime(data);

	}
	//初始化binary
	osSemaphoreDef(SEM);
	osSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 1);
	//mutex
	osMutexDef(rtc_mutex); 
	rtc_mutex = osMutexCreate(osMutex(rtc_mutex));

}
