/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
//for debug
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include "stm32f4xx_hal.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */
extern RTC_HandleTypeDef hrtc;

//thread
osThreadId Lcd_disp_TaskHandle;//display
osThreadId touch_TaskHandle;//touch
osThreadId sensor_TaskHandle;//touch
osThreadId uart_TaskHandle;

void func_DispTask(void const * argument);
void func_touchTask(void const * argument);
void func_sensorTask(void const * argument);
void func_uartTask(void const * argument);

uint8_t init_finish=0;
extern volatile uint8_t work_flag;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
//---------------------------------
//error print
void Error_Handler(char * err)
{
	SEGGER_RTT_printf(0,"error:%s\r\n",err);
}
//---------------------------------------
void Info_Handler(char * info)
{
	SEGGER_RTT_printf(0,"info:%s\r\n",info);
}
//------------------------------------------------
//send message func
void send_message(uint8_t message)
{
	if(message==1)//touch
		osSignalSet(touch_TaskHandle, message);
	else if(message==2)//sensor
		osSignalSet(sensor_TaskHandle, message);
	else if(message==3)//uart
		osSignalSet(uart_TaskHandle, message);

}
//---------------------------------------------------------

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */


  //²âÊÔ as7000
  //as7000_deinit();
  //²âÊÔlis2ds
  if(lis2ds12_init()==0)
		Error_Handler("g-sensor is error!");
	else
		Info_Handler("g-sensor is ok!");		
	
	//Loop_Test_Tap();
	Loop_Test_Pedometer();
	
//	if(iqs263_init()==0)
//		Error_Handler("tp is error!");
//	else
//		Info_Handler("tp is ok!");
	
	//as7000
	as7000_init();

	//display thread
	osThreadDef(disp_Task, func_DispTask, osPriorityNormal, 0, 128);
	Lcd_disp_TaskHandle = osThreadCreate(osThread(disp_Task), NULL);
	//touch thread
	osThreadDef(touch_tTask, func_touchTask, osPriorityNormal, 0, 128);
  touch_TaskHandle = osThreadCreate(osThread(touch_tTask), NULL);
	//sensor thread
	osThreadDef(sensor_tTask, func_sensorTask, osPriorityNormal, 0, 128);
  sensor_TaskHandle = osThreadCreate(osThread(sensor_tTask), NULL);
	//uart thread
	osThreadDef(uart_tTask, func_uartTask, osPriorityNormal, 0, 128);
  uart_TaskHandle = osThreadCreate(osThread(uart_tTask), NULL);

	init_finish=1;//init finish
	osThreadTerminate(defaultTaskHandle);
  /* Infinite loop */
  for(;;)
  {
		//power_init();
    	osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
//-------------------------------------------------------------------------
//lcd disp thread
void func_DispTask(void const * argument)
{
		uint8_t curr_time[3];

	//lcd_init();
	while(1)
	{
		osDelay(5000);
		//osSignalWait(0x1, osWaitForever);
		//lcd_test();
  

//osMutexWait(rtc_mutex, osWaitForever);
			/* Get the RTC current Time */



	}
}
//-------------------------------------------------------------------
//touch thread
void func_touchTask(void const * argument)
{
	while(1)
	{
		osSignalWait(0x1, osWaitForever);
		if(work_flag==0)
		{
			work_flag=1;
			handleEvents();
		}
	}

}
//------------------------------------------------------------------
//sensor thread
void func_sensorTask(void const * argument)
{
	uint8_t curr_time[3],hour,min;
	while(1)
	{
		osSignalWait(0x2, osWaitForever);
		//osDelay(2000);
		//step
		//LIS2DS12_ACC_GYRO_Pedo_Callback();
		//hrs
		
		read_rtc_status();
		RTC_Read_datetime(curr_time,1);
		SEGGER_RTT_printf(0,"TIMER:%d:%d:%d\r\n",curr_time[0],curr_time[1],curr_time[2]);			

		hour=curr_time[0];
		min=curr_time[1]+1;
		if(min>59)
		{	
			min-=60;
			if(hour==23)
				hour=0;
			else
				hour++;
		}
		RTC_AlarmConfig(hour,min);

		//send_message(3);
		

	}
}
//----------------------------------------------------------------------
//uart thread
void func_uartTask(void const * argument)
{
	uart2_read(1);
	while(1)
	{
		osSignalWait(0x3, osWaitForever);
		send_sensor_data();
		//send_shakehand(1);
	}
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
