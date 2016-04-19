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

#include "eric_flash.h"

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

volatile uint8_t init_finish=0;


extern volatile uint8_t current_datetime[];//current datetime


volatile uint8_t sensor_flag=0;
volatile uint8_t current_mode=1;//normal,sport,sleep
volatile uint8_t batt_status=50;//current battery percent
volatile uint8_t disp_sort=0;//display lcd sortno,0-time ,1-step,2-hrs,3-consume
volatile uint8_t step_len=50;//step length

volatile uint8_t disp_flag=0;//displaying lcd,don't do other


stru_region current_sensor_data;
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
	else if(message==4)//display
		osSignalSet(Lcd_disp_TaskHandle, message);


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
	//as7000_init();

  //²âÊÔlis2ds
  if(lis2ds12_init()==0)
		Error_Handler("g-sensor is error!");
	else
		Info_Handler("g-sensor is ok!");		
	
	//Loop_Test_Tap();
	Loop_Test_Pedometer();
	
	if(iqs263_init()==0)
		Error_Handler("tp is error!");
	else
		Info_Handler("tp is ok!");
	
	
	//flash init
	flash_init();
	//lcd init
	lcd_init();
	//display default lcd

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
	char data[10];
	uint8_t count=0;//10 times,gc4 update

	//display lcd default datetime
	send_message(4);
	while(1)
	{
		osSignalWait(0x4, osWaitForever);
		disp_flag=1;
		#if 1
		read_batt();
		lcd_display_on(1);
		lcd_disp_clear();
		if(disp_sort==0)
		{
			//lcd_disp_bmp(1,80);
			//lcd_disp_font(1,170,1,29,29);
			//lcd_disp_number(10,200,data,33,12);
			sprintf(data,"%02d%02d",current_datetime[3],current_datetime[4]);
			lcd_disp_time(1,40,data,80,40);
			sprintf(data,"%02d%02d%d",current_datetime[1],current_datetime[2],current_datetime[6]);
			lcd_disp_date(1,205,data,29,15);
		}
		else if(disp_sort==1)//²½Êý
		{
			lcd_disp_bmp(1,40,1);
			lcd_disp_font(1,130,1,2,29,29);
			sprintf(data,"%d",current_sensor_data.step_count);
			lcd_disp_number(10,160,data,29,12);
			lcd_disp_font(1,190,2,2,29,29);
			sprintf(data,"%d",(current_sensor_data.step_count*step_len)/100);
			lcd_disp_number(10,220,data,29,12);
			lcd_disp_font(1,250,3,1,24,24);
			
		}
		else if(disp_sort==2)//consume
		{
			lcd_disp_bmp(1,40,2);
			lcd_disp_font(1,140,4,3,29,29);
			sprintf(data,"%d",current_sensor_data.step_count);
			lcd_disp_number(10,170,data,29,12);
			
		}
		else if(disp_sort==3)//hrs rate
		{
			lcd_disp_bmp(1,40,3);
			lcd_disp_font(1,140,5,2,29,29);
			sprintf(data,"%d",current_sensor_data.step_count);
			lcd_disp_number(10,170,data,29,12);
			
		}
		lcd_disp_battery(batt_status);
		count++;
		if(count>9)
		{	
			count=0;
			lcd_display_update(1);
		}
		else
			lcd_display_update(2);

		lcd_display_on(0);
	  #endif
		disp_flag=0;
	}
}
//-------------------------------------------------------------------
//touch thread
void func_touchTask(void const * argument)
{
	uint8_t temp=0,old_disp_sort=0;
	while(1)
	{
		osSignalWait(0x1, osWaitForever);
		temp=handleEvents();
		if(disp_flag==1)
			continue;
		old_disp_sort=disp_sort;
		if(temp==2)//left
		{
			if(disp_sort>0)
				disp_sort--;
		}
		else if(temp==3)//right
		{
			if(disp_sort<3)
				disp_sort++;
		}
		if(disp_sort!=old_disp_sort)
			send_message(4);

	}

}
//------------------------------------------------------------------
//sensor thread
void func_sensorTask(void const * argument)
{
	uint8_t curr_time[3],hour,min;
	uint8_t flag=4;

	while(1)
	{
		osSignalWait(0x2, osWaitForever);
		sensor_flag=1;
		//osDelay(2000);
		//step
		LIS2DS12_ACC_GYRO_Pedo_Callback();
		//hrs
		
		current_sensor_data.hrs_rate=50;

		if(current_mode==1)//normal
			current_sensor_data.sleep_status=0xffffffff;
		else if(current_mode==3)//sleep
			current_sensor_data.step_count=0xffff;
		 
		current_sensor_data.bld_press=0xff;//now we have not blood pressure

		flash_write_movedata(&current_sensor_data);
		
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
//				osDelay(2000);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
//				led_flash(flag);
//				flag--;
//		    if(flag<1)
//					flag=4;
		//sprintf(data,"%d",current_sensor_data.step_count);
		if(disp_sort==0 || disp_sort==1 || disp_sort==2 || disp_sort==3)
			if(disp_flag==0)
				send_message(4);
		sensor_flag=0;
		

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
