//ÐÄÂÊ¶ÁÈ¡
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"

#include "power.h"


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  /* Get the converted value of regular channel */
  uint32_t temp;
  temp= HAL_ADC_GetValue(AdcHandle);
  temp=0;
}


//------------------------------------------------------
static void read_batt()
{
	HAL_ADC_Start_IT(&hadc1);
	

}
//------------------------------------------------------
unsigned char power_init()
{
	read_batt();

}
//------------------------------------------
