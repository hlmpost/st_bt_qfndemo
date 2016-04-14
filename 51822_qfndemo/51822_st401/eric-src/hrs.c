//心率读取
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "gpio.h"

#include "hrs.h"

//-------------------------------------------------------
#define AS7000_ADDRESS       (0x60)
static unsigned char data_buffer[30];

//------------------------------------------------------
//as7000 power control,pb12
static void as7000_power(uint8_t flag)
{
	if(flag==1)//on
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	else//off
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}
}


//---------------------------------------------------
static void as7000_WriteBytes(unsigned char address,unsigned char data)
{
   uint8_t temp[2]={address,data}; 
  while(HAL_I2C_Master_Transmit(&hi2c2, AS7000_ADDRESS,temp , 2, 1000)!= HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
    {
      Error_Handler("as7000 write error!");
			return;
    }
  }
  return;
}
//---------------------------------------------
static void as7000_ReadBytes(unsigned char address,unsigned char* data)
{
	while(HAL_I2C_Master_Transmit(&hi2c2, AS7000_ADDRESS,&address , 1, 1000)!= HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
    {
      Error_Handler("as7000 write error!");
			return;
    }
  }
//-----------------------------------------------------------------------
	//读取寄存器内容
	while(HAL_I2C_Master_Receive(&hi2c2, AS7000_ADDRESS, data, 1, 1000) != HAL_OK)
  {
    /* Error_Handler() function is called when Timeout error occurs.
       When Acknowledge failure occurs (Slave don't acknowledge it's address)
       Master restarts communication */
    if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
    {
      Error_Handler("g-sensor read error!");
			return;
    }   
  }
	return;     

}

//--------------------------------------------
unsigned char as7000_init()
{
as7000_power(0);
	return 0;
as7000_ReadBytes(0x00,&data_buffer[0]);
as7000_ReadBytes(0x01,&data_buffer[1]);
as7000_ReadBytes(0x02,&data_buffer[2]);
as7000_ReadBytes(0x03,&data_buffer[3]);
as7000_ReadBytes(0x04,&data_buffer[4]);
as7000_ReadBytes(0x05,&data_buffer[5]);
if(data_buffer[0]==1 && data_buffer[0]==3)
		return 1;
else
		return 0;

}
//------------------------------------------------
//读心率
unsigned char as7000_hrs_rate()
{
as7000_ReadBytes(0x08,&data_buffer[0]);
as7000_ReadBytes(0x09,&data_buffer[1]);
as7000_ReadBytes(0x0a,&data_buffer[2]);
as7000_ReadBytes(0x0b,&data_buffer[3]);
as7000_ReadBytes(0x0c,&data_buffer[4]);
as7000_ReadBytes(0x0d,&data_buffer[5]);
as7000_ReadBytes(0x0e,&data_buffer[6]);
return data_buffer[2];

}
