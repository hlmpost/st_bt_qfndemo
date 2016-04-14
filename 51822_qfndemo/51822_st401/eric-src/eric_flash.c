#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "eric_flash.h"
#define FLASH_DATA1_SECTOR_ADDR   ADDR_FLASH_SECTOR_5   /* Start @ of user Flash area */
#define FLASH_DATA2_SECTOR_ADDR   ADDR_FLASH_SECTOR_6   /* End @ of user Flash area */
#define FLASH_PARA_SECTOR_ADDR   ADDR_FLASH_SECTOR_1   /* End @ of user Flash area */

//----------------------------------------------------
#define DAY_DATA_LENGTH  (11520)//0x2d00,ÿ�����ݰ�8���ֽ�

//--------------------------------------------------
static uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0;
static FLASH_EraseInitTypeDef EraseInitStruct;
static uint32_t SectorError = 0;

//���ݴ洢������
//static stru_header data_header;//��ʱ��7��洢
stru_region data_region;

//��¼ϵͳ�����粽��
stru_para sys_para;


//����14����Ϣͷ���飬ÿ��flash����7,��������������
stru_header data_header[14];
static uint8_t curr_index=0;//��ǰ����д����Ϣͷ����

extern uint8_t alarm_flag;

//-------------------------------------------------------------------
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;  
  }
  else if(Address >= ADDR_FLASH_SECTOR_7)
  {
    sector = FLASH_SECTOR_7;  
  }

  return sector;
}

//------------------------------------------------------------
void flash_read(uint8_t * Address)
{
	for(int i=0;i<4;i++)
	{
		SEGGER_RTT_printf(0,"flash_read:0x%x=0x%x\r\n",(Address+i),*(Address+i));
	}
	SEGGER_RTT_printf(0,"flash_read:index=%d\r\n",curr_index);
}
//----------------------------------------------------------
//���������ݼ�¼
//sector_no=1-����1��2-����2,curr_date=д�뵱ǰ����
static void flash_erase_sector_init(uint8_t sector_no,uint32_t curr_date)
{
	HAL_FLASH_Unlock();
	  if(sector_no==1)
			EraseInitStruct.Sector =  GetSector(FLASH_DATA1_SECTOR_ADDR);
	  else if(sector_no==2)
			EraseInitStruct.Sector =  GetSector(FLASH_DATA2_SECTOR_ADDR);
		else
		{	
			SEGGER_RTT_printf(0,"flash_erase_sector:sector_no is error\r\n");
			return;
		}
	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		EraseInitStruct.NbSectors = 1;
		if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
		{ 
					SEGGER_RTT_printf(0,"erase is error\r\n");
		}
		
				//clear buffer
		__HAL_FLASH_DATA_CACHE_DISABLE();
		__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
		__HAL_FLASH_DATA_CACHE_RESET();
		__HAL_FLASH_INSTRUCTION_CACHE_RESET();
		__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
		__HAL_FLASH_DATA_CACHE_ENABLE();
		
		
		//д�벽��	
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)FLASH_DATA1_SECTOR_ADDR, sys_para.step_len);

		HAL_FLASH_Lock();

}
//----------------------------------------------------------
//ϵͳ������¼���粽���ȣ����ڣ�sector 1
//
void flash_erase_para_sector()
{
	HAL_FLASH_Unlock();
	EraseInitStruct.Sector =  GetSector(FLASH_PARA_SECTOR_ADDR);
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.NbSectors = 1;
		if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
		{ 
					SEGGER_RTT_printf(0,"erase is error\r\n");
		}
		
		//clear buffer
		__HAL_FLASH_DATA_CACHE_DISABLE();
		__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
		__HAL_FLASH_DATA_CACHE_RESET();
		__HAL_FLASH_INSTRUCTION_CACHE_RESET();
		__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
		__HAL_FLASH_DATA_CACHE_ENABLE();
		HAL_FLASH_Lock();

}


//------------------------------------------------------
//�����㷨�����Ҵ洢��ַ
//���ȶ�ȡ��ǰ���ڣ�ʱ�䣬ͨ�����ڲ������ĸ���Ϣͷ�У�ͨ��ʱ���ж�Ӧ�洢���ĸ���ַ�����û�еĻ�����д�뵱ǰ��һ������Ϣͷ��
void flash_write_movedata(uint16_t step_count,uint16_t hrs_rate)
{
	uint8_t i=0;
	uint32_t curr_date=0;
	uint8_t curr_time[3];
	uint16_t offset=0;//����ƫ��ֵ
	uint8_t* temp_add=0;
	uint8_t hour,min;
	
	if(RTC_get_state()==1)
	{
		RTC_Read_datetime(curr_time,1);
		RTC_Read_datetime((uint8_t *)(&curr_date),2);
	}
	else
		SEGGER_RTT_printf(0,"flash_write_movedata:rtc state is error\r\n");

	SEGGER_RTT_printf(0,"flash_write_movedata:current_date=%x;alarm=%x,%02d:%02d:%02d\r\n",data_header[curr_index].curr_date,curr_date,curr_time[0],curr_time[1],curr_time[2]);

	//�͵�ǰ��Ϣͷ�Ƚ�
	if(data_header[curr_index].curr_date!=curr_date)
	{
		if(curr_index==6)//�ߵ�data1�Ľ���
		{
			//earse data1
			flash_erase_sector_init(2,curr_date);
			curr_index=7;
		}
		else if(curr_index==13)//�Ѿ�data2�Ľ���
		{
			//earse data2
			flash_erase_sector_init(1,curr_date);
			curr_index=0;
		}
		else
			curr_index++;
		data_header[curr_index].curr_date=curr_date;
	}
	
	
	//����Ҫд�������ƫ�Ƶ�ַ
	offset=curr_time[0]*60+curr_time[1];
//	if((offset%2)>0)//��������
//	{
//		SEGGER_RTT_printf(0,"flash_write_movedata:no equal\r\n");
//	}
//	offset=offset/2;
	temp_add=(uint8_t *)(data_header[curr_index].start_add+offset*4);
	if( *temp_add!=0xff && *(temp_add+1)!=0xff)
	{
		SEGGER_RTT_printf(0,"flash_write_movedata:data have been writed\r\n");
		//return;
	}
	else
	{
		HAL_FLASH_Unlock();
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(uint32_t)(temp_add), step_count);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(uint32_t)(temp_add+2), hrs_rate);
		HAL_FLASH_Lock();
	}
	flash_read(temp_add);
	

	hour=curr_time[0];
	//min=(curr_time[1]/2);
	//min=min*2+2;
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
	SEGGER_RTT_printf(0,"flash_write_movedata:index=%d;address=%x\r\n",curr_index,data_header[curr_index].start_add);

}

//----------------------------------------------------------------------

void flash_init()
{
	
	/* Unlock the Flash to enable the flash control register access *************/ 
  HAL_FLASH_Unlock();
    
  /* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  /* Get the 1st sector to erase */

  FirstSector = GetSector(FLASH_DATA1_SECTOR_ADDR);
  /* Get the number of sector to erase from 1st sector*/
  NbOfSectors = GetSector(FLASH_DATA2_SECTOR_ADDR) - FirstSector + 1;
	/* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FirstSector;
  EraseInitStruct.NbSectors = NbOfSectors;
  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  { 
    		SEGGER_RTT_printf(0,"erase is error\r\n");
  }
	else
			SEGGER_RTT_printf(0,"erase is ok\r\n");
	 /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();

  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();

//��ʼ���Ʋ������ʵ�������,��ʼ����һ������
		//init header info array
		for(int i=0;i<14;i++)
		{	
			memset(data_header+i,0,sizeof(stru_header));//emtpy memory
			//init start address
			if(i<7)
				data_header[i].start_add=(uint8_t *)(FLASH_DATA1_SECTOR_ADDR+DAY_DATA_LENGTH*i);
			else
				data_header[i].start_add=(uint8_t *)(FLASH_DATA2_SECTOR_ADDR+DAY_DATA_LENGTH*(i-7));
		}
		
		RTC_Read_datetime((uint8_t *)(&(data_header[0].curr_date) ),2);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(data_header[0].start_add), data_header[0].curr_date);
		curr_index=0;
  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock(); 
}