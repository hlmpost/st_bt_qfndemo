//�����51822 ����ͨѶЭ��
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "usart.h"
#include "comm.h"
#include "eric_flash.h"


extern uint8_t buffer[];


extern stru_region current_sensor_data;

extern volatile uint8_t current_mode;//normal,sport,sleep
extern volatile uint8_t batt_status;//current battery percent


//extern stru_para sys_para;

//��¼�汾��Ϣ
static uint8_t * serian_no=(uint8_t *)(0x1FFF7A10);//Ӳ�����к�

//1.0
static uint8_t main_ver=1;//���汾��
static uint8_t sec_ver=0;//���汾��

//��ǰģʽ
uint8_t curr_mode=0x01;//1-normal,2-sport,3-sleep

//�����ϴ���־
uint8_t bat_upload=0;

//------------------------------------------------------
//����У���
uint8_t check_sum(uint8_t* data,uint8_t len)
{
	uint8_t temp=0;
	for(int i=0;i<len;i++)
		temp+=data[i];
	return temp;
}

//��������
//================================================================
//����ȷ������
//fe 05 01 01 ab b0 ��fe 05 01 01 ae b3
//flag=1 ���ͳɹ�ȷ�ϣ�0-����ʧ��ȷ��
void send_shakehand(uint8_t flag)
{
	buffer[0]=0xfe;
	buffer[1]=0x05;
	buffer[2]=0x01;//command
	buffer[3]=0x01;
	if(flag==1)
	{
		buffer[4]=0xab;
		buffer[5]=0xb0;
	}
	else
	{
		buffer[4]=0xae;
		buffer[5]=0xb3;
	}
	uart2_send(buffer,6);
}
//================================================================
//���ذ汾��Ϣ��
void send_version_info()
{
	uint8_t i=0;
	buffer[0]=0xfe;
	buffer[1]=0x05;
	buffer[2]=0x05;//command
	buffer[3]=0x0e;
	for(;i<12;i++)
		buffer[4+i]=serian_no[i];
	buffer[4+i]=main_ver;
	buffer[5+i]=sec_ver;
	buffer[6+i]=check_sum(buffer,6+i);
	uart2_send(buffer,7+i);

}
//---------------------------------------------------
//ģʽ�л��ϱ�
void send_mode()
{
	buffer[0]=0xfe;
	buffer[1]=0x05;
	buffer[2]=0x08;//command
	buffer[3]=0x0a;
	buffer[4]=curr_mode;
	buffer[5]=check_sum(buffer,5);
	uart2_send(buffer,6);
}
//---------------------------------------------------
//���䴫��������
void send_sensor_data()
{
	uint8_t *temp=NULL;
	uint8_t i=0;
	buffer[0]=0xfe;
	buffer[1]=0x05;
	buffer[2]=0xaa;//command
	buffer[3]=0x05;
	current_sensor_data.step_count++;
	if(current_mode==3)
		temp=(uint8_t *)&(current_sensor_data.sleep_status);
	else
		temp=(uint8_t *)&(current_sensor_data.step_count);
	memcpy(&buffer[4],temp,3);
	buffer[7]=current_sensor_data.hrs_rate;
	buffer[8]=batt_status;
	buffer[9]=check_sum(buffer,9);
	uart2_send(buffer,10);

}
//-----------------------------------------------------------
//�ϴ���ɣ��ϱ�
void send_finish()
{
	buffer[0]=0xfe;
	buffer[1]=0x05;
	buffer[2]=0x08;//command
	buffer[3]=0x0a;
	buffer[4]=0xab;
	buffer[5]=0xb7;
	uart2_send(buffer,6);
	bat_upload=0;

}


//===============================================================
//--------------------------------------
//������������
uint16_t get_len(uint8_t *data)
{
	if(data[2]==0xaa && (data[5]==check_sum(data,5)) )
	{
		return data[4];
	}
	else
		return 0xff;
}
//-----------------------------------------------------
//������յ�������
//return:1-�յ�������ȷ��0-�յ��������
void rece_dispatch(uint8_t *data)
{
	uint8_t len=data[0];
	uint8_t error=0;;
	if(data[1]!=0xfe || data[2]!=0x04 || (data[len]!=check_sum(&data[1],len-1)) )
	{
		//�յ���������ظ�
		send_shakehand(0);
	}
	//��������
	switch(data[3])
	{
		case 0x01://ȷ������
			break;
		case 0x02://��ʼ��flash�洢��
			//flash_init();
			break;
		case 0x03://�յ�ʱ����������
			//RTC_Set_datetime(&data[5]);
			//RTC_AlarmConfig(data[8],data[9]+1);
			//SEGGER_RTT_printf(0,"set_time=%02d-%02d-%02d;%02d-%02d-%02d;\r\n",data[5],data[6],data[7],data[8],data[9],data[10]);
			break;
		case 0x04://step length
			//sys_para.step_len=data[5];
			//flash_erase_para_sector();
			break;
		case 0x05://return serial number
			send_version_info();		
			break;
		case 0x0a://ģʽ�л�
			if(data[5]>0x03)
				error=1;
			else
				curr_mode=data[5];
			break;
		case 0x06://������������
			bat_upload=1;
			break;
		case 0x09://�ж���������
			bat_upload=0;
			break;
		default:
			send_shakehand(0);//�յ��쳣����S
			return;
	};
	if(data[3]!=0x01)
	{
		//�ظ�ȷ���յ�
		if(error==0)
			send_shakehand(1);
		else
			send_shakehand(0);
	}
}