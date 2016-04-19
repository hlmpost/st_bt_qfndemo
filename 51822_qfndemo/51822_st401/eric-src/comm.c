//处理和51822 数据通讯协议
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

//记录版本信息
static uint8_t * serian_no=(uint8_t *)(0x1FFF7A10);//硬件序列号

//1.0
static uint8_t main_ver=1;//主版本号
static uint8_t sec_ver=0;//副版本号

//当前模式
uint8_t curr_mode=0x01;//1-normal,2-sport,3-sleep

//批量上传标志
uint8_t bat_upload=0;

//------------------------------------------------------
//计算校验和
uint8_t check_sum(uint8_t* data,uint8_t len)
{
	uint8_t temp=0;
	for(int i=0;i<len;i++)
		temp+=data[i];
	return temp;
}

//发送命令
//================================================================
//发送确认命令
//fe 05 01 01 ab b0 ；fe 05 01 01 ae b3
//flag=1 发送成功确认，0-发送失败确认
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
//返回版本信息，
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
//模式切换上报
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
//传输传感器数据
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
//上传完成，上报
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
//解析长度命令
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
//处理接收到的数据
//return:1-收到命令正确；0-收到命令错误
void rece_dispatch(uint8_t *data)
{
	uint8_t len=data[0];
	uint8_t error=0;;
	if(data[1]!=0xfe || data[2]!=0x04 || (data[len]!=check_sum(&data[1],len-1)) )
	{
		//收到错误命令回复
		send_shakehand(0);
	}
	//解析命令
	switch(data[3])
	{
		case 0x01://确认命令
			break;
		case 0x02://初始化flash存储区
			//flash_init();
			break;
		case 0x03://收到时间设置命令
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
		case 0x0a://模式切换
			if(data[5]>0x03)
				error=1;
			else
				curr_mode=data[5];
			break;
		case 0x06://启动批量传输
			bat_upload=1;
			break;
		case 0x09://中断批量传输
			bat_upload=0;
			break;
		default:
			send_shakehand(0);//收到异常命令S
			return;
	};
	if(data[3]!=0x01)
	{
		//回复确认收到
		if(error==0)
			send_shakehand(1);
		else
			send_shakehand(0);
	}
}