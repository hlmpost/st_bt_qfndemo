#ifndef __ERIC_FLASH_H
#define __ERIC_FLASH_H
#include <stdint.h>
//放在ram中
//记录区的信息头结构体
typedef struct Header
{
	uint32_t curr_date;//当前日期
	uint8_t up_flag;//上传的标志；0-未上传，1-已上传
	uint32_t curr_step_data;//step:记录当天的当前总步数；
	uint8_t curr_hrs_data;//hrs：最新测量的心率数据
	uint8_t batt_status;//battery percent
	uint8_t *start_add;//起始地址
} stru_header;//32位对齐

//存储在flash中
typedef struct Region
{
	stru_header * curr_header;//当前的头信息flash地址
	//uint16_t * step_add;//两次时间间隔内的步数存储位置
	//uint16_t * hrs_add;//当前读到心率数据存储位置
} stru_region;

//存储在flash中，记录一些参数，如步数
typedef struct
{
	uint8_t * step_len;//step length,单位 cm
} stru_para;

//日期时间对应结构
//typedef struct date_struct
//{
//	uint32_t 日期;//当前日期
//	uint8_t up_flag;//上传的标志；0-未上传，1-已上传
//	uint16_t curr_step_data;//step:记录当天的当前总步数；
//	uint8_t curr_hrs_data;//hrs：最新测量的心率数据
//	uint8_t *start_add;//起始地址
//} date_add;//32位对齐

#endif