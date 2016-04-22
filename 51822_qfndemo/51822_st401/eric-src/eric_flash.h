#ifndef __ERIC_FLASH_H
#define __ERIC_FLASH_H
#include <stdint.h>
//flash address
#define ADDR_FLASH_SECTOR_0     (0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     (0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     (0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     (0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     (0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     (0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     (0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     (0x08060000) /* Base @ of Sector 7, 128 Kbytes */

//����ram��
//��¼������Ϣͷ�ṹ��,8���ֽ�
typedef struct Header
{
	uint32_t curr_step_data;//step:��¼����ĵ�ǰ�ܲ�����
	uint8_t curr_hrs_data;//hrs�����²�������������
	uint16_t cur_bld_press;//blood pressure
	uint8_t *start_add;//��ʼ��ַ
} stru_header;//32λ����

//�洢��flash��
typedef struct
{
	uint16_t step_count;
	uint32_t sleep_status;
	uint8_t  hrs_rate;
	uint16_t bld_press;// blood pressure
	
} stru_region;

//�洢��flash�У���¼һЩ�������粽��
typedef struct
{
	uint8_t   step_len;//step length,��λ cm
	uint16_t  height;//person height
	uint8_t   bt_name[8];
	
} stru_para;

//����ʱ���Ӧ�ṹ
//typedef struct date_struct
//{
//	uint32_t ����;//��ǰ����
//	uint8_t up_flag;//�ϴ��ı�־��0-δ�ϴ���1-���ϴ�
//	uint16_t curr_step_data;//step:��¼����ĵ�ǰ�ܲ�����
//	uint8_t curr_hrs_data;//hrs�����²�������������
//	uint8_t *start_add;//��ʼ��ַ
//} date_add;//32λ����

#endif