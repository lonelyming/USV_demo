/*******************************************************************************
* �ļ����ƣ�md_led_driver.c
*
* ժ    Ҫ��STM32F1��LED�м�������ʵ�֣��ײ�Ӳ������������Ӧ�ò�֮��Ĳ㣩
*
* ��ǰ�汾��
* ��    �ߣ�
* ��    �ڣ�2017/12/18
* ���뻷����keil5
*
* ��ʷ��Ϣ��
*******************************************************************************/

#include "md_led_driver.h"
#include "main.h"
/*******************************��������****************************************
* ��������: void MD_LED_AMBER_Control(uint8_t status)
* �������: status�����ƵƵ�����״̬
* ���ز���:  
* ��    ��:
* ��    ��: 
* ��    ��: 2017/12/18
*******************************************************************************/ 
void MD_LED_AMBER_Control(uint8_t status)
{
	if(status==1)
	{
		//����
	  HAL_GPIO_WritePin(FMC_LED_AMBER_GPIO_Port,FMC_LED_AMBER_Pin,GPIO_PIN_RESET);
	}
	else
	{
		//Ϩ��
	   HAL_GPIO_WritePin(FMC_LED_AMBER_GPIO_Port,FMC_LED_AMBER_Pin,GPIO_PIN_SET);
	}
}
