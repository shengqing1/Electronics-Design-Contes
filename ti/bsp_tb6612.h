/*
 * ������������Ӳ�������������չ����Ӳ�����Ϲ���ȫ����Դ
 * �����������www.lckfb.com
 * ����֧�ֳ�פ��̳���κμ������⻶ӭ��ʱ����ѧϰ
 * ������̳��https://oshwhub.com/forum
 * ��עbilibili�˺ţ������������塿���������ǵ����¶�̬��
 * ��������׬Ǯ���������й�����ʦΪ����
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-31     LCKFB-LP    first version
 */

#ifndef _BSP_TB6612_H
#define _BSP_TB6612_H

#include "ti_msp_dl_config.h"


#define AIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_AIN1_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN1_PIN)) )
#define AIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_AIN2_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN2_PIN)) )
#define BIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_BIN1_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN1_PIN)) )
#define BIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_BIN2_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN2_PIN)) )
#define Left (int)0
#define Right (int)1

void AIN_GPIO_INIT(void);
void BIN_GPIO_INIT(void);
void TB6612_Init(uint16_t pre,uint16_t per);
void AO_Control(uint8_t dir, uint32_t speed);
void BO_Control(uint8_t dir, uint32_t speed);
void Motor_straight(int speed,int wheel);
void Motor_Control(void);

#endif  /* _BSP_TB6612_H */
