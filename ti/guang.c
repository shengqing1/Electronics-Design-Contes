#include "guang.h"
extern int LineL1, LineL2, LineR1, LineR2 ;
extern int pin2,pin9,pin21,pin14;
// ��ȡ����״̬�ĺ���
void readGuangStatus() {
    // ��ȡPA15���ŵ�״̬
    LineR1 = !DL_GPIO_readPins(GUANG_PORT, GUANG_PIN_15_PIN);
	// ��ȡPA16���ŵ�״̬
    LineR2 = !DL_GPIO_readPins(GUANG_PORT, GUANG_PIN_16_PIN);
	// ��ȡPA17���ŵ�״̬
    LineL1 = !DL_GPIO_readPins(GUANG_PORT, GUANG_PIN_17_PIN);
	// ��ȡPA18���ŵ�״̬
    LineL2 = !DL_GPIO_readPins(GUANG_PORT, GUANG_PIN_18_PIN);
	
	 pin2 = !DL_GPIO_readPins(GUANG_PORT, DL_GPIO_PIN_2);
	// ��ȡPA18���ŵ�״̬
    pin9 = !DL_GPIO_readPins(GUANG_PORT, DL_GPIO_PIN_9);
	
	pin21 = !!DL_GPIO_readPins(GUANG_PORT, GUANG_PIN_21_PIN);//����
	// ��ȡPA18���ŵ�״̬
    pin14 = !!DL_GPIO_readPins(GUANG_PORT, GUANG_PIN_14_PIN);//����
}