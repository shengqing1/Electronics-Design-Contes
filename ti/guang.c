#include "guang.h"
extern int LineL1, LineL2, LineR1, LineR2 ;
extern int pin2,pin9,pin21,pin14;
// 读取光电管状态的函数
void readGuangStatus() {
    // 读取PA15引脚的状态
    LineR1 = !DL_GPIO_readPins(GUANG_PORT, GUANG_PIN_15_PIN);
	// 读取PA16引脚的状态
    LineR2 = !DL_GPIO_readPins(GUANG_PORT, GUANG_PIN_16_PIN);
	// 读取PA17引脚的状态
    LineL1 = !DL_GPIO_readPins(GUANG_PORT, GUANG_PIN_17_PIN);
	// 读取PA18引脚的状态
    LineL2 = !DL_GPIO_readPins(GUANG_PORT, GUANG_PIN_18_PIN);
	
	 pin2 = !DL_GPIO_readPins(GUANG_PORT, DL_GPIO_PIN_2);
	// 读取PA18引脚的状态
    pin9 = !DL_GPIO_readPins(GUANG_PORT, DL_GPIO_PIN_9);
	
	pin21 = !!DL_GPIO_readPins(GUANG_PORT, GUANG_PIN_21_PIN);//左蓝
	// 读取PA18引脚的状态
    pin14 = !!DL_GPIO_readPins(GUANG_PORT, GUANG_PIN_14_PIN);//右蓝
}