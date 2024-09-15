#include "ti_msp_dl_config.h"
#ifndef u8
#define u8 uint8_t
#endif

#ifndef u16
#define u16 uint16_t
#endif

#ifndef u32
#define u32 uint32_t
#endif
extern int openmv[7];//stm32接收数据数组
extern int16_t data1;
extern int16_t data2;
extern int16_t data3;
extern int16_t data4;
 
void Openmv_Receive_Data(int16_t data);
void Openmv_Data(void);
