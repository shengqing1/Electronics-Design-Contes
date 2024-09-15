#include "Encoder.h"

extern int Distance;//����������
extern int openEncoder;
extern int currentA;
extern int currentB;
extern int lastA;  // ��һ�ζ�ȡ��Aͨ��״̬
extern int lastB;  // ��һ�ζ�ȡ��Bͨ��״̬
extern int position;  // λ�ü���
extern int currentA_;
extern int currentB_;
extern int lastA_;  // ��һ�ζ�ȡ��Aͨ��״̬
extern int lastB_;  // ��һ�ζ�ȡ��Bͨ��״̬
extern int position_;  // λ�ü���
extern float Speed_Left;
extern float Speed_Right;
extern float Speed_Current;
int speed_lv_r[5] = {0};
int speed_lv_l[5] = {0};
//---------------------------------------------------------------------------------------------
void Encoder_Integral(void)
{
//	if(openEncoder == 1){
//		uint64_t temp = (uint64_t)position + position_;
//    Distance = (int)(temp / 2);
//	}
}

void readEncoder(void){
	           currentB=!!DL_GPIO_readPins(Encoder_PORT, Encoder_Encoder_A_PIN);
				     currentA=!!DL_GPIO_readPins(Encoder_PORT, Encoder_Encoder_B_PIN);
							if (lastA != currentA || lastB != currentB) {
							if (lastA == 0 && currentA == 1) {  // Aͨ����0��1
									if (lastB == 0) {  // Bͨ��Ϊ0��˳ʱ����ת
											position++;
									} else {  // Bͨ��Ϊ1����ʱ����ת
											position--;
									}
							} else if (lastA == 1 && currentA == 0) {  // Aͨ����1��0
									if (lastB == 1) {  // Bͨ��Ϊ1��˳ʱ����ת
											position++;
									} else {  // Bͨ��Ϊ0����ʱ����ת
											position--;
									}
							}

							// ��������״̬
							lastA = currentA;
							lastB = currentB;
							}
							
							currentB_=!!DL_GPIO_readPins(Encoder_PORT, Encoder_Dncoder_A_PIN);
				      currentA_=!!DL_GPIO_readPins(Encoder_PORT, Encoder_Dncoder_B_PIN);
							if (lastA_ != currentA_ || lastB_ != currentB_) {
							if (lastA_ == 0 && currentA_ == 1) {  // Aͨ����0��1
									if (lastB_ == 0) {  // Bͨ��Ϊ0��˳ʱ����ת
											position_++;
									} else {  // Bͨ��Ϊ1����ʱ����ת
											position_--;
									}
							} else if (lastA_ == 1 && currentA_ == 0) {  // Aͨ����1��0
									if (lastB_ == 1) {  // Bͨ��Ϊ1��˳ʱ����ת
											position_++;
									} else {  // Bͨ��Ϊ0����ʱ����ת
											position_--;
									}
							}

							// ��������״̬
							lastA_ = currentA_;
							lastB_ = currentB_;
							}
}
void Get_Speed(void)
{
	int i,max,min;

	 Speed_Left = position_;
	 position_ = 0;
	 speed_lv_l[0] = speed_lv_l[1];
		speed_lv_l[1] = speed_lv_l[2];
		speed_lv_l[2] = speed_lv_l[3];
		speed_lv_l[3] = speed_lv_l[4];
		speed_lv_l[4] = Speed_Left;
		max = speed_lv_l[0];
		min = speed_lv_l[0];
		for(i = 1; i < 5; i++)
		{
			if(speed_lv_l[i] > max) 
				max = speed_lv_l[i];
			
			if(speed_lv_l[i] < min) 
				min = speed_lv_l[i];
		}
		Speed_Left = ((speed_lv_l[0] + speed_lv_l[1] + speed_lv_l[2] + speed_lv_l[3] + speed_lv_l[4] - max - min)/3);

	 Speed_Right = position;
	 position = 0;
		speed_lv_r[0] = speed_lv_r[1];
		speed_lv_r[1] = speed_lv_r[2];
		speed_lv_r[2] = speed_lv_r[3];
		speed_lv_r[3] = speed_lv_r[4];
		
		speed_lv_r[4] = Speed_Right;
		
		max = speed_lv_r[0];
		min = speed_lv_r[0];
		
		for(i = 1; i < 5; i++)
		{
			if(speed_lv_r[i] > max) 
				max = speed_lv_r[i];
			
			if(speed_lv_r[i] < min) 
				min = speed_lv_r[i];
		}
		
		Speed_Right = ((speed_lv_r[0] + speed_lv_r[1] + speed_lv_r[2] + speed_lv_r[3] + speed_lv_r[4] - max - min)/3);

		
		Speed_Current = (Speed_Left + Speed_Right)/2.0f;
}