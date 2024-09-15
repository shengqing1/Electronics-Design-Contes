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
#include "srevo.h"

unsigned int Servo_Angle = 0;//����Ƕ�

/******************************************************************
       ����ռ�ձ� ��Χ 0 ~ (per-1)
   t = 0.5ms������������-�����ת�� 0 ��
   t = 1.0ms������������-�����ת�� 45��
   t = 1.5ms������������-�����ת�� 90��
   t = 2.0ms������������-�����ת�� 135��
   t = 2.5ms������������-�����ת��180��
******************************************************************/


/******************************************************************
 * �� �� �� �ƣ�Set_Servo_Angle
 * �� �� ˵ �������ýǶ�
 * �� �� �� �Σ�angle=Ҫ���õĽǶȣ���Χ0-180
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void Set_Servo_Angle(unsigned int angle)
{
    uint32_t period = 400;

    if(angle > 180)
    {
        angle = 180; // ���ƽǶ���0��180��֮��
    }

    Servo_Angle = angle;

    // ����PWMռ�ձ�
    // 0.5ms��Ӧ�ļ��� = 10
    // 2.5ms��Ӧ�ļ��� = 50
    float min_count = 10.0f;
    float max_count = 50.0f;
    float range = max_count - min_count;
    float ServoAngle = min_count + (((float)angle / 180.0f) * range);

    //DL_TimerG_setCaptureCompareValue(PWM_0_INST, (unsigned int)(ServoAngle + 0.5f), GPIO_PWM_0_C0_IDX);
}



/******************************************************************
 * �� �� �� �ƣ���ȡ��ǰ�Ƕ�
 * �� �� ˵ ����Get_Servo_Angle
 * �� �� �� �Σ���
 * �� �� �� �أ���ǰ�Ƕ�
 * ��       �ߣ�LC
 * ��       ע��ʹ��ǰ����ȷ��֮ǰʹ�ù� 
                void Set_Servo_Angle(unsigned int angle) 
                �������ù��Ƕ�
******************************************************************/
unsigned int Get_Servo_Angle(void)
{
        return Servo_Angle;
}