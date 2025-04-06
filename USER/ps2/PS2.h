#ifndef _PS2_H_
#define _PS2_H_
#include "main.h"

/*
��Ҫ
4��GPIO
    1.3���������ģʽ CLK DO CS
    2.1����������ģʽ DI
*/

#define PS2_CS_GPIOx GPIOD
#define PS2_CS_Pin GPIO_PIN_4

#define PS2_CLK_GPIOx GPIOD
#define PS2_CLK_Pin GPIO_PIN_5

#define PS2_DO_GPIOx GPIOD
#define PS2_DO_Pin GPIO_PIN_7

#define PS2_DI_GPIOx GPIOD
#define PS2_DI_Pin GPIO_PIN_6

typedef struct
{
    uint8_t A_D;                                       //ģ��(���)Ϊ1 ����(�޵�)Ϊ0
    uint8_t Rocker_RX, Rocker_RY, Rocker_LX, Rocker_LY; //ҡ��ֵ(ģ��״̬Ϊʵ��ֵ0-0xFF)(����̬Ϊ��Ч��ֵ0,0x80,0xFF)
    //����ֵ0Ϊδ����,1Ϊ����̬
    uint8_t Key_L1, Key_L2, Key_R1, Key_R2;                //���󰴼�
    uint8_t Key_L_Right, Key_L_Left, Key_L_Up, Key_L_Down; //��ఴ��
    uint8_t Key_R_Right, Key_R_Left, Key_R_Up, Key_R_Down; //�Ҳఴ��
    uint8_t Key_Select;                                    //ѡ���
    uint8_t Key_Start;                                     //��ʼ��
    uint8_t Key_Rocker_Left, Key_Rocker_Right;             //ҡ�˰���

} PS2_TypeDef;
extern PS2_TypeDef PS2_Data;
void PS2_Read_Data(void);
#endif
