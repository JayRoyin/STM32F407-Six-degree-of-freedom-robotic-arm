#include "delay.h"
void delay_us(u32 us)
{//����stm32f4ϵ�� 168mhz������1us
		u8 i = 0;
		while(us--)
		{
			i = 42;
			while(i--);
		};
}
void delay_ms(u16 t)
{
	HAL_Delay(t);
}
