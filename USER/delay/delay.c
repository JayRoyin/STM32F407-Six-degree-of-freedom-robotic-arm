#include "delay.h"
void delay_us(u32 us)
{//对于stm32f4系列 168mhz大致是1us
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
