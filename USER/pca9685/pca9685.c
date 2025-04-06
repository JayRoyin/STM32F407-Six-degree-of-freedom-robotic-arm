#include "pca9685.h"
#include "myiic.h"
#include "delay.h"
#include "math.h"
void PCA9685_write(unsigned char reg,unsigned char data)
{
    IIC_Start();
    IIC_Send_Byte(PCA9685_adrr);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Send_Byte(data);
    IIC_Wait_Ack();
    IIC_Stop();
}
u8 PCA9685_read(unsigned char reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte(PCA9685_adrr);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();    
        IIC_Start();                
    IIC_Send_Byte(PCA9685_adrr|0X01);
    IIC_Wait_Ack();
    res=IIC_Read_Byte(0);       
    IIC_Stop();             
    return res;  
}
void setPWMFreq(u8 freq)
{
   u8 prescale,oldmode,newmode;
   double prescaleval;
   prescaleval = 25000000.0f/(4096*freq*0.97);
   prescale = (u8)floor(prescaleval+0.5)-1;

   oldmode = PCA9685_read(PCA9685_MODE1);
   newmode = (oldmode&0x7F) | 0x10; // sleep
   PCA9685_write(PCA9685_MODE1, newmode); // go to sleep
   PCA9685_write(PCA9685_PRESCALE, prescale); // set the prescaler
   PCA9685_write(PCA9685_MODE1, oldmode);
   delay_ms(5);
   PCA9685_write(PCA9685_MODE1, oldmode | 0xa1); 
}
void setPWM(u8 num, u16 on, u16 off) 
{
    PCA9685_write(LED0_ON_L+4*num,on);
    PCA9685_write(LED0_ON_H+4*num,on>>8);
    PCA9685_write(LED0_OFF_L+4*num,off);
    PCA9685_write(LED0_OFF_H+4*num,off>>8);
}
void PCA9685_write(unsigned char reg,unsigned char data);
u8 PCA9685_read(unsigned char reg);
void setPWMFreq(u8 freq);
void setPWM(u8 num, u16 on, u16 off);

void setServoAngle(u8 servoNum, float angle) 
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // 转换角度到脉宽
    
		float pulseWidth = (angle / 180.0) * (2.0 - 1.0) + 1.0; // 1ms到2ms之间
	
    float pulseLength = 1000.0 / 50.0; // 50Hz的周期（20ms）
    u16 pulse = (u16)(4096 * pulseWidth / pulseLength);
    
    setPWM(servoNum, 0, pulse);
}

