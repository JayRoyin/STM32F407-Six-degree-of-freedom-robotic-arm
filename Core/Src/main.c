/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pca9685.h"
#include "math.h"
#include "key.h"
#include "oled.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
	
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159f
#define L1 103.8f
#define L2 96.3f
#define L3 154.4f
#define MAX_SPEED 30
//#define PS2_MODE 0
#define USER_PRINTF
#ifdef USER_PRINTF
#define user_printf(aaa,...) printf(aaa"\n",##__VA_ARGS__)
#else
#define user_printf(format,...)
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int fputc(int ch,FILE *f)
{
	uint8_t temp[1]={ch};
	if(HAL_UART_Transmit(&huart1,temp,1,0xffff)!=HAL_OK)
		Error_Handler();
	return ch;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void joint_init(void);
void joint_angle(uint8_t joint,int angle,uint8_t sped);
//#if !PS2_MODE
//uint8_t robot_control(float y,float z,float Alpha);
//void joint_angle_mode0(int ang1,int ang2,int ang3,uint8_t speed);
//#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t speed=10;
uint16_t joint_zero[7]={307,307,315,307,307,315,307};
uint16_t joint_max_angle[7]={300,270,270,270,270,180,270};
int joint_angle_now[7]={0,0,0,0,0,0,0};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	uint8_t key=0;
	joint_init();
	OLED_Init();
	OLED_ShowChinese(0,24,4,24,1);
	OLED_ShowChinese(24,24,5,24,1);
	OLED_ShowChar(48,24,':',24,1);
	OLED_Refresh();
	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_IT(&hadc1);
	HAL_Delay(1000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */
		    // 初始化I2C
    IIC_Init();

    // 设置PWM频率为50Hz
    setPWMFreq(50);

    // 设置第0通道的PWM，on为100，off为400
    setPWM(0, 100, 400);
		setServoAngle(0,90.0);
    while (1)
    {
        setServoAngle(0, 90.0);
        delay_ms(1000);
    }
    /* USER CODE BEGIN 3 */
		#if PS2_MODE
		static int ps_angle[7]={0};
		key = KEY_SCAN();
		user_printf("J1=%d,J2=%d,J3=%d,J4=%d",joint_angle_now[1],joint_angle_now[2],joint_angle_now[3],joint_angle_now[4]);
		user_printf("J5=%d,J6=%d,speed=%d",joint_angle_now[5],joint_angle_now[6],speed);
		if(1)
		{
			if(key==1&&ps_angle[1]!=joint_max_angle[1]/2)
				ps_angle[1]++;
			if(key==2&&ps_angle[1]!=-joint_max_angle[1]/2)
				ps_angle[1]--;
			joint_angle(1,ps_angle[1],speed);
			if(key==3&&ps_angle[2]!=90)
				ps_angle[2]++;
			if(key==4&&ps_angle[2]!=-90)
				ps_angle[2]--;
			joint_angle(2,ps_angle[2],speed);
			
			if(key==5&&ps_angle[3]!=55)
				ps_angle[3]++;
			if(key==6&&ps_angle[3]!=-joint_max_angle[3]/2)
				ps_angle[3]--;
			joint_angle(3,ps_angle[3],speed);
			
			if(key==7&&ps_angle[4]!=joint_max_angle[4]/2)
				ps_angle[4]++;
			if(key==8&&ps_angle[4]!=-20)
				ps_angle[4]--;
			joint_angle(4,ps_angle[4],speed);
			
			if(key==9&&ps_angle[5]!=joint_max_angle[5]/2)
				ps_angle[5]++;
			if(key==10&&ps_angle[5]!=-joint_max_angle[5]/2)
				ps_angle[5]--;
			joint_angle(5,ps_angle[5],speed);
			
			if(key==11&&ps_angle[6]!=90)
				ps_angle[6]++;
			if(key==12&&ps_angle[6]!=-70)
				ps_angle[6]--;
			joint_angle(6,ps_angle[6],speed);
			
			if(key==13&&speed!=0)
			{
				speed--;
				HAL_Delay(500);
			}
			if(key==14&&speed!=MAX_SPEED)
			{
				speed++;
				HAL_Delay(500);
			}
			
		}
		else
			user_printf("error");
		#else
		static float y=L2+L3;
		static float z=L1;
		static float alpha =0;
		static int ps_angle[3]={0};
		static float last_y=L2+L3;
		static float last_z=L1;
		static float last_alpha =0;
		//user_printf("%d",speed);
		key = KEY_SCAN();
		if(1)
		{
				if(key==1)
					y+=5;
				if(key==2)
					y-=5;
				if(key==3)
					z+=5;
				if(key==4)
					z-=5;
				if(key==13&&alpha<0)
					alpha+=0.1f;
				if(key==14&&alpha>-PI/2)
					alpha-=0.1f;
				if(last_y!=y||last_z!=z||last_alpha!=alpha)
				{
					if(robot_control(y,z,alpha))
					{
						last_y=y;
						last_z=z;
						last_alpha=alpha;
					}
					else
						y=last_y;
						z=last_z;
						alpha=last_alpha;
				}
				if(key==5&&ps_angle[0]!=joint_max_angle[1]/2)
					ps_angle[0]++;
				if(key==6&&ps_angle[0]!=-joint_max_angle[1]/2)
					ps_angle[0]--;
				joint_angle(1,ps_angle[0],speed);
				
				if(key==7&&ps_angle[1]!=joint_max_angle[5]/2)
					ps_angle[1]++;
				if(key==8&&ps_angle[1]!=-joint_max_angle[5]/2)
					ps_angle[1]--;
				joint_angle(5,ps_angle[1],speed);
				
				if(key==9&&ps_angle[2]!=90)
					ps_angle[2]++;
				if(key==10&&ps_angle[2]!=-70)
					ps_angle[2]--;
				joint_angle(6,ps_angle[2],speed);
				
				if(key==11&&speed!=0)
				{
					speed--;
					HAL_Delay(500);
				}
				if(key==12&&speed!=MAX_SPEED)
				{
					speed++;
					HAL_Delay(500);
				}
		}
		HAL_Delay(5);
		#endif
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	int32_t adc_data = HAL_ADC_GetValue(&hadc1);
	OLED_ShowNum(0,0,adc_data,4,24,1);
	adc_data-=2930;
	float data = adc_data*1.0f/1166;
	if(data<0)
		data=0;
	adc_data=data*100;
	OLED_ShowNum(60,24,adc_data,2,24,1);
	OLED_Refresh();
}
#if !PS2_MODE
//1:可以到达，0：不能到达
uint8_t robot_control(float x,float y,float Alpha)
{
		int ang[3];
		float theta1,theta2,theta3;
		float x1,y1;
		float bet,t;
		x1=x-L3*cosf(Alpha);
		y1=y-L3*sinf(Alpha);
		if((x1*x1+y1*y1)>(L1+L2)*(L1+L2))
		{
			user_printf("无解");
			return 0;
		}
		bet=(x1*x1+y1*y1+L1*L1-L2*L2)/(2*L1*sqrtf(x1*x1+y1*y1));
		bet=acosf(bet);
		t=atan2f(y1,x1);
		theta1=(bet+t)*180/PI;
		theta2=(L1*L1+L2*L2-x1*x1-y1*y1)/(2*L1*L2);
		theta2=acosf(theta2)*180/PI-180;
		theta3=Alpha*180/PI-theta1-theta2;
		
		user_printf("j2=%.0f,j3=%.0f,j4=%.0f",theta1,theta2,theta3);
		user_printf("y=%.1f,z=%.1f,alpha=%.1f",x,y,Alpha*180/PI);
		
		if(theta1>=180||theta1<=0)
		{
			user_printf("关节2受限");
			return 0;
		}
		if(theta2<=-145||theta2>=45)
		{
			user_printf("关节3受限");
			return 0;
		}
		if(theta3>=20||theta3<=-135)
		{
			user_printf("关节4受限");
			return 0;
		}
		ang[0]=(int)round(theta1-90);
		ang[1]=(int)round(theta2+90);
		ang[2]=(int)round(theta3);
		
		joint_angle_mode0(ang[0],-ang[1],-ang[2],speed);
		return 1;
}
void joint_angle_mode0(int ang1,int ang2,int ang3,uint8_t speed)
{
	int pwm[3];
	while(joint_angle_now[2]!=ang1||joint_angle_now[3]!=ang2||joint_angle_now[4]!=ang3)
	{
		if(joint_angle_now[2]<ang1)
			joint_angle_now[2]++;
		else if(joint_angle_now[2]>ang1)
			joint_angle_now[2]--;
		pwm[0]=(int)round(joint_angle_now[2]*4096*2.0/joint_max_angle[2]/20);
		
		
		if(joint_angle_now[3]<ang2)
			joint_angle_now[3]++;
		else if(joint_angle_now[3]>ang2)
			joint_angle_now[3]--;
		pwm[1]=(int)round(joint_angle_now[3]*4096*2.0/joint_max_angle[3]/20);
		
		
		if(joint_angle_now[4]<ang3)
			joint_angle_now[4]++;
		else if(joint_angle_now[4]>ang3)
			joint_angle_now[4]--;
		pwm[2]=(int)round(joint_angle_now[4]*4096*2.0/joint_max_angle[4]/20);
		setPWM(2,0,joint_zero[2]+pwm[0]);
		setPWM(3,0,joint_zero[3]+pwm[1]);
		setPWM(4,0,joint_zero[4]+pwm[2]);
		HAL_Delay(MAX_SPEED-speed);
	}
}
#endif
void joint_init()
{
	IIC_Init();
	PCA9685_write(PCA9685_MODE1,0x00);
	setPWMFreq(50);
	for(uint8_t i=0;i<7;i++)
	{
		setPWM(i,0,joint_zero[i]);

	}
}
void joint_angle(uint8_t joint,int angle,uint8_t speed)
{
	int pwm;
	if(joint<=6&&joint>=0)
	{
		while(joint_angle_now[joint]!=angle)
		{
			if(joint_angle_now[joint]<angle)
				joint_angle_now[joint]++;
			else if(joint_angle_now[joint]>angle)
				joint_angle_now[joint]--;
			pwm=(int)round(joint_angle_now[joint]*4096*2.0/joint_max_angle[joint]/20);
			setPWM(joint,0,joint_zero[joint]+pwm);
			HAL_Delay(MAX_SPEED+5-speed);
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
