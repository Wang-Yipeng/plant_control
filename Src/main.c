/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "mb.h"
#include "led.h"
#include "pid_control.h"
#include "uart.h"
#include "math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;   //普通串口
UART_HandleTypeDef huart2;   //print,调试串口
UART_HandleTypeDef huart3;	//RS422B
UART_HandleTypeDef huart6;   //RS422A


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

struct SAngle
{
	short Angle[3];
	short T;
};


struct SAngle stcAngle;

unsigned char aRxBuffer1[1];
unsigned char aRxBuffer2[1];
unsigned char aRxBuffer3[1];
 
unsigned char Ini_Flag=0;

unsigned char Home_Flag=0;
unsigned char plate_state=0x08;

uint8_t open_control=0;

double SetHeading=0; 
//设置航向，单位为度，0为东向，逆时针为正
unsigned char sensordata[8];

double DoubleHomeHeading=0; 
short FloatHomeHeading=0;
//设置航向，单位为度，0为东向，逆时针为正

int32_t PID_K1=0,PID_K2=0,PID_DK=0;
uint32_t pos_home=39000;

double diffheading=0;



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim3.Instance) {
		unxx_led_timer_callback();
	//	uart_enc();
		//printf("\r\n unxx_led_timer_callback(); \r\n");
	}
		//printf("\r\n HAL_TIM_PeriodElapsedCallback \r\n");
	if(htim->Instance == htim7.Instance) {
		pxMBPortCBTimerExpired();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if(huart->Instance == huart6.Instance) {
		pxMBFrameCBTransmitterEmpty();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//printf("\r\n H1 \r\n");
	if(huart->Instance == huart6.Instance) {
		//printf("\r\n HAL_UART_RxCpltCallback2 \r\n");
		pxMBFrameCBByteReceived();
		//printf("\r\n HAL_UART_RxCpltCallback3 \r\n");
		
	}
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM7_Init();
  MX_UART4_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  
  HAL_UART_Receive_IT(&huart4, (unsigned char *)aRxBuffer2, 1);	
  
 //   eMBInit(MB_RTU, 0x04, 0x06, 115200, MB_PAR_NONE);
	// 从机地址为04，使用串口06，设置波特率为115200，没有奇偶校验
	//单条指令格式为：  地址	功能代码	数据地址	数据	LRC高字节	LRC低字节
	//04 03 00 00 00 01 84 5F
	//04 03 00 01 00 01 D5 9F
	//04 03 00 02 00 01 25 5F
	//04 03 00 03 00 01 74 5F
	//04 03 00 04 00 01 C5 9E
	//04 03 00 05 00 01 94 5E
	//04 03 00 06 00 01 64 5E
	//04 03 00 07 00 01 35 9E
	
	//测试写对地址0x0000的读写
	//04 03 00 00 00 01 84 5F
	//04 06 00 00 FF EE 48 23
	//04 03 00 00 00 01 84 5F
	
//	eMBEnable();
  
 //LEDIO测试
 
	uint8_t K1_RxBuffer6[20]="K1_RxBuffer6-T \r\n";	
	uint8_t K1_RxBuffer3[20]="K1_RxBuffer3-T \r\n";
	uint8_t K1_RxBuffer2[20]="K1_RxBuffer2-T \r\n";
	uint8_t K1_RxBuffer4[20]="K1_RxBuffer4-T \r\n";
	HAL_UART_Transmit(&huart6, K1_RxBuffer6,20,1000);
	HAL_UART_Transmit(&huart3, K1_RxBuffer3,20,1000);
	HAL_UART_Transmit(&huart2, K1_RxBuffer2,20,1000);
	HAL_UART_Transmit(&huart4, K1_RxBuffer4,20,1000);
	
	
//	uint8_t K2_RxBuffer1[20]="422B发送正常 \r\n";	
//	HAL_UART_Transmit(&huart4, K2_RxBuffer1,20,1000);
//	uint8_t K3_RxBuffer1[20]="422A发送正常 \r\n";	
//	HAL_UART_Transmit(&huart4, K3_RxBuffer1,20,1000);
 
  	Ini_LED();
		
	HAL_DAC_Start(&hdac,DAC_CHANNEL_1); //开启DAC通道1
	HAL_DAC_Start(&hdac,DAC_CHANNEL_2); //开启DAC通道1	
	
//	AA 44 13
//	58 FC 01 7C 08 18 CD 09 11 7C 08 00 00 00 00 00 00 7C 72 11 41
//	EE 18 AA 05 78 05 37 40
//	9F 35 09 F5 D9 4E 5C 40
//	00 00 E0 AB 5B 7B 42 40
//	8A C1 BC 4F 6F E3 8C BF
//	6D 17 10 AD 6A D2 A5 3F
//	30 28 45 51 2C CD BA 3F
//	FC 77 D8 53 C9 91 DC BF
//	C7 ED C3 2C CA 70 DB BF
//	00 00 00 00 00 00 00 80
//	0A 00 00 00
//	35 25 91 F8 
	
	
	double apps;
	char app1[8]={0xEE, 0x18 ,0xAA ,0x05 ,0x78, 0x05 ,0x37 ,0x40};
	memcpy(&apps,&app1,8);
	printf("\r\n apps: %0.12f \r\n",apps);



  /* 队列测试----------------------------------------------------------*/
	
//	uint16_t j=1;
//	uint16_t arr_num=0;
//	uint16_t buff_sizeA=5;
//	uint8_t latitude_arrA[5];
//	while(j<9){
//	
//			
//		if(arr_num<buff_sizeA){
//			
//			latitude_arrA[arr_num]=j;
//			printf("\r\n latitude_arrA: %d %d %d %d %d  \r\n",latitude_arrA[0],latitude_arrA[1], 
//			latitude_arrA[2],latitude_arrA[3],latitude_arrA[4]);
//			arr_num++;
//		}
//		else{
//			uint16_t i=0;
//			while(i!=buff_sizeA-1){
//				latitude_arrA[i]=latitude_arrA[i+1];
//				i++;					
//			}
//		latitude_arrA[buff_sizeA-1]=j;
//			
//		printf("\r\n latitude_arrA: %d %d %d %d %d  \r\n",latitude_arrA[0],latitude_arrA[1], 
//			latitude_arrA[2],latitude_arrA[3],latitude_arrA[4]);

//		}
//		j++;
//			
//}
	
  /* 队列测试----------------------------------------------------------*/	
		
	
	
//	platform_ini();
//	
//	GoHome();

//	memcpy(&FloatHomeHeading,&sensordata[4],2);
//	
//	printf("\r\n 进入主函数初始化... \r\n");




	

	//关闭电机使能，控制信号置零
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET); 		//关闭转台电机使能
//		
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET); 		//关闭转台电机使能
//	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,3000);    //关闭控制信号
//	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);	//关闭控制信号
//		
//	HAL_Delay(3000);	

//	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);    //关闭控制信号
//	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,3000);	//关闭控制信号	
//		
//	HAL_Delay(3000);	
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET); 		//关闭转台电机使能
		
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	//	eMBPoll();
		
	//	printf("\r\n WHILE_print \r\n");
//		uint8_t K1_RxBuffer6[20]="K1_RxBuffer6-T \r\n";	
//		uint8_t K1_RxBuffer3[20]="K1_RxBuffer3-T \r\n";
//		uint8_t K1_RxBuffer2[20]="K1_RxBuffer2-T \r\n";
//		uint8_t K1_RxBuffer4[20]="K1_RxBuffer4-T \r\n";
//		HAL_UART_Transmit(&huart6, K1_RxBuffer6,20,1000);
//		HAL_UART_Transmit(&huart3, K1_RxBuffer3,20,1000);
//		HAL_UART_Transmit(&huart2, K1_RxBuffer2,20,1000);
//		HAL_UART_Transmit(&huart4, K1_RxBuffer4,20,1000);
		
		HAL_Delay(3000);
		printf("\r\n diffheading：%f \r\n",diffheading);
//		short newheading1;
//		memcpy(&newheading1,&sensordata[4],2);
//		float diffheading=(float)(FloatHomeHeading-newheading1)/32768*180;
//		
//	
//		uint32_t control_z=(pos_home+diffheading*(pos_home-9000)/30);
//		

//			
//		pid_control(control_z,PID_K1, PID_K2, PID_DK);
	  
	  
	  if (open_control==1){
	  
	 		uint32_t control_z=(pos_home+diffheading*(pos_home)/30);
			
			pid_control(control_z,PID_K1, PID_K2, PID_DK);
		  printf("\r\n diffheading：%f \r\n",diffheading);
		  printf("\r\n control_z：%d \r\n",control_z);
	  
	  }
	  
	  
	
	  
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_EnableOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DAC init function */
void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  HAL_DAC_Init(&hdac);

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);

    /**DAC channel OUT2 config 
    */
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  HAL_TIM_Encoder_Init(&htim2, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* TIM7 init function */
void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 99;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 27;
  HAL_TIM_Base_Init(&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

}

/* UART4 init function */
void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart4);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

/* USART6 init function */
void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart6);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin 
                          |LED5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Enable_Pin */
  GPIO_InitStruct.Pin = Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_EXTI12_Z_Pin */
  GPIO_InitStruct.Pin = GPIO_EXTI12_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIO_EXTI12_Z_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_EXTI13_HOME_Pin GPIO_EXTI14_N_Pin GPIO_EXTI15_P_Pin */
  GPIO_InitStruct.Pin = GPIO_EXTI13_HOME_Pin|GPIO_EXTI14_N_Pin|GPIO_EXTI15_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin 
                           LED5_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin 
                          |LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	
printf("HAL_GPIO_EXTI_Callback \r\n");
	switch(GPIO_Pin){
		
		case GPIO_PIN_11: //按键3
			printf("按键3_开启航向稳定 \r\n");
			HAL_Delay(100);
			open_control=!open_control;
		

		break;
		
		case GPIO_PIN_0: //按键2，顺时针转动
		
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET); //  PE5变高电平，使能电机		
			
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET); //LED4亮  顺时针向右转动，向右
			
		HAL_Delay(100);	

			while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==0){
								  
				HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,4000);	  
				HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);
			
				
			}
			
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET); //LED0亮
			
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET); //LED0亮
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);
		
		break;
		
		case GPIO_PIN_1: //按键1，逆时针转动		
		   
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET); // PE5变高电平，使能电机	
			
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET); //LED5亮 从上往下看逆时针，向左转动

			HAL_Delay(100);
		
			while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0){
				
				//逆时针旋转	
			  
				HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);				
				HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,4000);
								
			}
			
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET); //LED0亮
			
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET); //LED0亮
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);
	
		break;
		

		case GPIO_PIN_12:  
		
			//	printf("Z脉冲来了 \r\n");
		
				if(Ini_Flag==1){
					Ini_Flag=2;					
					printf("找到右极限旁第一个Z脉冲 \r\n");
					// 此处装载位置计数器初值,Z脉冲初值		
				}
				
				if(Ini_Flag==2){
					
//					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET); //LED0亮
//				    HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);
//		  	        HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,0);
				    printf(" Ini_Flag==2 Z脉冲来了 \r\n");
					uart_enc();
			
					// 此处装载位置计数器初值，Z脉冲初值
		
				}
				
			break;
						
		case GPIO_PIN_13:  
			
				HAL_Delay(100);			
				printf("顺时针极限 \r\n");
		
				if(Ini_Flag==0){
					Ini_Flag=1;
					
					
					printf("初始化到到达右极限 \r\n");	

					
				} 
				if(Ini_Flag==2){
					
					printf("工作到到达右极限 \r\n");	
					
				
				}
		

			break;
								
		case GPIO_PIN_14:
			
				HAL_Delay(100);
				printf("逆时针极限 \r\n");	

			break;
										
		case GPIO_PIN_15:
				
				printf("Home中断 \r\n");	

				
			break;
		default:
			break;
	
	}

}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
