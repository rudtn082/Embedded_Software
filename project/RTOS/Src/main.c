/**
******************************************************************************
* @file    TIM/TIM_PWMOutput/Src/main.c
* @author  MCD Application Team
* @version V1.0.1
* @date    26-February-2014
* @brief   This sample code shows how to use STM32F4xx TIM HAL API to generate
*          4 signals in PWM.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "main.h"
#include <stdio.h>
#include "cmsis_os.h"
#include <SEMPHR.H>

/** @addtogroup STM32F4xx_HAL_Examples
* @{
*/

/** @addtogroup TIM_PWM_Output
* @{
*/

/* Private typedef -----------------------------------------------------------*/
#define  PERIOD_VALUE       0xFFFF  /* Period Value  */
#define  PULSE1_VALUE       0xFFFF        /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       900         /* Capture Compare 2 Value  */
#define  PULSE3_VALUE       600         /* Capture Compare 3 Value  */
#define  PULSE4_VALUE       450         /* Capture Compare 4 Value  */

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle1, TimHandle2, TimHandle3, TimHandle4;
TIM_IC_InitTypeDef     sICConfig;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig1, sConfig2, sConfig3;

/* Counter Prescaler value */
uint32_t uwPrescalerValue = 0;
uint16_t motorInterrupt1 = 0;
uint16_t motorInterrupt2 = 0;

uint8_t encoder_right = READY;
uint8_t encoder_left = READY;

/* Captured Values */
uint32_t               uwIC2Value1 = 0;
uint32_t               uwIC2Value2 = 0;
uint32_t               uwDiffCapture1 = 0;

uint32_t               uwIC2Value3 = 0;
uint32_t               uwIC2Value4 = 0;
uint32_t               uwDiffCapture2 = 0;

uint32_t               uwIC2Value5 = 0;
uint32_t               uwIC2Value6 = 0;
uint32_t               uwDiffCapture3 = 0;

uint32_t               uwFrequency = 0;

/* ADC handler declaration */
ADC_HandleTypeDef    AdcHandle1, AdcHandle2, AdcHandle3;
ADC_ChannelConfTypeDef adcConfig1, adcConfig2, adcConfig3;
ADC_ChannelConfTypeDef sConfig;

/* Variable used to get converted value */
__IO uint32_t uhADCxRight;
// __IO uint32_t uhADCxForward;
__IO uint32_t uhADCxLeft;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
void Motor_Forward(void);
void Motor_Backward(void);
void Motor_Left(void);
void Motor_Right(void);
void Motor_Stop(void);
void Motor_Speed_Up_Config(void);
void Motor_Speed_Down_Config(void);
void leftmoterspeedDown();
void leftmoterspeedUp();
static void EXTILine_Config(void);
static void Error_Handler(void);
void IR_Sensor();
/* Private functions ---------------------------------------------------------*/

extern UART_HandleTypeDef UartHandle1, UartHandle2;

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&UartHandle1, (uint8_t *)&ch, 1, 0xFFFF);

return ch;
}

/**
* @brief  Main program.
* @param  None
* @retval None
*/

uint32_t frontResult = 0;		// 0 이면 장애물 X 
uint32_t rightResult = 0;
uint32_t leftResult = 0;
uint32_t diagonalLeft = 0;
uint32_t diagonalRight = 0;
uint32_t LFF = 0;						// 기본 0, 있었다 1
uint32_t RFF = 0;						// 기본 0, 있었다 1
uint32_t when90_01 = 0;						// 기본 0, 돌았다 1
uint32_t when90_02 = 0;						// 기본 0, 돌았다 1
int32_t dgree = 0;						// 오른쪽이면 +
xSemaphoreHandle xSemaphore = NULL;


//왼쪽으로 6도 돌기위한 함수
void turnLeft() {
	if (xSemaphore != NULL)
	{
		if (xSemaphoreTake(xSemaphore, 100))
		{
			while (diagonalRight == 1 || frontResult == 1) {
				Motor_Stop();
				osDelay(10); // 여기 딜레이를 낮추면 좀더 부드럽게 돌 수 있다.

				motorInterrupt1 = 1;		// 바퀴 회전 값 초기화
				Motor_Left();

				while (motorInterrupt1 < 60) { 										// 1회 회전시 바퀴 회전수 60만큼 회전 (약 6도) 
					vTaskDelay(1 / portTICK_RATE_MS);  // motorInterrupt1 값을 읽어오기 위한 딜레이
					dgree -= motorInterrupt1;
				}
				Motor_Stop();
			}
			xSemaphoreGive(xSemaphore);
		}
	}
}

//왼쪽으로 90도 돌기위한 함수
void turnLeft90() {
	if (xSemaphore != NULL)
	{
		if (xSemaphoreTake(xSemaphore, 100))
		{
			Motor_Stop();
			osDelay(10); // 여기 딜레이를 낮추면 좀더 부드럽게 돌 수 있다.

			motorInterrupt1 = 1;		// 바퀴 회전 값 초기화
			Motor_Left();

			while (motorInterrupt1 < 950) { 										// 1회 회전시 바퀴 회전수 950만큼 회전 (약 90도) 
				vTaskDelay(1 / portTICK_RATE_MS);  // motorInterrupt1 값을 읽어오기 위한 딜레이
				dgree -= motorInterrupt1;
			}
			xSemaphoreGive(xSemaphore);
			when90_02 = 1;
		}
	}
}

//오른쪽으로 6도 돌기위한 함수
void turnRight() {
	if (xSemaphore != NULL)
	{
		if (xSemaphoreTake(xSemaphore, 100))
		{
			while (diagonalLeft == 1 || frontResult == 1) {
				//IR_Sensor();		// 대각 IR 센서 탐지 1회 수행 
				Motor_Stop();
				osDelay(10); // 여기 딜레이를 낮추면 좀더 부드럽게 돌 수 있다.

				motorInterrupt2 = 1;		// 바퀴 회전 값 초기화
				Motor_Right();

				while (motorInterrupt2 < 60) { 										// 1회 회전시 바퀴 회전수 60만큼 회전 (약 6도) 
					vTaskDelay(1 / portTICK_RATE_MS);  // motorInterrupt1 값을 읽어오기 위한 딜레이
					dgree += motorInterrupt2;
				}
				Motor_Stop();
			}
			xSemaphoreGive(xSemaphore);
		}
	}
}

//오른쪽으로 90도 돌기위한 함수
void turnRight90() {
	if (xSemaphore != NULL)
	{
		if (xSemaphoreTake(xSemaphore, 100))
		{
			Motor_Stop();
			osDelay(10); // 여기 딜레이를 낮추면 좀더 부드럽게 돌 수 있다.

			motorInterrupt2 = 1;		// 바퀴 회전 값 초기화
			Motor_Right();

			while (motorInterrupt2 < 900) { 										// 1회 회전시 바퀴 회전수 900만큼 회전 (약 90도) 
				vTaskDelay(1 / portTICK_RATE_MS);  // motorInterrupt1 값을 읽어오기 위한 딜레이
				dgree += motorInterrupt2;
			}
			xSemaphoreGive(xSemaphore);
			when90_01 = 1;
		}
	}
}

/* 적외선 감지 함수 */
void IR_Sensor() {
	HAL_ADC_Start(&AdcHandle1);
	uhADCxLeft = HAL_ADC_GetValue(&AdcHandle1);
	HAL_ADC_PollForConversion(&AdcHandle1, 0xFF);
	if (uhADCxLeft > 2000) uhADCxLeft = 2000;
	else if (uhADCxLeft < 100) uhADCxLeft = 100;

	HAL_ADC_Start(&AdcHandle2);
	uhADCxRight = HAL_ADC_GetValue(&AdcHandle2);
	HAL_ADC_PollForConversion(&AdcHandle2, 0xFF);
	if (uhADCxRight > 2000) uhADCxRight = 2000;
	else if (uhADCxRight < 100) uhADCxRight = 100;


	if ((uhADCxLeft > 550 && uhADCxLeft < 1200 && uwDiffCapture3 / 58 > 0 && uwDiffCapture3 / 58 < 2.2) || uhADCxLeft > 845) {
		diagonalLeft = 1;
	}
	else { diagonalLeft = 0; }

	if ((uhADCxRight > 550 && uhADCxRight < 1200 && uwDiffCapture1 / 58 > 0 && uwDiffCapture1 / 58 < 2.2) || uhADCxRight > 845) {
		diagonalRight = 1;
	}
	else { diagonalRight = 0; }
}

/*********************************  task ************************************/
uint32_t temp = 100; // 연속으로 90도 회전하는 것을 방지하는 임시 변수

void Detect_obstacle() {
	osDelay(200);  // 태스크 만든 후 약간의 딜레이
	for (;;)		// 탐지 전체 루프 시작
	{
		osDelay(30);
		IR_Sensor();		// 대각 IR 센서 탐지 1회 수행 

		// 전좌 초음파 센서 수행 시작
		if (uwDiffCapture1 / 58 > 0 && uwDiffCapture1 / 58 < 18) 	// 오른쪽 탐지 0~18
		{
			rightResult = 1;
		}
		else {																																			// 오른쪽 미탐
			rightResult = 0;
		}
		if (uwDiffCapture2 / 58 > 0 && uwDiffCapture2 / 58 < 13) {	// 전방 탐지 0 ~ 13
			frontResult = 1;
		}
		else {																																			// 전방 미탐
			frontResult = 0;
		}
		if (uwDiffCapture3 / 58 > 0 && uwDiffCapture3 / 58 < 18) { 	// 왼쪽 탐지 0 ~ 18
			leftResult = 1;
		}
		else																																			// 왼쪽 미탐
		{
			leftResult = 0;
		}
		temp++;
		if (temp < 150) continue; // temp가 150 미만일 때는 continue

		if (RFF == 0 && uwDiffCapture1 / 58 > 0 && uwDiffCapture1 / 58 < 25 && leftResult == 0 && frontResult == 0 && diagonalLeft == 0 && when90_02 == 1) {
			RFF = 1;
			temp = 0;
			when90_02 = 0;
		}
		if (LFF == 0 && uwDiffCapture3 / 58 > 0 && uwDiffCapture3 / 58 < 25 && rightResult == 0 && frontResult == 0 && diagonalRight == 0 && when90_01 == 1) {
			LFF = 1;
			temp = 0;
			when90_01 = 0;
		}
		// 전좌 초음파 센서 수행 종료
	}		// 탐지 전체 루프 종료
}

void Motor_control() {
	osDelay(200);  // 태스크 만든 후 약간의 딜레이
	Motor_Forward();


	for (;;)
	{
		// 전방에 물체를 감지했을 때
		if (frontResult == 1)
		{
			// 왼쪽, 오른쪽에 감지된 물체가 없을 때
			if (leftResult == 0 && rightResult == 0) {
				if (dgree >= 0) {
					osDelay(10);
					Motor_Stop();
					turnLeft90();
					Motor_Stop();
					osDelay(10);
				}
				else if (dgree < 0) {
					osDelay(10);
					Motor_Stop();
					turnRight90();

					Motor_Stop();
					osDelay(10);
				}
			}
			// 왼쪽에만 감지된 물체가 있을 때
			else if (leftResult == 1 && rightResult == 0) {
				Motor_Stop();
				turnRight90();
				Motor_Stop();
			}
			// 오른쪽에만 감지된 물체가 있을 때
			else if (leftResult == 0 && rightResult == 1) {
				Motor_Stop();
				turnLeft90();
				Motor_Stop();

			}
			// 왼쪽, 오른쪽에 감지된 물체가 있을 때
			else if (leftResult == 1 && rightResult == 1) {
				Motor_Stop();
				if (dgree > 0) { turnLeft(); }
				else if (dgree <= 0) { turnRight(); }
				Motor_Stop();
			}
		}
		// 전방에 감지된 물체가 없을 때
		else {
			// 오른쪽으로 회전해야할 때
			if (RFF == 1 && rightResult == 0 && diagonalRight == 0) {
				osDelay(1200);
				Motor_Stop();
				turnRight90();
				Motor_Stop();
				RFF = 0;
			}
			// 왼쪽으로 회전해야할 때
			if (LFF == 1 && leftResult == 0 && diagonalLeft == 0) {
				osDelay(1200);
				Motor_Stop();
				turnLeft90();
				Motor_Stop();
				LFF = 0;
			}
			// IR 센서 활용, 미감지 시 직진
			if (diagonalLeft == 1) {
				Motor_Stop();
				turnRight();
				Motor_Stop();
			}
			else if (diagonalRight == 1) {
				Motor_Stop();
				turnLeft();
				Motor_Stop();
			}
			else Motor_Forward();
		}
	}

}




/***************************************************************************/

int main(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* STM32F4xx HAL library initialization:
	- Configure the Flash prefetch, instruction and Data caches
	- Configure the Systick to generate an interrupt each 1 msec
	- Set NVIC Group Priority to 4
	- Global MSP (MCU Support Package) initialization
	*/
	HAL_Init();


	/* Configure the system clock to have a system clock = 180 Mhz */
	SystemClock_Config();

	BSP_COM1_Init();



	/************************************** 모터 시작 **************************************/
	uwPrescalerValue = (SystemCoreClock / 2) / 1000000;


	// PB2 모터 전원 인가를 위한 GPIO 초기화
	__GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // MC_EN(PB2) 모터 전원 

	sConfig1.OCMode = TIM_OCMODE_PWM1;
	sConfig1.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig1.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig1.Pulse = 16700;

	TimHandle1.Instance = TIM8;
	TimHandle1.Init.Prescaler = uwPrescalerValue;
	TimHandle1.Init.Period = 20000;
	TimHandle1.Init.ClockDivision = 0;
	TimHandle1.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&TimHandle1);

	HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_2);

	/* Common configuration for all channels */
	sConfig2.OCMode = TIM_OCMODE_PWM1;
	sConfig2.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig2.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig2.Pulse = 17000; 

	TimHandle2.Instance = TIM4;
	TimHandle2.Init.Prescaler = uwPrescalerValue;
	TimHandle2.Init.Period = 20000;
	TimHandle2.Init.ClockDivision = 0;
	TimHandle2.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&TimHandle2);

	HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_2);

	EXTILine_Config(); // Encoder Interrupt Setting
					   /************************************** 모터 끝 **************************************/




					   /************************************** 초음파 시작 **************************************/
	uwPrescalerValue = ((SystemCoreClock / 2) / 1000000) - 1;

	/* Set TIMx instance */
	TimHandle3.Instance = TIM3;

	TimHandle3.Init.Period = 0xFFFF;
	TimHandle3.Init.Prescaler = uwPrescalerValue;
	TimHandle3.Init.ClockDivision = 0;
	TimHandle3.Init.CounterMode = TIM_COUNTERMODE_UP;

	if (HAL_TIM_IC_Init(&TimHandle3) != HAL_OK) { Error_Handler(); }



	/* Configure the Input Capture of channel 2 */
	sICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
	sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
	sICConfig.ICFilter = 0;

	HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_1);
	HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_2);
	HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_3);
	HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_4);

	HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_4);

	uwPrescalerValue = (SystemCoreClock / 2 / 131099) - 1;


	TimHandle4.Instance = TIM10;

	TimHandle4.Init.Prescaler = uwPrescalerValue;
	TimHandle4.Init.Period = 0xFFFF;
	TimHandle4.Init.ClockDivision = 0;
	TimHandle4.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&TimHandle4);

	/* Common configuration for all channels */
	sConfig3.OCMode = TIM_OCMODE_PWM1;
	sConfig3.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig3.OCFastMode = TIM_OCFAST_DISABLE;

	/* Set the pulse value for channel 1 */
	sConfig3.Pulse = 2;
	HAL_TIM_PWM_ConfigChannel(&TimHandle4, &sConfig3, TIM_CHANNEL_1);

	/* Start channel 3 */
	HAL_TIM_PWM_Start(&TimHandle4, TIM_CHANNEL_1);
	/************************************** 초음파 끝**************************************/


	/************************************** 적외선 시작**************************************/

	AdcHandle1.Instance = ADC3;   // ADC 3번분

	AdcHandle1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	AdcHandle1.Init.Resolution = ADC_RESOLUTION12b;
	AdcHandle1.Init.ScanConvMode = DISABLE;
	// Mode 설정
	AdcHandle1.Init.ContinuousConvMode = DISABLE;
	AdcHandle1.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle1.Init.NbrOfDiscConversion = 0;
	AdcHandle1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AdcHandle1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	AdcHandle1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle1.Init.NbrOfConversion = 1;
	//DMA(Direct Memory Access)
	AdcHandle1.Init.DMAContinuousRequests = DISABLE;
	AdcHandle1.Init.EOCSelection = DISABLE;

	HAL_ADC_Init(&AdcHandle1);//ADC Initialized

	adcConfig1.Channel = ADC_CHANNEL_11; //채널 설정
	adcConfig1.Rank = 1;
	adcConfig1.SamplingTime = ADC_SAMPLETIME_480CYCLES; //샘플링 주기 설정
	adcConfig1.Offset = 0;

	HAL_ADC_ConfigChannel(&AdcHandle1, &adcConfig1);

	AdcHandle2.Instance = ADC2;   // ADC부분

	AdcHandle2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	AdcHandle2.Init.Resolution = ADC_RESOLUTION12b;
	AdcHandle2.Init.ScanConvMode = DISABLE;
	AdcHandle2.Init.ContinuousConvMode = DISABLE;
	AdcHandle2.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle2.Init.NbrOfDiscConversion = 0;
	AdcHandle2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AdcHandle2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	AdcHandle2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle2.Init.NbrOfConversion = 1;
	AdcHandle2.Init.DMAContinuousRequests = DISABLE;
	AdcHandle2.Init.EOCSelection = DISABLE;

	HAL_ADC_Init(&AdcHandle2);

	adcConfig2.Channel = ADC_CHANNEL_14;
	adcConfig2.Rank = 1;
	adcConfig2.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	adcConfig2.Offset = 0;

	HAL_ADC_ConfigChannel(&AdcHandle2, &adcConfig2);

	AdcHandle3.Instance = ADC1;   // ADC부분

	AdcHandle3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	AdcHandle3.Init.Resolution = ADC_RESOLUTION12b;
	AdcHandle3.Init.ScanConvMode = DISABLE;
	AdcHandle3.Init.ContinuousConvMode = DISABLE;
	AdcHandle3.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle3.Init.NbrOfDiscConversion = 0;
	AdcHandle3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AdcHandle3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	AdcHandle3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle3.Init.NbrOfConversion = 1;
	AdcHandle3.Init.DMAContinuousRequests = DISABLE;
	AdcHandle3.Init.EOCSelection = DISABLE;

	HAL_ADC_Init(&AdcHandle3);

	adcConfig3.Channel = ADC_CHANNEL_15;
	adcConfig3.Rank = 1;
	adcConfig3.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	adcConfig3.Offset = 0;
	HAL_ADC_ConfigChannel(&AdcHandle3, &adcConfig3);
	/************************************** 적외선 끝**************************************/


	/******* 학번 : 201604140, 201604149 이름 : 박경수, 조민석 *******/

	xTaskCreate(Detect_obstacle, "obstacle", 1000, NULL, 2, NULL);
	xTaskCreate(Motor_control, "motor", 1000, NULL, 2, NULL);
	// 세마포어 생성
	xSemaphore = xSemaphoreCreateMutex();

	vTaskStartScheduler();

}


void Motor_Forward(void)
{
	HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
}

void Motor_Backward(void)
{
	HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
}
void Motor_Left(void)
{
	HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
}
void Motor_Right(void)
{
	HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
}

void Motor_Stop(void)
{
	HAL_TIM_PWM_Stop(&TimHandle1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&TimHandle1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&TimHandle2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&TimHandle2, TIM_CHANNEL_2);
}

void Motor_Speed_Up_Config(void)
{
	sConfig1.Pulse = sConfig1.Pulse + 100;
	sConfig2.Pulse = sConfig2.Pulse + 100;
	TIM8->CCR1 = sConfig1.Pulse;
	TIM8->CCR2 = sConfig1.Pulse;
	TIM4->CCR1 = sConfig2.Pulse;
	TIM4->CCR2 = sConfig2.Pulse;
}

void Motor_Speed_Down_Config(void)
{
	sConfig1.Pulse = sConfig1.Pulse - 100;
	sConfig2.Pulse = sConfig2.Pulse - 100;
	TIM8->CCR1 = sConfig1.Pulse;
	TIM8->CCR2 = sConfig1.Pulse;
	TIM4->CCR1 = sConfig2.Pulse;
	TIM4->CCR2 = sConfig2.Pulse;
}
/**
* @brief  System Clock Configuration
*         The system Clock is configured as follow :
*            System Clock source            = PLL (HSE)
*            SYSCLK(Hz)                     = 180000000
*            HCLK(Hz)                       = 180000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 4
*            APB2 Prescaler                 = 2
*            HSE Frequency(Hz)              = 25000000
*            PLL_M                          = 25
*            PLL_N                          = 360
*            PLL_P                          = 2
*            PLL_Q                          = 7
*            VDD(V)                         = 3.3
*            Main regulator output voltage  = Scale1 mode
*            Flash Latency(WS)              = 5
* @param  None
* @retval None
*/
static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the device is
	clocked below the maximum system frequency, to update the voltage scaling value
	regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 360;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* Activate the Over-Drive mode */
	HAL_PWREx_ActivateOverDrive();

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}

#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
	case GPIO_PIN_15:
		encoder_right = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
		if (encoder_right == 0)
		{
			if (motorInterrupt1 == 0)
				motorInterrupt1 = 20000;
			motorInterrupt1--;
			encoder_right = 3;

		}
		else if (encoder_right == 1)
		{
			motorInterrupt1++;
			encoder_right = 3;
			if (motorInterrupt1 == 20000)
				motorInterrupt1 = 0;
		}
		break;

	case GPIO_PIN_4:
		encoder_left = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
		if (encoder_left == 0)
		{
			motorInterrupt2++;
			encoder_left = 3;
			if (motorInterrupt2 == 20000)
				motorInterrupt2 = 0;
		}
		else if (encoder_left == 1)
		{
			if (motorInterrupt2 == 0)
				motorInterrupt2 = 20000;
			motorInterrupt2--;
			encoder_left = 3;

		}
		break;
	}
}


/**
* @brief  Conversion complete callback in non blocking mode
* @param  htim : hadc handle
* @retval None
*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if ((TIM3->CCER & TIM_CCER_CC2P) == 0)
			{
				/* Get the 1st Input Capture value */
				uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				TIM3->CCER |= TIM_CCER_CC2P;
			}
			else if ((TIM3->CCER & TIM_CCER_CC2P) == TIM_CCER_CC2P)
			{
				/* Get the 2nd Input Capture value */
				uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

				/* Capture computation */
				if (uwIC2Value2 > uwIC2Value1)
				{
					uwDiffCapture1 = (uwIC2Value2 - uwIC2Value1);
				}
				else if (uwIC2Value2 < uwIC2Value1)
				{
					uwDiffCapture1 = ((0xFFFF - uwIC2Value1) + uwIC2Value2);
				}
				else
				{
					uwDiffCapture1 = 0;
				}

				uwFrequency = (2 * HAL_RCC_GetPCLK1Freq()) / uwDiffCapture1;
				TIM3->CCER &= ~TIM_CCER_CC2P;
			}
		}

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if ((TIM3->CCER & TIM_CCER_CC3P) == 0)
			{
				/* Get the 1st Input Capture value */
				uwIC2Value3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
				TIM3->CCER |= TIM_CCER_CC3P;
			}
			else if ((TIM3->CCER & TIM_CCER_CC3P) == TIM_CCER_CC3P)
			{
				/* Get the 2nd Input Capture value */
				uwIC2Value4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

				/* Capture computation */
				if (uwIC2Value4 > uwIC2Value3)
				{
					uwDiffCapture2 = (uwIC2Value4 - uwIC2Value3);
				}
				else if (uwIC2Value4 < uwIC2Value3)
				{
					uwDiffCapture2 = ((0xFFFF - uwIC2Value3) + uwIC2Value4);
				}
				else
				{
					uwDiffCapture2 = 0;
				}
				//printf("\r\n Value Forward : %d cm", uwDiffCapture2/58);

				uwFrequency = (2 * HAL_RCC_GetPCLK1Freq()) / uwDiffCapture2;
				TIM3->CCER &= ~TIM_CCER_CC3P;
			}
		}

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if ((TIM3->CCER & TIM_CCER_CC4P) == 0)
			{
				/* Get the 1st Input Capture value */
				uwIC2Value5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				TIM3->CCER |= TIM_CCER_CC4P;
			}
			else if ((TIM3->CCER & TIM_CCER_CC4P) == TIM_CCER_CC4P)
			{
				/* Get the 2nd Input Capture value */
				uwIC2Value6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

				/* Capture computation */
				if (uwIC2Value6 > uwIC2Value5)
				{
					uwDiffCapture3 = (uwIC2Value6 - uwIC2Value5);
				}
				else if (uwIC2Value6 < uwIC2Value5)
				{
					uwDiffCapture3 = ((0xFFFF - uwIC2Value5) + uwIC2Value6);
				}
				else
				{
					uwDiffCapture3 = 0;
				}
				//printf("\r\n Value Left: %d cm", uwDiffCapture3/58);

				uwFrequency = (2 * HAL_RCC_GetPCLK1Freq()) / uwDiffCapture3;
				TIM3->CCER &= ~TIM_CCER_CC4P;
			}
		}
	}
}

/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
static void Error_Handler(void)
{
	/* Turn LED3 on */
	BSP_LED_On(LED3);
	while (1)
	{
	}
}

/**
* @brief  Configures EXTI Line (connected to PA15, PB3, PB4, PB5 pin) in interrupt mode
* @param  None
* @retval None
*/
static void EXTILine_Config(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;

	/* Enable GPIOA clock */
	__GPIOA_CLK_ENABLE();

	/* Configure PA15 pin  */
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable and set EXTI Line15 Interrupt to the lowest priority */
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


	/* Enable GPIOB clock */
	__GPIOB_CLK_ENABLE();

	/* Configure PB3, PB4, PB5 pin  */
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Enable and set EXTI Line4 Interrupt to the lowest priority */
	HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/