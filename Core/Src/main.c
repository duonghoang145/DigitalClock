/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CLCD_I2C.h"
#include "DS3231.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum
{
	ModeIdle = 0,
	ModeSetAlarm,
	ModeSetTime,
	ModeChangeDisplay,
	ModeAlarm,
	ModeAdjustAlarm,
	ModeAdjustTime
}ModeEnum;
uint8_t Mode = ModeIdle;

enum
{
	Mode24h = 0,
	Mode12h
}ModeDisplayEnum;
uint8_t ModeDisplay = Mode24h;

enum
{
	AdjustHour = 0,
	AdjustMin,
	AdjustSec,
	AdjustDate,
	AdjustMonth,
	AdjustYear,
}AdjustEnum;
uint8_t Adjust = -1;

uint8_t tim2_flag = 0;// flag 1s 
uint8_t alarm_flag = 0;// flag check status alarm
uint8_t finish_flag = 0;// flag finish adjust time and date

uint8_t isChangeDisplay = 0;

uint8_t count_5s = 0;// count 5s for off alarm

uint8_t AlarmHours;
uint8_t AlarmMin;
uint8_t AlarmDate;

uint8_t device;
DS3231_Name DS3231;
CLCD_I2C_Name LCD1;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void MonthPrint(uint8_t Month, char bufferm [16])
{
	if (Month == 1)
		sprintf(bufferm, "January");
	else if (Month == 2)
		sprintf(bufferm, "Febnuary");
	else if (Month == 3)
		sprintf(bufferm, "March");
	else if (Month == 4)
		sprintf(bufferm, "April");
	else if (Month == 5)
		sprintf(bufferm, "May");
	else if (Month == 6)
		sprintf(bufferm, "June");
	else if (Month == 7)
		sprintf(bufferm, "July");
	else if (Month == 8)
		sprintf(bufferm, "August");
	else if (Month == 9)
		sprintf(bufferm, "September");
	else if (Month == 10)
		sprintf(bufferm, "October");
	else if (Month == 11)
		sprintf(bufferm, "November");
	else if (Month == 12)
		sprintf(bufferm, "December");
}

void Display_SetAlarm()
{
	CLCD_I2C_Clear(&LCD1);
	CLCD_I2C_SetCursor(&LCD1, 0, 0);
	CLCD_I2C_WriteString(&LCD1,"Set Alarm");
}

void Display_SetTime()
{
	CLCD_I2C_Clear(&LCD1);
	CLCD_I2C_SetCursor(&LCD1, 0, 0);
	CLCD_I2C_WriteString(&LCD1,"Set Time");
}

void Display_Time()
{
	char buffer[16];
	switch(ModeDisplay){
		case(Mode24h):
			DS3231_GetTime(&DS3231);
		sprintf(buffer, "%02d:%02d:%02d", DS3231.Hours, DS3231.Min, DS3231.Sec);
		CLCD_I2C_SetCursor(&LCD1, 0, 0);
		CLCD_I2C_WriteString(&LCD1, buffer);
		break;
		case(Mode12h):
			DS3231_GetTime(&DS3231);
		if(DS3231.Hours < 12){
			sprintf(buffer, "%02d:%02d:%02d AM", DS3231.Hours, DS3231.Min, DS3231.Sec);
		} 
		else if(DS3231.Hours == 12){
			DS3231_SetTime(&DS3231, DS3231.Hours, DS3231.Min, DS3231.Sec);
			sprintf(buffer, "%02d:%02d:%02d PM", DS3231.Hours, DS3231.Min, DS3231.Sec);
		}
		else if(DS3231.Hours > 12){
			DS3231_SetTime(&DS3231, DS3231.Hours - 12, DS3231.Min, DS3231.Sec);
			sprintf(buffer, "%02d:%02d:%02d PM", DS3231.Hours, DS3231.Min, DS3231.Sec);
		}
		else if(DS3231.Hours == 24){
			DS3231_SetTime(&DS3231, DS3231.Hours - 12, DS3231.Min, DS3231.Sec);
			sprintf(buffer, "%02d:%02d:%02d AM", DS3231.Hours, DS3231.Min, DS3231.Sec);
		}
	CLCD_I2C_SetCursor(&LCD1, 0, 0);
	CLCD_I2C_WriteString(&LCD1, buffer);
	}
}

void Display_Date()
{
	char buffer[16], bufferm[16];
	DS3231_GetDate(&DS3231);
	MonthPrint(DS3231.Month, bufferm);	
	sprintf(buffer, "%02d,%s,20%02d", DS3231.Date, bufferm, DS3231.Year);
	CLCD_I2C_SetCursor(&LCD1, 0, 1);
	CLCD_I2C_WriteString(&LCD1, buffer);
}

void Display_Temp()
{
	char buffer[16];
	uint8_t Temp;
	Temp = (int)DS3231_GetTemp(&DS3231);
	CLCD_I2C_SetCursor(&LCD1, 12, 0);
	sprintf(buffer, "%02d", Temp);
	CLCD_I2C_WriteString(&LCD1, buffer);
  CLCD_I2C_WriteChar(&LCD1, 223);
	CLCD_I2C_WriteChar(&LCD1, 67);
}

void Display_AlarmMode()
{
	char buffer[16];
	CLCD_I2C_SetCursor(&LCD1,0,0);
	sprintf(buffer, "Time %02d:%02d", AlarmHours, AlarmMin);
	CLCD_I2C_WriteString(&LCD1, buffer);
	CLCD_I2C_SetCursor(&LCD1,0,1);
	sprintf(buffer, "Date %02d", AlarmDate);
	CLCD_I2C_WriteString(&LCD1, buffer);
}

void Display_TimeMode()
{
	char buffer[16];
	CLCD_I2C_SetCursor(&LCD1,0,0);
	sprintf(buffer, "Time %02d:%02d:%02d", DS3231.Hours, DS3231.Min, DS3231.Sec);
	CLCD_I2C_WriteString(&LCD1, buffer);
	CLCD_I2C_SetCursor(&LCD1,0,1);
	sprintf(buffer, "Date %02d-%02d-%02d", DS3231.Date, DS3231.Month, DS3231.Year);
	CLCD_I2C_WriteString(&LCD1, buffer);
}

void Display_ChangeDisplay()
{
	CLCD_I2C_Clear(&LCD1);
	CLCD_I2C_SetCursor(&LCD1, 0, 0);
	CLCD_I2C_WriteString(&LCD1,"Change Display");
}

void Display_Adjust()
{
	switch(Adjust)
	{
		case AdjustHour:
			CLCD_I2C_SetCursor(&LCD1, 5, 0);			
			break;
		case AdjustMin:
			CLCD_I2C_SetCursor(&LCD1, 8, 0);
			break;
		case AdjustSec:
			CLCD_I2C_SetCursor(&LCD1, 11, 0);
			break;
		case AdjustDate:
			CLCD_I2C_SetCursor(&LCD1, 5, 1);
			break;
			case AdjustMonth:
			CLCD_I2C_SetCursor(&LCD1, 8, 1);
			break;
		case AdjustYear:
			CLCD_I2C_SetCursor(&LCD1, 11, 1);
			break;
	}
}

void Display_Alarm()
{
	char buffer[16];
	CLCD_I2C_SetCursor(&LCD1,5,0);
	CLCD_I2C_WriteString(&LCD1, "Alarm");
	CLCD_I2C_SetCursor(&LCD1,5,1);
	sprintf(buffer, "%02d:%02d", DS3231.Hours, DS3231.Min);
	CLCD_I2C_WriteString(&LCD1, buffer);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
	
	if(GPIO_Pin == Menu_Pin)
	{
		switch(Mode)
		{
			case ModeIdle:
		    Mode = ModeSetAlarm;
			  Display_SetAlarm();
			break;
			case ModeSetAlarm:
			  Mode = ModeSetTime;
				Display_SetTime();
			break;
			case ModeSetTime:
				Mode = ModeChangeDisplay;
			  Display_ChangeDisplay();
			break;
			case ModeChangeDisplay:
				Mode = ModeIdle;
			CLCD_I2C_Clear(&LCD1);
			break;
			case ModeAdjustAlarm:
				CLCD_I2C_BlinkOff(&LCD1);
					Mode = ModeIdle;
			CLCD_I2C_Clear(&LCD1);
			break;
			case ModeAdjustTime:
			CLCD_I2C_BlinkOff(&LCD1);
			DS3231_SetTime(&DS3231, DS3231.Hours, DS3231.Min, DS3231.Sec);
			DS3231_SetDate(&DS3231, DS3231.Day, DS3231.Date, DS3231.Month, DS3231.Year);
				Mode = ModeIdle;
			CLCD_I2C_Clear(&LCD1);
			Display_Time();
			Display_Temp();
			Display_Date();
			break;
		}
	}
	if(GPIO_Pin == Enter_Pin)
	{
		switch(Mode)
		{
			case ModeSetAlarm:
			Mode = ModeAdjustAlarm;
			CLCD_I2C_Clear(&LCD1);
			Display_AlarmMode();
			Adjust = AdjustHour;
	    CLCD_I2C_BlinkOn(&LCD1);
			Display_Adjust();
			break;
			case ModeSetTime:
			Mode = ModeAdjustTime;
			CLCD_I2C_Clear(&LCD1);
			Display_TimeMode();
			Adjust = AdjustHour;
	    CLCD_I2C_BlinkOn(&LCD1);
			Display_Adjust();
			break;
			case ModeChangeDisplay:
			if (ModeDisplay == Mode24h)
				ModeDisplay = Mode12h;
			else
				ModeDisplay = Mode24h;
			Mode = ModeIdle;
			CLCD_I2C_Clear(&LCD1);
			break;
			case ModeAdjustAlarm:	
			switch(Adjust)
			{
				case AdjustHour:
					Adjust = AdjustMin;
				break;
				case AdjustMin:
					Adjust = AdjustDate;
				break;
				case AdjustDate:
					Adjust = AdjustHour;
				break;
			}
			Display_Adjust();	
			break;
			case ModeAdjustTime:	
			if (Adjust == AdjustYear)	
			Adjust = -1;
			Adjust++;
			Display_Adjust();	
			break;
			case ModeAlarm:
				alarm_flag = 1;
			break;
		}
	}
	if(GPIO_Pin == Up_Pin)
	{
		if(Mode == ModeAdjustAlarm){
		switch(Adjust)
		{
			case AdjustHour:
				if(AlarmHours < 23)
					AlarmHours += 1;
				else AlarmHours = 0;
				
			Display_AlarmMode();
			Adjust = AdjustHour;
			Display_Adjust();
			break;
				
			case AdjustMin:
				if(AlarmMin < 59)
					AlarmMin += 1;
				else AlarmMin = 0;
				
			Display_AlarmMode();
			Adjust = AdjustMin;
			Display_Adjust();
			break;
				
			case AdjustDate:
				if(AlarmDate < 31)
					AlarmDate += 1;
				else AlarmDate = 1;
				
			Display_AlarmMode();
			Adjust = AdjustDate;
			Display_Adjust();
			break;
		}
	}
		if(Mode == ModeAdjustTime){
		switch(Adjust)
		{
			case AdjustHour:
				if(DS3231.Hours < 23)
					DS3231.Hours += 1;
				else DS3231.Hours = 0;
				
				Display_TimeMode();
			  Adjust = AdjustHour;
				Display_Adjust();
				break;
				
			case AdjustMin:
				if(DS3231.Min < 59)
					DS3231.Min += 1;
				else DS3231.Min = 0;
				
				Display_TimeMode();
				Adjust = AdjustMin;
				Display_Adjust();
				break;
				
			case AdjustSec:
				if(DS3231.Sec < 59)
					DS3231.Sec += 1;
				else DS3231.Sec = 0;
				
			Display_TimeMode();
			Adjust = AdjustSec;
				Display_Adjust();
				break;
				
			case AdjustYear:
			if (DS3231.Year < 99)	
			DS3231.Year += 1;
			else
			DS3231.Year = 0;
				
			Display_TimeMode();
			Adjust = AdjustYear;
				Display_Adjust();
				break;
			
			case AdjustMonth:
				if(DS3231.Month < 12)
					DS3231.Month += 1;
				else DS3231.Month = 1;
				
			Display_TimeMode();
			Adjust = AdjustMonth;
				Display_Adjust();
				break;
				
			case AdjustDate:
				if(DS3231.Date < 31)
					DS3231.Date += 1;
				else DS3231.Date = 1;
				
		  Display_TimeMode();
			Adjust = AdjustDate;
				Display_Adjust();
				break;
		}
	}
}
	if(GPIO_Pin == Down_Pin)
	{
		if(Mode == ModeAdjustAlarm){
		switch(Adjust)
		{
			case AdjustHour:
				if(AlarmHours > 0)
					AlarmHours -= 1;
				else AlarmHours = 23;
				
			Display_AlarmMode();
			Adjust = AdjustHour;
				Display_Adjust();
				break;
				
			case AdjustMin:
				if(AlarmMin > 0)
					AlarmMin -= 1;
				else AlarmMin = 59;
				
			Display_AlarmMode();
			Adjust = AdjustMin;
				Display_Adjust();
				break;
				
			case AdjustDate:
				if(AlarmDate > 1)
					AlarmDate -= 1;
				else AlarmDate = 31;
				
			Display_AlarmMode();
			Adjust = AdjustDate;
				Display_Adjust();
				break;
		}
	}
		if(Mode == ModeAdjustTime){
		switch(Adjust)
		{
			case AdjustHour:
				if( DS3231.Hours > 0)
					DS3231.Hours -= 1;
				else DS3231.Hours = 23;
				
			Display_TimeMode();
			Adjust = AdjustHour;
				Display_Adjust();
				break;
				
			case AdjustMin:
				if(DS3231.Min > 0)
					DS3231.Min -= 1;
				else DS3231.Min = 59;
				
			Display_TimeMode();
			Adjust = AdjustMin;
				Display_Adjust();
				break;
				
			case AdjustSec:
				if(DS3231.Sec > 0)
					DS3231.Sec -= 1;
				else DS3231.Sec = 59;
					
			Display_TimeMode();
			Adjust = AdjustSec;
				Display_Adjust();
				break;
				
			case AdjustYear:
			if(DS3231.Year > 0)	
			DS3231.Year -= 1;
			else 
				DS3231.Year = 99;
					
			Display_TimeMode();
			Adjust = AdjustYear;
				Display_Adjust();
				break;
			
			case AdjustMonth:
				if(DS3231.Month > 1)
					DS3231.Month -= 1;
				else DS3231.Month = 12;
					
			Display_TimeMode();
			Adjust = AdjustMonth;
				Display_Adjust();
				break;
				
			case AdjustDate:
				if(DS3231.Date > 1)
					DS3231.Date -= 1;
				else DS3231.Date = 31;
					
			Display_TimeMode();
			Adjust = AdjustDate;
				Display_Adjust();
				break;
			}
		}
	}
}
void AlarmTone(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
		if(alarm_flag){
			alarm_flag = 0;
			break;
		}
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
		HAL_Delay(150);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
		HAL_Delay(70);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
		HAL_Delay(150);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
		HAL_Delay(500);
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)/*Ham timer ngat*/
{
  UNUSED(htim);
	if(Mode == ModeIdle)
	{
		Display_Time();
		Display_Temp();
		Display_Date();
		if (DS3231.Hours == AlarmHours && DS3231.Min == AlarmMin && DS3231.Sec == 0 && DS3231.Date == AlarmDate)
			{
			  Mode = ModeAlarm;
				CLCD_I2C_Clear(&LCD1);
				Display_Alarm();
				AlarmTone(5000);
			  Mode = ModeIdle;
				CLCD_I2C_Clear(&LCD1);
			}
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	__disable_irq();
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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	for(int i = 0; i<255; i++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c1,i,1,100) == HAL_OK)
		{	
			device = i;
		}
	}
	
	DS3231_Init(&DS3231, &hi2c1);
	CLCD_I2C_Init(&LCD1,&hi2c1,0x4E,16,2);
	__enable_irq();
	HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Menu_Pin Enter_Pin Up_Pin Down_Pin */
  GPIO_InitStruct.Pin = Menu_Pin|Enter_Pin|Up_Pin|Down_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
