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
	ModeAdjust
}ModeEnum;
uint8_t Mode = ModeIdle;

enum
{
	AdjustHour = 0,
	AdjustMin,
	AdjustSec,
	AdjustYear,
	AdjustMonth,
	AdjustDate,
	AdjustFinish
}AdjustEnum;
uint8_t Adjust = AdjustHour;

uint8_t tim2_flag = 0;// flag 1s 
uint8_t alarm_flag = 0;// flag check status alarm
uint8_t finish_flag = 0;// flag finish adjust time and date

uint8_t isSetAlarm = 0;
uint8_t isSetTime = 0;
uint8_t isChangeDisplay = 0;

uint8_t count_5s = 0;// count 5s for off alarm

uint8_t device;
DS3231_Name DS3231;
CLCD_I2C_Name LCD1;
char buffer[16];
char ASCII[10];
float Temp;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIME_OFF_ALARM 5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void BCD2ASCII(uint8_t bcd_value, char * p_ascii_text)
{
  *p_ascii_text++ = (bcd_value >> 4)  + '0';
  *p_ascii_text++ = (bcd_value & 0x0f) + '0';
	*p_ascii_text = '\0';
  return;
}
void MonthPrint(uint8_t Month)
{
	if (Month == 1)
		sprintf(ASCII, "January");
	else if (Month == 2)
		sprintf(ASCII, "Febnuary");
	else if (Month == 3)
		sprintf(ASCII, "March");
	else if (Month == 4)
		sprintf(ASCII, "April");
	else if (Month == 5)
		sprintf(ASCII, "May");
	else if (Month == 6)
		sprintf(ASCII, "June");
	else if (Month == 7)
		sprintf(ASCII, "July");
	else if (Month == 8)
		sprintf(ASCII, "August");
	else if (Month == 9)
		sprintf(ASCII, "September");
	else if (Month == 10)
		sprintf(ASCII, "October");
	else if (Month == 11)
		sprintf(ASCII, "November");
	else if (Month == 12)
		sprintf(ASCII, "December");
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
	DS3231_GetTime(&DS3231);
	sprintf(buffer, "%02d:%02d:%02d", DS3231.Hours, DS3231.Min, DS3231.Sec);
	CLCD_I2C_SetCursor(&LCD1, 0, 1);
	CLCD_I2C_WriteString(&LCD1, buffer);
}


void Display_Date()
{
	DS3231_GetDate(&DS3231);
	//MonthPrint(DS3231.Month);	
	sprintf(buffer, "%02d,%02d,20%02d", DS3231.Date, DS3231.Month, DS3231.Year);
	CLCD_I2C_SetCursor(&LCD1, 0, 1);
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
	CLCD_I2C_CursorOn(&LCD1);
	switch(Adjust)
	{
		case AdjustHour:
			CLCD_I2C_SetCursor(&LCD1, 0, 1);			
			break;
		case AdjustMin:
			CLCD_I2C_SetCursor(&LCD1, 3, 1);
			break;
		case AdjustSec:
			CLCD_I2C_SetCursor(&LCD1, 6, 1);
			break;
		case AdjustYear:
			CLCD_I2C_SetCursor(&LCD1, 6, 1);
			break;
		case AdjustMonth:
			CLCD_I2C_SetCursor(&LCD1, 3, 1);
			break;
		case AdjustDate:
			CLCD_I2C_SetCursor(&LCD1, 0, 1);
			break;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  if(GPIO_Pin == Nav_Pin)
	{
		switch(Adjust)
		{
			case AdjustHour:
				if( DS3231.Hours > 0)
					DS3231.Hours -= 1;
				else DS3231.Hours = 23;
				sprintf(buffer, "%02d", DS3231.Hours);
				break;
			case AdjustMin:
				if(DS3231.Min > 0)
					DS3231.Min -= 1;
				else DS3231.Min = 59;
				sprintf(buffer, "%02d", DS3231.Min);
				break;
			case AdjustSec:
				if(DS3231.Sec > 0)
					DS3231.Sec -= 1;
				else DS3231.Sec = 59;
				sprintf(buffer, "%02d", DS3231.Sec);
				break;
			case AdjustYear:
				DS3231.Year -= 1;
				sprintf(buffer,"20%02d", DS3231.Year);
				break;
			case AdjustMonth:
				if(DS3231.Month > 1)
					DS3231.Month -= 1;
				else DS3231.Month = 12;
				sprintf(buffer,"%02d", DS3231.Month);
				break;
			case AdjustDate:
				if(DS3231.Date > 1)
					DS3231.Date -= 1;
				else DS3231.Date = 31;
				sprintf(buffer,"%02d", DS3231.Day);
				break;
		}
		CLCD_I2C_WriteString(&LCD1, buffer);
	}
	if(GPIO_Pin == Inc_Pin)
	{
		switch(Adjust)
		{
			case AdjustHour:
				if( DS3231.Hours < 23)
					DS3231.Hours += 1;
				else DS3231.Hours = 0;
				sprintf(buffer, "%02d", DS3231.Hours);
				break;
			case AdjustMin:
				if(DS3231.Min < 59)
					DS3231.Min += 1;
				else DS3231.Min = 0;
				sprintf(buffer, "%02d", DS3231.Min);
				break;
			case AdjustSec:
				if(DS3231.Sec < 59)
					DS3231.Sec += 1;
				else DS3231.Sec = 0;
				sprintf(buffer, "%02d", DS3231.Sec);
				break;
			case AdjustYear:
				DS3231.Year += 1;
				sprintf(buffer,"20%02d", DS3231.Year);
				break;
			case AdjustMonth:
				if(DS3231.Month < 12)
					DS3231.Month += 1;
				else DS3231.Month = 1;
				sprintf(buffer,"%02d", DS3231.Month);
				break;
			case AdjustDate:
				if(DS3231.Date < 31)
					DS3231.Date += 1;
				else DS3231.Date = 1;
				sprintf(buffer,"%02d", DS3231.Day);
				break;
		}
		CLCD_I2C_WriteString(&LCD1, buffer);
	}
	if(GPIO_Pin == Mode_Pin)
	{
		switch(Mode)
		{
			case ModeIdle:
				Mode = ModeSetAlarm;
				break;
			case ModeSetAlarm:
				Mode = ModeSetTime;
				break;
			case ModeSetTime:
				Mode = ModeChangeDisplay;
				break;
			case ModeChangeDisplay:
				Mode = ModeIdle;
				break;
			case ModeAdjust:
				finish_flag = 1;
				break;
		}
	}
	if(GPIO_Pin == Alarm_Pin)
	{
		Mode = ModeAlarm;
	}
	if(GPIO_Pin == OK_Pin)
	{
		switch(Mode)
		{
			case ModeAlarm:
				alarm_flag = 1;
				break;
			case ModeSetAlarm:
				isSetAlarm = 1;
				break;
			case ModeSetTime:
				isSetTime = 1;
				break;
			case ModeChangeDisplay:
				isChangeDisplay = 1;
				break;
			case ModeAdjust:
				Adjust += 1;					
				if( Adjust == AdjustYear)
					Display_Date();
				if( Adjust == AdjustFinish)
					finish_flag = 1;
				break;
		}
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)/*Ham timer ngat*/
{
  UNUSED(htim);
	switch(Mode)
	{
		case ModeIdle:
			tim2_flag = 1;
			break;
		case ModeAlarm:
			count_5s += 1;
			if(count_5s == TIME_OFF_ALARM)
			{
				alarm_flag = 1;
				count_5s = 0;
			}
			break;
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
	HAL_TIM_Base_Start_IT(&htim2);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		switch( Mode)
		{
			case ModeIdle:
				if(tim2_flag)
				{
						DS3231_GetTime(&DS3231);
						sprintf(buffer, "%02d:%02d:%02d", DS3231.Hours, DS3231.Min, DS3231.Sec);
						CLCD_I2C_SetCursor(&LCD1, 0, 0);
						CLCD_I2C_WriteString(&LCD1, buffer);
						
						DS3231_GetDate(&DS3231);
						MonthPrint(DS3231.Month);	
						sprintf(buffer, "%02d,%s,20%02d", DS3231.Date, ASCII, DS3231.Year);
						CLCD_I2C_SetCursor(&LCD1, 0, 1);
						CLCD_I2C_WriteString(&LCD1, buffer);
							
						Temp = DS3231_GetTemp(&DS3231);
						CLCD_I2C_SetCursor(&LCD1, 10, 0);
						sprintf(buffer, "%.1f", Temp);
						CLCD_I2C_WriteString(&LCD1, buffer);
						CLCD_I2C_WriteChar(&LCD1, 223);
						CLCD_I2C_WriteChar(&LCD1, 67);
					tim2_flag = 0;
				}
				break;
			case ModeSetTime:
				Display_SetTime();
				if(isSetTime)
				{
					Mode = ModeAdjust;
					Display_Time();
					Adjust = AdjustHour;
				}
				break;
			case ModeChangeDisplay:
				Display_ChangeDisplay();
				if(isChangeDisplay)
				{
					// change display func
				}
				break;
			case ModeSetAlarm:
				Display_SetAlarm();
				if(isSetAlarm)
				{
					Mode = ModeAdjust;
					Display_Time();
					Adjust = AdjustHour;
				}
				break;
			case ModeAlarm:
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
				if(alarm_flag)
				{
					HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
					alarm_flag = 0;
					Mode = ModeIdle;
				}
				break;
			case ModeAdjust:
				if(isSetAlarm)
				{					
					Display_Adjust();
					if(finish_flag)
					{
						//set alarm time and date func
						DS3231_SetAlarmTime(&DS3231, DS3231.Hours, DS3231.Min, DS3231.Sec);
						DS3231_SetAlarmDate(&DS3231, DS3231.Day, DS3231.Date);
						Mode = ModeSetTime;
						finish_flag = 0;
						isSetAlarm = 0;
					}
				}
				if(isSetTime)
				{
					Display_Adjust();
					if(finish_flag)
					{
						//set time and date func
						DS3231_SetTime(&DS3231, DS3231.Hours, DS3231.Min, DS3231.Sec);
						DS3231_SetDate(&DS3231, DS3231.Day, DS3231.Date, DS3231.Month, DS3231.Year);
						Mode = ModeChangeDisplay;
						finish_flag = 0;
						isSetTime = 0;
					}
				}
				break;
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  htim2.Init.Prescaler = 8000;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
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

  /*Configure GPIO pins : Mode_Pin OK_Pin Inc_Pin Nav_Pin */
  GPIO_InitStruct.Pin = Mode_Pin|OK_Pin|Inc_Pin|Nav_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Alarm_Pin */
  GPIO_InitStruct.Pin = Alarm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Alarm_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
