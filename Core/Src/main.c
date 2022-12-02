/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NO_OF_TIMERS 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
const int MAX_LED_MATRIX = 8;
int index_col_led_matrix = 0;

enum status{
	RED,
	GREEN,
	MIX
};

enum direction{
	VER,
	VER_REV,
	HOR,
	HOR_REV
};

//uint8_t matrix_buffer[8] = {0x01, 0x06, 0x38, 0xC8, 0xC8, 0x38, 0x06, 0x01};
//
uint8_t matrix_buffer[8] = {0xff, 0xff,0x18,0x18,0x18,0x18,0xff,0xff};
uint8_t charA[8] = {0x00,0x06,0x38,0x48,0x48,0x38,0x06,0x00};
uint8_t charAR[8] = {0x81,0x42,0x42,0x3c,0x24,0x24,0x18,0x18};
uint8_t charH[8] = {0xff, 0xff,0x18,0x18,0x18,0x18,0xff,0xff};

GPIO_TypeDef * Port[8]={GPIOA,GPIOB,GPIOB,GPIOA,GPIOA,GPIOC,GPIOC,GPIOC};
uint16_t row_red_pin[8] = {Red1_Pin,Red2_Pin,Red3_Pin,Red4_Pin,Red5_Pin,Red6_Pin,Red7_Pin,Red8_Pin};
uint16_t row_green_pin[8] = {Green1_Pin,Green2_Pin,Green3_Pin,Green4_Pin,Green5_Pin,Green6_Pin,Green7_Pin,Green8_Pin};
uint16_t col_pin[8] = {EN_ROW1_Pin,EN_ROW2_Pin,EN_ROW3_Pin,EN_ROW4_Pin,EN_ROW5_Pin,EN_ROW6_Pin,EN_ROW7_Pin,EN_ROW8_Pin};


int TIMER_CYCLE;

int timer_counter[NO_OF_TIMERS];
int timer_flag[NO_OF_TIMERS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void clearLED(void);
void updateLEDMatrixBuffer(uint8_t *);
void updateLEDMatrix(int,enum status,enum direction);
void shiftLeftMatrix();
void shiftRightMatrix();
void shiftUpMatrix();
void shiftDownMatrix();

void setTimer(int, int);
void timer_run(void);
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  setTimer(0,TIMER_CYCLE);
  setTimer(1,TIMER_CYCLE);
  setTimer(2,TIMER_CYCLE);
  enum status curent_state = GREEN;
  enum direction flow= VER;
  updateLEDMatrixBuffer(charA);
  char is_First = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(timer_flag[0]){
		  timer_flag[0] = 0;
		  setTimer(0,200);
		  if(is_First) is_First = 0;
		  else{
			  shiftLeftMatrix();
//			  shiftUpMatrix();
//			  shiftDownMatrix();
//			  shiftRightMatrix();
		  }
	  }

	  if(timer_flag[1]) {
		  timer_flag[1] = 0;
		  setTimer(1,2);
		  updateLEDMatrix(index_col_led_matrix++,curent_state,flow);
		  if(index_col_led_matrix == MAX_LED_MATRIX) index_col_led_matrix = 0;
	  }
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
  htim2.Init.Prescaler = 799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|EN_ROW7_Pin|GPIO_PIN_1
                          |GPIO_PIN_2|EN_ROW8_Pin|GPIO_PIN_10|GPIO_PIN_11
                          |EN_ROW5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6
                          |EN_ROW1_Pin|EN_ROW4_Pin|GPIO_PIN_13|GPIO_PIN_14
                          |EN_ROW6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|EN_ROW3_Pin|GPIO_PIN_14
                          |GPIO_PIN_15|EN_ROW2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 EN_ROW7_Pin PC1
                           PC2 EN_ROW8_Pin PC10 PC11
                           EN_ROW5_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|EN_ROW7_Pin|GPIO_PIN_1
                          |GPIO_PIN_2|EN_ROW8_Pin|GPIO_PIN_10|GPIO_PIN_11
                          |EN_ROW5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA5 PA6
                           EN_ROW1_Pin EN_ROW4_Pin PA13 PA14
                           EN_ROW6_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6
                          |EN_ROW1_Pin|EN_ROW4_Pin|GPIO_PIN_13|GPIO_PIN_14
                          |EN_ROW6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 EN_ROW3_Pin PB14
                           PB15 EN_ROW2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|EN_ROW3_Pin|GPIO_PIN_14
                          |GPIO_PIN_15|EN_ROW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void clearLEDPin(uint16_t * pin,int enable){
	for(int i = 0 ; i < 8; i++){
		if(i == 0 || i == 3 || i == 4){
			HAL_GPIO_WritePin(GPIOA, pin[i], enable);
		}
		else if(i == 1 || i ==2){
			HAL_GPIO_WritePin(GPIOB, pin[i], enable);
		}
		else{
			HAL_GPIO_WritePin(GPIOC, pin[i], enable);
		}
	}
}

void clearLED(){
	clearLEDPin(row_red_pin,0);
	clearLEDPin(row_green_pin,0);
	clearLEDPin(col_pin,1);
}

void updateLEDMatrixBuffer(uint8_t * character){
	for(int i = 0 ; i < 8; i++){
		matrix_buffer[i] = character[i];
	}
}

void updatePin(enum status state_current,int index, int enable){
	switch(state_current){
	case RED:
		if(index == 0 || index == 3 || index == 4){
			HAL_GPIO_WritePin(GPIOA, row_red_pin[index], enable);
		}
		else if(index == 1 || index == 2){
			HAL_GPIO_WritePin(GPIOB, row_red_pin[index], enable);
		}
		else{
			HAL_GPIO_WritePin(GPIOC, row_red_pin[index], enable);
		}
		break;
	case GREEN:
		if(index == 0 || index == 3 || index == 4){
			HAL_GPIO_WritePin(GPIOA, row_green_pin[index], enable);
		}
		else if(index == 1 || index == 2){
			HAL_GPIO_WritePin(GPIOB, row_green_pin[index], enable);
		}
		else{
			HAL_GPIO_WritePin(GPIOC, row_green_pin[index], enable);
		}
		break;
	default: break;
	}
}

void updateColPin(int index,int enable){
	if(index == 0 || index == 3 || index == 4){
		HAL_GPIO_WritePin(GPIOA, col_pin[index], enable);
	}
	else if(index == 1 || index == 2){
		HAL_GPIO_WritePin(GPIOB, col_pin[index], enable);
	}
	else{
		HAL_GPIO_WritePin(GPIOC, col_pin[index], enable);
	}
}

void updateLEDMatrix(int index, enum status state_current,enum direction flow) {
	clearLED();
	int flag_led = 0;
	switch(state_current){
	case RED:
		switch(flow){
		case VER:
		case HOR:
			for(int i = 0; i < 8; i++) {
				if(matrix_buffer[index] & (0x80 >> i)) {
//					updatePin(RED,i,1);
					HAL_GPIO_WritePin(Port[i],row_red_pin[i],1);
				}
			}
			break;
		case VER_REV:
			for(int i = 0; i < 8; i++) {
				if(matrix_buffer[index] & (0x01 << i)) {
					updatePin(RED,i,1);
					HAL_GPIO_WritePin(Port[i],row_red_pin[i],1);
				}
			}
			break;
		case HOR_REV:
			for(int i = 0; i < 8; i++) {
				if(matrix_buffer[7-index] & (0x80 >> i)) {
//					updatePin(RED,i,1);
					HAL_GPIO_WritePin(Port[i],row_red_pin[i],1);
				}
			}
			break;
		}
		break;
	case GREEN:
		switch(flow){
		case VER:
		case HOR:
			for(int i = 0; i < 8; i++) {
				if(matrix_buffer[index] & (0x80 >> i)) {
//					updatePin(GREEN,i,1);
					HAL_GPIO_WritePin(Port[i],row_green_pin[i],1);
				}
			}
			break;
		case VER_REV:
			for(int i = 0; i < 8; i++) {
				if(matrix_buffer[index] & (0x01 << i)) {
//					updatePin(GREEN,i,1);
					HAL_GPIO_WritePin(Port[i],row_green_pin[i],1);
				}
			}
			break;
		case HOR_REV:
			for(int i = 0; i < 8; i++) {
				if(matrix_buffer[7-index] & (0x80 >> i)) {
//					updatePin(GREEN,i,1);
					HAL_GPIO_WritePin(Port[i],row_green_pin[i],1);
				}
			}
			break;
		}
		break;
	case MIX:
		switch(flow){
		case VER:
		case HOR:
			for(int i = 0; i < 8; i++) {
				if(matrix_buffer[index] & (0x80 >> i)) {
					if(flag_led){
//						updatePin(GREEN,i,1);
						HAL_GPIO_WritePin(Port[i],row_green_pin[i],1);
						flag_led = 0;
					}
					else{
//						updatePin(RED,i,1);
						HAL_GPIO_WritePin(Port[i],row_red_pin[i],1);
						flag_led = 1;
					}
				}
			}
			break;
		case VER_REV:
			for(int i = 0; i < 8; i++) {
				if(matrix_buffer[index] & (0x01 << i)) {
					if(flag_led){
//						updatePin(GREEN,i,1);
						HAL_GPIO_WritePin(Port[i],row_green_pin[i],1);
						flag_led = 0;
					}
					else{
//						updatePin(RED,i,1);
						HAL_GPIO_WritePin(Port[i],row_red_pin[i],1);
						flag_led = 1;
					}
				}
			}
			break;
		case HOR_REV:
			for(int i = 0; i < 8; i++) {
				if(matrix_buffer[7-index] & (0x80 >> i)) {
					if(flag_led){
//						updatePin(GREEN,i,1);
						HAL_GPIO_WritePin(Port[i],row_green_pin[i],1);
						flag_led = 0;
					}
					else{
//						updatePin(RED,i,1);
						HAL_GPIO_WritePin(Port[i],row_red_pin[i],1);
						flag_led = 1;
					}
				}
			}
			break;
		}
		break;
	default: break;
	}
//	updateColPin(index,0);
	HAL_GPIO_WritePin(Port[index],col_pin[index],0);
}

void shiftRightMatrix() {
	uint8_t old = matrix_buffer[0];
	for(int i = 0 ; i < 8; i++)  matrix_buffer[i] = matrix_buffer[i+1];
	matrix_buffer[7] = old;
}

void shiftLeftMatrix(){
	uint8_t old = matrix_buffer[7];
	for(int i = 7; i > 0; i--) matrix_buffer[i] = matrix_buffer[i-1];
	matrix_buffer[0] = old;
}

void shiftUpMatrix(){
	for(int i = 0; i < 8; i++){
		int b = matrix_buffer[i] & 0x01;
		matrix_buffer[i] >>= 1;
		if(b == 1) matrix_buffer[i] = matrix_buffer[i] | (1 << 7);
		else matrix_buffer[i] = matrix_buffer[i] & ~(1 << 7);
	}
}

void shiftDownMatrix(){
	for(int i = 0; i < 8; i++){
		int b = matrix_buffer[i] & 0x80;
		matrix_buffer[i] <<= 1;
		if(b == 128) matrix_buffer[i] = matrix_buffer[i] | (1 << 0);
		else matrix_buffer[i] = matrix_buffer[i] & ~(1 << 0);
	}
}

void setTimer(int timer, int duration) {
	timer_counter[timer] = duration / TIMER_CYCLE;
	timer_flag[timer] = 0;
}

void timer_run() {
	for(int i = 0; i < NO_OF_TIMERS; i++) {
		if(timer_counter[i] > 0) {
			timer_counter[i]--;
			if(timer_counter[i] == 0) {
				timer_flag[i] = 1;
			}
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim) {
	timer_run();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
