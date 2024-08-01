/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t pin[] = {D0_Pin,D1_Pin,D2_Pin,D3_Pin,D4_Pin,D5_Pin,D6_Pin,D7_Pin};
uint8_t i=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void DataByte_Write(char data) ;
void LCD_Cmd(char cmd);
void LCD_Data(char data);
void LCD_Init();
void LCD_String(char *ptr);
uint8_t Read_Keypad( void );
void Calculator( void );
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
  /* USER CODE BEGIN 2 */
  LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Calculator();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, R1_Pin|R2_Pin|R3_Pin|R4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS_Pin|RW_Pin|EN_Pin|D0_Pin
                          |D1_Pin|D2_Pin|D3_Pin|D4_Pin
                          |D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : R1_Pin R2_Pin R3_Pin R4_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R2_Pin|R3_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin RW_Pin EN_Pin D0_Pin
                           D1_Pin D2_Pin D3_Pin D4_Pin
                           D5_Pin D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = RS_Pin|RW_Pin|EN_Pin|D0_Pin
                          |D1_Pin|D2_Pin|D3_Pin|D4_Pin
                          |D5_Pin|D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C1_Pin C2_Pin C3_Pin C4_Pin */
  GPIO_InitStruct.Pin = C1_Pin|C2_Pin|C3_Pin|C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void Calculator( void ) {

	char num1[10],num2[10],op;
	uint8_t  i=0,j=0;
	uint16_t result =0;
	HAL_GPIO_WritePin(GPIOC, C1_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, C2_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, C3_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, C4_Pin, 1);
	LCD_Cmd(0x80);
	LCD_String("Enter Num1:       ");

	while(1) {

		char key = Read_Keypad();
		if(key != 0){
			if(key >= '0' && key <= '9') {

				num1[i] = key;
				i++;
				//num1[i] = '\0';
				LCD_Cmd(0x8b);
				LCD_String(num1);
			}
			else if( key == '=') {
				 i=0;
				 break;
			}
		}
	}

	LCD_Cmd(0x80);
	LCD_String("Enter Op :       ");
	while(1) {

		char key = Read_Keypad();
		if(key != 0){
			if(key == '+' || key == '-' || key == '*' || key == '/') {

				op = key;
				LCD_Cmd(0x80);
				LCD_String("You Want:");
				LCD_Cmd(0x8b);
				LCD_Data(key);
				HAL_Delay(1000);
				break;
			}
		}
	}

	LCD_Cmd(0x80);
	LCD_String("Enter Num2 :     ");

	while(1) {

		char key = Read_Keypad();
		if(key >= '0' && key <= '9') {

			num2[j] = key;
			j++;
			//num1[j] = '\0';
			LCD_Cmd(0x8d);
			LCD_String(num2);
		}
		else if( key == '=') {
			 j=0;
			 break;
		}
	}

	int n1 = atoi(num1);
	int n2 = atoi(num2);

	switch(op) {

	case '+':
		result = n1+n2;
		break;

	case '-':
		result = n1-n2;
		break;
	case '*':
		result = n1*n2;
		break;
	case '/':
		result = n1/n2;
		break;
	default:
		break;
	}
	LCD_Cmd(0x01);
	char str[16];
	sprintf(str, "Result : %02d",result);
	LCD_Cmd(0x80);
	LCD_String(str);
	HAL_Delay(5000);

}
void DataByte_Write(char data) {

	for( i=0;i<8;i++) {

		HAL_GPIO_WritePin(GPIOA, pin[i], ((data>>i)&0x01) ? GPIO_PIN_SET:GPIO_PIN_RESET);
	}
}

void LCD_Cmd(char cmd) {

	DataByte_Write(cmd);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(2);
}

void LCD_Data(char data) {

	DataByte_Write(data);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(2);
}

void LCD_Init() {

	LCD_Cmd(0x01);
	LCD_Cmd(0x02);
	LCD_Cmd(0x06);
	LCD_Cmd(0x38);
	LCD_Cmd(0x0C);

}

void LCD_String(char *ptr) {

	while(*ptr) {
		LCD_Data(*ptr++);
	}
}

uint8_t Read_Keypad( void ) {

	HAL_GPIO_WritePin(GPIOC, R1_Pin, 0);
	HAL_GPIO_WritePin(GPIOC, R2_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, R3_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, R4_Pin, 1);
	HAL_Delay(20);
	if(HAL_GPIO_ReadPin(GPIOC, C1_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C1_Pin) == 0);
		return '7';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C2_Pin) == 0) {


		while(HAL_GPIO_ReadPin(GPIOC, C2_Pin) == 0);
		return '8';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C3_Pin) == 0) {


		while(HAL_GPIO_ReadPin(GPIOC, C3_Pin) == 0);
		return '9';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C4_Pin) == 0) {


		while(HAL_GPIO_ReadPin(GPIOC, C4_Pin) == 0);
		return '/';
	}




	HAL_GPIO_WritePin(GPIOC, R1_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, R2_Pin, 0);
	HAL_GPIO_WritePin(GPIOC, R3_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, R4_Pin, 1);
	HAL_Delay(20);
	if(HAL_GPIO_ReadPin(GPIOC, C1_Pin) ==0) {

		while(HAL_GPIO_ReadPin(GPIOC, C1_Pin) == 0);
		return '4';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C2_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C2_Pin) == 0);
		return '5';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C3_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C3_Pin) == 0);
		return '6';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C4_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C4_Pin) == 0);
		return '*';
	}





	HAL_GPIO_WritePin(GPIOC, R1_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, R2_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, R3_Pin, 0);
	HAL_GPIO_WritePin(GPIOC, R4_Pin, 1);
	HAL_Delay(20);
	if(HAL_GPIO_ReadPin(GPIOC, C1_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C1_Pin) == 0);
		return '1';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C2_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C2_Pin) == 0);
		return '2';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C3_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C3_Pin) == 0);
		return '3';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C4_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C4_Pin) == 0);
		return '-';
	}





	HAL_GPIO_WritePin(GPIOC, R1_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, R2_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, R3_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, R4_Pin, 0);
	HAL_Delay(20);
	if(HAL_GPIO_ReadPin(GPIOC, C1_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C1_Pin) == 0);
		return 'C';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C2_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C2_Pin) == 0);
		return '0';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C3_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C3_Pin) == 0);
		return '=';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C4_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C4_Pin) == 0);
		return '+';
	}
	return 0;
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
