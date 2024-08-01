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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROWS 4
#define COLS 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t Data_Pins[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void LCD_Initialize( void );
void LCD_WriteCommand(uint8_t cmd);
void LCD_WriteData(uint8_t data);
void LCD_WriteString(const char *ptr);

uint8_t  Get_Key( void );
uint8_t Read_Keypad( void );
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Data_Pins[0] = D0_Pin;
  Data_Pins[1] = D1_Pin;
  Data_Pins[2] = D2_Pin;
  Data_Pins[3] = D3_Pin;
  Data_Pins[4] = D4_Pin;
  Data_Pins[5] = D5_Pin;
  Data_Pins[6] = D6_Pin;
  Data_Pins[7] = D7_Pin;



  LCD_Initialize();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOC, C1_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, C2_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, C3_Pin, 1);
	  HAL_GPIO_WritePin(GPIOC, C4_Pin, 1);

	  uint8_t key = Read_Keypad();
	  LCD_WriteCommand(0x80);
	  LCD_WriteString("Key Prassed : ");
	  if(key != 0) {
		  LCD_WriteCommand(0xC0);
		  LCD_WriteData(key);

	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, R1_Pin|R2_Pin|R3_Pin|R4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D0_Pin|D1_Pin|D2_Pin|EN_Pin
                          |D3_Pin|D4_Pin|D5_Pin|D6_Pin
                          |D7_Pin|RS_Pin|Rw_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin R2_Pin R3_Pin R4_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R2_Pin|R3_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : C1_Pin C2_Pin C3_Pin C4_Pin */
  GPIO_InitStruct.Pin = C1_Pin|C2_Pin|C3_Pin|C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : D0_Pin D1_Pin D2_Pin EN_Pin
                           D3_Pin D4_Pin D5_Pin D6_Pin
                           D7_Pin RS_Pin Rw_Pin */
  GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D2_Pin|EN_Pin
                          |D3_Pin|D4_Pin|D5_Pin|D6_Pin
                          |D7_Pin|RS_Pin|Rw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void LCD_Initialize( void ){

	HAL_Delay(100);

	LCD_WriteCommand(0x38);
	LCD_WriteCommand(0x38);
	LCD_WriteCommand(0x38);
	LCD_WriteCommand(0x0C);
	LCD_WriteCommand(0x06);
	LCD_WriteCommand(0x01);
	LCD_WriteCommand(0x80);
}
void LCD_WriteCommand(uint8_t cmd){

	HAL_GPIO_WritePin(GPIOB, RS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Rw_Pin, GPIO_PIN_RESET);

	for(int8_t i = 0;i<8;i++){

		HAL_GPIO_WritePin(GPIOB, Data_Pins[i], ((cmd >> i)&0x01)?GPIO_PIN_SET : GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(GPIOB, EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, EN_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}
void LCD_WriteData(uint8_t data)
{

	HAL_GPIO_WritePin(GPIOB, RS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, Rw_Pin, GPIO_PIN_RESET);

	for(int8_t i = 0;i<8;i++){

		HAL_GPIO_WritePin(GPIOB, Data_Pins[i], ((data >> i)&0x01)?GPIO_PIN_SET : GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(GPIOB, EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, EN_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}
void LCD_WriteString(const char *ptr){

	while(*ptr != '\0'){

		LCD_WriteData(*ptr++);
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
		return 'A';
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
		return 'B';
	}





	HAL_GPIO_WritePin(GPIOC, R1_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, R2_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, R3_Pin, 0);
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
		return 'C';
	}





	HAL_GPIO_WritePin(GPIOC, R1_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, R2_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, R3_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, R4_Pin, 0);
	HAL_Delay(20);
	if(HAL_GPIO_ReadPin(GPIOC, C1_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C1_Pin) == 0);
		return '*';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C2_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C2_Pin) == 0);
		return '0';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C3_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C3_Pin) == 0);
		return '#';
	}

	if(HAL_GPIO_ReadPin(GPIOC, C4_Pin) == 0) {

		while(HAL_GPIO_ReadPin(GPIOC, C4_Pin) == 0);
		return 'D';
	}
	return 0;
}
//uint8_t Get_Key( void ){
//
//	HAL_GPIO_WritePin (R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);  //Pull the R1 low
//	HAL_GPIO_WritePin (R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);  // Pull the R2 High
//	HAL_GPIO_WritePin (R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);  // Pull the R3 High
//	HAL_GPIO_WritePin (R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);  // Pull the R4 High
//
//	if ((HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin))== GPIO_PIN_RESET)   // if the Col 1 is low
//	{
//		while ((HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return '1';
//	}
//
//	if ((HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin))==GPIO_PIN_RESET)   // if the Col 2 is low
//	{
//		while ((HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return '2';
//	}
//
//	if ((HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin))==GPIO_PIN_RESET)   // if the Col 3 is low
//	{
//		while ((HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return '3';
//	}
//
//	if((HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin))==GPIO_PIN_RESET)   // if the Col 4 is low
//	{
//
//		while ((HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return 'A';
//	}
//
//
//
//
//	HAL_GPIO_WritePin (R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);  //Pull the R1 low
//	HAL_GPIO_WritePin (R2_GPIO_Port, R2_Pin, GPIO_PIN_RESET);  // Pull the R2 High
//	HAL_GPIO_WritePin (R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);  // Pull the R3 High
//	HAL_GPIO_WritePin (R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);  // Pull the R4 High
//
//	if ((HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin))==GPIO_PIN_RESET)   // if the Col 1 is low
//	{
//		while ((HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return '4';
//
//	}
//
//	if ((HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin))==GPIO_PIN_RESET)   // if the Col 2 is low
//	{
//		while ((HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return '5';
//	}
//
//	if ((HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin))==GPIO_PIN_RESET)   // if the Col 3 is low
//	{
//		while ((HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return '6';
//	}
//
//	if ((HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin))==GPIO_PIN_RESET)   // if the Col 4 is low
//	{
//		while ((HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return 'B';
//	}
//
//
//	HAL_GPIO_WritePin (R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);  //Pull the R1 low
//	HAL_GPIO_WritePin (R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);  // Pull the R2 High
//	HAL_GPIO_WritePin (R3_GPIO_Port, R3_Pin, GPIO_PIN_RESET);  // Pull the R3 High
//	HAL_GPIO_WritePin (R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);  // Pull the R4 High
//
//	if ((HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin))==GPIO_PIN_RESET)   // if the Col 1 is low
//	{
//		while ((HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return '7';
//	}
//
//	if ((HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin))==GPIO_PIN_RESET)   // if the Col 2 is low
//	{
//		while ((HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return '8';
//	}
//
//	if ((HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin))==GPIO_PIN_RESET)   // if the Col 3 is low
//	{
//		while ((HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return '9';
//	}
//
//	if ((HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin))==GPIO_PIN_RESET)   // if the Col 4 is low
//	{
//		while ((HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return 'C';
//	}
//
//
//
//
//	HAL_GPIO_WritePin (R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);  //Pull the R1 low
//	HAL_GPIO_WritePin (R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);  // Pull the R2 High
//	HAL_GPIO_WritePin (R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);  // Pull the R3 High
//	HAL_GPIO_WritePin (R4_GPIO_Port, R4_Pin, GPIO_PIN_RESET);  // Pull the R4 High
//
//	if ((HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin))==GPIO_PIN_RESET)   // if the Col 1 is low
//	{
//		while ((HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return '*';
//	}
//
//	if ((HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin))==GPIO_PIN_RESET)   // if the Col 2 is low
//	{
//		while ((HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return '0';
//	}
//
//	if ((HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin))==GPIO_PIN_RESET)   // if the Col 3 is low
//	{
//		while ((HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return '#';
//	}
//
//	if ((HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin))==GPIO_PIN_RESET)   // if the Col 4 is low
//	{
//		while ((HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin))==GPIO_PIN_RESET);   // wait till the button is pressed
//		return 'D';
//	}
//	return 0;
//}

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
