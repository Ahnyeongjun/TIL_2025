/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define MODE_OFF    2
#define MODE_A      0
#define MODE_B      1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
GPIO_TypeDef* led_ports[8] = {LED00_GPIO_Port, LED01_GPIO_Port, LED02_GPIO_Port, LED03_GPIO_Port,
                              LED04_GPIO_Port, LED05_GPIO_Port, LED06_GPIO_Port, LED07_GPIO_Port};
uint16_t led_pins[8] = {LED00_Pin, LED01_Pin, LED02_Pin, LED03_Pin,
                        LED04_Pin, LED05_Pin, LED06_Pin, LED07_Pin};

uint8_t message_1[] = "\n\n**************************\n\r";
uint8_t message_2[] = "Emergency Light Control\n\r";
uint8_t message_3[] = "**************************\n\n\r";
uint8_t message_4[] = "1. Set to [Ambulance_A] mode\n\r";
uint8_t message_5[] = "2. Set to [Ambulance_B] mode\n\r";
uint8_t message_6[] = "3. Set to [Off] mode\n\r";
uint8_t message_7[] = "4. Inquire current Emergency mode\n\n\r";
uint8_t message_8[] = "Type number : \r";
uint8_t message_9[] = "\n\nNow, [Ambulance_A] mode\n\r";
uint8_t message_10[] = "\n\nNow, [Ambulance_B] mode\n\r";
uint8_t message_11[] = "\n\nNow, [Off] mode\n\r";
uint8_t message_12[] = "\n\nCurrent mode is [Ambulance_A] mode\n\r";
uint8_t message_13[] = "\n\nCurrent mode is [Ambulance_B] mode\n\r";
uint8_t message_14[] = "\n\nCurrent mode is [Off] mode\n\r";

uint8_t RxBuffer[1];
uint8_t ambulance_mode = MODE_OFF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void display_menu(void);
void ambulance_a(void);
void ambulance_b(void);
void ambulance_off(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void display_menu(void)
{
    HAL_UART_Transmit(&huart2, message_1, sizeof(message_1)-1, 100);
    HAL_UART_Transmit(&huart2, message_2, sizeof(message_2)-1, 100);
    HAL_UART_Transmit(&huart2, message_3, sizeof(message_3)-1, 100);
    HAL_UART_Transmit(&huart2, message_4, sizeof(message_4)-1, 100);
    HAL_UART_Transmit(&huart2, message_5, sizeof(message_5)-1, 100);
    HAL_UART_Transmit(&huart2, message_6, sizeof(message_6)-1, 100);
    HAL_UART_Transmit(&huart2, message_7, sizeof(message_7)-1, 100);
    HAL_UART_Transmit(&huart2, message_8, sizeof(message_8)-1, 100);
}

void ambulance_a(void)
{
    for (int i=0; i<4; i++)
    {
        HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_SET);
        HAL_GPIO_WritePin(led_ports[i+4], led_pins[i+4], GPIO_PIN_RESET);
    }
    HAL_Delay(500);

    if (ambulance_mode == MODE_B)
        if ((ambulance_mode == MODE_B) || (ambulance_mode == MODE_OFF))
            return;

    for (int i=0; i<4; i++)
    {
        HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET);
        HAL_GPIO_WritePin(led_ports[i+4], led_pins[i+4], GPIO_PIN_SET);
    }
    HAL_Delay(500);

    if (ambulance_mode == MODE_B)
        if ((ambulance_mode == MODE_B) || (ambulance_mode == MODE_OFF))
            return;
}

void ambulance_b(void)
{
    for (int i=7; i>=1; i--)
    {
        for (int j=0; j<=7; j++)
        {
            if (i==j)
                HAL_GPIO_WritePin(led_ports[j], led_pins[j], GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(led_ports[j], led_pins[j], GPIO_PIN_RESET);
        }
        HAL_Delay(100);

        if (ambulance_mode == MODE_A)
            if ((ambulance_mode == MODE_A) || (ambulance_mode == MODE_OFF))
                return;
    }

    for (int i=0; i<=6; i++)
    {
        for (int j=0; j<=7; j++)
        {
            if (i==j)
                HAL_GPIO_WritePin(led_ports[j], led_pins[j], GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(led_ports[j], led_pins[j], GPIO_PIN_RESET);
        }
        HAL_Delay(100);

        if (ambulance_mode == MODE_A)
            if ((ambulance_mode == MODE_A) || (ambulance_mode == MODE_OFF))
                return;
    }
}

void ambulance_off(void)
{
    for (int i=0; i<=7; i++)
    {
        HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET);
    }
    HAL_Delay(100);
}

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
  // 1. UART 입력 인터럽트 활성화
  HAL_UART_Receive_IT(&huart2, RxBuffer, 1);

  // 2. 메뉴 디스플레이 및 현재 모드를 [Off]로 설정
  display_menu();
  ambulance_mode = MODE_OFF;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 3. 현재 모드에 따라 실행
    if (ambulance_mode == MODE_A)
    {
        ambulance_a();  // A. [Ambulance-A] 모드 동작 1회 실행
    }
    else if (ambulance_mode == MODE_B)
    {
        ambulance_b();  // B. [Ambulance-B] 모드 동작 1회 실행
    }
    else  // MODE_OFF
    {
        ambulance_off();  // C. [Off] 모드 - 비상등(LED) 모두 Off
    }
    // 4. (3)을 반복
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED04_Pin|LED05_Pin|LED06_Pin|LED07_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED00_Pin|LED01_Pin|LED02_Pin|LED03_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED04_Pin LED05_Pin LED06_Pin LED07_Pin */
  GPIO_InitStruct.Pin = LED04_Pin|LED05_Pin|LED06_Pin|LED07_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SW2_Pin */
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED00_Pin LED01_Pin LED02_Pin LED03_Pin */
  GPIO_InitStruct.Pin = LED00_Pin|LED01_Pin|LED02_Pin|LED03_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 1. 수신된 데이터(키 입력값)를 검사하여 처리
    switch (RxBuffer[0])
    {
        case '1':
            // A. '1'이 입력된 경우
            ambulance_mode = MODE_A;
            HAL_UART_Transmit(&huart2, RxBuffer, 1, 100);  // i. '1' echo back
            HAL_UART_Transmit(&huart2, message_9, sizeof(message_9)-1, 100);  // iii. 메시지 디스플레이
            display_menu();  // 3. 메뉴 디스플레이
            break;

        case '2':
            // B. '2'가 입력된 경우
            ambulance_mode = MODE_B;
            HAL_UART_Transmit(&huart2, RxBuffer, 1, 100);  // i. '2' echo back
            HAL_UART_Transmit(&huart2, message_10, sizeof(message_10)-1, 100);  // iii. 메시지 디스플레이
            display_menu();  // 3. 메뉴 디스플레이
            break;

        case '3':
            // C. '3'이 입력된 경우
            ambulance_mode = MODE_OFF;
            HAL_UART_Transmit(&huart2, RxBuffer, 1, 100);  // i. '3' echo back
            HAL_UART_Transmit(&huart2, message_11, sizeof(message_11)-1, 100);  // iii. 메시지 디스플레이
            display_menu();  // 3. 메뉴 디스플레이
            break;

        case '4':
            // D. '4'가 입력된 경우
            HAL_UART_Transmit(&huart2, RxBuffer, 1, 100);  // i. '4' echo back
            // ii. 현재 모드를 검사하여 메시지 디스플레이
            if (ambulance_mode == MODE_A)
                HAL_UART_Transmit(&huart2, message_12, sizeof(message_12)-1, 100);
            else if (ambulance_mode == MODE_B)
                HAL_UART_Transmit(&huart2, message_13, sizeof(message_13)-1, 100);
            else
                HAL_UART_Transmit(&huart2, message_14, sizeof(message_14)-1, 100);
            display_menu();  // 3. 메뉴 디스플레이
            break;

        default:
            // E. '1', '2', '3', '4'가 아닌 키가 입력된 경우
            RxBuffer[0] = '\a';  // i. '\a'(알람, 삐~ 소리) echo back
            HAL_UART_Transmit(&huart2, RxBuffer, 1, 100);
            break;
    }

    // 4. 1개의 다음 키 입력을 수신할 수 있도록 UART 입력 인터럽트 활성화
    HAL_UART_Receive_IT(&huart2, RxBuffer, 1);
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
