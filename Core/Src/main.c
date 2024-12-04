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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HIGH GPIO_PIN_RESET
#define LOW GPIO_PIN_SET
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

osThreadId defaultTaskHandle;
osThreadId Green_Led_FlashHandle;
osThreadId Red_Led_FlashHandle;
osThreadId Orange_Led_FlasHandle;
/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */
uint8_t StartFlag;
volatile uint8_t RedFlag;
volatile uint8_t GreenFlag;
uint16_t WaitTimeMilliseconds = 550;

osSemaphoreDef(CriticalResourceSemaphore);
osSemaphoreId CriticalResourceSemaphoreHandle;
/* USER CODE END PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void const * argument);
void GreenTask(void const * argument);
void RedTask(void const * argument);
void OrangeTask(void const * argument);

/* USER CODE BEGIN PFP */

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
	StartFlag = 1;
	GreenFlag = 1;
	RedFlag = 1;
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, HIGH);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  // Inisialisasi semaphore menggunakan osSemaphoreNew (CMSIS-RTOS2)
  CriticalResourceSemaphoreHandle = osSemaphoreCreate(osSemaphore(CriticalResourceSemaphore), 1);
  if (CriticalResourceSemaphoreHandle == NULL) {
      Error_Handler(); // Handle semaphore creation error
  }

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Green_Led_Flash */
  osThreadDef(Green_Led_Flash, GreenTask, osPriorityIdle, 0, 128);
  Green_Led_FlashHandle = osThreadCreate(osThread(Green_Led_Flash), NULL);

  /* definition and creation of Red_Led_Flash */
  osThreadDef(Red_Led_Flash, RedTask, osPriorityIdle, 0, 128);
  Red_Led_FlashHandle = osThreadCreate(osThread(Red_Led_Flash), NULL);

  /* definition and creation of Orange_Led_Flas */
  osThreadDef(Orange_Led_Flas, OrangeTask, osPriorityIdle, 0, 128);
  Orange_Led_FlasHandle = osThreadCreate(osThread(Orange_Led_Flas), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led1_Pin|led2_Pin|led3_Pin|led4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : led1_Pin led2_Pin led3_Pin led4_Pin */
  GPIO_InitStruct.Pin = led1_Pin|led2_Pin|led3_Pin|led4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void AccessSharedData(void) {
	//osSemaphoreAcquire(CriticalResourceSemaphoreHandle, osWaitForever);
    if (StartFlag == 1) {
        // Set Start flag to Down to indicate resource is in use
        StartFlag = 0;
    } else {
        // Resource contention: Turn on Blue LED
        HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, HIGH);
    }
    // Simulate read/write operations with a delay of 500 milliseconds
  //    SimulateReadWriteOperation();
      HAL_Delay(500);

      // Set Start flag back to Up to indicate resource is free
      StartFlag = 1;

      // Turn off Blue LED (if it was turned on during contention)
      HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, LOW);

      //osSemaphoreRelease(CriticalResourceSemaphoreHandle);
  }

  void SimulateReadWriteOperation(void) {
      volatile uint32_t delay_count = 0;
      const uint32_t delay_target = 2000000; // Adjust this value to approximate 500 ms

      // Dummy loop to simulate processing time
      for (delay_count = 0; delay_count < delay_target; delay_count++) {
          __asm("nop"); // No Operation: Keeps the processor busy without changing code behavior
      }
  }
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_GreenTask */
/**
* @brief Function implementing the Green_Led_Flash thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GreenTask */
void GreenTask(void const * argument)
{
  /* USER CODE BEGIN GreenTask */
  /* Infinite loop */
  for(;;)
  {
	  GreenFlag = 1;
	        HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, HIGH);

	        // Masuk critical section
	        osSemaphoreWait(CriticalResourceSemaphoreHandle, osWaitForever);
	        AccessSharedData(); // Fungsi yang menggunakan resource bersama
	        osSemaphoreRelease(CriticalResourceSemaphoreHandle);


	        HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, LOW);
	        GreenFlag = 0;

	        osDelay(400); // Delay untuk simulasi
	    }

  /* USER CODE END GreenTask */
}

/* USER CODE BEGIN Header_RedTask */
/**
* @brief Function implementing the Red_Led_Flash thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RedTask */
void RedTask(void const * argument)
{
  /* USER CODE BEGIN RedTask */
  /* Infinite loop */
  for(;;)
  {
	  RedFlag = 1;
	        HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, HIGH);

	        // Masuk critical section
	        osSemaphoreWait(CriticalResourceSemaphoreHandle, osWaitForever);
	        AccessSharedData(); // Fungsi yang menggunakan resource bersama
	        osSemaphoreRelease(CriticalResourceSemaphoreHandle);

	        HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, LOW);
	        RedFlag = 0;

	        osDelay(450); // Delay untuk simulasi
	    }
  /* USER CODE END RedTask */
}

/* USER CODE BEGIN Header_OrangeTask */
/**
* @brief Function implementing the Orange_Led_Flas thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_OrangeTask */
void OrangeTask(void const * argument)
{
  /* USER CODE BEGIN OrangeTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(led4_GPIO_Port, led4_Pin);
	  osDelay(50);
  }
  /* USER CODE END OrangeTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
}

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
