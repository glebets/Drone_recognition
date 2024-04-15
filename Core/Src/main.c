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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_SIZE 1024
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
 I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_rx;
/* Definitions for procFFT */
osThreadId_t procFFTHandle;
const osThreadAttr_t procFFT_attributes = {
  .name = "procFFT",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dataGet */
osThreadId_t dataGetHandle;
const osThreadAttr_t dataGet_attributes = {
  .name = "dataGet",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for datasetFlow */
osMessageQueueId_t datasetFlowHandle;
const osMessageQueueAttr_t datasetFlow_attributes = {
  .name = "datasetFlow"
};
/* USER CODE BEGIN PV */
//Буферные массивы хранения выборок
int32_t bufInput[FFT_SIZE*4] = {0};
int32_t bufCopy[FFT_SIZE*4] = {0};
float32_t bufDataset[FFT_SIZE] = {0};
float32_t bufFFTRes[FFT_SIZE] = {0};
//Флаг готовности выборки
int DMAFlag = -1;
//Передаваемый массив
uint8_t bufRes[FFT_SIZE/2] = {0};
//Маркировка начала сообщения
uint8_t marker[4] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2S3_Init(void);
static void MX_DMA_Init(void);
void StartDataSend(void *argument);
void StartFFT(void *argument);
void StartGet(void *argument);

//Функция приёма выборки по обратному вызову контроллера DMA
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){
	if(hi2s == &hi2s3){
		HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);	//Отключение прерывания DMA
		memcpy(&bufCopy, &bufInput, 16384);		//Защита от повреждения данных
		DMAFlag = 1;							//Установка флага готовности данных
	}
}
//Функция преобразования выборки данных микрофона
void makeDataset(int* input, float* output, size_t size){
	int temp;
	for(size_t i = 0; i < size ;i += 4){
		temp = (input[i]<<8)+(input[i+1]);
		if(temp & 0x80000000){
			output[i/4] = (float)(0xff000000|temp);
		}else{
			output[i/4] = (float)temp;
		}
	}
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();
  HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*)bufInput, FFT_SIZE*2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the queue(s) */
  /* creation of datasetFlow */
  datasetFlowHandle = osMessageQueueNew (1024, sizeof(float32_t), &datasetFlow_attributes);

  /* Create the thread(s) */
  /* creation of procFFT */
  procFFTHandle = osThreadNew(StartFFT, NULL, &procFFT_attributes);

  /* creation of dataGet */
  dataGetHandle = osThreadNew(StartGet, NULL, &dataGet_attributes);

  /* Start scheduler */
  osKernelStart();
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

}

/* USER CODE BEGIN Header_StartFFT */
/**
* @brief Function implementing the procFFT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFFT */
void StartFFT(void *argument)
{
  /* USER CODE BEGIN StartFFT */
	//Блок инициализации БПФ
	arm_rfft_fast_instance_f32 S;
	arm_rfft_fast_init_f32(&S, FFT_SIZE);
  /* Infinite loop */
  for(;;)
  {
	  //Блок получения выборки посредством очереди
	  osMessageQueueGet(datasetFlowHandle, &bufDataset, 0, osWaitForever);
	  //Блок преобразования данных БПФ
	  arm_rfft_fast_f32(&S, &bufDataset, &bufFFTRes, 0);	//Функция возвращает массив комплексных чисел
	  arm_cmplx_mag_f32(&bufFFTRes, &bufFFTRes, FFT_SIZE);	//Преобразование комплексных чисел в амплитуды
	  //Блок передачи данных на устройство обработки
	  CDC_Transmit_FS((uint8_t*)bufFFTRes, FFT_SIZE*2);		//Передача массива значений амплитуд
	  CDC_Transmit_FS((uint8_t*)marker, 4);					//Последовательность завершения передачи
	  //Вызов планировщика
	  taskYIELD();
  }
  /* USER CODE END StartFFT */
}

/* USER CODE BEGIN Header_StartGet */
/**
* @brief Function implementing the dataGet thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGet */
void StartGet(void *argument)
{
  /* USER CODE BEGIN StartGet */

  /* Infinite loop */
  for(;;)
  {
	  // enable interrupts
	  //Разрешение прерывания ПДП
	  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	  //Блок ожидания получения данных
	  if(DMAFlag == 1){
		  DMAFlag = 0;
		  //Формирование выборки
		  makeDataset(&bufCopy, &bufDataset, FFT_SIZE*4);
		  //Загрузка выборки в очередь
		  osMessageQueuePut(datasetFlowHandle, &bufDataset, 0, osWaitForever);
		  //Вызов планировщика
		  taskYIELD();
	  }
  }
  /* USER CODE END StartGet */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
