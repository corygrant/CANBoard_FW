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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_TASK_DELAY 10
#define ADC_1_COUNT 5
#define ADC_2_COUNT 1
#define DI_DEBOUNCE_TIME 10
#define CAN_BASE_ID 0x640
#define CAN_TX_DELAY 50
#define CAN_TX_MSG_SPLIT 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CAN_HandleTypeDef hcan;

CRC_HandleTypeDef hcrc;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CanTxTask */
osThreadId_t CanTxTaskHandle;
const osThreadAttr_t CanTxTask_attributes = {
  .name = "CanTxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
const uint16_t* const STM32_TEMP_3V3_30C =  (uint16_t*)(0x1FFFF7B8);
const uint16_t* const STM32_TEMP_3V3_110C =  (uint16_t*)(0x1FFFF7C2);

volatile uint16_t anInAdc1[ADC_1_COUNT];
volatile uint16_t anInAdc2[ADC_2_COUNT];

static volatile uint8_t digIn[8];
static volatile uint8_t digOut[4];
static volatile uint16_t anIn[5];
static volatile uint16_t temperature;

static volatile uint8_t lastDigIn[8];
static volatile uint32_t digInTrigTime[8];
static volatile uint8_t checkDigInTime[8];

static CAN_TxHeaderTypeDef   CanTxHeader;
static CAN_RxHeaderTypeDef   CanRxHeader;
static uint8_t               CanTxData[8];
static uint8_t               CanRxData[8];
static uint32_t              CanTxMailbox;
static volatile uint8_t CanHeartbeat;
static volatile uint8_t nCanDigOut[4];
static volatile uint8_t nCanRotarySwitch[5];
static volatile uint8_t nCanAnalogSwitch[5];
static volatile uint8_t nCanRotaryInvert[5];

static volatile uint8_t nCanBaseIdOffset;
static volatile uint8_t nCanId1;
static volatile uint8_t nCanId2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_CRC_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);
void StartCanTxTask(void *argument);

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_CRC_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) anInAdc1, ADC_1_COUNT);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*) anInAdc2, ADC_2_COUNT);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CanTxTask */
  CanTxTaskHandle = osThreadNew(StartCanTxTask, NULL, &CanTxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  //---------------------------------------------------------------
  //Set CAN RX filter
  //---------------------------------------------------------------
  CAN_FilterTypeDef  sFilterConfig;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  CanTxHeader.StdId = CAN_BASE_ID + nCanBaseIdOffset;
  CanTxHeader.ExtId = 0;
  CanTxHeader.RTR = CAN_RTR_DATA;
  CanTxHeader.IDE = CAN_ID_STD;
  CanTxHeader.DLC = 8;
  CanTxHeader.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  //---------------------------------------------------------------
  //Set CAN RX interrupt
  //---------------------------------------------------------------
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DO2_Pin|DO3_Pin|DO4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DI1_Pin DI2_Pin DI3_Pin DI6_Pin
                           DI7_Pin DI8_Pin */
  GPIO_InitStruct.Pin = DI1_Pin|DI2_Pin|DI3_Pin|DI6_Pin
                          |DI7_Pin|DI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DI4_Pin DI5_Pin */
  GPIO_InitStruct.Pin = DI4_Pin|DI5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DO1_Pin */
  GPIO_InitStruct.Pin = DO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DO2_Pin DO3_Pin DO4_Pin */
  GPIO_InitStruct.Pin = DO2_Pin|DO3_Pin|DO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_ID_2_Pin CAN_ID_1_Pin */
  GPIO_InitStruct.Pin = CAN_ID_2_Pin|CAN_ID_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxHeader, CanRxData) != HAL_OK)
  {
    Error_Handler();
  }

  if((CanRxHeader.StdId == CAN_BASE_ID + nCanBaseIdOffset + 3) && (CanRxHeader.IDE == CAN_ID_STD) && (CanRxHeader.DLC >= 4))
  {
    nCanDigOut[0] = CanRxData[0] & 0x01;
    nCanDigOut[1] = CanRxData[1] & 0x01;
    nCanDigOut[2] = CanRxData[2] & 0x01;
    nCanDigOut[3] = CanRxData[3] & 0x01;
  }
}

void TxCanMsgs()
{
  //=======================================================
  //Build Msg 0 (Analog inputs 1-4 millivolts)
  //=======================================================
  CanTxHeader.StdId = CAN_BASE_ID + nCanBaseIdOffset + 0;
  CanTxHeader.DLC = 8; //Bytes to send
  CanTxData[0] = ((float)anIn[0] / 4096) * 4850; //Scaled to 4.85v - voltage divider on input, 3.3V = 4.85V on input
  CanTxData[1] = ((float)(anIn[0] >> 8) / 4096) * 4850;
  CanTxData[2] = ((float)anIn[1] / 4096) * 4850;
  CanTxData[3] = ((float)(anIn[1] >> 8) / 4096) * 4850;
  CanTxData[4] = ((float)anIn[2] / 4096) * 4850;
  CanTxData[5] = ((float)(anIn[2] >> 8) / 4096) * 4850;
  CanTxData[6] = ((float)anIn[3] / 4096) * 4850;
  CanTxData[7] = ((float)(anIn[3] >> 8) / 4096) * 4850;

  //=======================================================
  //Send CAN msg
  //=======================================================
  if(HAL_CAN_AddTxMessage(&hcan, &CanTxHeader, CanTxData, &CanTxMailbox) != HAL_OK){
    Error_Handler();
  }

  osDelay(CAN_TX_MSG_SPLIT);

  //=======================================================
  //Build Msg 1 (Analog input 5 millivolts and temperature)
  //=======================================================
  CanTxHeader.StdId = CAN_BASE_ID + nCanBaseIdOffset + 1;
  CanTxHeader.DLC = 8; //Bytes to send
  CanTxData[0] = ((float)anIn[4] / 4096) * 4850; //Scaled to 4.85v - voltage divider on input, 3.3V = 4.85V on input
  CanTxData[1] = ((float)(anIn[4] >> 8) / 4096) * 4850;
  CanTxData[2] = 0;
  CanTxData[3] = 0;
  CanTxData[4] = 0;
  CanTxData[5] = 0;
  CanTxData[6] = temperature;
  CanTxData[7] = (temperature >> 8);

  //=======================================================
  //Send CAN msg
  //=======================================================
  if(HAL_CAN_AddTxMessage(&hcan, &CanTxHeader, CanTxData, &CanTxMailbox) != HAL_OK){
    Error_Handler();
  }

  osDelay(CAN_TX_MSG_SPLIT);

  for(int i=0; i<5; i++)
  {
    if(nCanRotaryInvert[i]){
      //Analog as rotary switch
      if(anIn[i] > 3300)
        nCanRotarySwitch[i] = 0;
      if((anIn[i] <= 3300) && (anIn[i] > 2700))
        nCanRotarySwitch[i] = 1;
      if((anIn[i] <= 2700) && (anIn[i] > 2200))
        nCanRotarySwitch[i] = 2;
      if((anIn[i] <= 2200) && (anIn[i] > 1800))
        nCanRotarySwitch[i] = 3;
      if((anIn[i] <= 1800) && (anIn[i] > 1400))
        nCanRotarySwitch[i] = 4;
      if((anIn[i] <= 1400) && (anIn[i] > 1000))
        nCanRotarySwitch[i] = 5;
      if((anIn[i] <= 1000) && (anIn[i] > 600))
        nCanRotarySwitch[i] = 6;
      if(anIn[i] <= 600)
        nCanRotarySwitch[i] = 7;
    }
    else
    {
      //Analog as rotary switch
      if(anIn[i] > 3300)
        nCanRotarySwitch[i] = 7;
      if((anIn[i] <= 3300) && (anIn[i] > 2700))
        nCanRotarySwitch[i] = 6;
      if((anIn[i] <= 2700) && (anIn[i] > 2200))
        nCanRotarySwitch[i] = 5;
      if((anIn[i] <= 2200) && (anIn[i] > 1800))
        nCanRotarySwitch[i] = 4;
      if((anIn[i] <= 1800) && (anIn[i] > 1400))
        nCanRotarySwitch[i] = 3;
      if((anIn[i] <= 1400) && (anIn[i] > 1000))
        nCanRotarySwitch[i] = 2;
      if((anIn[i] <= 1000) && (anIn[i] > 600))
        nCanRotarySwitch[i] = 1;
      if(anIn[i] <= 600)
        nCanRotarySwitch[i] = 0;
    }

    //Analog as digital switch
    nCanAnalogSwitch[i] = anIn[i] > 2048;
  }

  //=======================================================
  //Build Msg 2 (Rotary switches, dig inputs, analog input switches, low side output status, heartbeat)
  //=======================================================
  CanTxHeader.StdId = CAN_BASE_ID + nCanBaseIdOffset + 2;
  CanTxHeader.DLC = 8; //Bytes to send
  CanTxData[0] = (nCanRotarySwitch[1] << 4) + nCanRotarySwitch[0];
  CanTxData[1] = (nCanRotarySwitch[3] << 4) + nCanRotarySwitch[2];
  CanTxData[2] = nCanRotarySwitch[4];
  CanTxData[3] = 0; //Empty
  CanTxData[4] = (digIn[7] << 7) + (digIn[6] << 6) + (digIn[5] << 5) + (digIn[4] << 4) + (digIn[3] << 3) + (digIn[2] << 2) + (digIn[1] << 1) + digIn[0];
  CanTxData[5] = (nCanAnalogSwitch[4] << 4) + (nCanAnalogSwitch[3] << 3) + (nCanAnalogSwitch[2] << 2) + (nCanAnalogSwitch[1] << 1) + nCanAnalogSwitch[0];
  CanTxData[6] = (nCanDigOut[3] << 3) + (nCanDigOut[2] << 2) + (nCanDigOut[1] << 1) + nCanDigOut[0];;
  CanTxData[7] = CanHeartbeat;

  //=======================================================
  //Send CAN msg
  //=======================================================
  if(HAL_CAN_AddTxMessage(&hcan, &CanTxHeader, CanTxData, &CanTxMailbox) != HAL_OK){
    Error_Handler();
  }

  CanHeartbeat++;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  nCanRotaryInvert[0] = 1;
  nCanRotaryInvert[1] = 0;
  nCanRotaryInvert[2] = 0;
  nCanRotaryInvert[3] = 0;
  nCanRotaryInvert[4] = 0;

  /* Infinite loop */
  for(;;)
  {
    //=======================================================
    //Set CAN base ID
    //=======================================================
    nCanId1 = HAL_GPIO_ReadPin(CAN_ID_1_GPIO_Port, CAN_ID_1_Pin);
    nCanId2 = HAL_GPIO_ReadPin(CAN_ID_2_GPIO_Port, CAN_ID_2_Pin);
    nCanBaseIdOffset = (nCanId1 << 4) + (nCanId2 << 5);

    //=======================================================
    //Set digital outputs
    //=======================================================
    if(nCanDigOut[0] == 0)
      DO1_GPIO_Port->ODR &= ~DO1_Pin;
    else
      DO1_GPIO_Port->ODR |= DO1_Pin;

    if(nCanDigOut[1] == 0)
      DO2_GPIO_Port->ODR &= ~DO2_Pin;
    else
      DO2_GPIO_Port->ODR |= DO2_Pin;

    if(nCanDigOut[2] == 0)
      DO3_GPIO_Port->ODR &= ~DO3_Pin;
    else
      DO3_GPIO_Port->ODR |= DO3_Pin;

    if(nCanDigOut[3] == 0)
      DO4_GPIO_Port->ODR &= ~DO4_Pin;
    else
      DO4_GPIO_Port->ODR |= DO4_Pin;

    osDelay(MAIN_TASK_DELAY);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCanTxTask */
/**
* @brief Function implementing the CanTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTxTask */
void StartCanTxTask(void *argument)
{
  /* USER CODE BEGIN StartCanTxTask */
  /* Infinite loop */
  for(;;)
  {
    //=======================================================
    //Read digital input pins
    //=======================================================
    digIn[0] = !HAL_GPIO_ReadPin(DI1_GPIO_Port, DI1_Pin);
    digIn[1] = !HAL_GPIO_ReadPin(DI2_GPIO_Port, DI2_Pin);
    digIn[2] = !HAL_GPIO_ReadPin(DI3_GPIO_Port, DI3_Pin);
    digIn[3] = !HAL_GPIO_ReadPin(DI4_GPIO_Port, DI4_Pin);
    digIn[4] = !HAL_GPIO_ReadPin(DI5_GPIO_Port, DI5_Pin);
    digIn[5] = !HAL_GPIO_ReadPin(DI6_GPIO_Port, DI6_Pin);
    digIn[6] = !HAL_GPIO_ReadPin(DI7_GPIO_Port, DI7_Pin);
    digIn[7] = !HAL_GPIO_ReadPin(DI8_GPIO_Port, DI8_Pin);

    //=======================================================
    //Debounce and latch logic
    //=======================================================
    /*
    //Last transition recorded
    if(digIn[2] != lastDigIn[2])
      if(digIn[2] == GPIO_PIN_SET){
        digInTrigTime[2] = HAL_GetTick();
        checkDigInTime[2] = 1;
      }

    lastDigIn[2] = digIn[2];

    if(checkDigInTime[2] && ((HAL_GetTick() - digInTrigTime[2]) > DI_DEBOUNCE_TIME)){
      digOut[2] = !digOut[2];
      checkDigInTime[2] = 0;
    }
    */

    //=======================================================
    //Copy analog input DMA to local vars (single array)
    //=======================================================
    anIn[0] = anInAdc1[0];
    anIn[1] = anInAdc1[1];
    anIn[2] = anInAdc1[2];
    anIn[3] = anInAdc1[3];
    anIn[4] = anInAdc2[0];

    //=======================================================
    //Scale temperature based on factory calibration
    //=======================================================
    temperature = (uint16_t)((80.0 / ((float)(*STM32_TEMP_3V3_110C) - (float)(*STM32_TEMP_3V3_30C)) *
                              (((float)anInAdc2[0]) - (float)(*STM32_TEMP_3V3_30C)) + 30.0) * 10.0);

    //=======================================================
    //Tx CAN messages
    //=======================================================
    TxCanMsgs();
    osDelay(CAN_TX_DELAY);
  }
  /* USER CODE END StartCanTxTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
