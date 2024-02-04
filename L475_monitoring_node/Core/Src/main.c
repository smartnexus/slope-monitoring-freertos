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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "es_wifi.h"
#include "wifi.h"
#include "mqtt_priv.h"
// Drivers
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_tsensor.h"
// Otros
#include <math.h>
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
#define SSID     "Celia"
#define PASSWORD "holahola"
#define WIFISECURITY WIFI_ECN_WPA2_PSK
#define SOCKET 0
#define WIFI_WRITE_TIMEOUT 10
#define WIFI_READ_TIMEOUT 10
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for networkSetup */
osThreadId_t networkSetupHandle;
const osThreadAttr_t networkSetup_attributes = {
  .name = "networkSetup",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for stopWatch */
osThreadId_t stopWatchHandle;
const osThreadAttr_t stopWatch_attributes = {
  .name = "stopWatch",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for collectMeasures */
osThreadId_t collectMeasuresHandle;
const osThreadAttr_t collectMeasures_attributes = {
  .name = "collectMeasures",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MQTTPublish */
osThreadId_t MQTTPublishHandle;
const osThreadAttr_t MQTTPublish_attributes = {
  .name = "MQTTPublish",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for inputConfig */
osThreadId_t inputConfigHandle;
const osThreadAttr_t inputConfig_attributes = {
  .name = "inputConfig",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for printUART */
osThreadId_t printUARTHandle;
const osThreadAttr_t printUART_attributes = {
  .name = "printUART",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for measuresQueue */
osMessageQueueId_t measuresQueueHandle;
const osMessageQueueAttr_t measuresQueue_attributes = {
  .name = "measuresQueue"
};
/* Definitions for receiveQueue */
osMessageQueueId_t receiveQueueHandle;
const osMessageQueueAttr_t receiveQueue_attributes = {
  .name = "receiveQueue"
};
/* Definitions for printQueue */
osMessageQueueId_t printQueueHandle;
const osMessageQueueAttr_t printQueue_attributes = {
  .name = "printQueue"
};
/* USER CODE BEGIN PV */
extern  SPI_HandleTypeDef hspi;
static  uint8_t  IP_Addr[4];
static uint8_t operationMode;						/* Modo de operacion: normal/alarma*/
static NetworkContext_t xNetworkContext = { 0 };
static MQTTContext_t xMQTTContext;

bool ctrlAcceleration = false; /* Controla el envío de la notificación en el callback de la interrupción del acelerómetro*/

uint8_t drdyPulsedCfg = 0;
uint8_t ctrlDrdy = 0;
uint8_t ctrlMaster = 0;

uint8_t rec_data;		/* Almacena el caracter recibido por UART */
RTC_DateTypeDef GetDate; 		/* Estructura para fijar/leer fecha */
RTC_TimeTypeDef GetTime;		/* Estructura para fijar/leer hora */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_RTC_Init(void);
void networkSetupTask(void *argument);
void stopWatchTask(void *argument);
void collectMeasuresTask(void *argument);
void MQTTPublishTask(void *argument);
void inputConfigTask(void *argument);
void printUARTTask(void *argument);

/* USER CODE BEGIN PFP */
void LSM6DSL_AccInt_Drdy(void);							/* Función para la activación de la interrupción Data Ready */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);			/* Función para el callback de la interrupción */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void osMessageQueuePutChecker(char *nombre_tarea, char *nombre_cola, osStatus_t estado);
void leerDatosCola(char * recibido);
uint8_t extraerNumero(char *digitos, uint8_t *rango);
void publishMeasurements(char *accel_x, char *accel_y, char *accel_z, char *humedad, char *temperatura);
void separarCadena(char *cadena, char *delimitador, char *partes[], int numPartes);
void MQTTSubTask(void);


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
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  printf("****** Sistemas Ciberfisicos ****** \n\n");
  // Activamos la interrupcion UART_RECEIVE antes de arrancar el scheduler
  HAL_UART_Receive_IT(&huart1, &rec_data, 1);

  printf("----> Inicializando el sensor LSM6DSL\r\n");
  BSP_ACCELERO_Init();									        /* Inicialización del acelerómetro */
  LSM6DSL_AccInt_Drdy();										/* Configuración del acelerómetro*/
  BSP_ACCELERO_LowPower(0);										/* Deshabilitado del modo de bajo consumo*/
  printf("----> Sensor LSM6DSL inicializado\r\n");

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

  /* Create the queue(s) */
  /* creation of measuresQueue */
  measuresQueueHandle = osMessageQueueNew (3, sizeof(uintptr_t), &measuresQueue_attributes);

  /* creation of receiveQueue */
  receiveQueueHandle = osMessageQueueNew (3, sizeof(char), &receiveQueue_attributes);

  /* creation of printQueue */
  printQueueHandle = osMessageQueueNew (8, sizeof(uintptr_t), &printQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of networkSetup */
  networkSetupHandle = osThreadNew(networkSetupTask, NULL, &networkSetup_attributes);

  /* creation of stopWatch */
  stopWatchHandle = osThreadNew(stopWatchTask, NULL, &stopWatch_attributes);

  /* creation of collectMeasures */
  collectMeasuresHandle = osThreadNew(collectMeasuresTask, NULL, &collectMeasures_attributes);

  /* creation of MQTTPublish */
  MQTTPublishHandle = osThreadNew(MQTTPublishTask, NULL, &MQTTPublish_attributes);

  /* creation of inputConfig */
  inputConfigHandle = osThreadNew(inputConfigTask, NULL, &inputConfig_attributes);

  /* creation of printUART */
  printUARTHandle = osThreadNew(printUARTTask, NULL, &printUART_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000E14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 12;
  sTime.Minutes = 30;
  sTime.Seconds = 35;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
	int DataIdx;
	for(DataIdx=0; DataIdx<len; DataIdx++){
		ITM_SendChar(*ptr++);
	}
	return len;
}
static int wifi_start(void) {
  uint8_t  MAC_Addr[6];

  /*Initialize and use WIFI module */
  if(WIFI_Init() ==  WIFI_STATUS_OK) {
	  printf("ES-WIFI Initialized.\n");
	  if(WIFI_GetMAC_Address(MAC_Addr, 32) == WIFI_STATUS_OK) {
		  printf("> eS-WiFi module MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\n",
               MAC_Addr[0],
               MAC_Addr[1],
               MAC_Addr[2],
               MAC_Addr[3],
               MAC_Addr[4],
               MAC_Addr[5]);
	  } else {
		  printf("> ERROR : CANNOT get MAC address\n");
		  return -1;
	  }
  } else {
	  return -1;
  }
  return 0;
}
int wifi_connect(void) {

  wifi_start();

  printf("\nConnecting to %s , %s\n",SSID,PASSWORD);
  if( WIFI_Connect(SSID, PASSWORD, WIFISECURITY) == WIFI_STATUS_OK) {
	  if(WIFI_GetIP_Address(IP_Addr, 32) == WIFI_STATUS_OK) {
		  printf("> es-wifi module connected: got IP Address : %d.%d.%d.%d\n",
               IP_Addr[0],
               IP_Addr[1],
               IP_Addr[2],
               IP_Addr[3]);
	  } else {
		  printf(" ERROR : es-wifi module CANNOT get IP address\n");
		  return -1;
	  }
  } else {
	  printf("ERROR : es-wifi module NOT connected\n");
	  return -1;
  }
  return 0;
}

void MQTTSubTask(void) {
	TransportStatus_t xNetworkStatus;
	/* Conexión al broker específico */
	xNetworkStatus = prvConnectToServer( &xNetworkContext );
	configASSERT( xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS );
	prvCreateMQTTConnectionWithBroker( &xMQTTContext, &xNetworkContext );
	/* Subscripción a los diferentes canales. */
	prvMQTTSubscribeToTopic(&xMQTTContext, topicModoOperacion);
	for( ; ; ) {
		  if (xNetworkContext.socket_open == 0){
			  break;
		  }
	 	MQTT_ProcessLoop(&xMQTTContext); /* Comprueba nuevos mensajes cada segundo */
		osDelay(pdMS_TO_TICKS(1000));
	}
}

void separarCadena(char *cadena, char *delimitador, char *partes[], int numPartes) {
    char *token = strtok(cadena, delimitador);

    int i = 0;
    while (token != NULL && i < numPartes) {
        partes[i] = token;
        // printf("Parte %d = %.*s \r\n", i, strlen(partes[i]),(char *) partes[i]);
        i++;
        token = strtok(NULL, delimitador);
    }
}

void publishMeasurements(char *accel_x, char *accel_y, char *accel_z, char *humedad, char *temperatura) {
    char payload[256];
    char *template = "{\"aceleraciones\":{\"value\":1,\"context\":{\"accel_x\":%s,\"accel_y\":%s,\"accel_z\":%s}},\"humedad\":%s,\"temperatura\":%s}";
    sprintf(payload, template, accel_x, accel_y, accel_z, humedad, temperatura);
    printf("%s\r\n", payload);
    prvMQTTPublishToTopic(&xMQTTContext, topicGeneral, payload);
}

/* Funcion para leer de la cola de recepcion y formar el mensaje recibido por UART */
void leerDatosCola(char * recibido) {
	bool leer_cola = true;
	uint8_t num_msg_leido = 0;
	char rec;
	osStatus_t estado;

	while (leer_cola) {
		estado = osMessageQueueGet(receiveQueueHandle, &rec, NULL, pdMS_TO_TICKS(200));

		if (estado == osOK) {
			printf("[RTC_set] (%c) leido de receive_queue\r\n", rec);

			if ('\n' == rec || '\r' == rec) {
				recibido[num_msg_leido] = '\0'; // fin de string
				leer_cola = false;
			} else {
				recibido[num_msg_leido] = rec;
				num_msg_leido++;
			}
		} else if (estado == osErrorTimeout) {
			printf("[RTC_set] Timeout lectura receive_queue\r\n");
		}
	}
}

/* Funcion para extraer un número de una cadena de dígitos en formato ASCII y verificar si
 * ese número se encuentra dentro de un rango especificado. Si devuelve 255 significa que
 * el numero no es valido*/
uint8_t extraerNumero(char *digitos, uint8_t *rango) {
	int valor;

	// Cada dígito se resta por el valor ASCII del '0' (48) para obtener su valor numérico
	if (strlen(digitos) > 1)
		valor = (((digitos[0] - 48) * 10) + (digitos[1] - 48));
	else
		valor = digitos[0] - 48;

	// Se verifica si el número se encuentra dentro del rando admisible
	if (valor < rango[0] || valor > rango[1])
		valor = 255; // Si devuelve 255 significa que el numero no es valido
	return valor;
}

/* Comprobacion escritura en cola*/
void osMessageQueuePutChecker(char *nombre_tarea, char *nombre_cola, osStatus_t estado) {
	if (estado == osOK)
		printf("[%s] Msg enviado a %s\r\n", nombre_tarea, nombre_cola);
	else if (estado == osErrorTimeout)
		printf("[%s] Timeout put %s agotado\r\n", nombre_tarea, nombre_cola);
}

/**/
void MQTTSubscribeCallback(char topic[128], char content[128]) {
	if (strcmp(topic, topicModoOperacion) == 0) {
		if (strcmp(content, MODO_NORMAL) == 0)
			operationMode = MODO_NORMAL_COD;
		else if (strcmp(content, MODO_ALARMA) == 0)
			operationMode = MODO_ALARMA_COD;
	}
}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
		case (GPIO_PIN_1): {
			SPI_WIFI_ISR();
			break;
		}
		case (LSM6DSL_INT1_EXTI11_Pin): { /* Comprobación del Pin de Interrupción --> INT1, buscar en main.h */
			if (ctrlAcceleration == 1)
				osThreadFlagsSet(collectMeasuresHandle, 0x0002U);
			break;
		}
		default: {
			break;
		}
	}
}

/* Interrupcion callback UART */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	osStatus_t estado;
	char dato = (char) rec_data;

	if (huart == &huart1) {

		printf("[UART_IT] Recibido un caracter: %c\r\n", rec_data);

		// Enviar mensaje a la cola de recepcion
		estado = osMessageQueuePut(receiveQueueHandle, &dato, 0, 0);

		if (estado == osOK) {
			printf("[UART_IT] Put (%c) en receive_queue\r\n", dato);

			if (rec_data == '\r' || rec_data == '\n') {
				// Envia notificacion a RTC_set si el caracter implica finalizacion de linea
				osThreadFlagsSet(inputConfigHandle, 0x0004U);
				printf("[UART_IT] Notificacion flag 4 a RTC_set\r\n");
			}

		} else if (estado == osErrorResource) {
			printf("[UART_IT] Overflow en receive_queue\r\n");
			// Envia notificacion a RTC_set si no se ha podido añadir a la cola (overflow)
			osThreadFlagsSet(inputConfigHandle, 0x0008U);
			printf("[UART_IT] Notificacion flag 8 a RTC_set\r\n");
		} else if (estado == osErrorTimeout) {
			printf("[UART_IT] Timeout put (%c) en receive_queue\r\n", dato);
		}

		// Reactivamos la interrupcion UART_RECEIVE
		HAL_UART_Receive_IT(&huart1, &rec_data, 1);
	}
}

/* Inicialización del acelerómetro */
void LSM6DSL_AccInt_Drdy(void) {

	// LEO->APLICO MASCARA->ESCRIBO

	// DRDY_PULSE_CFG_G
	/* Read DRDY_PULSE_CFG_G value */
	drdyPulsedCfg = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_DRDY_PULSE_CFG_G); /*Buscar en archivo lsm6dsl.h*/

	/* Set Drdy interruption to INT1  */
	drdyPulsedCfg |= 0b10000000;

	/* write back control register */
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_DRDY_PULSE_CFG_G, drdyPulsedCfg);

	// INT1_CTRL
	/* Read INT1_CTRL value */
	ctrlDrdy = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT1_CTRL);

	/* Set Drdy interruption to INT1  */
	ctrlDrdy |= 0b00000001;

	/* write back control register */
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT1_CTRL, ctrlDrdy);

	// MASTER_CONFIG
	/* Read MASTER_CONFIG value */
	ctrlMaster = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MASTER_CONFIG);

	/* Set Drdy interruption to INT1  */
	ctrlMaster |= 0b10000000;

	/* write back control register */
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MASTER_CONFIG, ctrlMaster);
}

/**
  * @brief  SPI3 line detection callback.
  * @param  None
  * @retval None
  */
void SPI3_IRQHandler(void) {
  HAL_SPI_IRQHandler(&hspi);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_networkSetupTask */
/**
  * @brief  Function implementing the networkSetup thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_networkSetupTask */
void networkSetupTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {
	  if (xNetworkContext.socket_open == 0){
		  wifi_connect();
	  }

	  MQTTSubTask();
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_stopWatchTask */
/**
* @brief Function implementing the stopWatch thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_stopWatchTask */
void stopWatchTask(void *argument)
{
  /* USER CODE BEGIN stopWatchTask */
	// Modo de operacion por defecto
	operationMode = MODO_ALARMA_COD;

	// Espera a que el RTC este inicializado
	while (osThreadFlagsWait(0x0010U, osFlagsWaitAll, pdMS_TO_TICKS(20000)) != 0x0010U) {
		printf("Esperando a que se inicialice RTC\r\n");
	}

	/* Infinite loop */
	for (;;) {

		// Esperamos el tiempo de modo alarma (20 segundos)
		osDelay(pdMS_TO_TICKS((TIEMPO_MODO_ALARMA)*1000));

		// Comprobamos el modo de operacion.
		// Si modo normal, esperamos 40 segundos mas para llegar a los 60 segundos
		printf("\nMODO_OPERACION: %d\r\n", operationMode);

		if (operationMode == MODO_NORMAL_COD) {
			osDelay(pdMS_TO_TICKS((TIEMPO_MODO_NORMAL-TIEMPO_MODO_ALARMA)*1000));
		}

		// Enviamos notificacion para que comiencen las mediciones
		printf("Enviando notificacion 0x0001...\r\n");
		osThreadFlagsSet(collectMeasuresHandle, 0x0001U);

	}
  /* USER CODE END stopWatchTask */
}

/* USER CODE BEGIN Header_collectMeasuresTask */
/**
* @brief Function implementing the collectMeasures thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_collectMeasuresTask */
void collectMeasuresTask(void *argument)
{
  /* USER CODE BEGIN collectMeasuresTask */
  uint32_t flag_state = 0U;
	osStatus_t queue_state;
	char queue_msg[100] = "";
	char *queue_msg_ptr = queue_msg; // Puntero a queue_msg

	// Inicializamos el sensor de temperatura.
	printf("----> Inicializando el sensor de temperatura\r\n");
	while (BSP_TSENSOR_Init()) {
		printf("----> Error al inicializar\r\n");
	}
	printf("----> Sensor de temperatura inicializado\r\n");

	// Inicializamos el sensor de humedad.
	printf("----> Inicializando el sensor de humedad\r\n");
	while (BSP_HSENSOR_Init()) {
		printf("----> Error al inicializar\r\n");
	}
	printf("----> Sensor de humedad inicializado\n\n");

	/* Infinite loop */
	for (;;) {

		// Espera notificación de stopWatchTask
		flag_state = osThreadFlagsWait(0x0001, osFlagsWaitAll, pdMS_TO_TICKS(80 * 1000));


		if (flag_state == osFlagsErrorTimeout)
			printf("Timeout: espera notificacion.\r\n");
		else if (flag_state == 0x0001U) {
			printf("Notificacion recibida.\r\n");

			ctrlAcceleration = true;

			// Medir tempetarura
			float temp_float = BSP_TSENSOR_ReadTemp();

			// Conversión de la temperatura a formato int
			int temp_int1 = (int) temp_float;
			float temp_frac = temp_float - temp_int1;
			int temp_int2 = trunc(temp_frac * 100);

			printf("Temperatura: %d.%02d grados.\r\n", temp_int1, temp_int2);

			// Medir humedad
			float hum_float = BSP_HSENSOR_ReadHumidity();

			// Conversión de la temperatura a formato int
			int hum_int1 = (int) hum_float;
			float hum_frac = hum_float - hum_int1;
			int hum_int2 = trunc(hum_frac * 100);

			printf("Humedad: %d.%02d%%.\r\n", hum_int1, hum_int2);

			// Medir acceleración
			int16_t pDataAccAvg[3] = {0, 0, 0};

			for (uint8_t i=0; i<10; i++) {
				uint32_t flag_acc_state = 0U;
				int16_t pDataAcc[3];

				flag_acc_state = osThreadFlagsWait(0x0002, osFlagsWaitAll, pdMS_TO_TICKS(1 * 1000));

				if (flag_acc_state == osFlagsErrorTimeout)
						printf("Timeout: espera notificacion.\r\n");
				else if (flag_acc_state == 0x0002U) {

					// Obtencion aceleracion en los 3 ejes
					BSP_ACCELERO_AccGetXYZ(pDataAcc);

					pDataAccAvg[0] += 0.1 * pDataAcc[0];
					pDataAccAvg[1] += 0.1 * pDataAcc[1];
					pDataAccAvg[2] += 0.1 * pDataAcc[2];
				}
			}

			printf("Aceleracion: Eje_X = %d, Eje_Y = %d, Eje_Z = %d, \r\n",
					pDataAccAvg[0], pDataAccAvg[1], pDataAccAvg[2]);

			ctrlAcceleration = false;

			// Añadir medidas a la cola de publicación MQTT
			// Orden: temperatura, humedad, accX, accY, accZ
			snprintf(queue_msg, 100, "%d.%02d;%d.%02d;%d;%d;%d",
					temp_int1, temp_int2,
					hum_int1, hum_int2,
					pDataAccAvg[0], pDataAccAvg[1], pDataAccAvg[2]);

			queue_state = osMessageQueuePut(measuresQueueHandle, &queue_msg_ptr, 0, pdMS_TO_TICKS(200));

			if (queue_state == osErrorTimeout)
				printf("Timeout: put cola\r\n");
			else if (queue_state == osOK)
				printf("Medidas insertadas en la cola.\r\n");
		}
	}
  /* USER CODE END collectMeasuresTask */
}

/* USER CODE BEGIN Header_MQTTPublishTask */
/**
* @brief Function implementing the MQTTPublish thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MQTTPublishTask */
void MQTTPublishTask(void *argument)
{
  /* USER CODE BEGIN MQTTPublishTask */
	osStatus_t queue_state;
	char *rec;

	/* Infinite loop */
	for (;;) {
		char *datos_rx[3] = { NULL, NULL, NULL};
		char *temperatura = "-1";
		char *humedad="-1";
		char *accel_x="-1";
		char *accel_y="-1";
		char *accel_z="-1";
		// Obtiene medidas de la cola
		queue_state = osMessageQueueGet(measuresQueueHandle, &rec, NULL, pdMS_TO_TICKS(70 * 1000));
		if (queue_state == osErrorTimeout) {
			printf("Timeout: get cola\r\n");
		} else if (queue_state == osOK) {
			printf("Medidas obtenidadas de la cola: %s\r\n", (uint8_t*) rec);

			separarCadena(rec, ";", datos_rx, 5);
			temperatura = datos_rx[0];
			humedad = datos_rx[1];
			accel_x = datos_rx[2];
			accel_y = datos_rx[3];
			accel_z = datos_rx[4];

			publishMeasurements(accel_x, accel_y, accel_z, humedad, temperatura);
		}
	}
  /* USER CODE END MQTTPublishTask */
}

/* USER CODE BEGIN Header_inputConfigTask */
/**
* @brief Function implementing the inputConfig thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_inputConfigTask */
void inputConfigTask(void *argument) {
	/* USER CODE BEGIN inputConfigTask */
	osStatus_t estado;
	uint32_t flag_rec;
	char recibido[3];
	const char *msg_hora_ok = "\r\nHora cambiada correctamente\r\n";
	const char *msg_fecha_ok = "Fecha cambiada correctamente\r\n";
	const char *msg_error = "\r\nERROR: Valor no válido\r\n";
	const char *msg_rtc1 = "\r\n\r\n========================\r\n"
			"| Configurar rtc |\r\n"
			"========================\r\n\r\n";
	const char *msg_fin = "fin";
	const char *msg[6] = { "Hora (0-23): ", "\r\nMinuto (0-59): ",
			"\r\nSegundo (0-59): ", "\r\nDía (1-31): ", "\r\nMes (1-12): ",
			"\r\nAño (0-99): " };
	uint8_t limit[6][2] = { { 0, 23 }, { 0, 59 }, { 0, 59 }, { 1, 31 },
			{ 1, 12 }, { 0, 99 } };
	uint8_t *toChange[6] = { &GetTime.Hours, &GetTime.Minutes, &GetTime.Seconds,
			&GetDate.Date, &GetDate.Month, &GetDate.Year };

	uint8_t num_msg = 0;

	/* Infinite loop */
	for (;;) {

		if (num_msg == 0) {
			// Primer mensaje
			estado = osMessageQueuePut(printQueueHandle, &msg_rtc1, 0, pdMS_TO_TICKS(200));
			osMessageQueuePutChecker("RTC_set", "print_queue", estado);
		}

		if (num_msg < 6) {
			// Mensajes de configuracion
			estado = osMessageQueuePut(printQueueHandle, &msg[num_msg], 0, pdMS_TO_TICKS(200));
			osMessageQueuePutChecker("RTC_set", "print_queue", estado);

			// ESPERA NOTIFICACION EN FLAG 4 u 8 (0000 1100)
			flag_rec = osThreadFlagsWait(0x000CU, osFlagsWaitAny, osWaitForever); // TODO: pongo timeout?

			switch (flag_rec) {
			case 0x0004U: 	// FLAG 4: se han recibido todos los datos
				// LECTURA DATO DE LA COLA
				leerDatosCola(recibido);

				// COMPROBACION DATO CORRECTO
				uint8_t to_change = extraerNumero(recibido, limit[num_msg]);
				if (to_change != 255) {
					// Si el numero es correcto, se actualiza el valor de las variables GetTime/GetDate
					*(toChange[num_msg]) = to_change;
					num_msg++;

				} else {
					// Si numero recibido no es valido se comienza de nuevo
					num_msg = 0;
					estado = osMessageQueuePut(printQueueHandle, &msg_error, 0,
							pdMS_TO_TICKS(200));
					osMessageQueuePutChecker("RTC_set", "print_queue", estado);
				}
				break;

			case 0x0008U: // FLAG 8: Overflow en la cola
				estado = osMessageQueuePut(printQueueHandle, &msg_error, 0,
						pdMS_TO_TICKS(200));
				osMessageQueuePutChecker("RTC_set", "print_queue", estado);
				num_msg = 0;
				// Se resetea la cola de recepcion para comenzar de nuevo
				osMessageQueueReset(receiveQueueHandle);
				break;

			default:
				break;
			}
		} else if (num_msg == 6) {
			// FECHA Y HORA CORRECTAS
			estado = osMessageQueuePut(printQueueHandle, &msg_hora_ok, 0, pdMS_TO_TICKS(200));
			osMessageQueuePutChecker("RTC_set", "print_queue", estado);
			estado = osMessageQueuePut(printQueueHandle, &msg_fecha_ok, 0, pdMS_TO_TICKS(200));
			osMessageQueuePutChecker("RTC_set", "print_queue", estado);

			// Fija fecha y hora de RTC
			HAL_RTC_SetTime(&hrtc, &GetTime, RTC_FORMAT_BIN);
			HAL_RTC_SetDate(&hrtc, &GetDate, RTC_FORMAT_BIN);

			// Se envía notificación a la tarea stopWatch para que comience a muestrear la temperatura
			osThreadFlagsSet(stopWatchHandle, 0x0010U);
			printf("[RTC_set] Notificacion flag 0 a temp_task\r\n");

			estado = osMessageQueuePut(printQueueHandle, &msg_fin, 0, pdMS_TO_TICKS(200));
			osMessageQueuePutChecker("RTC_set", "print_queue", estado);


			// Tarea se suspende a sí misma para no solicitar otra vez fecha/hora
			printf("[RTC_set] Tarea se suspende a si misma\r\n");
			osThreadSuspend(inputConfigHandle);
		}
		/* USER CODE END inputConfigTask */
	}
}

/* USER CODE BEGIN Header_printUARTTask */
/**
* @brief Function implementing the printUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_printUARTTask */
void printUARTTask(void *argument)
{
  /* USER CODE BEGIN printUARTTask */
	osStatus_t estado;
	char *rec;
	char msg[100] = "";

	/* Infinite loop */
	for (;;) {
		estado = osMessageQueueGet(printQueueHandle, &rec, NULL,0);

		if (estado == osOK) {
			if (strcmp("fin", rec) == 0){
				osThreadSuspend(printUARTHandle);
			}
			HAL_UART_Transmit(&huart1, (uint8_t*) rec, strlen(rec), 1000);

		} else if (estado == osErrorTimeout) {
			snprintf(msg, 100, "Timeout lectura print_queue\r\n");

			HAL_UART_Transmit(&huart1, (uint8_t*) msg, sizeof(msg), 1000);
		}
	}
  /* USER CODE END printUARTTask */
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
