/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "bme68x.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for SharedMemoryTas */
osThreadId_t SharedMemoryTasHandle;
const osThreadAttr_t SharedMemoryTas_attributes = {
  .name = "SharedMemoryTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sensorQueue */
osMessageQueueId_t sensorQueueHandle;
const osMessageQueueAttr_t sensorQueue_attributes = {
  .name = "sensorQueue"
};
/* USER CODE BEGIN PV */
uint8_t rx = '1';
uint8_t x;
uint32_t counter=0;

struct bme68x_dev bme;
struct bme68x_data data;
uint8_t n_fields;
uint8_t dev_addr = 0x76;
//volatile uint32_t *shared_data = (uint32_t *)0x38001000;

typedef struct
{
    float temperature;
    float humidity;
    float pressure;
} SensorData_t;

#define SHARED_MEM_ADDR ((uint32_t)0x38001000U)

volatile SensorData_t * const shared_data =
    (volatile SensorData_t *)SHARED_MEM_ADDR;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void SensorTaskFunc(void *argument);
void SharedMemoryTaskFunc(void *argument);

/* USER CODE BEGIN PFP */
int8_t bme68x_i2c_read(uint8_t reg_addr,
                       uint8_t *reg_data,
                       uint32_t len,
                       void *intf_ptr);

int8_t bme68x_i2c_write(uint8_t reg_addr,
                        const uint8_t *reg_data,
                        uint32_t len,
                        void *intf_ptr);

void bme68x_delay_us(uint32_t period,
                     void *intf_ptr);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int8_t bme68x_i2c_read(uint8_t reg_addr,
                       uint8_t *reg_data,
                       uint32_t len,
                       void *intf_ptr)
{
    HAL_I2C_Mem_Read(&hi2c1,
                     dev_addr << 1,
                     reg_addr,
                     I2C_MEMADD_SIZE_8BIT,
                     reg_data,
                     len,
                     HAL_MAX_DELAY);

    return 0;
}

int8_t bme68x_i2c_write(uint8_t reg_addr,
                        const uint8_t *reg_data,
                        uint32_t len,
                        void *intf_ptr)
{
    HAL_I2C_Mem_Write(&hi2c1,
                      dev_addr << 1,
                      reg_addr,
                      I2C_MEMADD_SIZE_8BIT,
                      (uint8_t*)reg_data,
                      len,
                      HAL_MAX_DELAY);

    return 0;
}

void bme68x_delay_us(uint32_t period,
                     void *intf_ptr)
{
    HAL_Delay((period + 999) / 1000);
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

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
//  HAL_PWREx_ClearPendingEvent();
//  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  bme.intf = BME68X_I2C_INTF;
  bme.read = bme68x_i2c_read;
  bme.write = bme68x_i2c_write;
  bme.delay_us = bme68x_delay_us;
  bme.intf_ptr = NULL;
  bme.amb_temp = 25;

  bme68x_init(&bme);

  struct bme68x_conf conf;

  conf.os_hum  = BME68X_OS_2X;
  conf.os_pres = BME68X_OS_4X;
  conf.os_temp = BME68X_OS_8X;
  conf.filter  = BME68X_FILTER_SIZE_3;

  bme68x_set_conf(&conf, &bme);
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
  /* creation of sensorQueue */
  sensorQueueHandle = osMessageQueueNew (10, sizeof(SensorData_t), &sensorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(SensorTaskFunc, NULL, &SensorTask_attributes);

  /* creation of SharedMemoryTas */
  SharedMemoryTasHandle = osThreadNew(SharedMemoryTaskFunc, NULL, &SharedMemoryTas_attributes);

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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void *argument)
//{
//  /* USER CODE BEGIN 5 */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END 5 */
//}

/* USER CODE BEGIN Header_SensorTaskFunc */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorTaskFunc */
void SensorTaskFunc(void *argument)
{
  /* USER CODE BEGIN SensorTaskFunc */
  /* Infinite loop */
	SensorData_t sensor;
	    uint8_t n_fields;

	    for(;;)
	    {
	        bme68x_set_op_mode(
	                BME68X_FORCED_MODE,
	                &bme);

	        osDelay(200);

	        bme68x_get_data(
	                BME68X_FORCED_MODE,
	                &data,
	                &n_fields,
	                &bme);

	        sensor.temperature = data.temperature;
	        sensor.humidity    = data.humidity;
	        sensor.pressure    = data.pressure;

	        osMessageQueuePut(
	                sensorQueueHandle,
	                &sensor,
	                0,
	                0);

	        osDelay(1000);
	    }
  /* USER CODE END SensorTaskFunc */
}

/* USER CODE BEGIN Header_SharedMemoryTaskFunc */
/**
* @brief Function implementing the SharedMemoryTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SharedMemoryTaskFunc */
void SharedMemoryTaskFunc(void *argument)
{
  /* USER CODE BEGIN SharedMemoryTaskFunc */
  /* Infinite loop */
	SensorData_t sensor;

	    for(;;)
	    {
	        osMessageQueueGet(
	                sensorQueueHandle,
	                &sensor,
	                NULL,
	                osWaitForever);

	        while(HAL_HSEM_FastTake(0) != HAL_OK);

	        shared_data->temperature =
	                sensor.temperature;

	        shared_data->humidity =
	                sensor.humidity;

	        shared_data->pressure =
	                sensor.pressure;

	        HAL_HSEM_Release(0,0);
	    }
  /* USER CODE END SharedMemoryTaskFunc */
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
#ifdef USE_FULL_ASSERT
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
