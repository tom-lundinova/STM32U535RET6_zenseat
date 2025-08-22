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
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>

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

OSPI_HandleTypeDef hospi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_ICACHE_Init(void);
/* USER CODE BEGIN PFP */
void RunHyperRAMTest(void);
void OctoSPI_RegisterDump(OSPI_HandleTypeDef *hospi);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
  MX_USART3_UART_Init();
  MX_OCTOSPI1_Init();
  MX_ICACHE_Init();
  /* USER CODE BEGIN 2 */
  OctoSPI_RegisterDump(&hospi1);
  RunHyperRAMTest();

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    printf("Hello!\n");
    HAL_Delay(1000);

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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPI_HyperbusCfgTypeDef sHyperBusCfg = {0};
  HAL_OSPI_DLYB_CfgTypeDef HAL_OSPI_DLYB_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 4;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_HYPERBUS;
  hospi1.Init.DeviceSize = 24;
  hospi1.Init.ChipSelectHighTime = 8;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 2;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
  hospi1.Init.ChipSelectBoundary = 23;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED;
  hospi1.Init.MaxTran = 0;
  hospi1.Init.Refresh = 250;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  sHyperBusCfg.RWRecoveryTime = 3;
  sHyperBusCfg.AccessTime = 6;
  sHyperBusCfg.WriteZeroLatency = HAL_OSPI_LATENCY_ON_WRITE;
  sHyperBusCfg.LatencyMode = HAL_OSPI_FIXED_LATENCY;
  if (HAL_OSPI_HyperbusCfg(&hospi1, &sHyperBusCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_OSPI_DLYB_Cfg_Struct.Units = 0;
  HAL_OSPI_DLYB_Cfg_Struct.PhaseSel = 4;
  if (HAL_OSPI_DLYB_SetConfig(&hospi1, &HAL_OSPI_DLYB_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

// *******************
// Delay block tuning: (override CUBE MX settings)
// *******************
// Settings:
// #define TUNE_DELAY_BLOCK           // define to use tuning and print values
// #define RESTORE_CUBE_MX_SETTINGS   // define to restore original CUBE MX settings (print values only)

#ifdef TUNE_DELAY_BLOCK
  HAL_OSPI_DLYB_CfgTypeDef dlyb_cfg, dlyb_cfg_mx_settings;

  // save CUBE MX settings (done this way since we can't modify above CUBE MX generated code)
  dlyb_cfg_mx_settings.Units = HAL_OSPI_DLYB_Cfg_Struct.Units;
  dlyb_cfg_mx_settings.PhaseSel = HAL_OSPI_DLYB_Cfg_Struct.PhaseSel;

  HAL_OSPI_DLYB_GetConfig(&hospi1,&dlyb_cfg);
  printf("\n\nDelay block tuning:\n");
  
  while(1)
  {
    // bug in HAL code might cause the call to fail or return data == 0 so attempt repeatedly
    if(HAL_OSPI_DLYB_GetClockPeriod(&hospi1,&dlyb_cfg) != HAL_OK || dlyb_cfg.Units == 0 || dlyb_cfg.PhaseSel == 0)
    { 
      printf("HAL_OSPI_DLYB_GetClockPeriod() failed - retrying...\n");
      HAL_Delay(100);
    }
    else
      break;
  }

  printf("Units: %08" PRIX32 " PhaseSel %08" PRIX32 " after call to HAL_OSPI_DLYB_GetClockPeriod() \n", dlyb_cfg.Units, dlyb_cfg.PhaseSel);

  /*when DTR, PhaseSel is divided by 4 (emperic value)*/
  dlyb_cfg.PhaseSel /=4;

  #ifdef RESTORE_CUBE_MX_SETTINGS
    // don't use tuning result - restore original settings
    // HAL_OSPI_DLYB_Cfg_Struct.Units = 0;
    // HAL_OSPI_DLYB_Cfg_Struct.PhaseSel = 4;
    HAL_OSPI_DLYB_Cfg_Struct.Units = dlyb_cfg_mx_settings.Units;
    HAL_OSPI_DLYB_Cfg_Struct.PhaseSel = dlyb_cfg_mx_settings.PhaseSel; 
  #else
    // use tuning result
    HAL_OSPI_DLYB_Cfg_Struct.Units = dlyb_cfg.Units;
    HAL_OSPI_DLYB_Cfg_Struct.PhaseSel = dlyb_cfg.PhaseSel;
  #endif

  printf("Units: %08" PRIX32 " PhaseSel %08" PRIX32 " (values used)\n", HAL_OSPI_DLYB_Cfg_Struct.Units, HAL_OSPI_DLYB_Cfg_Struct.PhaseSel);

  if (HAL_OSPI_DLYB_SetConfig(&hospi1, &HAL_OSPI_DLYB_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }


#endif // TUNE_DELAY_BLOCK

  /* USER CODE END OCTOSPI1_Init 2 */

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
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HYPER_MUX_GPIO_Port, HYPER_MUX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HYPER_MUX_Pin */
  GPIO_InitStruct.Pin = HYPER_MUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HYPER_MUX_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void OctoSPI_RegisterDump(OSPI_HandleTypeDef *hospi)
{
  printf("\nOCTOSPI registers hex dump:\n");
  printf("----------------------------\n");
  printf("CR   : %08" PRIX32 "\n", hospi->Instance->CR);   
  printf("DCR1 : %08" PRIX32 "\n", hospi->Instance->DCR1); 
  printf("DCR2 : %08" PRIX32 "\n", hospi->Instance->DCR2);
  printf("DCR3 : %08" PRIX32 "\n", hospi->Instance->DCR3); 
  printf("DCR4 : %08" PRIX32 "\n", hospi->Instance->DCR4); 
  printf("SR   : %08" PRIX32 "\n", hospi->Instance->SR);   
  printf("FCR  : %08" PRIX32 "\n", hospi->Instance->FCR);  
  printf("DLR  : %08" PRIX32 "\n", hospi->Instance->DLR);  
  printf("AR   : %08" PRIX32 "\n", hospi->Instance->AR);   
  printf("DR   : %08" PRIX32 "\n", hospi->Instance->DR);   
  printf("PSMKR: %08" PRIX32 "\n", hospi->Instance->PSMKR);
  printf("PSMAR: %08" PRIX32 "\n", hospi->Instance->PSMAR);
  printf("PIR  : %08" PRIX32 "\n", hospi->Instance->PIR);  
  printf("CCR  : %08" PRIX32 "\n", hospi->Instance->CCR);  
  printf("TCR  : %08" PRIX32 "\n", hospi->Instance->TCR);  
  printf("IR   : %08" PRIX32 "\n", hospi->Instance->IR);   
  printf("ABR  : %08" PRIX32 "\n", hospi->Instance->ABR);  
  printf("LPTR : %08" PRIX32 "\n", hospi->Instance->LPTR); 
  printf("WPCCR: %08" PRIX32 "\n", hospi->Instance->WPCCR);
  printf("WPTCR: %08" PRIX32 "\n", hospi->Instance->WPTCR);
  printf("WPIR : %08" PRIX32 "\n", hospi->Instance->WPIR); 
  printf("WPABR: %08" PRIX32 "\n", hospi->Instance->WPABR);
  printf("WCCR : %08" PRIX32 "\n", hospi->Instance->WCCR); 
  printf("WTCR : %08" PRIX32 "\n", hospi->Instance->WTCR); 
  printf("WIR  : %08" PRIX32 "\n", hospi->Instance->WIR);  
  printf("WABR : %08" PRIX32 "\n", hospi->Instance->WABR); 
  printf("HLCR : %08" PRIX32 "\n", hospi->Instance->HLCR); 
  printf("DONE!\n");
}

// Define to use SRAM instead of HyperRAM:
// #define USE_LOCAL_SRAM     
// 
// Define only one:
// #define TEST_8_BIT
#define TEST_16_BIT
// #define TEST_32_BIT

uint8_t sram_memory_test_block[8192] = {0};

void RunHyperRAMTest(void)
{
  printf("HyperRAM test...\n");

  OSPI_HyperbusCmdTypeDef sCommand = {0};
  OSPI_MemoryMappedTypeDef sMemMappedCfg = {0};
    
  sCommand.AddressSpace = HAL_OSPI_MEMORY_ADDRESS_SPACE;
  sCommand.AddressSize  = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.DQSMode      = HAL_OSPI_DQS_ENABLE;
  sCommand.Address      = 0;
  sCommand.NbData       = 1;
  if (HAL_OSPI_HyperbusCmd(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
      Error_Handler();
  }
  
  // disable timeout (which will keep nCS low)
  sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;    
  sMemMappedCfg.TimeOutPeriod     = 0x00;  
  // sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_ENABLE;
  // sMemMappedCfg.TimeOutPeriod     = 0x08; // experimented with 6 clocks as specified in errata but system crashed
  if (HAL_OSPI_MemoryMapped(&hospi1, &sMemMappedCfg) != HAL_OK)
  {
      Error_Handler();
  }
  
  HAL_Delay(10);  // just to be sure..
  
  printf("Register dump post memory mapping\n");
  OctoSPI_RegisterDump(&hospi1);

  // "manufacture" a 32-bit value memory address that can be cast to point to anything:
  #ifdef USE_LOCAL_SRAM
    uint8_t *mem_ptr = &sram_memory_test_block[0];
    uint32_t mem_addr = (uint32_t)mem_ptr;
    printf("Using SRAM - mem_addr: %08" PRIX32 "\n", mem_addr);
  #else
    printf("Using HyperRAM - mem_addr: %08" PRIX32 "\n", OCTOSPI1_BASE);
  #endif

  while(true)
  {
#ifdef TEST_8_BIT
    // TEST 2
    // 1) write 4 bytes
    // 2) read 4 bytes
    // 3) dump read and write arrays
    // uint8_t w_array[] = {0x01, 0x23, 0x45, 0x67};
    uint8_t w_array[] = {0xFE, 0xDC, 0xBA, 0x98};
    uint8_t r_array[4];
    uint8_t index = 0;
    uint8_t address_offset = 0;

    for(address_offset = 0x5A, index = 0; index < 4; address_offset++, index++)
    {
      #ifdef USE_LOCAL_SRAM
      *((uint8_t*)(mem_addr + address_offset)) = w_array[index];       // write data to hyperram at increasing address
      #else
      *((uint8_t*)(OCTOSPI1_BASE + address_offset)) = w_array[index];       // write data to hyperram at increasing address
      #endif
    }

    for(address_offset = 0x5A, index = 0; index < 4; address_offset++, index++)
    {
      r_array[index] = 0x00;
      #ifdef USE_LOCAL_SRAM
      r_array[index] = *((uint8_t*)(mem_addr + address_offset));         // read data from hyperram into val
      #else
      r_array[index] = *((uint8_t*)(OCTOSPI1_BASE + address_offset));         // read data from hyperram into val
      #endif
    }

    printf("W: ");
    for(index = 0; index < 4; index++)
    {
      printf("%02X ", w_array[index]);
    }
    printf("\n");

    printf("R: ");
    for(index = 0; index < 4; index++)
    {
      printf("%02X ", r_array[index]);
    }
    printf("\n\n");

    HAL_Delay(100);   // slow down prints..
#endif

#ifdef TEST_32_BIT
    // TEST 3
    // 1) write 4 32-bit words 
    // 2) read 4 32-bit words
    // 3) dump read and write arrays
    // uint32_t w_array[] = {0xAAAAAAAA, 0xBBBBBBBB, 0xCCCCCCCC, 0xDDDDDDDD};
    uint32_t w_array[] = {0x01234567, 0x89ABCDEF, 0x01234567, 0x89ABCDEF};    
    uint32_t r_array[4];
    uint32_t index = 0;
    uint32_t address_offset = 0;

    for(address_offset = 0x5A, index = 0; index < 4; address_offset+=4, index++)
    {
      #ifdef USE_LOCAL_SRAM
      *((uint32_t*)(mem_addr + address_offset)) = w_array[index];       // write data to hyperram at increasing address      
      #else
      *((uint32_t*)(OCTOSPI1_BASE + address_offset)) = w_array[index];       // write data to hyperram at increasing address
      #endif
    }

    for(address_offset = 0x5A, index = 0; index < 4; address_offset+=4, index++)
    {
      r_array[index] = 0x00;
      #ifdef USE_LOCAL_SRAM
      r_array[index] = *((uint32_t*)(mem_addr + address_offset));         // read data from hyperram into val
      #else
      r_array[index] = *((uint32_t*)(OCTOSPI1_BASE + address_offset));         // read data from hyperram into val
      #endif
    }

    printf("W: ");
    for(index = 0; index < 4; index++)
    {
      printf("%08" PRIX32 " ", w_array[index]);
    }
    printf("\n");

    printf("R: ");
    for(index = 0; index < 4; index++)
    {
      printf("%08" PRIX32 " ", r_array[index]);
    }
    printf("\n\n");

    HAL_Delay(100);   // slow down prints..
#endif

#ifdef TEST_16_BIT
    // TEST 4
    // 1) write 4 16-bit words 
    // 2) read 4 16-bit words
    // 3) dump read and write arrays
    uint16_t w_array[] = {0x0123, 0x4567, 0x89AB, 0xCDEF};
    // uint16_t w_array[] = {0xFEDC, 0xBA98, 0x7654, 0x3210};
    uint16_t r_array[4];
    uint16_t index = 0;
    uint16_t address_offset = 0;

    for(address_offset = 0x5A, index = 0; index < 4; address_offset+=2, index++)
    {
      #ifdef USE_LOCAL_SRAM
      *((uint16_t*)(mem_addr + address_offset)) = w_array[index];       // write data to sram at increasing address
      #else
      *((uint16_t*)(OCTOSPI1_BASE + address_offset)) = w_array[index];       // write data to hyperram at increasing address
      #endif
    }

    for(address_offset = 0x5A, index = 0; index < 4; address_offset+=2, index++)
    {
      r_array[index] = 0x00;
      #ifdef USE_LOCAL_SRAM
      r_array[index] = *((uint16_t*)(mem_addr + address_offset));         // read data from sram into r_array
      #else
      r_array[index] = *((uint16_t*)(OCTOSPI1_BASE + address_offset));         // read data from hyperram into r_array
      #endif
    }

    printf("W: ");
    for(index = 0; index < 4; index++)
    {
      printf("%04X ", w_array[index]);
    }
    printf("\n");

    printf("R: ");
    for(index = 0; index < 4; index++)
    {
      printf("%04X ", r_array[index]);
    }
    printf("\n\n");

    HAL_Delay(100);   // slow down prints..
#endif


  }

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
