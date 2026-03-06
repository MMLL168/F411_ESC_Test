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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ESC_PULSE_MIN_US            1000U
#define ESC_PULSE_MID_US            1500U
#define ESC_PULSE_MAX_US            2000U
#define ESC_PULSE_ESTOP_US          0U
#define UART_RX_TIMEOUT_MS          10U
#define UART_CMD_BUFFER_SIZE        16U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static uint16_t g_escPulseUs = ESC_PULSE_MIN_US;
static char g_uartCmdBuffer[UART_CMD_BUFFER_SIZE];
static uint8_t g_uartCmdIndex = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void UART_SendString(const char *text);
static uint16_t ESC_ClampPulseUs(uint16_t pulseUs);
static void ESC_SetPulseUs(uint16_t pulseUs);
static void ESC_EmergencyStop(void);
static void ESC_ProcessCommand(const char *cmd);
static void UART_ProcessInput(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void UART_SendString(const char *text)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

static uint16_t ESC_ClampPulseUs(uint16_t pulseUs)
{
  if (pulseUs < ESC_PULSE_MIN_US)
  {
    return ESC_PULSE_MIN_US;
  }

  if (pulseUs > ESC_PULSE_MAX_US)
  {
    return ESC_PULSE_MAX_US;
  }

  return pulseUs;
}

static void ESC_SetPulseUs(uint16_t pulseUs)
{
  g_escPulseUs = ESC_ClampPulseUs(pulseUs);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, g_escPulseUs);
}

static void ESC_EmergencyStop(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_PULSE_ESTOP_US);
  UART_SendString("ESTOP: PWM=0us\r\n");
}

static void ESC_ProcessCommand(const char *cmd)
{
  if ((strcmp(cmd, "1000") == 0) || (strcmp(cmd, "1") == 0))
  {
    ESC_SetPulseUs(ESC_PULSE_MIN_US);
    UART_SendString("OK: PWM=1000us\r\n");
    return;
  }

  if ((strcmp(cmd, "1500") == 0) || (strcmp(cmd, "5") == 0))
  {
    ESC_SetPulseUs(ESC_PULSE_MID_US);
    UART_SendString("OK: PWM=1500us\r\n");
    return;
  }

  if ((strcmp(cmd, "2000") == 0) || (strcmp(cmd, "2") == 0))
  {
    ESC_SetPulseUs(ESC_PULSE_MAX_US);
    UART_SendString("OK: PWM=2000us\r\n");
    return;
  }

  if ((strcmp(cmd, "STOP") == 0) || (strcmp(cmd, "ESTOP") == 0) ||
      (strcmp(cmd, "S") == 0) || (strcmp(cmd, "s") == 0))
  {
    ESC_EmergencyStop();
    return;
  }

  UART_SendString("ERR: cmd? use 1000/1500/2000/STOP\r\n");
}

static void UART_ProcessInput(void)
{
  uint8_t rxChar = 0U;

  if (HAL_UART_Receive(&huart2, &rxChar, 1U, UART_RX_TIMEOUT_MS) != HAL_OK)
  {
    return;
  }

  if ((rxChar == '\r') || (rxChar == '\n'))
  {
    if (g_uartCmdIndex > 0U)
    {
      g_uartCmdBuffer[g_uartCmdIndex] = '\0';
      ESC_ProcessCommand(g_uartCmdBuffer);
      g_uartCmdIndex = 0U;
    }
    return;
  }

  if ((rxChar < 32U) || (rxChar > 126U))
  {
    return;
  }

  if (g_uartCmdIndex < (UART_CMD_BUFFER_SIZE - 1U))
  {
    g_uartCmdBuffer[g_uartCmdIndex++] = (char)rxChar;
  }
  else
  {
    g_uartCmdIndex = 0U;
    UART_SendString("ERR: cmd too long\r\n");
  }
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  ESC_SetPulseUs(ESC_PULSE_MIN_US);
  UART_SendString("ESC ready: 1000us, cmd=1000/1500/2000/STOP\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    UART_ProcessInput();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
