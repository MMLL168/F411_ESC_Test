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
#include <ctype.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  ESC_STATE_DISARMED = 0,
  ESC_STATE_ARMED,
  ESC_STATE_ESTOP
} ESC_State_t;

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
static ESC_State_t g_escState = ESC_STATE_DISARMED;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void UART_SendString(const char *text);
static uint16_t ESC_ClampPulseUs(uint16_t pulseUs);
static void ESC_ApplyPulseUs(uint16_t pulseUs);
static void ESC_SetPulseUs(uint16_t pulseUs);
static void ESC_SetDutyPercent(uint8_t dutyPercent);
static void ESC_SetState(ESC_State_t state);
static void ESC_EmergencyStop(void);
static void UART_NormalizeCommand(char *cmd);
static void UART_SanitizeCommand(char *cmd);
static uint8_t UART_TryParseUint16(const char *text, uint16_t *outValue);
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

static void ESC_ApplyPulseUs(uint16_t pulseUs)
{
  g_escPulseUs = ESC_ClampPulseUs(pulseUs);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, g_escPulseUs);
}

static void ESC_SetPulseUs(uint16_t pulseUs)
{
  ESC_ApplyPulseUs(pulseUs);
}

static void ESC_SetDutyPercent(uint8_t dutyPercent)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
  uint32_t compare = ((arr + 1UL) * dutyPercent) / 100UL;

  if (compare > arr)
  {
    compare = arr;
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)compare);
}

static void ESC_SetState(ESC_State_t state)
{
  g_escState = state;

  if (state == ESC_STATE_ESTOP)
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_PULSE_ESTOP_US);
    return;
  }

  ESC_ApplyPulseUs(ESC_PULSE_MIN_US);
}

static void ESC_EmergencyStop(void)
{
  ESC_SetState(ESC_STATE_ESTOP);
  UART_SendString("ESTOP: PWM=0us\r\n");
}

static void UART_NormalizeCommand(char *cmd)
{
  uint8_t i = 0U;

  while ((cmd[i] != '\0') && (i < (UART_CMD_BUFFER_SIZE - 1U)))
  {
    cmd[i] = (char)toupper((unsigned char)cmd[i]);
    i++;
  }
}

static void UART_SanitizeCommand(char *cmd)
{
  uint8_t start = 0U;
  uint8_t end = (uint8_t)strlen(cmd);
  uint8_t i = 0U;

  while ((cmd[start] == ' ') || (cmd[start] == '\t'))
  {
    start++;
  }

  while (end > start)
  {
    char tail = cmd[end - 1U];
    if ((tail == ' ') || (tail == '\t') || (tail == ':'))
    {
      end--;
    }
    else
    {
      break;
    }
  }

  while ((start + i) < end)
  {
    cmd[i] = cmd[start + i];
    i++;
  }

  cmd[i] = '\0';
}

static uint8_t UART_TryParseUint16(const char *text, uint16_t *outValue)
{
  char *endPtr = NULL;
  unsigned long parsed = strtoul(text, &endPtr, 10);

  if ((text == endPtr) || (*endPtr != '\0') || (parsed > 65535UL))
  {
    return 0U;
  }

  *outValue = (uint16_t)parsed;
  return 1U;
}

static void ESC_ProcessCommand(const char *cmd)
{
  uint16_t parsedValue = 0U;

  if (strcmp(cmd, "ARM") == 0)
  {
    if (g_escState == ESC_STATE_ESTOP)
    {
      UART_SendString("ERR: in ESTOP, use CLEAR first\r\n");
      return;
    }

    ESC_SetState(ESC_STATE_ARMED);
    UART_SendString("OK: ARMED, PWM=1000us\r\n");
    return;
  }

  if (strcmp(cmd, "DISARM") == 0)
  {
    ESC_SetState(ESC_STATE_DISARMED);
    UART_SendString("OK: DISARMED, PWM=1000us\r\n");
    return;
  }

  if ((strcmp(cmd, "CLEAR") == 0) || (strcmp(cmd, "RESET") == 0))
  {
    if (g_escState == ESC_STATE_ESTOP)
    {
      ESC_SetState(ESC_STATE_DISARMED);
      UART_SendString("OK: ESTOP cleared, now DISARMED\r\n");
    }
    else
    {
      UART_SendString("OK: no ESTOP, state unchanged\r\n");
    }
    return;
  }

  if (strcmp(cmd, "STATUS") == 0)
  {
    if (g_escState == ESC_STATE_ARMED)
    {
      UART_SendString("STATE: ARMED\r\n");
    }
    else if (g_escState == ESC_STATE_DISARMED)
    {
      UART_SendString("STATE: DISARMED\r\n");
    }
    else
    {
      UART_SendString("STATE: ESTOP\r\n");
    }
    return;
  }

  if ((strncmp(cmd, "PWM ", 4) == 0) && UART_TryParseUint16(&cmd[4], &parsedValue))
  {
    if (g_escState != ESC_STATE_ARMED)
    {
      UART_SendString("ERR: not ARMED, use ARM first\r\n");
      return;
    }

    ESC_SetPulseUs(parsedValue);
    UART_SendString("OK: PWM set by pulse(us)\r\n");
    return;
  }

  if ((strncmp(cmd, "DUTY ", 5) == 0) && UART_TryParseUint16(&cmd[5], &parsedValue))
  {
    if (g_escState != ESC_STATE_ARMED)
    {
      UART_SendString("ERR: not ARMED, use ARM first\r\n");
      return;
    }

    if (parsedValue > 100U)
    {
      UART_SendString("ERR: DUTY range 0..100\r\n");
      return;
    }

    ESC_SetDutyPercent((uint8_t)parsedValue);
    UART_SendString("OK: PWM set by duty(%)\r\n");
    return;
  }

  if ((strcmp(cmd, "1000") == 0) || (strcmp(cmd, "1") == 0))
  {
    if (g_escState != ESC_STATE_ARMED)
    {
      UART_SendString("ERR: not ARMED, use ARM first\r\n");
      return;
    }

    ESC_SetPulseUs(ESC_PULSE_MIN_US);
    UART_SendString("OK: PWM=1000us\r\n");
    return;
  }

  if ((strcmp(cmd, "1500") == 0) || (strcmp(cmd, "5") == 0))
  {
    if (g_escState != ESC_STATE_ARMED)
    {
      UART_SendString("ERR: not ARMED, use ARM first\r\n");
      return;
    }

    ESC_SetPulseUs(ESC_PULSE_MID_US);
    UART_SendString("OK: PWM=1500us\r\n");
    return;
  }

  if ((strcmp(cmd, "2000") == 0) || (strcmp(cmd, "2") == 0))
  {
    if (g_escState != ESC_STATE_ARMED)
    {
      UART_SendString("ERR: not ARMED, use ARM first\r\n");
      return;
    }

    ESC_SetPulseUs(ESC_PULSE_MAX_US);
    UART_SendString("OK: PWM=2000us\r\n");
    return;
  }

  if ((strcmp(cmd, "STOP") == 0) || (strcmp(cmd, "ESTOP") == 0) ||
      (strcmp(cmd, "S") == 0))
  {
    ESC_EmergencyStop();
    return;
  }

  if (UART_TryParseUint16(cmd, &parsedValue))
  {
    if (g_escState != ESC_STATE_ARMED)
    {
      UART_SendString("ERR: not ARMED, use ARM first\r\n");
      return;
    }

    if ((parsedValue < ESC_PULSE_MIN_US) || (parsedValue > ESC_PULSE_MAX_US))
    {
      UART_SendString("ERR: raw number range 1000..2000\r\n");
      return;
    }

    ESC_SetPulseUs(parsedValue);
    UART_SendString("OK: PWM set by raw number\r\n");
    return;
  }

  UART_SendString("ERR: cmd? ARM/DISARM/STATUS/PWM <1000..2000>/DUTY <0..100>/STOP/CLEAR\r\n");
}

static void UART_ProcessInput(void)
{
  uint8_t rxChar = 0U;

  if (HAL_UART_Receive(&huart2, &rxChar, 1U, UART_RX_TIMEOUT_MS) != HAL_OK)
  {
    return;
  }

  if ((rxChar == '\r') || (rxChar == '\n') || (rxChar == '/'))
  {
    if (g_uartCmdIndex > 0U)
    {
      g_uartCmdBuffer[g_uartCmdIndex] = '\0';
      UART_NormalizeCommand(g_uartCmdBuffer);
      UART_SanitizeCommand(g_uartCmdBuffer);
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

  ESC_SetState(ESC_STATE_DISARMED);
  UART_SendString("ESC ready: DISARMED(1000us). Use ARM first.\r\n");
  UART_SendString("Ctrl: PWM <1000..2000>, DUTY <0..100>, DISARM/STOP/CLEAR/STATUS\r\n");
  UART_SendString("Tip: use '/' to chain commands, e.g. ARM/PWM 1200/PWM 1500\r\n");

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
