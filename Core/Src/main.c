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
#include <stdio.h>  /* 添加 stdio.h 用於 sprintf 函數 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  ESC_STATE_DISARMED = 0,
  ESC_STATE_ARMED,
  ESC_STATE_ESTOP
} ESC_State_t;

typedef enum
{
  ESC_MODE_PWM = 0,
  ESC_MODE_DSHOT
} ESC_Mode_t;

typedef enum
{
  DSHOT_RATE_150 = 0,
  DSHOT_RATE_300,
  DSHOT_RATE_600,
  DSHOT_RATE_1200
} DSHOT_Rate_t;

/* DSHOT 特殊命令定義 */
typedef enum {
  DSHOT_CMD_MOTOR_STOP = 0,
  DSHOT_CMD_BEEP1 = 1,
  DSHOT_CMD_BEEP2 = 2,
  DSHOT_CMD_BEEP3 = 3,
  DSHOT_CMD_BEEP4 = 4,
  DSHOT_CMD_BEEP5 = 5,
  DSHOT_CMD_ESC_INFO = 6,
  DSHOT_CMD_SPIN_DIRECTION_1 = 7,
  DSHOT_CMD_SPIN_DIRECTION_2 = 8,
  DSHOT_CMD_3D_MODE_OFF = 9,
  DSHOT_CMD_3D_MODE_ON = 10,
  DSHOT_CMD_SETTINGS_REQUEST = 11,
  DSHOT_CMD_SAVE_SETTINGS = 12,
  // 13-19: Reserved
  DSHOT_CMD_START_BOOTLOADER = 20,
  DSHOT_CMD_ROTATE_NORMAL = 21,
  DSHOT_CMD_LED0_ON = 22,
  DSHOT_CMD_LED1_ON = 23,
  DSHOT_CMD_LED2_ON = 24,
  DSHOT_CMD_LED3_ON = 25,
  DSHOT_CMD_LED0_OFF = 26,
  DSHOT_CMD_LED1_OFF = 27,
  DSHOT_CMD_LED2_OFF = 28,
  DSHOT_CMD_LED3_OFF = 29
  // 30-47: Reserved for future use
} DSHOT_Cmd_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ESC_PULSE_MIN_US            1000U
#define ESC_PULSE_MID_US            1500U
#define ESC_PULSE_MAX_US            2000U
#define ESC_PULSE_ESTOP_US          0U
#define UART_RX_TIMEOUT_MS          0U   /* 改為 0 以實現非阻塞接收，確保 DSHOT 發送流暢 */
#define UART_CMD_BUFFER_SIZE        32U  /* 增加命令緩衝區以支持更長命令 */

/* DSHOT 相關定義 */
#define DSHOT_DMA_BUFFER_SIZE       18   /* 16位數據 + 1位零結尾 + 1位保留 */
#define DSHOT_MIN_THROTTLE          48U  /* 最小油門值 (約 0%) */
#define DSHOT_MAX_THROTTLE          2047U /* 最大油門值 (100%) */
#define DSHOT_TELEMETRY_BIT         1U   /* 開啟遙測位 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static volatile uint16_t g_escPulseUs = ESC_PULSE_MIN_US;
static char g_uartCmdBuffer[UART_CMD_BUFFER_SIZE];
static uint8_t g_uartCmdIndex = 0U;
static volatile ESC_State_t g_escState = ESC_STATE_DISARMED;
static volatile ESC_Mode_t g_escMode = ESC_MODE_PWM;  /* 默認為 PWM 模式 */
static volatile uint16_t g_dshotThrottle = 0;         /* 當前 DSHOT 油門值 */
static DSHOT_Rate_t g_dshotRate = DSHOT_RATE_600;  /* 默認為 DSHOT600 */

/* DSHOT DMA 緩衝區 - 用於存放生成的信號時序 */
static uint16_t g_dshotDMABuffer[DSHOT_DMA_BUFFER_SIZE] = {0};
static uint8_t g_dshotConfigured = 0;  /* DSHOT 初始化標記 */

/* DSHOT 動態時序參數 (由 ConfigureTimer 計算) */
static uint16_t g_dshotT0H = 0; /* 0 bit high time (compare value) */
static uint16_t g_dshotT1H = 0; /* 1 bit high time (compare value) */

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

/* DSHOT 功能原型 */
static uint8_t DSHOT_SetupTimer(void);
static uint8_t DSHOT_CalcCRC(uint16_t value);
static uint16_t DSHOT_PreparePacket(uint16_t value, uint8_t telemetry);
static void DSHOT_PrepareBuffer(uint16_t packet);
static uint8_t DSHOT_SendCommand(uint16_t value, uint8_t telemetry);
static void DSHOT_ConfigureTimer(DSHOT_Rate_t rate);
static void PWM_ConfigureTimer(void);

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
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, g_escPulseUs);
}

static void ESC_SetPulseUs(uint16_t pulseUs)
{
  ESC_ApplyPulseUs(pulseUs);
}

static void ESC_SetDutyPercent(uint8_t dutyPercent)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
  uint32_t compare = ((arr + 1UL) * dutyPercent) / 100UL;

  if (compare > arr)
  {
    compare = arr;
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t)compare);
}

static void ESC_SetState(ESC_State_t state)
{
  g_escState = state;

  if (state == ESC_STATE_ESTOP)
  {
    if (g_escMode == ESC_MODE_PWM)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ESC_PULSE_ESTOP_US);
    }
    else
    {
      g_dshotThrottle = DSHOT_CMD_MOTOR_STOP;
    }
    return;
  }

  if (g_escMode == ESC_MODE_PWM)
  {
    ESC_ApplyPulseUs(ESC_PULSE_MIN_US);
  }
  else
  {
    g_dshotThrottle = (state == ESC_STATE_ARMED) ? DSHOT_MIN_THROTTLE : 0;
  }
}

static void ESC_EmergencyStop(void)
{
  ESC_SetState(ESC_STATE_ESTOP);
  UART_SendString("ESTOP: Motor stopped\r\n");
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

/* DSHOT 核心函數 */
static uint8_t DSHOT_CalcCRC(uint16_t dataWithTelemetry)
{
  /* 
   * DSHOT 校驗和計算
   * CRC = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F
   * data 為 12 bits (Throttle 11 bits + Telemetry 1 bit)
   */
  uint8_t crc = 0;
  uint16_t v = dataWithTelemetry;

  for (int i = 0; i < 3; i++)
  {
    crc ^= v & 0x0F;
    v >>= 4;
  }
  
  return crc;
}

static uint16_t DSHOT_PreparePacket(uint16_t value, uint8_t telemetry)
{
  uint16_t packet;
  
  /* 限制值範圍 */
  if (value > DSHOT_MAX_THROTTLE)
  {
    value = DSHOT_MAX_THROTTLE;
  }
  
  if (value < DSHOT_MIN_THROTTLE && value != 0)
  {
    value = DSHOT_MIN_THROTTLE;
  }
  
  /* 組合 12-bit 數據 (Throttle + Telemetry) */
  uint16_t data = (value << 1) | (telemetry ? 1 : 0);
  
  /* 生成完整封包: 12-bit 數據 + 4-bit CRC */
  packet = (data << 4) | DSHOT_CalcCRC(data);
  return packet;
}

static void DSHOT_PrepareBuffer(uint16_t packet)
{
  /* 
   * 將 16-bit DSHOT 數據包轉換為 DMA 緩衝區
   * 每位根據 0/1 設置為不同的 PWM 佔空比
   */
  const uint16_t bit0 = g_dshotT0H;
  const uint16_t bit1 = g_dshotT1H;
  
  /* 將 16-bit 包填入 DMA 緩衝區 */
  for (int i = 0; i < 16; i++)
  {
    if (packet & (1 << (15 - i)))
    {
      g_dshotDMABuffer[i] = bit1;
    }
    else
    {
      g_dshotDMABuffer[i] = bit0;
    }
  }
  
  /* 末尾清零確保最後信號為低電平 */
  g_dshotDMABuffer[16] = 0;
  g_dshotDMABuffer[17] = 0;
}

static uint8_t DSHOT_SetupTimer(void)
{
  /* 初始化 TIM2 和 DMA 用於 DSHOT 通訊 */
  if (!g_dshotConfigured)
  {
    /* 配置 TIM2 用於 DSHOT 協議 */
    DSHOT_ConfigureTimer(g_dshotRate);
    
    /* 啟動 DMA */
    HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
    
    g_dshotConfigured = 1;
  }
  
  return g_dshotConfigured;
}

static void DSHOT_ConfigureTimer(DSHOT_Rate_t rate)
{
  /* 根據 DSHOT 速率配置定時器 */
  uint32_t timerClock = HAL_RCC_GetPCLK1Freq() * 2; /* TIM2 is on APB1 */
  uint32_t targetFreq;
  uint16_t prescaler, period;
  
  /* 
   * DSHOT 頻率設定:
   * DSHOT600 = 600 kbit/s -> 600 kHz PWM
   * DSHOT300 = 300 kbit/s -> 300 kHz PWM
   * DSHOT150 = 150 kbit/s -> 150 kHz PWM
   */
  switch (rate)
  {
    case DSHOT_RATE_150:
      targetFreq = 150000;
      break;
    case DSHOT_RATE_300:
      targetFreq = 300000;
      break;
    case DSHOT_RATE_1200:
      targetFreq = 1200000;
      break;
    case DSHOT_RATE_600:
    default:
      targetFreq = 600000;
      break;
  }
  
  /* 計算預分頻器和周期 */
  prescaler = 0;
  period = (uint16_t)(timerClock / (targetFreq * (prescaler + 1))) - 1;
  
  /* 
   * 計算 T0H 和 T1H 的 Compare 值
   * T0H: 0 bit high time = 37.5% of period
   * T1H: 1 bit high time = 75.0% of period
   */
  g_dshotT0H = (uint16_t)((uint32_t)(period + 1) * 375 / 1000);
  g_dshotT1H = (uint16_t)((uint32_t)(period + 1) * 750 / 1000);

  /* 更新定時器配置 */
  htim2.Init.Prescaler = prescaler;
  htim2.Init.Period = period;
  HAL_TIM_Base_Init(&htim2);
  HAL_TIM_PWM_Init(&htim2); /* 確保 PWM 模式相關旗標被正確初始化 */
  
  /* 配置 PWM 輸出通道 */
  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  /* 啟用 Preload 以確保 DMA 更新時序正確 */
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
}

static void PWM_ConfigureTimer(void)
{
  /* 停止 DSHOT 可能正在使用的 DMA */
  HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

  /* 重置計數器，避免切換時 CNT > Period 導致延遲 */
  __HAL_TIM_SET_COUNTER(&htim2, 0);

  /* 配置 TIM2 為 400Hz PWM (PA0) */
  /* Clock = 100MHz (APB1 x2) */
  /* Prescaler = 99 -> 1MHz Counter Clock (1us resolution) */
  /* Period = 2499 -> 2500 ticks = 2.5ms = 400Hz */
  
  htim2.Init.Prescaler = 99;
  htim2.Init.Period = 2499;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = g_escPulseUs;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* 啟動 PWM */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  
  /* 標記 DSHOT 需要重新配置 */
  g_dshotConfigured = 0;
}

static uint8_t DSHOT_SendCommand(uint16_t value, uint8_t telemetry)
{
  /* 檢查並設置 DSHOT 定時器 */
  if (!DSHOT_SetupTimer())
  {
    return 0;
  }
  
  /* 準備 DSHOT 數據包 */
  uint16_t packet = DSHOT_PreparePacket(value, telemetry);
  
  /* 將數據包填入 DMA 緩衝區 */
  DSHOT_PrepareBuffer(packet);
  
  /* 重置計數器以確保從頭開始發送 */
  __HAL_TIM_SET_COUNTER(&htim2, 0);

  /* 使用 DMA 發送 DSHOT 命令 */
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*)g_dshotDMABuffer, DSHOT_DMA_BUFFER_SIZE);
  
  return 1;
}

static void ESC_ProcessCommand(const char *cmd)
{
  uint16_t parsedValue = 0U;
  char buffer[32];

    /* 處理模式切換命令 */
  if (strncmp(cmd, "MODE ", 5) == 0)
  {
    if (strcmp(&cmd[5], "PWM") == 0)
    {
      g_escMode = ESC_MODE_PWM;
      if (g_escState != ESC_STATE_ESTOP)
      {
        PWM_ConfigureTimer();
        
        if (g_escState == ESC_STATE_ARMED)
        {
          ESC_SetPulseUs(g_escPulseUs);
        }
        else
        {
          ESC_SetPulseUs(ESC_PULSE_MIN_US);
        }
      }
      UART_SendString("OK: Mode set to PWM\r\n");
      return;
    }
    else if (strcmp(&cmd[5], "DSHOT") == 0)
    {
      g_escMode = ESC_MODE_DSHOT;
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); /* 停止 PWM */
      
      if (!DSHOT_SetupTimer())
      {
        UART_SendString("ERR: Failed to setup DSHOT\r\n");
        g_escMode = ESC_MODE_PWM;
        PWM_ConfigureTimer();
        return;
      }
      UART_SendString("OK: Mode set to DSHOT\r\n");
      
      /* 切換模式時重置油門值 */
      if (g_escState == ESC_STATE_ARMED) g_dshotThrottle = DSHOT_MIN_THROTTLE;
      else g_dshotThrottle = 0;
      
      return;
    }
    else
    {
      UART_SendString("ERR: Invalid mode, use PWM or DSHOT\r\n");
      return;
    }
  }
  
  if (strcmp(cmd, "ARM") == 0)
  {
    if (g_escState == ESC_STATE_ESTOP)
    {
      UART_SendString("ERR: in ESTOP, use CLEAR first\r\n");
      return;
    }

    ESC_SetState(ESC_STATE_ARMED);
    UART_SendString("OK: ARMED, ESC ready\r\n");
    return;
  }

  if (strcmp(cmd, "DISARM") == 0)
  {
    ESC_SetState(ESC_STATE_DISARMED);
    UART_SendString("OK: DISARMED, ESC safe\r\n");
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
    sprintf(buffer, "STATE: %s, MODE: %s\r\n",
            g_escState == ESC_STATE_ARMED ? "ARMED" :
            g_escState == ESC_STATE_DISARMED ? "DISARMED" : "ESTOP",
            g_escMode == ESC_MODE_PWM ? "PWM" : "DSHOT");
    UART_SendString(buffer);
    
    if (g_escMode == ESC_MODE_DSHOT)
    {
      sprintf(buffer, "DSHOT: Rate=%d\r\n", 
              g_dshotRate == DSHOT_RATE_150 ? 150 :
              g_dshotRate == DSHOT_RATE_300 ? 300 :
              g_dshotRate == DSHOT_RATE_600 ? 600 : 1200);
      UART_SendString(buffer);
    }
    return;
  }

  /* 處理 DSHOT 速率設置命令 */
  if (strncmp(cmd, "DSRATE ", 7) == 0)
  {
    if (g_escMode != ESC_MODE_DSHOT)
    {
      UART_SendString("ERR: Must be in DSHOT mode first\r\n");
      return;
    }
    
    if (strcmp(&cmd[7], "150") == 0)
    {
      g_dshotRate = DSHOT_RATE_150;
      DSHOT_ConfigureTimer(g_dshotRate);
      UART_SendString("OK: DSHOT rate set to 150\r\n");
      return;
    }
    else if (strcmp(&cmd[7], "300") == 0)
    {
      g_dshotRate = DSHOT_RATE_300;
      DSHOT_ConfigureTimer(g_dshotRate);
      UART_SendString("OK: DSHOT rate set to 300\r\n");
      return;
    }
    else if (strcmp(&cmd[7], "600") == 0)
    {
      g_dshotRate = DSHOT_RATE_600;
      DSHOT_ConfigureTimer(g_dshotRate);
      UART_SendString("OK: DSHOT rate set to 600\r\n");
      return;
    }
    else if (strcmp(&cmd[7], "1200") == 0)
    {
      g_dshotRate = DSHOT_RATE_1200;
      DSHOT_ConfigureTimer(g_dshotRate);
      UART_SendString("OK: DSHOT rate set to 1200\r\n");
      return;
    }
    else
    {
      UART_SendString("ERR: Invalid DSHOT rate, use 150/300/600/1200\r\n");
      return;
    }
  }

  /* 處理 DSHOT 特殊命令 */
  if (strncmp(cmd, "DSCMD ", 6) == 0 && UART_TryParseUint16(&cmd[6], &parsedValue))
  {
    if (g_escMode != ESC_MODE_DSHOT)
    {
      UART_SendString("ERR: Must be in DSHOT mode first\r\n");
      return;
    }
    
    if (parsedValue > 47)
    {
      UART_SendString("ERR: DSHOT command range is 0-47\r\n");
      return;
    }
    
    DSHOT_SendCommand(parsedValue, DSHOT_TELEMETRY_BIT);
    sprintf(buffer, "OK: DSHOT special command %d sent\r\n", parsedValue);
    UART_SendString(buffer);
    return;
  }

  /* 處理 DSHOT 油門值 */
  if (strncmp(cmd, "DSHOT ", 6) == 0 && UART_TryParseUint16(&cmd[6], &parsedValue))
  {
    if (g_escMode != ESC_MODE_DSHOT)
    {
      UART_SendString("ERR: Must be in DSHOT mode first\r\n");
      return;
    }
    
    if (g_escState != ESC_STATE_ARMED)
    {
      UART_SendString("ERR: not ARMED, use ARM first\r\n");
      return;
    }
    
    if ((parsedValue < DSHOT_MIN_THROTTLE || parsedValue > DSHOT_MAX_THROTTLE) && parsedValue != 0)
    {
      sprintf(buffer, "ERR: DSHOT throttle range %d-%d\r\n", DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
      UART_SendString(buffer);
      return;
    }
    
    g_dshotThrottle = parsedValue;
    sprintf(buffer, "OK: DSHOT throttle set to %d\r\n", parsedValue);
    UART_SendString(buffer);
    return;
  }

  /* 處理 PWM 相關命令 */
  if (g_escMode == ESC_MODE_PWM)
  {
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
  }

  if ((strcmp(cmd, "STOP") == 0) || (strcmp(cmd, "ESTOP") == 0) ||
      (strcmp(cmd, "S") == 0))
  {
    ESC_EmergencyStop();
    return;
  }

  /* 提供命令幫助信息 */
  UART_SendString("ERR: Unknown command\r\n");
  UART_SendString("Common: ARM/DISARM/STATUS/STOP/CLEAR\r\n");
  UART_SendString("Modes: MODE PWM/MODE DSHOT\r\n");
  if (g_escMode == ESC_MODE_PWM)
  {
    UART_SendString("PWM: PWM <1000..2000>/DUTY <0..100>\r\n");
  }
  else
  {
    UART_SendString("DSHOT: DSHOT <48..2047>/DSCMD <0..47>/DSRATE <150/300/600/1200>\r\n");
  }
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

  /* 預設啟動 PWM 模式 (PA0) */
  PWM_ConfigureTimer();

  ESC_SetState(ESC_STATE_ARMED);
  ESC_SetPulseUs(ESC_PULSE_MID_US);
  UART_SendString("ESC ready: ARMED(1500us) for connection test.\r\n");
  UART_SendString("Ctrl: ARM/DISARM/STATUS, MODE PWM/DSHOT\r\n");
  UART_SendString("PWM: PWM <1000..2000>, DUTY <0..100>\r\n");
  UART_SendString("DSHOT: DSHOT <48..2047>, DSCMD <0..47>\r\n");
  UART_SendString("Tip: use '/' to chain commands\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t lastDshotTime = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    UART_ProcessInput();

    /* 在 DSHOT 模式下持續發送封包 (約 1kHz，使用非阻塞方式) */
    if (g_escMode == ESC_MODE_DSHOT)
    {
      if ((HAL_GetTick() - lastDshotTime) >= 1)
      {
        DSHOT_SendCommand(g_dshotThrottle, 0);
        lastDshotTime = HAL_GetTick();
      }
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
