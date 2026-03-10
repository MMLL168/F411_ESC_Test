# F411_ESC_Test 專案筆記

## 1. 專案目的（依目前程式與 ioc 設定推定）
- 專案名稱：`F411_ESC_Test`
- MCU：`STM32F411R(C-E)Tx`（STM32F411 系列）
- 開發目標：
  - 建立 ESC（Electronic Speed Controller）測試基礎。
  - 使用 `TIM3 CH1 (PA6)` 輸出標準伺服/ESC 類 PWM（目前為 50Hz，預設脈寬 1500us）。
  - 使用 `TIM2 CH1 (PA0)` 搭配 `DMA1_Stream5`，作為記憶體到周邊的 PWM 資料搬運通道（可擴充為波形串流或高速更新占空比）。
  - 提供 `USART2 (PA2/PA3)` 作為除錯或命令介面。

## 2. 專案架構與目錄職責
- `Core/Inc`：應用層標頭檔（各外設 init 介面、腳位定義、HAL 設定入口）。
- `Core/Src`：應用層與 CubeMX 產生程式碼（`main.c`、外設初始化、IRQ、MSP 等）。
- `Drivers/CMSIS`：ARM CMSIS 核心與裝置層定義。
- `Drivers/STM32F4xx_HAL_Driver`：STM32 HAL/LL 驅動實作。
- `cmake/`：工具鏈與 CubeMX 導出的 CMake 組態。
- `startup_stm32f411xe.s`：啟動碼與向量表。
- `STM32F411XX_FLASH.ld`：連結腳本（Flash/RAM 配置）。
- `F411_ESC_Test.ioc`：CubeMX 專案設定來源（外設、時鐘、腳位、多工）。

## 3. 主要檔案功能對照
- `CMakeLists.txt`
  - 定義專案 `F411_ESC_Test`，語言 C/ASM，預設 `Debug`。
  - 透過 `add_subdirectory(cmake/stm32cubemx)` 引入 CubeMX 產生的來源清單。
- `cmake/stm32cubemx/CMakeLists.txt`
  - 統一管理 `Core/Src/*.c`、HAL Driver 來源、Include 路徑與巨集（`USE_HAL_DRIVER`、`STM32F411xE`）。
- `CMakePresets.json`
  - 以 Ninja + `gcc-arm-none-eabi.cmake` 建置。
  - 提供 `Debug` 與 `Release` preset。
- `Core/Src/main.c`
  - 系統入口：`HAL_Init()` -> `SystemClock_Config()` -> `MX_GPIO_Init()` -> `MX_DMA_Init()` -> `MX_TIM2_Init()` -> `MX_TIM3_Init()` -> `MX_USART2_UART_Init()`。
  - 目前主迴圈為空，尚未加入 ESC 控制流程。
- `Core/Src/tim.c`
  - `TIM2`：`Period=166`，PWM CH1，並掛接 DMA（`DMA1_Stream5/Channel3`，M2P，halfword）。
  - `TIM3`：`Prescaler=99`、`Period=2499`，對應 1MHz 計數基準與 2.5ms 週期（400Hz），`Pulse=1500`。
- `Core/Src/dma.c`
  - 啟用 DMA1 時鐘與 `DMA1_Stream5_IRQn` 中斷。
- `Core/Src/stm32f4xx_it.c`
  - 提供 Cortex-M 例外處理與 `DMA1_Stream5_IRQHandler()`，呼叫 `HAL_DMA_IRQHandler(&hdma_tim2_ch1)`。
- `Core/Src/usart.c`
  - USART2 設定為 `115200 8N1`，TX/RX 模式。
- `Core/Src/gpio.c`
  - 設定按鍵 `PC13(B1)` 中斷下降沿、`PA5(LD2)` 輸出。
- `Core/Inc/main.h`
  - 集中定義板上腳位（`B1`、`LD2`、`USART2`、SWD 腳位）。

## 4. 目前程式狀態判讀
- 已完成時鐘、GPIO、DMA、PWM、UART 的初始化骨架。
- 已實作功能：
  - **PWM 控制**：支援 1000~2000us 脈寬控制，頻率 400Hz。
  - **DSHOT 控制**：支援 DSHOT150/300/600/1200 協定，包含特殊命令與遙測位。
  - **UART 命令介面**：支援 `ARM/DISARM`、`PWM/DUTY`、`DSHOT` 等指令。
  - **安全機制**：具備 Arming 狀態機與緊急停止 (ESTOP) 功能。

## 5. 建議的後續實作方向
- 在 `main.c` 使用者區塊補上：
  - TIM3 啟動與脈寬更新 API（以 us 為單位封裝）。
  - ESC arming 與安全輸出狀態機。
  - 若要做波形串流，加入 TIM2 DMA buffer 與完成中斷處理。
- 增加 UART 命令介面：
  - 設計簡單命令（例如設定 pulse、查詢狀態、急停）。
- 建立測試流程：
  - 上電安全輸出檢查。
  - 範圍測試（1000/1500/2000us）。
  - 異常條件測試（無效命令、DMA 錯誤）。

## 6. 判讀依據
- 主要依據：`F411_ESC_Test.ioc`、`Core/Src/main.c`、`Core/Src/tim.c`、`Core/Src/dma.c`、`Core/Src/usart.c`、`Core/Src/stm32f4xx_it.c`、`CMakeLists.txt`、`cmake/stm32cubemx/CMakeLists.txt`、`CMakePresets.json`。
- 本文件是以目前版本程式的靜態分析結果整理，後續若加入 user code，需同步更新。

## 7. 學習筆記 - DSHOT 協定實作細節
### 7.1 DSHOT 訊號原理
- **數位協定**：不同於傳統 PWM 依賴脈寬長度，DSHOT 是純數位訊號，抗干擾能力更強。
- **編碼方式**：透過 PWM 的佔空比 (Duty Cycle) 來表示位元 `0` 與 `1`。
  - `0`：佔空比約 37.5% (T0H)。
  - `1`：佔空比約 75.0% (T1H)。

### 7.2 STM32 實作機制 (TIM + DMA)
- **核心思路**：不使用 GPIO Bit-banging，而是利用 Timer 的 PWM 功能產生精確波形，並透過 DMA 自動搬運數據，釋放 CPU 資源。
- **DMA Buffer 設計**：
  - 每個 DSHOT 封包為 16 bits。
  - 為了用 PWM 模擬這 16 bits，我們建立一個大小為 18 的 `uint16_t` 陣列。
  - 前 16 個字組填入對應 `0` 或 `1` 的 Compare 值 (CCR)。
  - 最後 2 個字組填入 `0`，確保傳輸結束後 PWM 輸出維持在低電位 (Idle Low)。

### 7.3 封包結構
- **總長度**：16 bits
  - **Throttle (11 bits)**：油門數值 (0-2047)。其中 0-47 為特殊命令，48-2047 為油門。
  - **Telemetry (1 bit)**：遙測請求位。若設為 1，ESC 會在收到封包後回傳數據。
  - **CRC (4 bits)**：校驗碼，計算公式：`CRC = (Value ^ (Value >> 4) ^ (Value >> 8)) & 0x0F`。

## 7. 學習筆記 - DSHOT 協定實作細節
### 7.1 DSHOT 訊號原理
- **數位協定**：不同於傳統 PWM 依賴脈寬長度，DSHOT 是純數位訊號，抗干擾能力更強。
- **編碼方式**：透過 PWM 的佔空比 (Duty Cycle) 來表示位元 `0` 與 `1`。
  - `0`：佔空比約 37.5% (T0H)。
  - `1`：佔空比約 75.0% (T1H)。

### 7.2 STM32 實作機制 (TIM + DMA)
- **核心思路**：不使用 GPIO Bit-banging，而是利用 Timer 的 PWM 功能產生精確波形，並透過 DMA 自動搬運數據，釋放 CPU 資源。
- **DMA Buffer 設計**：
  - 每個 DSHOT 封包為 16 bits。
  - 為了用 PWM 模擬這 16 bits，我們建立一個大小為 18 的 `uint16_t` 陣列。
  - 前 16 個字組填入對應 `0` 或 `1` 的 Compare 值 (CCR)。
  - 最後 2 個字組填入 `0`，確保傳輸結束後 PWM 輸出維持在低電位 (Idle Low)。

### 7.3 封包結構
- **總長度**：16 bits
  - **Throttle (11 bits)**：油門數值 (0-2047)。其中 0-47 為特殊命令，48-2047 為油門。
  - **Telemetry (1 bit)**：遙測請求位。若設為 1，ESC 會在收到封包後回傳數據。
  - **CRC (4 bits)**：校驗碼，計算公式：`CRC = (Value ^ (Value >> 4) ^ (Value >> 8)) & 0x0F`。

## 8. UART 命令列表 (Command Reference)
以下指令可透過序列埠終端機輸入 (Baud: 115200)，支援以 `/` 分隔多個指令 (例如 `ARM/PWM 1500`)。

### 8.1 通用指令 (Common)
- `ARM`：解鎖 ESC (允許輸出油門訊號)。
- `DISARM`：上鎖 ESC (停止輸出或輸出安全值)。
- `STOP` / `ESTOP` / `S`：緊急停止 (強制停止輸出，進入 ESTOP 狀態)。
- `CLEAR` / `RESET`：清除 ESTOP 狀態，回到 DISARMED。
- `STATUS`：顯示目前狀態、模式與 DSHOT Rate。
- `MODE PWM`：切換至 PWM 模式 (50Hz/400Hz 視 Timer 設定)。
- `MODE DSHOT`：切換至 DSHOT 模式。

### 8.2 PWM 模式指令 (需先 `MODE PWM` 且 `ARM`)
- `PWM <1000~2000>`：設定脈寬 (微秒)。
- `DUTY <0~100>`：設定佔空比 (%)。
- `1000` / `1`：設定為 1000us (Min)。
- `1500` / `5`：設定為 1500us (Mid)。
- `2000` / `2`：設定為 2000us (Max)。

### 8.3 DSHOT 模式指令 (需先 `MODE DSHOT`)
- `DSRATE <150|300|600|1200>`：設定傳輸速率 (預設 600)。
- `DSHOT <48~2047>`：設定油門數值 (需 `ARM`)。
- `DSCMD <0~47>`：發送特殊命令 (無需 `ARM`)。常用命令：
  - `0`: Motor Stop
  - `1-5`: Beep
  - `7`: Spin Direction 1
  - `8`: Spin Direction 2
  - `12`: Save Settings (更改設定後必做)

### 8.4 測試流程範例
1. **PWM 測試**: `MODE PWM` -> `ARM` -> `PWM 1100` -> `PWM 1500` -> `DISARM`
2. **DSHOT 測試**: `MODE DSHOT` -> `DSRATE 600` -> `ARM` -> `DSHOT 100` -> `DSHOT 500` -> `DISARM`
3. **反轉設定**: `MODE DSHOT` -> `DSCMD 7` (或 8) -> `DSCMD 12` (儲存) -> 斷電重啟。