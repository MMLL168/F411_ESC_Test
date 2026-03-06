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
  - `TIM3`：`Prescaler=99`、`Period=19999`，對應 1MHz 計數基準與 20ms 週期（50Hz），`Pulse=1500`。
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
- 目前尚未看到：
  - 啟動 PWM 的程式（例如 `HAL_TIM_PWM_Start` / `HAL_TIM_PWM_Start_DMA`）。
  - ESC 解鎖（arming）流程與油門斜率控制。
  - UART 指令解析或遙測回傳。
  - 保護機制（失聯保護、範圍限制、錯誤回復）。

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