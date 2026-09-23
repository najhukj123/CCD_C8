/* USER CODE BEGIN Header */
// 程序入口只负责时钟、外设和应用初始化；小车业务逻辑位于 _car_app.c。
/* USER CODE END Header */
#include "main.h"
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
/* USER CODE BEGIN Includes */
#include "_car_app.h"
/* USER CODE END Includes */

void SystemClock_Config(void);
static void InitializePeripherals(void);

int main(void)
{
  // 初始化 HAL 和系统时钟，之后才能使用外设与 HAL 时间基准。
  HAL_Init();
  SystemClock_Config();

  // 保持 CubeMX 的外设初始化顺序；DMA 必须先于串口初始化。
  InitializePeripherals();

  /* USER CODE BEGIN 2 */
  // 初始化小车控制、CCD 和串口接收；电机在收到命令前保持停止。
  CarApp_Init();
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    // 应用轮询负责命令处理、采样和控制。
    CarApp_Poll();
    /* USER CODE END 3 */
  }
}

// 按 CubeMX 配置初始化引脚、DMA 和各外设。
static void InitializePeripherals(void)
{
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
}

// HSE 为 8 MHz，经 PLL 倍频 9 得到 72 MHz 系统时钟。
// AHB 和 APB2 为 72 MHz，APB1 分频 2 后为 36 MHz。
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef oscillator_config = {0};
  RCC_ClkInitTypeDef bus_clock_config = {0};

  // 配置外部晶振和 PLL；保留 CubeMX 启用 HSI 的原始设置。
  oscillator_config.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscillator_config.HSEState = RCC_HSE_ON;
  oscillator_config.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  oscillator_config.HSIState = RCC_HSI_ON;
  oscillator_config.PLL.PLLState = RCC_PLL_ON;
  oscillator_config.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  oscillator_config.PLL.PLLMUL = RCC_PLL_MUL9;

  if (HAL_RCC_OscConfig(&oscillator_config) != HAL_OK)
  {
    Error_Handler();
  }

  // PLL 输出直接供给 CPU；APB1 分频到 36 MHz。
  bus_clock_config.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  bus_clock_config.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  bus_clock_config.AHBCLKDivider = RCC_SYSCLK_DIV1;
  bus_clock_config.APB1CLKDivider = RCC_HCLK_DIV2;
  bus_clock_config.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&bus_clock_config, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

// 初始化失败后关闭中断并停留在此处，等待人工复位。
void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
  }
}
