/**
 * Minimal STM32Cube HAL example for F401CC (BlackPill F401CC)
 * Build with: pio run -e f401cc_cube
 *
 * LED: PC13 (change if your board uses another pin)
 *
 * Clocks (choose one by define):
 *   - Default: HSE 25 MHz → PLL → SYSCLK 84 MHz, USB 48 MHz (PLLI2S not used)
 *   - Or:      HSI 16 MHz → PLL → SYSCLK 84 MHz, USB 48 MHz
 */
#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    while (1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(500);
    }
}

/* --- Select clock source here --- */
#define USE_HSI_CLOCK 1   // uncomment to use internal HSI 16MHz
/* If not defined, code assumes external HSE 25 MHz.
 * If your board has HSE = 8 MHz, change HSE_VALUE (in build flags) to 8000000U
 * and set PLLM accordingly (see notes below).
*/

/* ---- SystemClock: 84 MHz, APB1=42, APB2=84 ---- */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    // F401 at 84 MHz → Voltage Scale 2
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

#if defined(USE_HSI_CLOCK)
    /* HSI 16 MHz → VCO 336 → SYSCLK 84 (PLLP=4), PLLQ=7 → 48 MHz (USB) */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM       = 16;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV4; // 336/4 = 84 MHz
    RCC_OscInitStruct.PLL.PLLQ       = 7;             // 336/7 = 48 MHz (USB)
#else
    /* HSE 25 MHz → VCO 336 → SYSCLK 84 (PLLP=4), PLLQ=7 → 48 MHz (USB) */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 25;            // HSE/PLLM = 1 MHz
    RCC_OscInitStruct.PLL.PLLN       = 336;           // 1 * 336 = 336 MHz VCO
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV4; // 336/4 = 84 MHz
    RCC_OscInitStruct.PLL.PLLQ       = 7;             // 336/7 = 48 MHz (USB)
#endif

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        while (1) { }
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;  // 84
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;    // 42
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    // 84

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        while (1) { }
    }

    // SysTick 1ms (HAL_Init() normally sets this, but safe to ensure)
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/* ---- PC13 LED ---- */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
