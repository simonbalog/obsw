#include "serial_monitor.h"
#include "main.h"
#include "supervisor.h"
#include "alarm.h"
#include "bme280.h"
#include "bno055.h"
#include "pca9685.h"
#include "countdown.h"
#include "logger.h"
#include "flight.h"
#include "watchdog.h"
#include "telemetry.h"
#include "stabilization.h"
#include "uplink.h"
#include "status_report.h"
#include "orientation.h"
#include "gps.h"

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim6;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);

int main(void)
{
    /* Capture reset cause before peripheral initialization can clear it. */
    uint32_t rst_cause = RCC->RSR;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_TIM6_Init();
    alarm_init();
    serial_init();

    serial_puts("\r\nCassiopeia v0.3\r\n");
    status_report_event("INFO", 1U, "boot");

    status_report_event((rst_cause & (RCC_RSR_IWDG1RSTF | RCC_RSR_WWDG1RSTF)) ?
                        "MASTER" : "INFO",
                        (rst_cause & (RCC_RSR_IWDG1RSTF | RCC_RSR_WWDG1RSTF)) ?
                        STATUS_CODE_RESET_WATCHDOG : 2U, "reset");

    /* smazat latchnute flagy, at pristi boot ukaze jen svuj duvod */
    RCC->RSR = RCC_RSR_RMVF;

    serial_puts("--- Supervisor init ---\r\n");
    supervisor_init();
    supervisor_self_test();
    supervisor_report();

    alarm_update_leds();

    serial_puts("\r\n--- Alarmy ---\r\n");
    serial_puts("master: ");
    print_unsigned(master_alarm_count());
    serial_puts("\r\n");
    serial_puts("alarms: ");
    print_unsigned(alarm_count());
    serial_puts("\r\n");
    serial_puts("warnings: ");
    print_unsigned(warning_count());
    serial_puts("\r\n");

    serial_puts("\r\n--- Flight init ---\r\n");
    stabilization_init();
    flight_init();
    orientation_init();   /* nulovani gyra + reference "nahoru" pri kazdem bootu */

    serial_puts("\r\n--- Odpocet ---\r\n");
    countdown_init();

    logger_init();
    telemetry_init();
    uplink_init();
    status_report_init();

    /* Start the runtime watchdog only after bounded boot/storage init has
       completed; boot diagnostics must not be reset by a missing SD card. */
    watchdog_init();
    alarm_update_leds();

    for (;;)
    {
        supervisor_warning_update();
        logger_update();   /* retry SD: pripoji se samo, kdyz karta odpovi */
        countdown_update();
        flight_update();
        gps_update();
        telemetry_update();
        orientation_update();
        bno055_calibration_update();
        /* stabilization_update() bezi v TIM6 preruseni (50 Hz, priorita) */
        uplink_update();
        status_report_update();
        watchdog_refresh();
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK)
        Error_Handler();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    uint32_t vos_start = HAL_GetTick();
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
        if ((uint32_t)(HAL_GetTick() - vos_start) >= 100U)
            Error_Handler();
    }

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_DIV1;
    osc.HSICalibrationValue = 64;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 32;
    osc.PLL.PLLN = 129;
    osc.PLL.PLLP = 2;
    osc.PLL.PLLQ = 2;
    osc.PLL.PLLR = 2;
    osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
    osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
        Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                    RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider = RCC_HCLK_DIV1;
    clk.APB3CLKDivider = RCC_APB3_DIV1;
    clk.APB1CLKDivider = RCC_APB1_DIV1;
    clk.APB2CLKDivider = RCC_APB2_DIV1;
    clk.APB4CLKDivider = RCC_APB4_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK)
        Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

    gpio.Pin = B1_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(B1_GPIO_Port, &gpio);

    gpio.Pin = SD_CS_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SD_CS_GPIO_Port, &gpio);

    gpio.Pin = LORA_NSS_Pin;
    HAL_GPIO_Init(LORA_NSS_GPIO_Port, &gpio);
    gpio.Pin = LORA_RST_Pin;
    HAL_GPIO_Init(LORA_RST_GPIO_Port, &gpio);

    gpio.Pin = LORA_DIO0_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(LORA_DIO0_GPIO_Port, &gpio);

    gpio.Pin = BNO_INT_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(BNO_INT_GPIO_Port, &gpio);
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x10707DBC;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK ||
        HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK ||
        HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
        Error_Handler();
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
        Error_Handler();
}

static void MX_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = HAL_RCC_GetPCLK1Freq() / 1000U - 1U;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = STAB_LOOP_MS - 1U;
    htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
        Error_Handler();
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
        Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    volatile uint32_t delay = 0;
    while (delay++ < 4000000U) {}
    NVIC_SystemReset();
    for (;;) {}
}
