#include "power.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "alarm.h"

/* Napajeni modul - mereni baterie a 5V vetve pres ADC.
 *
 * Zapojeni (TODO - uprav dle tveho delice):
 *   - baterie -> delic R1/R2 -> ADC pin
 *   - 5V      -> delic R1/R2 -> ADC pin
 *
 * Zakladni verze meri baterii na ADC1_IN0 (PA0) a 5V na ADC1_IN1 (PA1),
 * delici pomer je konfigurovatelny pres POWER_BATT_DIV a POWER_5V_DIV.
 */

#define POWER_BATT_DIV   (2.0f)   /* vstupni / mereny pomer (R1+R2)/R2 */
#define POWER_5V_DIV     (2.0f)
#define POWER_VDDA_MV    3300

static ADC_HandleTypeDef hadc1;
static int ready = 0;
static unsigned int last_error;

static void adc_pin_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static int adc_init(void)
{
    /* ADC kernel clock: na STM32H7 je nutny (PLL2P = 129 MHz) -
       bez nej ADC nebezi. Zde se PLL2 rovnou nakonfiguruje a zapne. */
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    PeriphClkInit.PLL2.PLL2M = 32;
    PeriphClkInit.PLL2.PLL2N = 129;
    PeriphClkInit.PLL2.PLL2P = 2;
    PeriphClkInit.PLL2.PLL2Q = 2;
    PeriphClkInit.PLL2.PLL2R = 2;
    PeriphClkInit.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
    PeriphClkInit.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInit.PLL2.PLL2FRACN = 0;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        return -1;

    __HAL_RCC_ADC12_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfDiscConversion = 0;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
        return -1;

    /* kalibrace ADC je na H7 nutna pro spravne vysledky */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
        return -1;

    return 0;
}

static int adc_read_mv(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        return -1;

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
        return -1;
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return -1;
    }
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return (int)((uint64_t)raw * POWER_VDDA_MV / 4096);
}

int power_init(void)
{
    ready = 0;
    adc_pin_init();
    if (adc_init() != HAL_OK)
        return -1;

    uint16_t mv = 0;
    if (power_read_battery_mv(&mv) != 0)
        return -1;
    ready = 1;
    return 0;
}

int power_self_test(void)
{
    return ready ? 0 : -1;
}

int power_read_battery_mv(uint16_t *mv)
{
    if (mv == NULL)
        return -1;
    int v = adc_read_mv(ADC_CHANNEL_0);
    if (v < 0 || v > 10000)
    {
        last_error = 1U;
        alarm_set(ALARM_BATTERY);
        return -1;
    }
    *mv = (uint16_t)(v * POWER_BATT_DIV);
    if (*mv == 0 || *mv > 20000U) { last_error = 2U; alarm_set(ALARM_BATTERY); return -1; }
    last_error = 0;
    return 0;
}

int power_read_5v_mv(uint16_t *mv)
{
    if (mv == NULL)
        return -1;
    int v = adc_read_mv(ADC_CHANNEL_1);
    if (v < 0 || v > 5000)
    {
        last_error = 1U;
        alarm_set(ALARM_BATTERY);
        return -1;
    }
    *mv = (uint16_t)(v * POWER_5V_DIV);
    if (*mv == 0 || *mv > 7000U) { last_error = 2U; alarm_set(ALARM_BATTERY); return -1; }
    return 0;
}

unsigned int power_last_error(void) { return last_error; }
