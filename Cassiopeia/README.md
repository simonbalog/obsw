# Cassiopeia

Čistý STM32CubeIDE/CubeMX základ pro **STM32H723ZGT6** na desce
**NUCLEO-H723ZG**.

Projekt obsahuje pouze generovanou HAL/CMSIS infrastrukturu:

- `Src/main.c` – HAL inicializace, systémové hodiny, GPIO, I2C1 a SPI1
- `Src/stm32h7xx_hal_msp.c` – MSP konfigurace periferií
- `Src/stm32h7xx_it.c` – obsluhy systémových a GPIO přerušení
- `Src/system_stm32h7xx.c` – CMSIS systémová vrstva
- `Src/syscalls.c`, `Src/sysmem.c` – newlib/syscall podpora
- `Startup/` – startup kód Cortex-M7
- `Drivers/` – STM32H7 HAL a CMSIS

Konfigurace MCU a pinů je v `Cassiopeia.ioc`. Projekt se sestavuje přes
STM32CubeIDE podle `.cproject`; vlastní aplikační moduly ani ručně psaný
`mx_init.*` zde nejsou.

## Preflight

Příkaz `PREFLIGHT` přes LoRa nebo jako řádek `PREFLIGHT` zakončený CR/LF na
diagnostickém USART3 spustí omezený checklist BME280, BNO055, ADC napájení,
PCA9685/serv, LoRa, SD/FAT loggeru, watchdogu, stavu letu a kalibrace.
Výsledek je současně vypsán na sériovou linku a
odeslán jako `PREFLIGHT result=PASS|FAIL BME280=<code> BNO055=<code>
POWER=<code> ALARMS=<code>`. Kód `0` znamená úspěch, chyby modulů používají
stabilní rozsah 401–411 a celkový FAIL 400. Kontrola je pouze diagnostická:
nečistí alarmy, nemění stav letu ani neobchází safety.

Před zpracováním letu se odmítají nefinite/mimo-rozsah vzorky BME280 (včetně
skoku tlaku), BNO055 (čtení, rozsah a stale data) a ADC napětí.
