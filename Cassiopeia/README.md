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
