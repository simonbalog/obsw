# OBSW

On-board software základ pro STM32H723ZGT6 / NUCLEO-H723ZG.

Verze **0.1** obsahuje čistý STM32CubeIDE/CubeMX skeleton bez aplikační
logiky. Projekt se nachází v adresáři `Cassiopeia/` a používá STM32Cube HAL,
CMSIS, startup kód Cortex-M7 a GCC toolchain `arm-none-eabi-*`.

## Build

Otevři adresář `Cassiopeia/` ve STM32CubeIDE a sestav konfiguraci Debug nebo
Release. Konfigurace MCU a periferií je v `Cassiopeia/Cassiopeia.ioc`.
