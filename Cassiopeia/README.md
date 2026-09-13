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
Výsledek je současně vypsán na sériovou linku a odeslán jako
`PREFLIGHT result=PASS|FAIL BME280=<code> BNO055=<code> POWER=<code>
SERVOS=<code> LORA=<code> SD=<code> LOGGER=<code> WATCHDOG=<code>
FLIGHT=<code> CALIBRATION=<code> ALARMS=<code>`. Kód `0` znamená úspěch, chyby modulů používají
stabilní rozsah 401–411 a celkový FAIL 400. Kontrola je pouze diagnostická:
nečistí alarmy, nemění stav letu ani neobchází safety.

## BNO055 ground calibration

`STAT` reports `imu_cal=SYS,GYR,ACC,MAG`, `imu_sys=<SYS_STATUS>` and
`imu_ready=1|0`. Calibration levels remain visible as a warning only;
`MASTER_IMU` is reserved for a missing/failed sensor, invalid samples, or a
fusion engine whose `imu_sys` is not `5`. `orientation.c`'s short
gravity/gyro reference calibration is separate and does not satisfy this
requirement.

After a cold boot, keep the vehicle still and level, then send `IMUCAL` over
USART3 or LoRa. Follow the BNO055 ground procedure by holding still first and
then moving the sensor through all required axes until `STAT` reports
`imu_sys=5 imu_ready=1`. The monitor times out after three
minutes and never accepts partial calibration. BNO055 offsets are volatile;
this firmware has no safe nonvolatile calibration store, so repeat the
procedure after a power cycle or reset that loses the sensor state. Launch
is blocked only while the sensor is unavailable, samples are invalid, or
fusion status is not `5`; incomplete component calibration remains visible
as `WRN_IMU_CAL` and in `imu_cal`.

## Release hardening limitations

Boot remains in `FLIGHT_IDLE`; flight control and liftoff acceptance require
the explicit countdown/`flight_start()` authorization. Safety detection latches
the launch master alarm and keeps stabilization engaged until an explicit
abort. The BNO055 and LoRa interrupt pins are intentionally polled from thread
context, so no I2C/SPI transaction is performed in an ISR.

The SD logger uses SPI block access with SDSC byte-address conversion and
bounded FAT32 geometry checks. `Cassiopeia.ioc` is aligned with the SPI1,
TIM6, and USART3 application and the CubeIDE GCC `.cproject`; regenerate
CubeMX code only with user code preservation enabled. Hardware card timing,
sensor wiring, and actuator behavior still require board-level validation.

Před zpracováním letu se odmítají nefinite/mimo-rozsah vzorky BME280 (včetně
skoku nebo nepřípustné rychlosti změny tlaku), BNO055 (čtení, rozsah a stale
data) a ADC napětí; odmítnuté vzorky se nepředávají do flight/orientation/
stabilization logiky a aktivují příslušný alarm.
