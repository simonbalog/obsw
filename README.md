# OBSW

On-board software základ pro STM32H723ZGT6 / NUCLEO-H723ZG.

Verze **0.1** obsahuje čistý STM32CubeIDE/CubeMX skeleton bez aplikační
logiky. Projekt se nachází v adresáři `Cassiopeia/` a používá STM32Cube HAL,
CMSIS, startup kód Cortex-M7 a GCC toolchain `arm-none-eabi-*`.

## Build

Otevři adresář `Cassiopeia/` ve STM32CubeIDE a sestav konfiguraci Debug nebo
Release. Konfigurace MCU a periferií je v `Cassiopeia/Cassiopeia.ioc`.

## Serial status protocol

The PC UART uses bounded, line-oriented ASCII output. The periodic line starts
with `STAT v=1` and uses stable key/value fields:

```
STAT v=1 flight=BOOT_HOLD count=STOPPED warn=1 alarm=0 master=1 codes=108,305 pressure_hpa=101325 altitude_cm=0 gyro_dps=0,0,0 nose=CALIBRATING stab=0:0d,1:0d,2:0d,3:0d batt_mv=7400 5v_mv=5000 safety=OFF
```

`codes=` contains numeric active conditions: warnings are 100-108, normal
alarms 200-207, and master alarms 300-310. The assignments are defined in
`Cassiopeia/Inc/alarm.h`; they are stable PC-facing identifiers. `EVENT`
lines use the same `level`, `code`, and terminology for boot, flight, safety,
and fault events. A missing code list is reported as `NONE`.

At every reset the BME280 is compensated using its factory calibration and a
valid ground pressure reference is captured. The BNO055 is reset, placed in
NDOF mode, allowed to become ready, and its calibration state is checked.
Until both sensors are ready, the corresponding numbered warning/master alarm
is reported and countdown/flight-critical arming is refused. `pressure_hpa`
and calculated `altitude_cm` are relative to that captured reference;
`gyro_dps`, `nose`, and `stab` expose the current gyro, clock-face orientation
(`UP`, `DOWN`, or `12H/3H/...` plus degrees), and commanded stabilization
channels. Hardware validation is still required for sensor mounting,
calibration, and actuator sign conventions.
