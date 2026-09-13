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

## SD telemetry logging

When the SD transport is available and the card contains a supported FAT32
volume, telemetry is appended as ordinary ASCII lines to **`LOG.TXT`** in the
root directory. Existing `LOG.TXT` content is preserved; each record is one
telemetry/status line followed by `LF`. The file is standard FAT32 and can be
opened directly by Windows. The logger writes complete sectors and updates the
directory size on every record; an abort also performs a final metadata/data
sync.

Use the periodic `STAT` line to distinguish failures:
`sd_transport=FAIL` means SPI/card initialization or sector I/O failed;
`sd_transport=OK logger_fs=FAIL` means the card transport works but the
filesystem is absent, unsupported, corrupt, or could not be updated. `LOGS`
reports the active log and sizes; `LOGSEL 0` selects `LOG.TXT` and `LOGDEL 1`
through `LOGDEL 3` removes only the backup logs. These commands are handled in
`Cassiopeia/Src/uplink.c`; there is no `commands.txt` file in this repository.

For safe extraction, issue `ABORT` (or otherwise allow the flight software to
finish its normal abort path), wait for the final status/serial output, remove
power, and then eject the card from Windows. Never remove the card during an
SD write. Cards must be preformatted on a PC as FAT32 with 512-byte sectors;
FAT12/FAT16, exFAT, unformatted media, and malformed volumes are rejected
without formatting or modifying existing user data. Both super-floppy FAT32
and normal MBR-partitioned FAT32 cards are supported.
