#include "bno055.h"
#include "bus_i2c.h"
#include "serial_monitor.h"
#include "stm32h7xx_hal.h"
#include "alarm.h"


#define BNO055_CHIP_ID_REG     0x00
#define BNO055_CHIP_ID         0xA0
#define BNO055_ACC_ID_REG      0x01
#define BNO055_ACC_ID_VAL      0xFB
#define BNO055_MAG_ID_REG      0x02
#define BNO055_MAG_ID_VAL      0x32
#define BNO055_GYR_ID_REG      0x03
#define BNO055_GYR_ID_VAL      0x0F
#define BNO055_PAGE_ID         0x07
#define BNO055_OPR_MODE        0x3D
#define BNO055_UNIT_SEL        0x3B
#define BNO055_PWR_MODE        0x3E
#define BNO055_SYS_TRIGGER     0x3F
#define BNO055_OPR_MODE_CONFIG 0x00
#define BNO055_OPR_MODE_NDOF   0x0C
#define BNO055_CALIB_STAT      0x35
#define BNO055_ST_RESULT       0x36
#define BNO055_SYS_STATUS      0x39
#define BNO055_SYS_ERR         0x3A
#define BNO055_ACC_DATA_START  0x08
#define BNO055_CALIBRATION_TIMEOUT_MS 180000U

static int present = 0;
static int calibration_state;
static uint32_t calibration_start;
static uint32_t calibration_next_poll;

/* gyro bias - nuluje se pri kazdem bootu (orientation.c) a odecita
   z kazdeho cteni gyroskopu, i z ISR stabilizace */
static volatile int16_t gyr_bias[3] = { 0, 0, 0 };
static volatile int gyr_calib = 0;
static int16_t last_sample[9];
static uint32_t same_since = 0;
static uint8_t stale_reads;
static uint8_t read_failures;

static void print_hex_byte(uint8_t v)
{
    const char *h = "0123456789ABCDEF";
    serial_putc('0');
    serial_putc('x');
    serial_putc(h[(v >> 4) & 0x0F]);
    serial_putc(h[v & 0x0F]);
}

int bno055_init(void)
{
    present = 0;
    calibration_state = 0;
    calibration_start = 0;
    calibration_next_poll = 0;
    gyr_bias[0] = gyr_bias[1] = gyr_bias[2] = 0;
    gyr_calib = 0;
    same_since = 0;
    stale_reads = 0;
    read_failures = 0;
    for (unsigned int i = 0; i < 9U; i++)
        last_sample[i] = 0;
    uint8_t id = 0;
    if (bus_i2c_read_reg(BNO055_ADDR, BNO055_CHIP_ID_REG, &id, 1) != 0 ||
        id != BNO055_CHIP_ID)
    {
        /* Recovery is only used when the device does not answer. A reset on
           every boot would discard the volatile BNO055 fusion calibration. */
        uint8_t reset = 0x20;
        if (bus_i2c_write_reg(BNO055_ADDR, BNO055_SYS_TRIGGER, &reset, 1) != 0)
            return -1;
        HAL_Delay(650);
        if (bus_i2c_read_reg(BNO055_ADDR, BNO055_CHIP_ID_REG, &id, 1) != 0)
            return -1;
    }
    if (id != BNO055_CHIP_ID)
        return -1;

    /* Do not switch an already-running fusion engine through CONFIG mode:
       that would discard the calibration accumulated before a warm reset. */
    uint8_t v = 0;
    uint8_t mode = 0;
    if (bus_i2c_read_reg(BNO055_ADDR, BNO055_OPR_MODE, &mode, 1) != 0)
        return -1;
    if (mode != BNO055_OPR_MODE_NDOF)
    {
        v = BNO055_OPR_MODE_CONFIG;
        if (bus_i2c_write_reg(BNO055_ADDR, BNO055_OPR_MODE, &v, 1) != 0)
            return -1;

        v = 0x00; /* page 0 */
        if (bus_i2c_write_reg(BNO055_ADDR, BNO055_PAGE_ID, &v, 1) != 0)
            return -1;

        v = 0x01; /* UNIT_SEL bits1:0 = 01 -> acceleration v mg (1000 = 1 g),
                     angular rate dps, temp degC. Puvodni 0x00 by byl m/s^2. */
        if (bus_i2c_write_reg(BNO055_ADDR, BNO055_UNIT_SEL, &v, 1) != 0)
            return -1;

        v = 0x00; /* normal power mode */
        if (bus_i2c_write_reg(BNO055_ADDR, BNO055_PWR_MODE, &v, 1) != 0)
            return -1;

        v = BNO055_OPR_MODE_NDOF; /* fusion mode */
        if (bus_i2c_write_reg(BNO055_ADDR, BNO055_OPR_MODE, &v, 1) != 0)
            return -1;

        /* senzor potrebuje cas na prepnuti modu, jinak jsou prvni data
           nestabilni/stara */
        HAL_Delay(100);
    }

    present = 1;
    return 0;
}

int bno055_self_test(void)
{
    if (!present)
        return -1;
    uint8_t id = 0;
    if (bus_i2c_read_reg(BNO055_ADDR, BNO055_CHIP_ID_REG, &id, 1) != 0)
        return -1;
    return (id == BNO055_CHIP_ID) ? 0 : -1;
}

/* diagnostika: precte ID vsech tri senzoru, self-test stav, kalibraci
   a chybovy registr. ACC_ID musi byt 0xFB, MAG_ID 0x32, GYR_ID 0x0F.
   Jinak jde o klon/falesny BNO055 a accel nemusi fungovat. */
int bno055_calib_status(uint8_t *sys)
{
    if (!present)
        return -1;
    uint8_t cal = 0;
    if (bus_i2c_read_reg(BNO055_ADDR, BNO055_CALIB_STAT, &cal, 1) != 0)
        return -1;
    /* CALIB_STAT: bit7:6=sys, bit5:4=gyr, bit3:2=acc, bit1:0=mag (0..3) */
    if (sys)
        *sys = (uint8_t)((cal >> 6) & 3);
    return 0;
}

int bno055_flight_status(uint8_t *calib_sys, uint8_t *sys_status)
{
    return bno055_flight_status_full(calib_sys, 0, 0, 0, sys_status);
}

int bno055_flight_status_full(uint8_t *calib_sys, uint8_t *calib_gyr,
                              uint8_t *calib_acc, uint8_t *calib_mag,
                              uint8_t *sys_status)
{
    uint8_t cal = 0;
    uint8_t status = 0;

    if (!present ||
        bus_i2c_read_reg(BNO055_ADDR, BNO055_CALIB_STAT, &cal, 1) != 0 ||
        bus_i2c_read_reg(BNO055_ADDR, BNO055_SYS_STATUS, &status, 1) != 0)
        return -1;

    if (calib_sys)
        *calib_sys = (uint8_t)((cal >> 6) & 3U);
    if (calib_gyr)
        *calib_gyr = (uint8_t)((cal >> 4) & 3U);
    if (calib_acc)
        *calib_acc = (uint8_t)((cal >> 2) & 3U);
    if (calib_mag)
        *calib_mag = (uint8_t)(cal & 3U);
    if (sys_status)
        *sys_status = status;
    return 0;
}

int bno055_flight_ready(void)
{
    uint8_t sys_status = 0;
    /*
     * Calibration quality is reported as WRN_IMU_CAL, but it is not a
     * launch interlock. The fusion engine must be running and reporting
     * SYS_STATUS=5; sample validity is checked by supervisor_warning_update.
     */
    if (bno055_flight_status(0, &sys_status) != 0)
        return 0;
    return sys_status == 5U;
}

int bno055_calibration_begin(void)
{
    if (!present)
        return -1;
    calibration_state = 1;
    calibration_start = HAL_GetTick();
    calibration_next_poll = calibration_start;
    serial_puts("bno055: ground calibration started; keep still, then move through all axes\r\n");
    return 0;
}

void bno055_calibration_update(void)
{
    uint8_t sys = 0, gyr = 0, acc = 0, mag = 0, status = 0;
    if (calibration_state != 1)
        return;
    if (HAL_GetTick() < calibration_next_poll)
        return;
    calibration_next_poll = HAL_GetTick() + 500U;
    if (bno055_flight_status_full(&sys, &gyr, &acc, &mag, &status) != 0)
    {
        calibration_state = -1;
        serial_puts("bno055: ground calibration failed: status read\r\n");
        return;
    }
    if (sys == 3U && gyr == 3U && acc == 3U && mag == 3U && status == 5U)
    {
        calibration_state = 2;
        serial_puts("bno055: ground calibration complete; offsets are volatile\r\n");
    }
    else if ((uint32_t)(HAL_GetTick() - calibration_start) >=
             BNO055_CALIBRATION_TIMEOUT_MS)
    {
        calibration_state = -1;
        serial_puts("bno055: ground calibration timed out; see STAT imu_cal\r\n");
    }
}

int bno055_calibration_active(void) { return calibration_state == 1; }
int bno055_calibration_state(void) { return calibration_state; }

void bno055_diag(void)
{
    if (!present)
    {
        serial_puts("bno055 diag: NOT PRESENT\r\n");
        return;
    }

    uint8_t acc_id = 0, mag_id = 0, gyr_id = 0;
    uint8_t st = 0, cal = 0, sys = 0, err = 0, unitsel = 0;

    bus_i2c_read_reg(BNO055_ADDR, BNO055_ACC_ID_REG, &acc_id, 1);
    bus_i2c_read_reg(BNO055_ADDR, BNO055_MAG_ID_REG, &mag_id, 1);
    bus_i2c_read_reg(BNO055_ADDR, BNO055_GYR_ID_REG, &gyr_id, 1);
    bus_i2c_read_reg(BNO055_ADDR, BNO055_ST_RESULT, &st, 1);
    bus_i2c_read_reg(BNO055_ADDR, BNO055_CALIB_STAT, &cal, 1);
    bus_i2c_read_reg(BNO055_ADDR, BNO055_SYS_STATUS, &sys, 1);
    bus_i2c_read_reg(BNO055_ADDR, BNO055_SYS_ERR, &err, 1);
    bus_i2c_read_reg(BNO055_ADDR, BNO055_UNIT_SEL, &unitsel, 1);

    serial_puts("bno055 diag: acc_id=");
    print_hex_byte(acc_id);
    serial_puts(" (exp 0xFB) mag_id=");
    print_hex_byte(mag_id);
    serial_puts(" (exp 0x32) gyr_id=");
    print_hex_byte(gyr_id);
    serial_puts(" (exp 0x0F)\r\n");

    serial_puts("bno055 diag: self_test=0x");
    print_hex_byte(st);
    serial_puts(" (bit2=ACC,bit3=GYR,bit1=MAG; 0x0E=OK) calib=");
    print_hex_byte(cal);
    serial_puts(" [SYS=");
    print_unsigned((cal >> 6) & 3U);
    serial_puts(" GYR=");
    print_unsigned((cal >> 4) & 3U);
    serial_puts(" ACC=");
    print_unsigned((cal >> 2) & 3U);
    serial_puts(" MAG=");
    print_unsigned(cal & 3U);
    serial_puts("]");
    serial_puts(" sys=");
    print_hex_byte(sys);
    serial_puts(" err=");
    print_hex_byte(err);
    serial_puts(" unitsel=");
    print_hex_byte(unitsel);
    serial_puts(" (ACC bits1:0: 00=m/s2, 01=mg, 10=g, 11=mg)\r\n");

    uint8_t d[18];
    if (bus_i2c_read_reg(BNO055_ADDR, BNO055_ACC_DATA_START, d, 18) == 0)
    {
        int16_t a[3], g[3], m[3];
        a[0] = (int16_t)((d[1] << 8) | d[0]);
        a[1] = (int16_t)((d[3] << 8) | d[2]);
        a[2] = (int16_t)((d[5] << 8) | d[4]);
        m[0] = (int16_t)((d[7] << 8) | d[6]);
        m[1] = (int16_t)((d[9] << 8) | d[8]);
        m[2] = (int16_t)((d[11] << 8) | d[10]);
        g[0] = (int16_t)((d[13] << 8) | d[12]);
        g[1] = (int16_t)((d[15] << 8) | d[14]);
        g[2] = (int16_t)((d[17] << 8) | d[16]);

        serial_puts("bno055 raw: acc=");
        print_int(a[0]); serial_puts(","); print_int(a[1]); serial_puts(","); print_int(a[2]);
        serial_puts(" mg gyr=");
        print_int(g[0]); serial_puts(","); print_int(g[1]); serial_puts(","); print_int(g[2]);
        serial_puts(" dps mag=");
        print_int(m[0]); serial_puts(","); print_int(m[1]); serial_puts(","); print_int(m[2]);
        serial_puts(" uT\r\n");
    }

    /* gravity vector + quaternion (fusion vystupy) */
    uint8_t dv[6], q[8];
    if (bus_i2c_read_reg(BNO055_ADDR, 0x2E, dv, 6) == 0)
    {
        int16_t gx = (int16_t)((dv[1] << 8) | dv[0]);
        int16_t gy = (int16_t)((dv[3] << 8) | dv[2]);
        int16_t gz = (int16_t)((dv[5] << 8) | dv[4]);
        serial_puts("bno055 grav_vec=");
        print_int(gx); serial_puts(","); print_int(gy); serial_puts(","); print_int(gz);
        serial_puts(" (0,0,~1000 = vodorovne, Z smerem nahoru)\r\n");
    }
    if (bus_i2c_read_reg(BNO055_ADDR, 0x34, q, 8) == 0)
    {
        int16_t qw = (int16_t)((q[1] << 8) | q[0]);
        int16_t qx = (int16_t)((q[3] << 8) | q[2]);
        int16_t qy = (int16_t)((q[5] << 8) | q[4]);
        int16_t qz = (int16_t)((q[7] << 8) | q[6]);
        serial_puts("bno055 quat=");
        print_int(qw); serial_puts(","); print_int(qx); serial_puts(","); print_int(qy); serial_puts(","); print_int(qz);
        serial_puts(" (scale 2^14)\r\n");
    }
}

/* isr != 0 = volano z TIM6 ISR stabilizace -> kratky I2C timeout
   (bus_i2c_read_reg_short), aby se ISR neblokovala na seknute lince. */
static int bno055_read_internal(int16_t *acc, int16_t *gyr, int16_t *mag, int isr)
{
    if (!present)
        return -1;

    uint8_t d[18];
    int r = isr ? bus_i2c_read_reg_short(BNO055_ADDR, BNO055_ACC_DATA_START, d, 18)
                : bus_i2c_read_reg(BNO055_ADDR, BNO055_ACC_DATA_START, d, 18);
    if (r != 0)
    {
        same_since = 0;
        stale_reads = 0;
        if (++read_failures >= 3U) alarm_set(ALARM_BNO055);
        return -1;
    }
    read_failures = 0;

    if (acc)
    {
        acc[0] = (int16_t)((d[1] << 8) | d[0]);
        acc[1] = (int16_t)((d[3] << 8) | d[2]);
        acc[2] = (int16_t)((d[5] << 8) | d[4]);
    }
    if (mag)
    {
        mag[0] = (int16_t)((d[7] << 8) | d[6]);
        mag[1] = (int16_t)((d[9] << 8) | d[8]);
        mag[2] = (int16_t)((d[11] << 8) | d[10]);
    }
    if (gyr)
    {
        gyr[0] = (int16_t)((d[13] << 8) | d[12]);
        gyr[1] = (int16_t)((d[15] << 8) | d[14]);
        gyr[2] = (int16_t)((d[17] << 8) | d[16]);
        if (gyr_calib)
        {
            gyr[0] = (int16_t)(gyr[0] - gyr_bias[0]);
            gyr[1] = (int16_t)(gyr[1] - gyr_bias[1]);
            gyr[2] = (int16_t)(gyr[2] - gyr_bias[2]);
        }
    }

    {
        int16_t sample[9];
        unsigned int i;
        for (i = 0; i < 3; i++) {
            sample[i] = (int16_t)((d[2U * i + 1U] << 8) | d[2U * i]);
            sample[3U + i] = (int16_t)((d[7U + 2U * i] << 8) | d[6U + 2U * i]);
            sample[6U + i] = (int16_t)((d[13U + 2U * i] << 8) | d[12U + 2U * i]);
        }
        if ((sample[0] == last_sample[0] && sample[1] == last_sample[1] &&
             sample[2] == last_sample[2] && sample[3] == last_sample[3] &&
             sample[4] == last_sample[4] && sample[5] == last_sample[5] &&
             sample[6] == last_sample[6] && sample[7] == last_sample[7] &&
             sample[8] == last_sample[8]))
        {
            if (same_since == 0U) same_since = HAL_GetTick();
            if (++stale_reads >= 20U && (HAL_GetTick() - same_since > 1000U &&
                sample[0] == 0 && sample[1] == 0 && sample[2] == 0 &&
                sample[3] == 0 && sample[4] == 0 && sample[5] == 0 &&
                sample[6] == 0 && sample[7] == 0 && sample[8] == 0)) {
                alarm_set(ALARM_BNO055);
                return -1;
            }
        } else {
            same_since = HAL_GetTick();
            stale_reads = 0;
        }
        for (i = 0; i < 9; i++) last_sample[i] = sample[i];
        /* Physical BNO055 output limits, not int16 limits (which are tautological). */
        if (sample[0] > 16000 || sample[0] < -16000 ||
            sample[1] > 16000 || sample[1] < -16000 ||
            sample[2] > 16000 || sample[2] < -16000 ||
            sample[3] > 8000 || sample[3] < -8000 ||
            sample[4] > 8000 || sample[4] < -8000 ||
            sample[5] > 8000 || sample[5] < -8000 ||
            sample[6] > 4000 || sample[6] < -4000 ||
            sample[7] > 4000 || sample[7] < -4000 ||
            sample[8] > 4000 || sample[8] < -4000) {
            alarm_set(ALARM_BNO055);
            return -1;
        }
    }
    alarm_clear(ALARM_BNO055);
    return 0;
}

int bno055_read(int16_t *acc, int16_t *gyr, int16_t *mag)
{
    return bno055_read_internal(acc, gyr, mag, 0);
}

int bno055_read_isr(int16_t *acc, int16_t *gyr, int16_t *mag)
{
    return bno055_read_internal(acc, gyr, mag, 1);
}

void bno055_gyro_bias_reset(void)
{
    gyr_bias[0] = gyr_bias[1] = gyr_bias[2] = 0;
    gyr_calib = 0;
}

void bno055_gyro_bias_set(const int16_t bias[3])
{
    gyr_bias[0] = bias[0];
    gyr_bias[1] = bias[1];
    gyr_bias[2] = bias[2];
    gyr_calib = 1;
}

int bno055_gyro_bias_get(int16_t bias[3])
{
    if (!gyr_calib)
        return -1;
    if (bias)
    {
        bias[0] = gyr_bias[0];
        bias[1] = gyr_bias[1];
        bias[2] = gyr_bias[2];
    }
    return 0;
}
