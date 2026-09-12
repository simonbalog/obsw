#include "countdown.h"
#include "serial_monitor.h"
#include "alarm.h"
#include "supervisor.h"
#include "safety.h"
#include "pca9685.h"
#include "lora.h"
#include "logger.h"
#include "flight.h"
#include "stabilization.h"
#include "telem_buf.h"
#include "main.h"
#include "stm32h7xx_hal.h"

/* Odpocitavani rideno tlacitkem B1 (externi, na PG14 / Arduino D2,
 * pull-up, aktivni low).
 *
 *  - 1 stisk        (< 1 s):     TEST MODE - stabilizace a klapky
 *                                  aktivni na zemi (zemnni test serv)
 *  - 2 stisky       (2x rychle):  start / pause odpoctu
 *  - dlhy stisk      (>= 1 s):    +1 minuta k nastavenemu casu
 *  - velmi dlouhy    (>= 5 s):    ABORT - zruseni startu, navrat do
 *                                  default stavu (LED probliknou)
 *
 *  Faze po startu:
 *   1) CHECK  - prvnich 30 s: znovu se zkontroluji systemy (supervisor
 *               self-test) a LED ukazuji alarmy jako po bootu
 *   2) WAVE   - LED problikavaji vlnou (priprava ke startu)
 *   3) WARNING- poslednich 30 s: vsechny 3 LED vyrazne blikaji naraz
 *   4) ARMED  - T-0, odpal
 *
 *  Safety watch (safety.c): prudky pokles tlaku nebo zrychleni -> MASTER_LAUNCH,
 *  spusteni stabilizace (serva).
 */

#define BTN_DEBOUNCE_MS   50
#define BTN_LONG_PRESS_MS 1000
#define BTN_ABORT_MS      5000
#define BTN_CLICK_WINDOW_MS 600

#define TICK_SEC  1000

#define CHECK_MS    30 * TICK_SEC
#define WARNING_MS  30 * TICK_SEC

#define LED_WAVE_MS      150
#define LED_WARNING_MS   100
#define LED_ABORT_BLINK_MS 150
#define LED_ABORT_BLINKS   6

static CountdownState state = COUNTDOWN_STOPPED;
static CountdownPhase phase = COUNTDOWN_PHASE_IDLE;
static uint32_t total_ms = COUNTDOWN_DEFAULT_MIN * 60 * TICK_SEC;
static uint32_t start_tick = 0;
static uint8_t remote_paused = 0;

static uint8_t test_mode = 0;
static uint8_t click_count = 0;
static uint32_t click_last_tick = 0;
static uint8_t abort_blink_count = 0;
static uint32_t abort_blink_tick = 0;
static uint8_t abort_blink_on = 0;

static void countdown_pause(void);
static void countdown_start(void);
static void countdown_add_min(void);
static void test_mode_toggle(void);

static uint8_t btn_prev = 1;
static uint32_t btn_change_tick = 0;
static uint32_t btn_down_tick = 0;

static uint32_t led_tick = 0;
static uint8_t led_wave_pos = 0;
static uint8_t led_warn_on = 0;

static uint8_t btn_read(void)
{
    return (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) ? 0 : 1;
}

static void print_time(uint32_t s)
{
    print_unsigned(s / 60);
    serial_puts(":");
    if (s % 60 < 10) serial_putc('0');
    print_unsigned(s % 60);
}

void countdown_init(void)
{
    state = COUNTDOWN_STOPPED;
    phase = COUNTDOWN_PHASE_IDLE;
    total_ms = COUNTDOWN_DEFAULT_MIN * 60 * TICK_SEC;
    start_tick = 0;
    remote_paused = 0;
    btn_prev = btn_read();
    btn_change_tick = HAL_GetTick();

    serial_puts("countdown: T-5:00 (stopped)\r\n");
    serial_puts("  B1 1x = test mode (stabilizace na zemi)\r\n");
    serial_puts("  B1 2x = start/pause\r\n");
    serial_puts("  B1 long = +1 min, B1 5s+ = ABORT\r\n");
}

CountdownState countdown_state(void)
{
    return state;
}

CountdownPhase countdown_phase(void)
{
    return phase;
}

uint32_t countdown_remaining_s(void)
{
    if (state == COUNTDOWN_RUNNING)
    {
        uint32_t elapsed = HAL_GetTick() - start_tick;
        if (elapsed >= total_ms)
            return 0;
        return (total_ms - elapsed + TICK_SEC - 1) / TICK_SEC;
    }
    return (total_ms + TICK_SEC - 1) / TICK_SEC;
}

void countdown_remote_pause(void)
{
    if (state == COUNTDOWN_RUNNING)
    {
        countdown_pause();
        remote_paused = 1;
        serial_puts("countdown: remote PAUSE\r\n");
    }
    else
    {
        serial_puts("countdown: remote PAUSE ignored (not running)\r\n");
    }
}

void countdown_remote_resume(void)
{
    if (state == COUNTDOWN_STOPPED && remote_paused && total_ms > 0)
    {
        state = COUNTDOWN_RUNNING;
        phase = COUNTDOWN_PHASE_CHECK;
        start_tick = HAL_GetTick();
        remote_paused = 0;
        serial_puts("countdown: remote RESUME\r\n");
        alarm_update_leds();
    }
    else
    {
        serial_puts("countdown: remote RESUME ignored\r\n");
    }
}

void countdown_remote_start(void)
{
    if (state == COUNTDOWN_STOPPED && !remote_paused)
    {
        countdown_start();
    }
    else
    {
        serial_puts("countdown: remote START ignored (not stopped)\r\n");
    }
}

void countdown_remote_add_min(void)
{
    countdown_add_min();
}

void countdown_remote_test_toggle(void)
{
    test_mode_toggle();
}

static void countdown_start(void)
{
    /* start = opusteni test modu */
    if (test_mode)
        test_mode_toggle();

    if (total_ms == 0)
        total_ms = COUNTDOWN_DEFAULT_MIN * 60 * TICK_SEC;

    state = COUNTDOWN_RUNNING;
    phase = COUNTDOWN_PHASE_CHECK;
    start_tick = HAL_GetTick();
    led_tick = start_tick;
    led_wave_pos = 0;
    led_warn_on = 0;

    serial_puts("\r\ncountdown: START T-5:00\r\n");
    serial_puts("--- system re-check ---\r\n");

    /* znovu kontrola systemu */
    supervisor_self_test();
    supervisor_report();

    if (master_alarm_count() > 0)
    {
        serial_puts("countdown: ABORT - master alarm!\r\n");
        state = COUNTDOWN_STOPPED;
        phase = COUNTDOWN_PHASE_IDLE;
        alarm_update_leds();
        return;
    }

    serial_puts("--- systems OK ---\r\n");
    safety_init();
    alarm_update_leds();
}

static void countdown_pause(void)
{
    uint32_t elapsed = HAL_GetTick() - start_tick;
    if (elapsed < total_ms)
        total_ms -= elapsed;
    else
        total_ms = 0;

    state = COUNTDOWN_STOPPED;
    phase = COUNTDOWN_PHASE_IDLE;
    serial_puts("countdown: PAUSE ");
    print_time(countdown_remaining_s());
    serial_puts("\r\n");
    alarm_update_leds();
}

static void countdown_add_min(void)
{
    total_ms += 60 * TICK_SEC;
    serial_puts("countdown: +1 min -> ");
    print_time(countdown_remaining_s());
    serial_puts("\r\n");
}

void countdown_abort(void)
{
    serial_puts("\r\ncountdown: ABORT - launch cancelled\r\n");

    /* 1) stabilizace vypnuta, serva do neutralu (vc. padaku) */
    stabilization_disengage();
    pca9685_set_servo_deg(PCA9685_SERVO_STAB1, 0);
    pca9685_set_servo_deg(PCA9685_SERVO_STAB2, 0);
    pca9685_set_servo_deg(PCA9685_SERVO_STAB3, 0);
    pca9685_set_servo_deg(PCA9685_SERVO_STAB4, 0);
    pca9685_set_servo_deg(PCA9685_SERVO_PARACHUTE, 0);

    /* 2) letovy FSM do IDLE */
    flight_abort();

    /* 3) countdown zpet do default */
    state = COUNTDOWN_STOPPED;
    phase = COUNTDOWN_PHASE_IDLE;
    total_ms = COUNTDOWN_DEFAULT_MIN * 60 * TICK_SEC;
    remote_paused = 0;
    test_mode = 0;

    /* 4) LED probliknou */
    abort_blink_count = LED_ABORT_BLINKS;
    abort_blink_tick = HAL_GetTick();
    abort_blink_on = 0;

    /* 5) dump cele telemetrie z RAM pres LoRa (zaznam celeho letu) */
    telem_buf_dump_start();

    serial_puts("countdown: T-5:00 (stopped, default)\r\n");
}

static void test_mode_toggle(void)
{
    test_mode = !test_mode;

    if (test_mode)
    {
        /* serva do neutralu, pak stabilizace na zemi */
        pca9685_set_servo_deg(PCA9685_SERVO_STAB1, 0);
        pca9685_set_servo_deg(PCA9685_SERVO_STAB2, 0);
        pca9685_set_servo_deg(PCA9685_SERVO_STAB3, 0);
        pca9685_set_servo_deg(PCA9685_SERVO_STAB4, 0);
        stabilization_engage();
        serial_puts("\r\ncountdown: TEST MODE ON - stabilization active (ground test)\r\n");
    }
    else
    {
        stabilization_disengage();
        pca9685_set_servo_deg(PCA9685_SERVO_STAB1, 0);
        pca9685_set_servo_deg(PCA9685_SERVO_STAB2, 0);
        pca9685_set_servo_deg(PCA9685_SERVO_STAB3, 0);
        pca9685_set_servo_deg(PCA9685_SERVO_STAB4, 0);
        serial_puts("countdown: TEST MODE OFF\r\n");
    }
}

uint8_t countdown_test_mode(void)
{
    return test_mode;
}

static void led_wave_step(void)
{
    switch (led_wave_pos)
    {
    case 0: led_set(1, 0, 0); break;  /* LD1 */
    case 1: led_set(0, 1, 0); break;  /* LD2 */
    case 2: led_set(0, 0, 1); break;  /* LD3 */
    case 3: led_set(0, 0, 0); break;  /* pauza */
    default: led_wave_pos = 0; break;
    }
    led_wave_pos = (led_wave_pos + 1) % 4;
}

static void countdown_launch(void)
{
    serial_puts("countdown: launch sequence\r\n");

    /* 1) stabilizace serva aktivni - vychozi (centrovana) poloha,
       padak zavreny */
    pca9685_set_servo_deg(PCA9685_SERVO_STAB1, 0);
    pca9685_set_servo_deg(PCA9685_SERVO_STAB2, 0);
    pca9685_set_servo_deg(PCA9685_SERVO_STAB3, 0);
    pca9685_set_servo_deg(PCA9685_SERVO_STAB4, 0);
    pca9685_set_servo_deg(PCA9685_SERVO_PARACHUTE, 0);
    serial_puts("countdown: stabilization servos active\r\n");

    /* 2) LoRa zprava LAUNCH */
    if (lora_send((const uint8_t *)"LAUNCH", 6) == 0)
        serial_puts("countdown: LoRa LAUNCH sent\r\n");
    else
        serial_puts("countdown: LoRa send FAIL\r\n");

    /* 3) SD log */
    if (logger_log("LAUNCH") == 0)
        serial_puts("countdown: SD log written\r\n");
    else
        serial_puts("countdown: SD log FAIL\r\n");

    /* 4) letovy FSM - PRE_LAUNCH (stabilizace se zapne pri liftoffu) */
    flight_start();
    serial_puts("countdown: flight FSM armed (PRE_LAUNCH)\r\n");

    alarm_update_leds();
}

static void countdown_toggle_run(void)
{
    if (state == COUNTDOWN_RUNNING)
        countdown_pause();
    else
        countdown_start();
}

static void btn_handle_edge(uint8_t now)
{
    if (now == 1 && btn_prev == 0)
    {
        uint32_t held = HAL_GetTick() - btn_down_tick;

        serial_puts("btn: up held=");
        print_unsigned((unsigned int)held);
        serial_puts("ms\r\n");

        if (held >= BTN_ABORT_MS)
        {
            countdown_abort();
        }
        else if (held >= BTN_LONG_PRESS_MS)
        {
            countdown_add_min();
        }
        else
        {
            /* kratky stisk (< 1 s): pocitame kliky v okne */
            click_count++;
            click_last_tick = HAL_GetTick();
        }
    }
}

void countdown_update(void)
{
    uint8_t now = btn_read();
    uint32_t t = HAL_GetTick();

    /* debounce + edge */
    if (now != btn_prev)
    {
        if ((t - btn_change_tick) >= BTN_DEBOUNCE_MS)
        {
            if (now == 0)
                btn_down_tick = t;
            btn_handle_edge(now);
            btn_prev = now;
            btn_change_tick = t;
        }
    }
    else
    {
        btn_change_tick = t;
    }

    /* odlozene stiskky - pocet kliku se vyhodnoti po vyprseni okna:
       1x = test mode, 2x = start/pause */
    if (click_count > 0 && (t - click_last_tick) > BTN_CLICK_WINDOW_MS)
    {
        if (click_count == 1)
            test_mode_toggle();
        else
            countdown_toggle_run();
        click_count = 0;
    }

    /* ABORT LED probliknuti */
    if (abort_blink_count > 0)
    {
        if (t - abort_blink_tick >= LED_ABORT_BLINK_MS)
        {
            abort_blink_tick = t;
            abort_blink_on = !abort_blink_on;
            led_set(abort_blink_on, abort_blink_on, abort_blink_on);
            abort_blink_count--;
            if (abort_blink_count == 0)
                alarm_update_leds();
        }
        return;
    }

    if (state != COUNTDOWN_RUNNING)
        return;

    /* safety watch bezi po cely cas odpocitavani */
    safety_update();
    if (safety_triggered())
    {
        serial_puts("\r\ncountdown: SAFETY HOLD - premature launch!\r\n");
        serial_puts("countdown: stabilization engaged\r\n");
        pca9685_set_servo_deg(PCA9685_SERVO_STAB1, 0);
        pca9685_set_servo_deg(PCA9685_SERVO_STAB2, 0);
        pca9685_set_servo_deg(PCA9685_SERVO_STAB3, 0);
        pca9685_set_servo_deg(PCA9685_SERVO_STAB4, 0);
        state = COUNTDOWN_STOPPED;
        phase = COUNTDOWN_PHASE_IDLE;
        alarm_update_leds();
        return;
    }

    uint32_t elapsed = t - start_tick;

    if (elapsed >= total_ms)
    {
        state = COUNTDOWN_ARMED;
        phase = COUNTDOWN_PHASE_IDLE;
        serial_puts("\r\n*** T-0: LAUNCH ***\r\n");
        countdown_launch();
        return;
    }

    uint32_t remain = total_ms - elapsed;

    /* konec CHECK faze; pod 30 s po celkou total_ms by total_ms - CHECK_MS
       underflowlo, proto osetrene saturovanym odecitanim */
    uint32_t check_end = (total_ms > CHECK_MS) ? (total_ms - CHECK_MS) : 0;

    if (remain > check_end)
    {
        /* CHECK faze - LED ukazuji alarmy */
        if (phase != COUNTDOWN_PHASE_CHECK)
        {
            phase = COUNTDOWN_PHASE_CHECK;
            serial_puts("countdown: CHECK phase (LED = alarms)\r\n");
            alarm_update_leds();
        }
    }
    else if (remain > WARNING_MS)
    {
        /* WAVE faze - LED vlnou */
        if (phase != COUNTDOWN_PHASE_WAVE)
        {
            phase = COUNTDOWN_PHASE_WAVE;
            serial_puts("countdown: launch prep - LED wave\r\n");
            led_tick = t;
            led_wave_pos = 0;
        }
        if (t - led_tick >= LED_WAVE_MS)
        {
            led_tick = t;
            led_wave_step();
        }
    }
    else
    {
        /* WARNING faze - poslednich 30 s, vyrazne blikani */
        if (phase != COUNTDOWN_PHASE_WARNING)
        {
            phase = COUNTDOWN_PHASE_WARNING;
            serial_puts("countdown: FINAL 30s WARNING\r\n");
            led_tick = t;
            led_warn_on = 0;
        }
        if (t - led_tick >= LED_WARNING_MS)
        {
            led_tick = t;
            led_warn_on = !led_warn_on;
            led_set(led_warn_on, led_warn_on, led_warn_on);
        }
    }
}
