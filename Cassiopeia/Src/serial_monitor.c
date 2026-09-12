#include "serial_monitor.h"
#include "stm32h7xx_hal.h"

#define SERIAL_TX_TIMEOUT_MS 100
#define SERIAL_TX_TIMEOUT_LOOPS 1000000U

#define RCC_AHB4ENR   (*(volatile unsigned int*)(0x580244E0))
#define RCC_APB1LENR  (*(volatile unsigned int*)(0x580244E8))
#define GPIOD_MODER   (*(volatile unsigned int*)(0x58020C00))
#define GPIOD_AFR2    (*(volatile unsigned int*)(0x58020C24))
#define USART3_CR1    (*(volatile unsigned int*)(0x40004800))
#define USART3_BRR    (*(volatile unsigned int*)(0x4000480C))
#define USART3_ISR    (*(volatile unsigned int*)(0x4000481C))
#define USART3_TDR    (*(volatile unsigned int*)(0x40004828))
#define USART3_RDR    (*(volatile unsigned int*)(0x40004824))
#define USART3_ISR_RXNE (1U << 5)

static unsigned char command_buf[64];
static unsigned char command_len;

void serial_init(void)
{
    RCC_AHB4ENR |= (1 << 3);
    RCC_APB1LENR |= (1 << 18);

    volatile int d;
    for (d = 0; d < 1000; d++);

    GPIOD_MODER = (GPIOD_MODER & ~(3 << 16)) | (2 << 16);
    GPIOD_AFR2 = (GPIOD_AFR2 & ~(0xFF << 0)) | (0x77 << 0);

    USART3_CR1 = 0;
    USART3_BRR = 556;
    for (d = 0; d < 100; d++);
    USART3_CR1 = (1 << 0) | (1 << 3);
}

void serial_putc(char c)
{
    uint32_t start = HAL_GetTick();
    uint32_t loops = 0;
    while (!(USART3_ISR & (1 << 7)))
    {
        if ((uint32_t)(HAL_GetTick() - start) >= SERIAL_TX_TIMEOUT_MS ||
            ++loops >= SERIAL_TX_TIMEOUT_LOOPS)
            return;
    }
    USART3_TDR = c;
}

void serial_puts(const char *s)
{
    if (s == 0)
        return;
    while (*s)
        serial_putc(*s++);
}

int serial_command_poll(unsigned char *buf, unsigned char *len, unsigned char max_len)
{
    while ((USART3_ISR & USART3_ISR_RXNE) != 0U)
    {
        unsigned char c = (unsigned char)USART3_RDR;
        if (c == '\r' || c == '\n')
        {
            if (command_len == 0U) continue;
            if (command_len > max_len) command_len = max_len;
            for (unsigned char i = 0; i < command_len; i++) buf[i] = command_buf[i];
            *len = command_len;
            command_len = 0;
            return 1;
        }
        if (c == 8U || c == 127U) {
            if (command_len) command_len--;
        } else if (command_len < sizeof(command_buf) - 1U && c >= 32U && c <= 126U) {
            command_buf[command_len++] = c;
        }
    }
    return 0;
}

void print_unsigned(unsigned int n)
{
    char buf[12];
    int i = 11;
    buf[11] = 0;
    if (n == 0) buf[--i] = '0';
    while (n) { buf[--i] = '0' + (n % 10); n /= 10; }
    serial_puts(buf + i);
}

void print_pad2(unsigned int n)
{
    if (n > 99) n = 99;
    char b[3];
    b[0] = '0' + (n / 10) % 10;
    b[1] = '0' + n % 10;
    b[2] = 0;
    serial_puts(b);
}

void print_pad4(unsigned int n)
{
    if (n > 9999) n = 9999;
    char b[5];
    b[0] = '0' + (n / 1000) % 10;
    b[1] = '0' + (n / 100) % 10;
    b[2] = '0' + (n / 10) % 10;
    b[3] = '0' + n % 10;
    b[4] = 0;
    serial_puts(b);
}

void print_int(int n)
{
    if (n < 0)
    {
        serial_putc('-');
        n = -n;
    }
    print_unsigned((unsigned int)n);
}
