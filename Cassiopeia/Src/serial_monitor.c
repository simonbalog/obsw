#include "serial_monitor.h"

#define RCC_AHB4ENR   (*(volatile unsigned int*)(0x580244E0))
#define RCC_APB1LENR  (*(volatile unsigned int*)(0x580244E8))
#define GPIOD_MODER   (*(volatile unsigned int*)(0x58020C00))
#define GPIOD_AFR2    (*(volatile unsigned int*)(0x58020C24))
#define USART3_CR1    (*(volatile unsigned int*)(0x40004800))
#define USART3_BRR    (*(volatile unsigned int*)(0x4000480C))
#define USART3_ISR    (*(volatile unsigned int*)(0x4000481C))
#define USART3_TDR    (*(volatile unsigned int*)(0x40004828))

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
    while (!(USART3_ISR & (1 << 7)));
    USART3_TDR = c;
}

void serial_puts(const char *s)
{
    while (*s)
        serial_putc(*s++);
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
