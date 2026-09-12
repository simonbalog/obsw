#ifndef SERIAL_MONITOR_H
#define SERIAL_MONITOR_H

void serial_init(void);
void serial_puts(const char *s);
void serial_putc(char c);
void print_unsigned(unsigned int n);
void print_pad2(unsigned int n);
void print_pad4(unsigned int n);
void print_int(int n);

#endif
