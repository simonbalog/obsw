#ifndef SERIAL_MONITOR_H
#define SERIAL_MONITOR_H

void serial_init(void);
void serial_puts(const char *s);
void serial_putc(char c);
void print_unsigned(unsigned int n);
void print_pad2(unsigned int n);
void print_pad4(unsigned int n);
void print_int(int n);
/* Non-blocking command line input on the same UART used for diagnostics. */
int serial_command_poll(unsigned char *buf, unsigned char *len, unsigned char max_len);

#endif
