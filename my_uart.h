#ifndef __my_uart_h
#define __my_uart_h

#include "messages.h"

#define MAXUARTBUF 4
#if (MAXUARTBUF > MSGLEN)
#define MAXUARTBUF MSGLEN
#endif
typedef struct __uart_comm {
    unsigned char buffer[MAXUARTBUF];
    unsigned char buflen;
} uart_comm;

void init_uart_recv(uart_comm *);
void uart_recv_int_handler(void);
void set_uart_bits();
void uart_send_int_handler();
void send_uart_msg(int len, unsigned char* msg_buffer);

#endif
