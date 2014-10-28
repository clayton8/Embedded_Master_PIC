#ifndef __my_i2c_h
#define __my_i2c_h

#include "messages.h"

#define MAXI2CBUF MSGLEN
typedef struct __i2c_comm {
    unsigned char buffer[MAXI2CBUF];
    unsigned char buflen;
    unsigned char event_count;
    unsigned char status;
    unsigned char error_code;
    unsigned char error_count;
    unsigned char outbuffer[MAXI2CBUF];
    unsigned char outbuflen;
    unsigned char outbufind;
    unsigned char slave_addr;
} i2c_comm;

#define I2C_IDLE 0x5
#define I2C_STARTED 0x6
#define	I2C_RCV_DATA 0x7
#define I2C_SLAVE_SEND 0x8
#define I2C_DATA_SENT 0x4
#define I2C_STARTED_FROM_CHECK_EMPTY 0x2
#define I2C_STARTED_FROM_MAIN 0x3
#define I2C_SLAVE_ADDR_LOADED 0x9
#define I2C_CHECK_EMPTY 0x1
#define I2C_READ_START_FROM_CHECK_EMPTY 0xA
#define I2C_READ_CHECK_ACK 0xB
#define I2C_READ_IN_BYTES 0xC
#define I2C_READ_NEEDS_MORE_BYTES 0xD
#define I2C_DONE_READING 0xE
#define I2C_READ_START_FROM_MAIN 0xF

#define I2C_ERR_THRESHOLD 1
#define I2C_ERR_OVERRUN 0x4
#define I2C_ERR_NOADDR 0x5
#define I2C_ERR_NODATA 0x6
#define I2C_ERR_MSGTOOLONG 0x7
#define I2C_ERR_MSG_TRUNC 0x8

void init_i2c(i2c_comm *);
void i2c_int_handler(void);
void start_i2c_slave_reply(unsigned char,unsigned char *);
void i2c_configure_slave(unsigned char);
void i2c_configure_master(unsigned char);
unsigned char i2c_master_send(unsigned char,unsigned char *);
unsigned char i2c_master_recv(unsigned char, unsigned char *);
// CURTIS CHANGES
void i2c_master_int_handler();
char format_uart_sensor_reply(unsigned char*);
// END CURTIS CHANGES

#endif