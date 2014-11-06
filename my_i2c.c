#include "maindefs.h"
#ifndef __XC8
#include <i2c.h>
#else
#include <plib/i2c.h>
#endif
#include "my_i2c.h"
#include "my_uart.h"
static i2c_comm *ic_ptr;
extern unsigned char GLOBAL_SENSOR_SEQUENCE_NUMBER;
extern unsigned char GLOBAL_MOTOR_SEQUENCE_NUMBER;

// Configure for I2C Master mode -- the variable "slave_addr" should be stored in
//   i2c_comm (as pointed to by ic_ptr) for later use.

void i2c_configure_master() {
    // Your code goes here
    // BEGIN CURTIS CHANGES

    SSPSTAT = 0x0;
    SSPCON1 = 0x0;
    SSPCON2 = 0x0;

    TRISBbits.TRISB4 = 1; // RB4 = SCL1
    TRISBbits.TRISB5 = 1; // RB5 = SDA1
    
    // REGISTER 19-5: SSPxSTAT: MSSPx STATUS REGISTER (I2C Mode)
    SSPSTATbits.SMP = 0x01; // Slew rate control is disabled for Standard Speed mode (100 kHz and 1 MHz)
    SSPSTATbits.CKE = 0x00; // Disable SMBus-specific inputs

    // REGISTER 19-6: SSPxCON1: MSSPx CONTROL REGISTER 1 (I2C Mode)
    SSP1CON1bits.SSPM = 0b1000;
    SSP1CON1bits.SSPEN = 0x01;

    SSP1ADD = 0x77;

    //ic_ptr->slave_addr = slave_addr;
    ic_ptr->status = I2C_IDLE;
}

static signed char length_of_message = 0;
static unsigned char msgtype;
static int cur_i2c_send_ind = 0;
static int cur_i2c_read_ind = 0;
void i2c_master_int_handler()
{
    switch(ic_ptr->status)
    {
        case I2C_IDLE:
        {
            break;
        }
        case I2C_CHECK_EMPTY:
        {
            // Check the queue to see if it is empty
            length_of_message = FromMainHigh_recvmsg(MAXI2CBUF,&msgtype,(void *) ic_ptr->outbuffer);
            if(length_of_message == MSGQUEUE_EMPTY)
            {
                // Message queue empty, no data to write
                // Should not come back to this ISR because we have not triggered
                // one of the five conditions to trigger the interrupt, see p. 308
                ic_ptr->status = I2C_IDLE;
            }
            else
            {
                if(msgtype == MSGT_I2C_SENSOR_READ || msgtype == MSGT_I2C_MOTOR_READ) // Check if next command is a read
                {
                    if(msgtype == MSGT_I2C_SENSOR_READ)
                    {
                        ic_ptr->slave_addr = 0x4F;
                    }
                    else // Motor Read
                    {
                        ic_ptr->slave_addr = 0x5C;
                    }
                    ic_ptr->buflen = length_of_message;
                    ic_ptr->status = I2C_READ_START_FROM_CHECK_EMPTY;
                    // Send the start bit to initiate read
                    SSP1CON2bits.SEN = 1;
                }
                else
                {
                    // There was something in the queue to be written. We have to send a start bit.
                    ic_ptr->outbuflen = length_of_message;
                    ic_ptr->status = I2C_STARTED_FROM_CHECK_EMPTY;
                    SSP1CON2bits.SEN = 1;
                }
            }
            break;
        }
        case I2C_STARTED_FROM_MAIN:
        {
            // Check the queue
            length_of_message = FromMainHigh_recvmsg(MAXI2CBUF,&msgtype,(void *) ic_ptr->outbuffer);

            ic_ptr->slave_addr = 0x5C; // Motor Slave Address
            // There was a message in the message queue
            // Load SSPBUF with slave address
            ic_ptr->outbuflen = length_of_message;
            SSPBUF = (ic_ptr->slave_addr << 1) | 0x0;
            ic_ptr->status = I2C_SLAVE_ADDR_LOADED;
            
            break;
        }
        case I2C_STARTED_FROM_CHECK_EMPTY:
        {
            ic_ptr->slave_addr = 0x5C; // Motor Slave Address

            // In this state, the queue has already been read from the IDLE state
            // Must load the SSPBUF with slave address
            SSPBUF = (ic_ptr->slave_addr << 1) | 0x0;
            ic_ptr->status = I2C_SLAVE_ADDR_LOADED;
            break;
        }
        case I2C_SLAVE_ADDR_LOADED:
        {
            unsigned char ack = SSPCON2bits.ACKSTAT;
            // Load SSPBUF with data
            SSPBUF = ic_ptr->outbuffer[cur_i2c_send_ind];
            cur_i2c_send_ind++;
            if( cur_i2c_send_ind == ic_ptr->outbuflen)
            {
                ic_ptr->status = I2C_DATA_SENT;
                cur_i2c_send_ind = 0;
            }
            break;
        }
        case I2C_DATA_SENT:
        {
            // Send the stop bit
            SSP1CON2bits.PEN = 1;
            // Go into the state to check if the queue was empty
            ic_ptr->status = I2C_CHECK_EMPTY;
            break;
        }
        case I2C_READ_START_FROM_MAIN:
        {
            length_of_message = FromMainHigh_recvmsg(MAXI2CBUF,&msgtype,(void *) ic_ptr->outbuffer);
            if(msgtype == MSGT_I2C_SENSOR_READ)
            {
                ic_ptr->slave_addr = 0x4F;
            }
            else // Motor Read
            {
                ic_ptr->slave_addr = 0x5C;
            }
            ic_ptr->buflen = length_of_message;
            SSPBUF = (ic_ptr->slave_addr << 1) | 0x1;
            ic_ptr->status = I2C_READ_CHECK_ACK;
            break;
        }
        case I2C_READ_START_FROM_CHECK_EMPTY:
        {
            // The start bit has been sent, so now we load the buffer with the slave address
            // Last bit is 1 for a read
            SSPBUF = (ic_ptr->slave_addr << 1) | 0x1;
            ic_ptr->status = I2C_READ_CHECK_ACK;
            break;
        }
        case I2C_READ_CHECK_ACK:
        {
            // We must check for the ACK from the slave before setting the 
            // read enable
            if(SSPCON2bits.ACKSTAT == 0)
            {
                SSPCON2bits.RCEN = 1;
                ic_ptr->status = I2C_READ_IN_BYTES;
            }
            break;
        }
        case I2C_READ_IN_BYTES:
        {
            ic_ptr->buffer[cur_i2c_read_ind] = SSPBUF;
            cur_i2c_read_ind++;
            if(cur_i2c_read_ind != ic_ptr->buflen)
            {
                SSPCON2bits.ACKEN = 1;
                SSPCON2bits.ACKDT = 0;
                ic_ptr->status = I2C_READ_NEEDS_MORE_BYTES;
            }
            else
            {
                SSPCON2bits.ACKEN = 1;
                SSPCON2bits.ACKDT = 1;
                cur_i2c_read_ind = 0;
                if(ic_ptr->slave_addr == 0x4F) // We did a sensor read
                {
                    unsigned char relay_sensor_data[12];
                    char check_empty = format_uart_sensor_reply(relay_sensor_data);
                    if(check_empty != 8)
                    {
                        ToMainLow_sendmsg(sizeof(relay_sensor_data), MSGT_UART_SEND_ARM, relay_sensor_data);
                    }
                }
                else // We did a motor read
                {
                    unsigned char relay_motor_data[6];
                    format_uart_motor_reply(relay_motor_data);
                    ToMainLow_sendmsg(sizeof(relay_motor_data), MSGT_UART_SEND_ARM, relay_motor_data);
                }
                ic_ptr->status = I2C_DONE_READING;
            }
            break;
        }
        case I2C_READ_NEEDS_MORE_BYTES:
        {
            SSPCON2bits.RCEN = 1;
            ic_ptr->status = I2C_READ_IN_BYTES;
            break;
        }
        case I2C_DONE_READING:
        {
            SSPCON2bits.PEN = 1;
            ic_ptr->status = I2C_CHECK_EMPTY;
            break;
        }
    }
}

// END CURTIS CHANGES

// Sending in I2C Master mode [slave write]
// 		returns -1 if the i2c bus is busy
// 		return 0 otherwise
// Will start the sending of an i2c message -- interrupt handler will take care of
//   completing the message send.  When the i2c message is sent (or the send has failed)
//   the interrupt handler will send an internal_message of type MSGT_MASTER_SEND_COMPLETE if
//   the send was successful and an internal_message of type MSGT_MASTER_SEND_FAILED if the
//   send failed (e.g., if the slave did not acknowledge).  Both of these internal_messages
//   will have a length of 0.
// The subroutine must copy the msg to be sent from the "msg" parameter below into
//   the structure to which ic_ptr points [there is already a suitable buffer there].

unsigned char i2c_master_send(unsigned char length, unsigned char *msg) {
    // Your code goes here
    // BEGIN CURTIS CHANGES
    
    if(ic_ptr->status == I2C_IDLE)
    {
        // Put the data that was given up into the queue
        signed char is_queue_full = FromMainHigh_sendmsg(length, MSGT_I2C_DATA, (void *) msg);
        // Send the start bit to trigger the interrupt handler
        SSP1CON2bits.SEN = 1;
        ic_ptr->status = I2C_STARTED_FROM_MAIN;
    }
    else
    {
        // Put the data that was given up into the queue
        signed char is_queue_full = FromMainHigh_sendmsg(length, MSGT_I2C_DATA, (void *) msg);
    }
    
    // END CURTIS CHANGES
    return(0);
}

// Receiving in I2C Master mode [slave read]
// 		returns -1 if the i2c bus is busy
// 		return 0 otherwise
// Will start the receiving of an i2c message -- interrupt handler will take care of
//   completing the i2c message receive.  When the receive is complete (or has failed)
//   the interrupt handler will send an internal_message of type MSGT_MASTER_RECV_COMPLETE if
//   the receive was successful and an internal_message of type MSGT_MASTER_RECV_FAILED if the
//   receive failed (e.g., if the slave did not acknowledge).  In the failure case
//   the internal_message will be of length 0.  In the successful case, the
//   internal_message will contain the message that was received [where the length
//   is determined by the parameter passed to i2c_master_recv()].
// The interrupt handler will be responsible for copying the message received into

unsigned char i2c_master_recv(unsigned char length, unsigned char * msg/*Not used in read*/, unsigned char slave_address) {
    // Your code goes here

    // BEGIN CURTIS CHANGES
    if(ic_ptr->status == I2C_IDLE)
    {
        if(slave_address == 0x4F) // Sensor Read
        {
            signed char is_queue_full = FromMainHigh_sendmsg(length, MSGT_I2C_SENSOR_READ, (void *) msg);
            ic_ptr->status = I2C_READ_START_FROM_MAIN;
            // Send the start bit
            SSP1CON2bits.SEN = 1;
        }
        else // Motor Read
        {
            signed char is_queue_full = FromMainHigh_sendmsg(length, MSGT_I2C_MOTOR_READ, (void *) msg);
            ic_ptr->status = I2C_READ_START_FROM_MAIN;
            // Send the start bit
            SSP1CON2bits.SEN = 1;
        }
    }
    else
    {
        if(slave_address == 0x4F) // Sensor Read
        {
            signed char is_queue_full = FromMainHigh_sendmsg(length, MSGT_I2C_SENSOR_READ, (void *) msg);
        }
        else // Motor Read
        {
            signed char is_queue_full = FromMainHigh_sendmsg(length, MSGT_I2C_MOTOR_READ, (void *) msg);
        }
    }
    // END CURTIS CHANGES
    return(0);
}

char format_uart_sensor_reply(unsigned char* final_buffer)
{
    char count_empty = 0;
    int len = ic_ptr->buflen + 4;
    final_buffer[0] = 0xFF; // API Start Byte
    final_buffer[1] = GLOBAL_SENSOR_SEQUENCE_NUMBER; // Sequence Number
    GLOBAL_SENSOR_SEQUENCE_NUMBER = (GLOBAL_SENSOR_SEQUENCE_NUMBER++) % 0xEF;
    final_buffer[2] = 0xFD;
    int i = 3;
    for(i = 3; i < (len - 1); i++)
    {
        final_buffer[i] = ic_ptr->buffer[i - 3];
        if(final_buffer[i] == 0)
        {
            count_empty++;
        }
    }
    final_buffer[len - 1] = 0xFE; // API End byte
    return count_empty;
}

void format_uart_motor_reply(unsigned char* mot_buffer)
{
    mot_buffer[0] = 0xFF;
    mot_buffer[1] = GLOBAL_MOTOR_SEQUENCE_NUMBER;
    GLOBAL_MOTOR_SEQUENCE_NUMBER = (GLOBAL_MOTOR_SEQUENCE_NUMBER++) % 0xEF;
    mot_buffer[2] = 0xFB;
    int ind_m = 3;
    int len_m = 6;
    for(ind_m = 3; ind_m < (len_m - 1); ind_m++)
    {
        mot_buffer[ind_m] = ic_ptr->buffer[ind_m - 3];
    }
    mot_buffer[len_m - 1] = 0xFE; // API End Byte
}

void start_i2c_slave_reply(unsigned char length, unsigned char *msg) {

    for (ic_ptr->outbuflen = 0; ic_ptr->outbuflen < length; ic_ptr->outbuflen++) {
        ic_ptr->outbuffer[ic_ptr->outbuflen] = msg[ic_ptr->outbuflen];
    }
    ic_ptr->outbuflen = length;
    ic_ptr->outbufind = 1; // point to the second byte to be sent

    // put the first byte into the I2C peripheral
    SSPBUF = ic_ptr->outbuffer[0];
    // we must be ready to go at this point, because we'll be releasing the I2C
    // peripheral which will soon trigger an interrupt
    SSPCON1bits.CKP = 1;

}

// an internal subroutine used in the slave version of the i2c_int_handler

void handle_start(unsigned char data_read) {
    ic_ptr->event_count = 1;
    ic_ptr->buflen = 0;
    // check to see if we also got the address
    if (data_read) {
        if (SSPSTATbits.D_A == 1) {
            // this is bad because we got data and
            // we wanted an address
            ic_ptr->status = I2C_IDLE;
            ic_ptr->error_count++;
            ic_ptr->error_code = I2C_ERR_NOADDR;
        } else {
            if (SSPSTATbits.R_W == 1) {
                ic_ptr->status = I2C_SLAVE_SEND;
            } else {
                ic_ptr->status = I2C_RCV_DATA;
            }
        }
    } else {
        ic_ptr->status = I2C_STARTED;
    }
}

// this is the interrupt handler for i2c -- it is currently built for slave mode
// -- to add master mode, you should determine (at the top of the interrupt handler)
//    which mode you are in and call the appropriate subroutine.  The existing code
//    below should be moved into its own "i2c_slave_handler()" routine and the new
//    master code should be in a subroutine called "i2c_master_handler()"

void i2c_int_handler() {
    unsigned char i2c_data;
    unsigned char data_read = 0;
    unsigned char data_written = 0;
    unsigned char msg_ready = 0;
    unsigned char msg_to_send = 0;
    unsigned char overrun_error = 0;
    unsigned char error_buf[3];

    // clear SSPOV
    if (SSPCON1bits.SSPOV == 1) {
        SSPCON1bits.SSPOV = 0;
        // we failed to read the buffer in time, so we know we
        // can't properly receive this message, just put us in the
        // a state where we are looking for a new message
        ic_ptr->status = I2C_IDLE;
        overrun_error = 1;
        ic_ptr->error_count++;
        ic_ptr->error_code = I2C_ERR_OVERRUN;
    }
    // read something if it is there
    if (SSPSTATbits.BF == 1) {
        i2c_data = SSPBUF;
        data_read = 1;
    }

    if (!overrun_error) {
        switch (ic_ptr->status) {
            case I2C_IDLE:
            {
                // ignore anything except a start
                if (SSPSTATbits.S == 1) {
                    handle_start(data_read);
                    // if we see a slave read, then we need to handle it here
                    if (ic_ptr->status == I2C_SLAVE_SEND) {
                        data_read = 0;
                        msg_to_send = 1;
                    }
                }
                break;
            }
            case I2C_STARTED:
            {
                // in this case, we expect either an address or a stop bit
                if (SSPSTATbits.P == 1) {
                    // we need to check to see if we also read an
                    // address (a message of length 0)
                    ic_ptr->event_count++;
                    if (data_read) {
                        if (SSPSTATbits.D_A == 0) {
                            msg_ready = 1;
                        } else {
                            ic_ptr->error_count++;
                            ic_ptr->error_code = I2C_ERR_NODATA;
                        }
                    }
                    ic_ptr->status = I2C_IDLE;
                } else if (data_read) {
                    ic_ptr->event_count++;
                    if (SSPSTATbits.D_A == 0) {
                        if (SSPSTATbits.R_W == 0) { // slave write
                            ic_ptr->status = I2C_RCV_DATA;
                        } else { // slave read
                            ic_ptr->status = I2C_SLAVE_SEND;
                            msg_to_send = 1;
                            // don't let the clock stretching bit be let go
                            data_read = 0;
                        }
                    } else {
                        ic_ptr->error_count++;
                        ic_ptr->status = I2C_IDLE;
                        ic_ptr->error_code = I2C_ERR_NODATA;
                    }
                }
                break;
            }
            case I2C_SLAVE_SEND:
            {
                if (ic_ptr->outbufind < ic_ptr->outbuflen) {
                    SSPBUF = ic_ptr->outbuffer[ic_ptr->outbufind];
                    ic_ptr->outbufind++;
                    data_written = 1;
                } else {
                    // we have nothing left to send
                    ic_ptr->status = I2C_IDLE;
                }
                break;
            }
            case I2C_RCV_DATA:
            {
                // we expect either data or a stop bit or a (if a restart, an addr)
                if (SSPSTATbits.P == 1) {
                    // we need to check to see if we also read data
                    ic_ptr->event_count++;
                    if (data_read) {
                        if (SSPSTATbits.D_A == 1) {
                            ic_ptr->buffer[ic_ptr->buflen] = i2c_data;
                            ic_ptr->buflen++;
                            msg_ready = 1;
                        } else {
                            ic_ptr->error_count++;
                            ic_ptr->error_code = I2C_ERR_NODATA;
                            ic_ptr->status = I2C_IDLE;
                        }
                    } else {
                        msg_ready = 1;
                    }
                    ic_ptr->status = I2C_IDLE;
                } else if (data_read) {
                    ic_ptr->event_count++;
                    if (SSPSTATbits.D_A == 1) {
                        ic_ptr->buffer[ic_ptr->buflen] = i2c_data;
                        ic_ptr->buflen++;
                    } else /* a restart */ {
                        if (SSPSTATbits.R_W == 1) {
                            ic_ptr->status = I2C_SLAVE_SEND;
                            msg_ready = 1;
                            msg_to_send = 1;
                            // don't let the clock stretching bit be let go
                            data_read = 0;
                        } else { /* bad to recv an address again, we aren't ready */
                            ic_ptr->error_count++;
                            ic_ptr->error_code = I2C_ERR_NODATA;
                            ic_ptr->status = I2C_IDLE;
                        }
                    }
                }
                break;
            }
        }
    }

    // release the clock stretching bit (if we should)
    if (data_read || data_written) {
        // release the clock
        if (SSPCON1bits.CKP == 0) {
            SSPCON1bits.CKP = 1;
        }
    }

    // must check if the message is too long, if
    if ((ic_ptr->buflen > MAXI2CBUF - 2) && (!msg_ready)) {
        ic_ptr->status = I2C_IDLE;
        ic_ptr->error_count++;
        ic_ptr->error_code = I2C_ERR_MSGTOOLONG;
    }

    if (msg_ready) {
        ic_ptr->buffer[ic_ptr->buflen] = ic_ptr->event_count;
        ToMainHigh_sendmsg(ic_ptr->buflen + 1, MSGT_I2C_DATA, (void *) ic_ptr->buffer);
        ic_ptr->buflen = 0;
    } else if (ic_ptr->error_count >= I2C_ERR_THRESHOLD) {
        error_buf[0] = ic_ptr->error_count;
        error_buf[1] = ic_ptr->error_code;
        error_buf[2] = ic_ptr->event_count;
        ToMainHigh_sendmsg(sizeof (unsigned char) *3, MSGT_I2C_DBG, (void *) error_buf);
        ic_ptr->error_count = 0;
    }
    if (msg_to_send) {
        // send to the queue to *ask* for the data to be sent out
        ToMainHigh_sendmsg(0, MSGT_I2C_RQST, (void *) ic_ptr->buffer);
        msg_to_send = 0;
    }
}

// set up the data structures for this i2c code
// should be called once before any i2c routines are called

void init_i2c(i2c_comm *ic) {
    ic_ptr = ic;
    ic_ptr->buflen = 0;
    ic_ptr->event_count = 0;
    ic_ptr->status = I2C_IDLE;
    ic_ptr->error_count = 0;
}

// setup the PIC to operate as a slave
// the address must include the R/W bit

void i2c_configure_slave(unsigned char addr) {

    // ensure the two lines are set for input (we are a slave)
#ifdef __USE18F26J50
    //THIS CODE LOOKS WRONG, SHOULDN'T IT BE USING THE TRIS BITS???
    PORTBbits.SCL1 = 1;
    PORTBbits.SDA1 = 1;
#else
#ifdef __USE18F46J50
    TRISBbits.TRISB4 = 1; //RB4 = SCL1
    TRISBbits.TRISB5 = 1; //RB5 = SDA1
#else
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
#endif
#endif

    // set the address
    SSPADD = addr;
    //OpenI2C(SLAVE_7,SLEW_OFF); // replaced w/ code below
    SSPSTAT = 0x0;
    SSPCON1 = 0x0;
    SSPCON2 = 0x0;
    SSPCON1 |= 0x0E; // enable Slave 7-bit w/ start/stop interrupts
    SSPSTAT |= SLEW_OFF;

#ifdef I2C_V3
    I2C1_SCL = 1;
    I2C1_SDA = 1;
#else 
#ifdef I2C_V1
    I2C_SCL = 1;
    I2C_SDA = 1;
#else
#ifdef __USE18F26J50
    PORTBbits.SCL1 = 1;
    PORTBbits.SDA1 = 1;
#else
#ifdef __USE18F46J50
    PORTBbits.SCL1 = 1;
    PORTBbits.SDA1 = 1;
#else
    __dummyXY=35;// Something is messed up with the #ifdefs; this line is designed to invoke a compiler error
#endif
#endif
#endif
#endif
    
    // enable clock-stretching
    SSPCON2bits.SEN = 1;
    SSPCON1 |= SSPENB;
    // end of i2c configure
}