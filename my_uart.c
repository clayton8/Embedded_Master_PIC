#include "maindefs.h"
#ifndef __XC8
#include <usart.h>
#else
#include <plib/usart.h>
#endif
#include "my_uart.h"
#include "debug.h"

static uart_comm *uc_ptr;

void uart_recv_int_handler() {
#ifdef __USE18F26J50
    if (DataRdy1USART()) {
        uc_ptr->buffer[uc_ptr->buflen] = Read1USART();
#else
#ifdef __USE18F46J50
    if (DataRdy1USART()) {
        uc_ptr->buffer[uc_ptr->buflen] = Read1USART();
#else
    if (DataRdyUSART()) {
        uc_ptr->buffer[uc_ptr->buflen] = ReadUSART();
#endif
#endif
        /*
         * THE FOLLOWING CODE IS HOW WE WOULD HANDLE AN ACK AND ERROR CHECKING
         * FOR A MOTOR COMMAND TO DETECT DROPPED PACKETS
        if(uc_ptr->buffer[uc_ptr->buflen] == 0xFE)
        {
            if(uc_ptr->buflen != MAXUARTBUF - 1)
            {

                // Send Error message
            }
            else
            {
                if(seqNumMotor != uc_ptr->buffer[SEQ_NUM_PLACE] )
                {
                    // Send Error Missing sequence
                }
                // Send Motor Ack
            }
        }
        else if(uc_ptr->buflen == MAXUARTBUF - 1)
        {
            // Send dropped byte error
        }
        */
        uc_ptr->buflen++;
        // check if a message should be sent
        if (uc_ptr->buflen == MAXUARTBUF) {
            // We have gotten a motor command. Respond with a motor response.
            /*
            unsigned char motor_ack[4];
            int i = 0;
            for(i = 0; i < 3; i++)
            {
                motor_ack[i] = uc_ptr->buffer[i];
            }
            motor_ack[3] = 0xFE; // API stop byte
            send_uart_msg(sizeof(motor_ack), motor_ack);
            */
            ToMainLow_sendmsg(uc_ptr->buflen, MSGT_UART_DATA, (void *) uc_ptr->buffer);
            uc_ptr->buflen = 0;
        }
    }
#ifdef __USE18F26J50
    if (USART1_Status.OVERRUN_ERROR == 1) {
#else
#ifdef __USE18F46J50
    if (USART1_Status.OVERRUN_ERROR == 1) {
#else
    if (USART_Status.OVERRUN_ERROR == 1) {
#endif
#endif
        // we've overrun the USART and must reset
        // send an error message for this
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
        ToMainLow_sendmsg(0, MSGT_OVERRUN, (void *) 0);
    }
}

void init_uart_recv(uart_comm *uc) {
    uc_ptr = uc;
    uc_ptr->buflen = 0;
}

// Interrupt driven UART send. Called from the Low Priority Interrupt Handler.
static unsigned char middle_of_message = 0;
static unsigned char cur_msg_ind = 0;
static signed char length_of_message = 0;
static unsigned char msgtype;
static unsigned char msgbuffer[MSGLEN + 1];
void uart_send_int_handler()
{
    // If we are not in the middle of a message, we should read from the queue.
    // Once we read from a queue, we are in the middle of transmitting a message.
    if(!middle_of_message)
    {
        length_of_message = FromMainLow_recvmsg(MSGLEN,&msgtype,(void *) &msgbuffer);
        // If the message queue we read from is empty, we must turn off the
        // trasmit UART interrupt and reset the static variables
        if(length_of_message == MSGQUEUE_EMPTY)
        {
            middle_of_message = 0;
            cur_msg_ind = 0;
            PIE1bits.TX1IE = 0x0;
        }
        else
        {
            middle_of_message = 1;
        }
    }
    // If we were in the middle of transmitting a message, then this else
    // block gets executed
    else
    {
        // We want to load the transmit register with one byte of information from the message.
        // We send each byte until the index is equal to the length of the message.
        if(cur_msg_ind < length_of_message)
        {
            TXREG1 = msgbuffer[cur_msg_ind];
            cur_msg_ind++;
        }
        else
        {
            // At this point, the current message index is equal to the length of the message.
            // We have now written all of the bytes in the message to UART.
            middle_of_message = 0;
            cur_msg_ind = 0;
        }

    }
}

// This function puts the message into the FromMainLow Queue, and the sending
// of the message through UART occurs in an interrupt driven way.
void send_uart_msg(int len, unsigned char* msg_buffer)
{
    signed char is_queue_full = FromMainLow_sendmsg(len, MSGT_UART_DATA, (void *) msg_buffer);
    if( is_queue_full == MSGQUEUE_FULL)
    {
        // Return that the queue was full
    }
    else
    {
        PIE1bits.TX1IE = 0x1;  // Enable low priority interrupt for transmit
    }
}

// Configure the necessary registers for asynchronous UART transmission
void set_uart_bits()
{
    TRISCbits.TRISC7 = 0x1; // Set to 1 for EUSART1
    TRISCbits.TRISC6 = 0x0; // Set to 0 for asynchronous master mode

    // Register 20-1: TXSTAx: TRANSMIT STATUS AND CONTROL REGISTER (ACCESS FADh, FA8h)
    TXSTA1bits.TXEN = 0x1; // Transmit is enabled
    TXSTA1bits.SYNC = 0x0; // Set EUSART mode to Asynchronous
    TXSTA1bits.BRGH = 0x1; // High speed Baud Rate

    // Register 20-2: RCSTAx: RECEIVE STATUS AND CONTROL REGISTER (ACCESS FACh, F9Ch)
    RCSTA1bits.SPEN = 0x1; // Serial port is enabled

    // Register 20-3: BAUDCONx: BAUD RATE CONTROL REGISTER (ACCESS F7Eh, F7Ch)
    BAUDCON1bits.BRG16 = 0x1; // 16-bit Baud Rate Generator

    // Set up baud rate generator for 57,600. See Baud Rate formula on page 328 of datasheet.
    // Actual baud rate generated here is 57,692 which is 0.16% error.
    SPBRGH1 = 0x0; // High byte
    SPBRG1 = 0xCF; // Low byte, 207 in decimal

    // UART Transmit is a low priority interrupt
    IPR1bits.TX1IP = 0x0;
}