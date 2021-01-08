#include "board.h"
#include "BR_CAN_IDs.h"
#include <mbed.h>

CAN can(CANRX_PIN, CANTX_PIN, BR_CAN_BAUD);
DigitalOut led(LED_PIN);
UnbufferedSerial xbee(XBEE_UART_TX_PIN, XBEE_UART_RX_PIN, XBEE_BAUD);

void initBoard()
{
    led.write(1);               // power indicator temporarily
    thread_sleep_for(250);      // wait for rails to stabilize
    can.frequency(BR_CAN_BAUD); // bug with MBED, must explicitly set baud
    xbee.baud(230400);          // set in XBEE XCTU software (max)
    thread_sleep_for(750);      // allow XBEE to boot
    led.write(0);               // led will be used as rf tx indicator
}