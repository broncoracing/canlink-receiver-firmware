#ifndef BOARD_H
#define BOARD_H

#include <mbed.h>

// Pins

// CAN Bus Transciever
#define CANTX_PIN           PA_12
#define CANRX_PIN           PA_11

// Accelerometer
#define ACCEL_SCL_PIN       PB_8
#define ACCEL_SDA_PIN       PB_9

// SD Card
#define SD_CARD_DETECT_PIN  PA_0
#define SD_SPISS_PIN        PB_12
#define SD_SCK_PIN          PB_13
#define SD_MISO_PIN         PB_14
#define SD_MOSI_PIN         PB_15

// XBEE Module
#define XBEE_UART_TX_PIN    PA_2
#define XBEE_UART_RX_PIN    PA_3

// LEDs
#define LED_PIN             PA_9

// AUX Pins
#define AUX1_PIN            PA_4
#define AUX2_PIN            PA_5

// Settings
#define XBEE_BAUD           230400

// prototypes
void initBoard();

#endif //BOARD_H