#include "CAN.h"
#include "SerialBase.h"
#include "can_helper.h"
#include "mbed.h"
#include "board.h"
#include <canSerializer.h>
#include <cstring>
#include <queue>

// External variables
extern CAN can;                 
extern UnbufferedSerial xbee;
extern DigitalOut led;

// All outgoing CAN frames go here
CANMessage outmsg;

char xbeeBuf = 0;       // UART bytes go here
queue<char> xbeeQueue;

// Prototypes
void xbeeISR();
void buildFrame(CANFrame *tempFrame);

// holds trial CAN frame before it is converted to an Mbed frame
uint8_t frameBuf[sizeof(CANFrame)] = {0};

bool gotByte = false;
Timer timer; 

int main()
{
    initBoard();
    timer.start();
    xbee.attach(xbeeISR, SerialBase::RxIrq);                                // dynamically set receive interrupt for XBEE transceiver

    while (1)
    {
        if (xbeeQueue.size() >= sizeof(CANFrame) * 2)                       // make sure we have enough data to build a packet
        {
            CANFrame tempFrame = {0};                                       // empty trial frame

            for (int i = 0; i < sizeof(CANFrame) - 1; i++)
            {
                frameBuf[i] = frameBuf[i + 1];
            }

            frameBuf[sizeof(frameBuf) - 1] = xbeeQueue.front();             // pull the latest value off the queue
            xbeeQueue.pop();                                                // delete it from queue

            memcpy(&tempFrame, frameBuf, sizeof(CANFrame));                 // build trial frame
            buildFrame(&tempFrame);                                         // build output frame if the checksum is valid
        }

        if (timer.read_ms() >= 500)                                         // we haven't gotten a frame in a while
        {
            led.write(0);
        }
        else
        {
            led.write(1);
        }
    }
}

void xbeeISR()
{
    if (xbeeQueue.size() >= 512)                                            // limit queue size
    {
        xbeeQueue.pop();
    }

    xbee.read((void*)&xbeeBuf, sizeof(xbeeBuf));
    xbeeQueue.push(xbeeBuf);
}

void buildFrame(CANFrame *tempFrame)
{
    if (tempFrame->syncWord == 0xA55A && validateChecksum(tempFrame)) // got a valid frame
    {
        outmsg.id = tempFrame->id;                               // build an Mbed CAN frame and send it
        outmsg.len = tempFrame->dlc;
        memcpy(outmsg.data, tempFrame->data, tempFrame->dlc); 
        
        if (tempFrame->extended)
        {
            outmsg.format = CANExtended;
        } 
        else
        {
            outmsg.format = CANStandard;
        }

        can.write(outmsg);

        timer.reset();
    }
}
