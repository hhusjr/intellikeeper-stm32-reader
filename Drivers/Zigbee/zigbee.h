//
// Created by 沈俊儒 on 2020/6/27.
//

#ifndef ZIGBEE_H
#define ZIGBEE_H

#define TX_BUFFER_SIZE 300
#define RX_BUFFER_SIZE 300

#define COORDINATOR 1
#define ROUTER      2
#define END_DEVICE  3

#define TRANS_TRANSPARENT     1
#define TRANS_TRANSPARENT_ADR 3
#define TRANS_TRANSPARENT_MAC 4
#define TRANS_N2N             5

#define BAUD_115200 0x08

#define AERIAL_INTERNAL 0
#define AERIAL_EXTERNAL 1

struct Config {
    uint8_t connected;
    double version;

    uint8_t type;
    uint8_t channel;
    uint8_t Transport_Mode;
    uint16_t address;
    uint16_t Pan_ID;

    uint8_t Baud_Rate;
    uint8_t Aerial_Type;
};

extern struct Config Zigbee_Config;

void Beep(uint16_t length, uint8_t times, uint16_t space);
void Zigbee_Involve(void);
void Zigbee_SayHello(void);
void Zigbee_UART_IRQ(void);
void Zigbee_ReceivedCount(void);
void Zigbee_ConfigureAs(uint8_t type);
void Zigbee_Read_Config();

#endif //ZIGBEE_H
