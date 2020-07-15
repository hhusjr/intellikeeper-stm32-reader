//
// Created by 沈俊儒 on 2020/6/27.
//

#include <usart.h>
#include <intellikeeper.h>
#include "stm32f1xx_hal.h"
#include "zigbee.h"

uint8_t Tx_Buf[TX_BUFFER_SIZE], Rx_Buf[RX_BUFFER_SIZE];
uint8_t *Send_Buf = Tx_Buf + 2;
volatile uint8_t Received_Length, Need_To_Receive, state = 0;

struct Config Zigbee_Config;

static const char Type_Mapping[][20] = {
        "",
        "COORDINATOR",
        "ROUTER",
        "END_DEVICE"
};

static const uint32_t Baud_Rate_Mapping[] = {
        0,
        1200,
        2400,
        4800,
        9600,
        19200,
        38400,
        57600,
        115200
};

#define OP_CONFIG 0x00
#define OP_PING 0x01

#define OP_READER_SAY_HELLO 0x01

static void Zigbee_Handle() {
    uint8_t len;
    uint8_t* data;
    uint16_t addr;

    switch (Rx_Buf[0]) {
        case 0xFD:
            if (Received_Length < 7 || Rx_Buf[1] != Received_Length - 6) {
                // Malformed package
                return;
            }
            len = Received_Length - 6;
            data = Rx_Buf + 4;
            addr = Rx_Buf[Received_Length - 1] + (Rx_Buf[Received_Length - 2] << 8u);
            goto individual;
        case 0x00:
            if (Received_Length < 5) {
                // Malformed package
                return;
            }
            len = Received_Length - 4;
            data = Rx_Buf + 2;
            addr = Rx_Buf[Received_Length - 1] + (Rx_Buf[Received_Length - 2] << 8u);
            goto together;
        default:
            return;
    }

    individual: {
        if (addr != 0) {
            // Only receive from COORDINATOR
            return;
        }

        uint8_t op = data[0];
        uint8_t Body_Length = len - 1;
        uint8_t *body = data + 1;
        switch (op) {
            case OP_CONFIG:
                // 传感器配置
                if (Body_Length < 3) {
                    return;
                }
                Sensor_Light_On = body[0];
                Sensor_Move_On = body[1];
                Mute_Mode = body[2];
                Beep(50, 3, 50);
                break;

            default:
                break;
        }
    }

    together: {
        uint8_t op = data[0];
        uint8_t Body_Length = len - 1;
        uint8_t *body = data + 1;

        switch (op) {
            case OP_PING:
                Zigbee_SayHello();
                break;

            default:
                break;
        }
    };
}

void Zigbee_UART_IRQ(void) {
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET) {
        uint8_t dt = huart2.Instance->DR & 0x00FFu;
        Rx_Buf[Received_Length++] = dt;
        switch (state) {
            case 0:
                state = 1;
                break;

            case 1:
                state = 2;
                /*
                 * 1. 0x00 len ... addr addr
                 * 2. 0xFC len addr addr ...
                 * so 2 + dt
                 */
                Need_To_Receive = 2 + dt;
                break;

            case 2:
                Need_To_Receive--;
                if (Need_To_Receive == 0) {
                    Zigbee_Handle();
                    Received_Length = 0;
                    state = 0;
                }
                break;
        }
    } else {
        __HAL_UART_CLEAR_PEFLAG(&huart2);
        __HAL_UART_CLEAR_FEFLAG(&huart2);
        __HAL_UART_CLEAR_NEFLAG(&huart2);
        __HAL_UART_CLEAR_OREFLAG(&huart2);
    }
}

uint8_t Calc_Checksum(uint8_t len) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += Tx_Buf[i];
    }
    return sum & 0x00FFu;
}

void Zigbee_Read_Config() {
    Tx_Buf[0] = 0xFC;
    Tx_Buf[1] = 0x06;
    Tx_Buf[2] = 0x05;
    Tx_Buf[3] = 0x44;
    Tx_Buf[4] = 0x54;
    Tx_Buf[5] = 0x4B;
    Tx_Buf[6] = 0x52;
    Tx_Buf[7] = 0x46;
    Tx_Buf[8] = Calc_Checksum(8);

    try:
    HAL_UART_Transmit(&huart2, Tx_Buf, 9, 0x2000);
    if (HAL_OK != HAL_UART_Receive(&huart2, Rx_Buf, 47, 0x1000)) {
        HAL_Delay(2000);
        goto try;
    }

    uint8_t* p = Rx_Buf + 4;
    Zigbee_Config.type = p[0];
    Zigbee_Config.channel = p[3];
    Zigbee_Config.Transport_Mode = p[4];
    Zigbee_Config.address = (p[5] << 8u) + p[6];
    Zigbee_Config.Baud_Rate = p[9];
    Zigbee_Config.Aerial_Type = p[15];
    Zigbee_Config.Pan_ID = (p[1] << 8u) + p[2];
}

void Beep(uint16_t length, uint8_t times, uint16_t space) {
    if (Mute_Mode) {
        return;
    }
    while (times--) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(length);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(space);
    }
}

void Zigbee_Connect() {
    Tx_Buf[0] = 0xFC;
    Tx_Buf[1] = 0x06;
    Tx_Buf[2] = 0x04;
    Tx_Buf[3] = 0x44;
    Tx_Buf[4] = 0x54;
    Tx_Buf[5] = 0x4B;
    Tx_Buf[6] = 0x52;
    Tx_Buf[7] = 0x46;
    Tx_Buf[8] = Calc_Checksum(8);

    try:
    HAL_UART_Transmit(&huart2, Tx_Buf, 9, 0x2000);
    if (HAL_OK != HAL_UART_Receive(&huart2, Rx_Buf, 10, 0x500)) {
        HAL_Delay(2000);
        goto try;
    }

    Zigbee_Config.connected = 1;
    Zigbee_Config.version = ((Rx_Buf[7] << 8u) + Rx_Buf[8]) / 10.0;
}

uint8_t Zigbee_Write_Config() {
    uint8_t* p = Tx_Buf + 3;

    p[0] = Zigbee_Config.type;
    p[1] = Zigbee_Config.Pan_ID >> 8u;
    p[2] = Zigbee_Config.Pan_ID & 0x00FFu;
    p[3] = Zigbee_Config.channel;
    p[4] = Zigbee_Config.Transport_Mode;
    p[5] = Zigbee_Config.address >> 8u;
    p[6] = Zigbee_Config.address & 0x00FFu;
    p[7] = 0xAA;
    p[8] = 0xBB;
    p[9] = Zigbee_Config.Baud_Rate;
    p[10] = 0x01;
    p[11] = 0x01;
    p[12] = 0x01;
    p[13] = 0x05;
    p[14] = 0xA6;
    p[15] = Zigbee_Config.Aerial_Type;

    p[16] = ROUTER;
    p[17] = Zigbee_Config.Pan_ID >> 8u;
    p[18] = Zigbee_Config.Pan_ID & 0x00FFu;
    p[19] = Zigbee_Config.channel;
    p[20] = Zigbee_Config.Transport_Mode;
    p[21] = Zigbee_Config.address >> 8u;
    p[22] = Zigbee_Config.address & 0x00FFu;
    p[23] = 0xCC;
    p[24] = 0xDD;
    p[25] = Zigbee_Config.Baud_Rate;
    p[26] = 0x01;
    p[27] = 0x01;
    p[28] = 0x01;
    p[29] = 0x05;
    p[30] = 0xA6;
    p[31] = Zigbee_Config.Aerial_Type;
    p[32] = 0x01;
    p[33] = 0x00;
    p[34] = 0x00;
    p[35] = 0x00;
    p[36] = 0x00;
    p[37] = 0x00;

    Tx_Buf[0] = 0xFC;
    Tx_Buf[1] = 0x27;
    Tx_Buf[2] = 0x07;

    Tx_Buf[41] = Calc_Checksum(41);

    try:
    HAL_UART_Transmit(&huart2, Tx_Buf, 42, 0x2000);
    if (HAL_OK != HAL_UART_Receive(&huart2, Rx_Buf, 5, 0x1000)) {
        HAL_Delay(2000);
        goto try;
    }

    if (Rx_Buf[2] != 0x0A) {
        return 0;
    }
    return 1;
}
#define PAN_ID 0x2A01
#define BAUD_RATE 0x06
#define CHANNEL 0x0F

void Zigbee_ConfigureAs(uint8_t type) {
    Zigbee_Connect();
    Zigbee_Config.type = type;
    Zigbee_Config.Pan_ID = PAN_ID;
    Zigbee_Config.Aerial_Type = (type == END_DEVICE ? AERIAL_INTERNAL : AERIAL_EXTERNAL);
    Zigbee_Config.Baud_Rate = BAUD_RATE;
    Zigbee_Config.address = 0x01;
    Zigbee_Config.channel = CHANNEL;
    Zigbee_Config.Transport_Mode = TRANS_TRANSPARENT_ADR;
    if (!Zigbee_Write_Config()) {
        return;
    }
    Zigbee_Read_Config();
}

void Zigbee_Involve() {
    Tx_Buf[0] = 0xFC;
    Tx_Buf[1] = 0x06;
    Tx_Buf[2] = 0x0C;
    Tx_Buf[3] = 0x44;
    Tx_Buf[4] = 0x54;
    Tx_Buf[5] = 0x4B;
    Tx_Buf[6] = 0x52;
    Tx_Buf[7] = 0x46;
    Tx_Buf[8] = Calc_Checksum(8);

    int Try_Times = 0;
    try:
    __HAL_UART_CLEAR_PEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    HAL_UART_Transmit(&huart2, Tx_Buf, 9, 500);
    if (HAL_OK != HAL_UART_Receive(&huart2, Rx_Buf, 8, 500)) {
        HAL_Delay(1000);
        Try_Times++;
        if (Try_Times > 5) {
            Try_Times = 0;
            goto involve;
        }
        goto try;
    }
    goto finish;

    involve:
    Beep(20, 5, 20);
    HAL_Delay(100);
    for (uint8_t i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_Delay(120);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_Delay(200);
    }
    HAL_Delay(3000);
    goto try;

    finish:
    Beep(50, 2, 50);

    __HAL_UART_CLEAR_PEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

void Zigbee_SendData(uint8_t len) {
    Tx_Buf[0] = 0x00;
    Tx_Buf[1] = len;
    HAL_UART_Transmit(&huart2, Tx_Buf, len + 2, 500);
    HAL_Delay(200);
}

void Zigbee_SayHello() {
    Send_Buf[0] = OP_READER_SAY_HELLO;
    *(uint16_t*) (Send_Buf + 1) = INTELLIKEEPER_ID;
    Zigbee_SendData(3);
    Beep(20, 3, 20);
}
