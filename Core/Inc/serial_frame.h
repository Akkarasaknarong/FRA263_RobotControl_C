/*
 * Serial_Header.h
 *
 *  Created on: Apr 21, 2026
 *      Author: Akkarasaknarong
 */

#ifndef INC_SERIAL_FRAME_H_
#define INC_SERIAL_FRAME_H_

#include "datatype.h"

//uint8_t rx_frame_size = 14 ;
//uint8_t tx_frame_size = 30 ;
//uint8_t RxBuffer_Raw[84];

#define SERIAL_HEADER1 0xAA
#define SERIAL_HEADER2 0x55
#define SERIAL_MAX_PAYLOAD 64
#define SERIAL_MAX_FRAME_SIZE 2 + SERIAL_MAX_PAYLOAD

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t tx_buf[SERIAL_MAX_FRAME_SIZE];
    uint16_t payload_len;
} SerialFrame_t;

void SerialFrame_Init(SerialFrame_t *frame, UART_HandleTypeDef *huart);
void SerialFrame_Reset(SerialFrame_t *frame);
HAL_StatusTypeDef SerialFrame_AddPayload(SerialFrame_t *frame, const void *data, uint16_t size);
HAL_StatusTypeDef SerialFrame_Transmit(SerialFrame_t *frame);

#endif /* INC_SERIAL_FRAME_H_ */
