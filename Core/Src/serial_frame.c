/*
 * Serial_frame.c
 *
 *  Created on: Apr 21, 2026
 *      Author: Akkarasaknarong
 */

#include "Serial_frame.h"
#include <string.h>

void SerialFrame_Init(SerialFrame_t *frame, UART_HandleTypeDef *huart) {
	frame->huart = huart;
	frame->payload_len = 0;
	memset(frame->tx_buf, 0, sizeof(frame->tx_buf));
}

void SerialFrame_Reset(SerialFrame_t *frame) {
	frame->payload_len = 0;
}

HAL_StatusTypeDef SerialFrame_AddPayload(SerialFrame_t *frame, const void *data,uint16_t size) {
	if (frame->huart->gState != HAL_UART_STATE_READY) {
			return HAL_BUSY;
	}

	if ((frame->payload_len + size) > SERIAL_MAX_PAYLOAD) {
		return HAL_ERROR;
	}

	memcpy(&frame->tx_buf[2 + frame->payload_len], data, size);
	frame->payload_len += size;

	return HAL_OK;
}

HAL_StatusTypeDef SerialFrame_Transmit(SerialFrame_t *frame) {
	uint16_t total_len;

	frame->tx_buf[0] = SERIAL_HEADER1;
	frame->tx_buf[1] = SERIAL_HEADER2;

	total_len = 2 + frame->payload_len;
	HAL_StatusTypeDef status = HAL_UART_Transmit(frame->huart, frame->tx_buf, total_len,10);
	frame->payload_len = 0 ;
	return status ;
}

//void UARTDMAConfig()
//{
//    HAL_UART_Receive_DMA(&STLINK_UART, RxBuffer_Raw, rx_frame_size);
//}

void UART_Unpack(){
//	memcpy(&RxBuffer_Unpack, &RxBuffer_Raw, rx_frame_size);
//	HAL_UART_Receive_DMA(&hlpuart1, RxBuffer_Raw, rx_frame_size);
//
//	if (RxBuffer_Unpack[0] == 0xAA && RxBuffer_Unpack[1] == 0x55) {
//		memcpy(&REFdata.ref_q, &RxBuffer_Unpack[2], sizeof(float));
//		memcpy(&REFdata.ref_qd, &RxBuffer_Unpack[6], sizeof(float));
//		memcpy(&pwm_input_matlab, &RxBuffer_Unpack[10], sizeof(float));
//		memset(RxBuffer_Unpack, 0, rx_frame_size);
//	}
}

