/*
 * TFLuna.cpp
 *
 *  Created on: Aug 5, 2023
 *      Author: Mateusz Gburek
 */

#include "TFLuna.h"

static int timeout_miliseconds = 500;

void TFLunaSetFrequency(TFLuna *sensor, TFLunaFrequencies frequency)
{
	// [Head(0x5a) Len ID FreqL FreqH  Check_sum]
	uint8_t command[6] = {0x5A, 0x06, 0x03, 0x00, 0x00, 0x00};
	command[3] = (uint8_t)frequency;
	command[4] = (uint8_t)frequency >> 8;
	HAL_UART_Transmit(sensor->uart_handler, command, 6, timeout_miliseconds);

	uint8_t response[6];
	HAL_UART_Receive(sensor->uart_handler, response, 6, timeout_miliseconds);
	sensor->frequency = response[4] << 8 | response[3];
}

void TFLunaGetSample(TFLuna *sensor, uint16_t *distance)
{
	// [Head(0x5a) Len ID  Check_sum]
	uint8_t command[4] = {0x5A, 0x04, 0x04, 0x00};
	HAL_UART_Transmit(sensor->uart_handler, command, 4, timeout_miliseconds);

	uint8_t response[9]; // for different modes it should be of different sizes
	HAL_UART_Receive(sensor->uart_handler, response, 9, timeout_miliseconds);

	*distance = response[3] << 8 |  response[2];
	//*distance = response[4] << 8 |  response[3];
}

void TFLunaSetOutputFormat(TFLuna *sensor, uint8_t format)
{
	// [Head(0x5a) Len ID Format Check_sum]
	uint8_t command[5] = {0x5A, 0x05, 0x05, format, 0x00};
	HAL_UART_Transmit(sensor->uart_handler, command, 5, timeout_miliseconds);

	uint8_t response[5];
	HAL_UART_Receive(sensor->uart_handler, response, 5, timeout_miliseconds);
	sensor->format = response[3];
}

void TFLunaEnableOutput(TFLuna *sensor, uint8_t enable)
{
	// [Head(0x5a) Len ID Enable Check_sum]
	uint8_t command[5] = {0x5A, 0x05, 0x07, enable, 0x00};
	HAL_UART_Transmit(sensor->uart_handler, command, 5, timeout_miliseconds);

	uint8_t response[5];
	HAL_UART_Receive(sensor->uart_handler, response, 5, timeout_miliseconds);
	sensor->isEnabled = response[3];
}

void TFLunaRestoreDefault(TFLuna *sensor, uint8_t *success)
{
	// [Head(0x5a) Len ID  Check_sum]
	uint8_t command[4] = {0x5A, 0x04, 0x10, 0x00};
	HAL_UART_Transmit(sensor->uart_handler, command, 4, timeout_miliseconds);

	uint8_t response[5];
	HAL_UART_Receive(sensor->uart_handler, response, 5, timeout_miliseconds);

	*success = response[3];
}
