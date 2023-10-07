/*
 * TFLuna.h
 *
 *  Created on: Aug 5, 2023
 *      Author: Mateusz Gburek
 */

#ifndef INC_TFLUNA_H_
#define INC_TFLUNA_H_

#include "stm32f4xx_hal.h"

typedef enum {
	FREQ_0Hz = 0,
	FREQ_1Hz = 1,
	FREQ_2Hz = 2,
	FREQ_100Hz = 100,
	FREQ_125Hz = 125,
	FREQ_166Hz = 166,
	FREQ_250Hz = 250
} TFLunaFrequencies;

typedef enum {
	CM_9BYTE = 0x01,
	PIX = 0x02,
	MM_9BYTE = 0x06,
	WITH_TIMESTAMP_32BYTE = 0x07,
	ID_0 = 0x08,
	CM_8BYTE = 0x09
} TFLunaOutputFormat;

typedef struct
{
	UART_HandleTypeDef *uart_handler;
	uint16_t frequency;
	TFLunaOutputFormat format;
	uint8_t isEnabled;

} TFLuna;

void TFLunaSetFrequency(TFLuna *sensor, TFLunaFrequencies frequency);

void TFLunaGetSample(TFLuna *sensor, uint16_t *distance);

void TFLunaSetOutputFormat(TFLuna *sensor, TFLunaOutputFormat format);

void TFLunaEnableOutput(TFLuna *sensor, uint8_t enable);

void TFLunaRestoreDefault(TFLuna *sensor, uint8_t *success);

#endif /* INC_TFLUNA_H_ */
