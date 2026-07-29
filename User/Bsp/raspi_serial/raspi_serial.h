
#ifndef __RASPI_SERIAL_H__
#define __RASPI_SERIAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

#define RASPI_SERIAL_FRAME_SIZE 96U

typedef struct {
    float x;
    float y;
    float th;
    float a;
    uint32_t sequence;
} raspi_serial_data_t;

bool raspi_serial_init(UART_HandleTypeDef *huart);

bool raspi_serial_get_latest(raspi_serial_data_t *data);
void send_reply(const char *reply);
#ifdef __cplusplus
}
#endif

#endif /* __RASPI_SERIAL_H__ */
