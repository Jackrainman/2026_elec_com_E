/**
 * @file    bsp.h
 * @author  Deadline039
 * @brief   Bsp layer export interface.
 * @version 1.0
 * @date    2024-09-18
 */

#ifndef __BSP_H
#define __BSP_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <cubemx.h>
#include "emm42/emm42.h"

#include "./core_delay/core_delay.h"
#include "./usart_ex/usart_ex.h"
#include "./can_list/can_list.h"
#include "./DJI-Motor/dji_bldc_motor.h"
#include "./arm/arm.h"
<<<<<<< HEAD
#include "./raspi_serial/raspi_serial.h"
=======
#include "raspi_serial/raspi_serial.h"
>>>>>>> e214c200d96f4d1acfd646f75e726734dc2b64ae


void bsp_init(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __BSP_H */
