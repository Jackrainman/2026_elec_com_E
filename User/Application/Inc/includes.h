/**
 * @file    includes.h
 * @author  Deadline039
 * @brief   Include files
 * @version 1.0
 * @date    2024-04-03
 */

#ifndef __INCLUDES_H
#define __INCLUDES_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <bsp.h>

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

void freertos_start(void);

/* 任务分组 init, 在 start_task 中手动注释/启用 */
void key_tasks_init(void);  /* 按键调试机械臂 */
void recv_tasks_init(void); /* 接收树莓派上位机数据 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCLUDES_H */
