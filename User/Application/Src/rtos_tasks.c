/**
 * @file    rtos_tasks.c
 * @author  Deadline039
 * @brief   RTOS tasks.
 * @version 1.0
 * @date    2024-01-31
 */

#include "includes.h"

static TaskHandle_t start_task_handle;
void start_task(void *pvParameters);

static TaskHandle_t task1_handle;
void task1(void *pvParameters);

static TaskHandle_t task4_handle;
void task4(void *pvParameters);

#define DEMO_MOTOR_SPEED_RPM  300U   /* 换向演示速度 (RPM) */
#define DEMO_MOTOR_TOGGLE_MS  2000U  /* 换向周期 (ms) */

/*****************************************************************************/

/**
 * @brief FreeRTOS start up.
 *
 */
void freertos_start(void) {
    xTaskCreate(start_task, "start_task", 256, NULL, 2, &start_task_handle);
    vTaskStartScheduler();
}

/**
 * @brief Start up task.
 *
 * @param pvParameters Start parameters.
 */
void start_task(void *pvParameters) {
    UNUSED(pvParameters);

    /* 必须有的任务: LED 心跳 */
    xTaskCreate(task1, "task1", 128, NULL, 2, &task1_handle);
    // xTaskCreate(task4, "task4", 128, NULL, 2, &task4_handle); /* 已无用, 保留备用 */

    /* 按需开关: 手动注释掉不用的一路 init 即可, 无需宏定义 */
    key_tasks_init();  /* 按键调试机械臂 (只有用 KEY 时需要) */
    recv_tasks_init(); /* 接收树莓派上位机数据 (只有接上位机时需要) */

    vTaskDelete(NULL);
}

/**
 * @brief Task1: Blink.
 *
 * @param pvParameters Start parameters.
 */
void task1(void *pvParameters) {
    UNUSED(pvParameters);

    LED0_OFF();
    LED1_OFF();

    while (1) {
        LED0_TOGGLE();
        vTaskDelay(1000);
    }
}

/**
 * @brief Task4: Emm42 motor toggle direction periodically (left/right).
 *
 * @param pvParameters Start parameters.
 */
void task4(void *pvParameters) {
    UNUSED(pvParameters);

    static emm42_motor_t demo_motor;
    uint8_t dir = 0U;

    /* 初始化电机: UART4 总线, 地址 1, RE 脚 PD14 */
    emm42_motor_init(&demo_motor, &huart4, 2, RS485_RE1_GPIO_Port,
                     RS485_RE1_Pin);
    emm42_en_control(&demo_motor, true, false);

    while (1) {
        /* 左转/右转交替 (0 = CW, 1 = CCW) */
        dir ^= 1U;

        emm42_vel_control(&demo_motor, dir, DEMO_MOTOR_SPEED_RPM, 0, false);

        vTaskDelay(DEMO_MOTOR_TOGGLE_MS);
    }
}

#ifdef configASSERT
/**
 * @brief FreeRTOS assert failed function.
 *
 * @param pcFile File name
 * @param ulLine File line
 */
void vAssertCalled(const char *pcFile, unsigned int ulLine) {
    fprintf(stderr, "FreeRTOS assert failed. File: %s, line: %u. \n", pcFile,
            ulLine);
}
#endif /* configASSERT */

#if configCHECK_FOR_STACK_OVERFLOW
/**
 * @brief The application stack overflow hook is called when a stack overflow is detected for a task.
 *
 * @param xTask the task that just exceeded its stack boundaries.
 * @param pcTaskName A character string containing the name of the offending task.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    UNUSED(xTask);
    fprintf(stderr, "Stack overflow! Taskname: %s. \n", pcTaskName);
}
#endif /* configCHECK_FOR_STACK_OVERFLOW */

#if configUSE_MALLOC_FAILED_HOOK
/**
 * @brief This hook function is called when allocation failed.
 *
 */
void vApplicationMallocFailedHook(void) {
    fprintf(stderr, "FreeRTOS malloc failed! \n");
}
#endif /* configUSE_MALLOC_FAILED_HOOK */
