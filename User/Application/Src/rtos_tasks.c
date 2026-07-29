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

static TaskHandle_t arm_test_handle;
void arm_test(void *pvParameters);

/*****************************************************************************/

/**
 * @brief FreeRTOS start up.
 *
 */
void freertos_start(void) {
    xTaskCreate(start_task, "start_task", 128, NULL, 2, &start_task_handle);
    vTaskStartScheduler();
}

/**
 * @brief Start up task.
 *
 * @param pvParameters Start parameters.
 */
void start_task(void *pvParameters) {
    UNUSED(pvParameters);
    taskENTER_CRITICAL();

    xTaskCreate(task1, "task1", 128, NULL, 2, &task1_handle);
    xTaskCreate(arm_test, "arm_test", 512, NULL, 3, &arm_test_handle);

    vTaskDelete(start_task_handle);
    taskEXIT_CRITICAL();
}

/**
 * @brief Task1: Blink.
 *
 * @param pvParameters Start parameters.
 */
void task1(void *pvParameters) {
    UNUSED(pvParameters);

    LED0_OFF();
    LED1_ON();

    while (1) {
        LED0_TOGGLE();
        LED1_TOGGLE();
        vTaskDelay(1000);
    }
}

/* ======================== 机械臂测试任务 ======================== */

/* 测试用运动参数 */
#define TEST_SPEED      60      /* 测试转速 RPM */
#define TEST_XY_DELTA   50.0f   /* XY轴测试移动量 mm */
#define TEST_XY_SMALL   30.0f   /* XY轴小移动量 mm */
#define TEST_R_DELTA    90.0f   /* R轴测试旋转角度 ° */
#define TEST_R_SMALL    45.0f   /* R轴小旋转角度 ° */

/**
 * @brief  等待指定轴到位
 * @param  axis  轴编号
 * @note   阻塞式轮询，超时约 5s
 */
static void arm_wait_axis(uint8_t axis)
{
    uint32_t timeout = 5000 / 10;  /* 5s / 10ms = 500次 */
    arm_is_arrived(axis);           /* 发送首次查询 */
    while (timeout--) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (arm_is_arrived(axis)) {
            break;
        }
    }
}

/**
 * @brief  机械臂测试任务
 * @note   KEY0 → 回零
 *         KEY1 → XY 顺序移动
 *         KEY2 → R轴自转90°
 *         KEY3(WAKEUP) → XYR 顺序旋转
 */
void arm_test(void *pvParameters) {
    UNUSED(pvParameters);

    key_press_t key;

    /* 初始化机械臂 */
    arm_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        key = key_scan(0);

        switch (key) {
        case KEY0_PRESS:
            arm_set_zero();
            LED0_ON();
            vTaskDelay(pdMS_TO_TICKS(200));
            LED0_OFF();
            break;

        case KEY1_PRESS:
            /* XY 顺序移动 */
            arm_axis_rel_move(0,  TEST_XY_DELTA, TEST_SPEED);  /* X */
            arm_wait_axis(0);
            arm_axis_rel_move(1,  TEST_XY_DELTA, TEST_SPEED);  /* Y */
            arm_wait_axis(1);
            break;

        case KEY2_PRESS:
            /* R轴自转90° */
            arm_axis_rel_move(2, TEST_R_DELTA, TEST_SPEED);
            arm_wait_axis(2);
            break;

        case WKUP_PRESS:
            /* XYR 顺序旋转 */
            arm_axis_rel_move(0,  TEST_XY_SMALL, TEST_SPEED);  /* X */
            arm_wait_axis(0);
            arm_axis_rel_move(1,  TEST_XY_SMALL, TEST_SPEED);  /* Y */
            arm_wait_axis(1);
            arm_axis_rel_move(2,  TEST_R_SMALL, TEST_SPEED);   /* R */
            arm_wait_axis(2);
            break;

        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
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