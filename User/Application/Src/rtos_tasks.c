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

static TaskHandle_t task2_handle;
void task2(void *pvParameters);

static TaskHandle_t arm_test_handle;
void arm_test(void *pvParameters);

static TaskHandle_t msg_receive_handle;
void msg_receive(void *pvParameters);

static TaskHandle_t arm_ctrl_handle;
void arm_ctrl(void *pvParameters);

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
    xTaskCreate(task2, "task2", 128, NULL, 2, &task2_handle);
    xTaskCreate(arm_test, "arm_test", 512, NULL, 3, &arm_test_handle);
    xTaskCreate(msg_receive, "msg_receive", 256, NULL, 3, &msg_receive_handle);
    xTaskCreate(arm_ctrl, "arm_ctrl", 512, NULL, 3, &arm_ctrl_handle);

    // vTaskSuspend(arm_test_handle);
    // vTaskSuspend(msg_receive_handle);
    // vTaskSuspend(arm_ctrl_handle);

    raspi_serial_init(&huart1);

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
    LED1_OFF();

    while (1) {
        LED0_TOGGLE();
        vTaskDelay(1000);
    }
}

/**
 * @brief Task2: print running time.
 *
 * @param pvParameters Start parameters.
 */
void task2(void *pvParameters) {
    UNUSED(pvParameters);

    while (1) {
        printf("STM32G4xx FreeRTOS project template. Running time: %u ms. \n",
               xTaskGetTickCount());
        vTaskDelay(1000);
    }
}

/**
 * @brief Arm test task: 按键控制机械臂动作（绝对位置）
 *
 * KEY0: X/Y/R 目标位置 (50mm, 50mm, 90°)
 * KEY1: X/Y/R 目标位置 (30mm, 30mm, 0°)
 * KEY2: X/Y/R 目标位置 (100mm, 100mm, 270°)
 * WKUP: X/Y/R 回零 (0, 0, 0)
 *
 * @param pvParameters Start parameters.
 */
void arm_test(void *pvParameters) {
    UNUSED(pvParameters);

    arm_init();

    key_press_t key;

    while (1) {
        key = key_scan(0);
        switch (key) {
            case KEY0_PRESS: {
                arm_axis_move(1, ARM_X_MM_TO_PULSE(50), 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(2, ARM_Y_MM_TO_PULSE(50), 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(3, ARM_DEG_TO_PULSE(90), 100);
            } break;

            case KEY1_PRESS: {
                arm_axis_move(1, ARM_X_MM_TO_PULSE(30), 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(2, ARM_Y_MM_TO_PULSE(30), 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(3, 0, 100);
            } break;

            case KEY2_PRESS: {
                arm_axis_move(1, ARM_X_MM_TO_PULSE(100), 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(2, ARM_Y_MM_TO_PULSE(100), 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(3, ARM_DEG_TO_PULSE(270), 100);
            } break;

            case WKUP_PRESS: {
                arm_axis_move(1, 0, 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(2, 0, 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(3, 0, 100);
            } break;

            default:
                break;
        }

        vTaskDelay(10);
    }
}

/**
 * @brief 消息接收任务: 轮询树莓派串口坐标数据, 通知 arm_ctrl 处理
 *
 * @param pvParameters Start parameters.
 */
void msg_receive(void *pvParameters) {
    UNUSED(pvParameters);

    raspi_serial_data_t command;
    static uint32_t last_sequence;
    static float last_x;

    while (1) {
        if (raspi_serial_get_latest(&command) &&
            command.sequence != last_sequence && command.x != last_x) {
            last_sequence = command.sequence;
            last_x = command.x;
            send_reply("OK\n");
            /* 通知 arm_ctrl 有新坐标 */
            xTaskNotifyGive(arm_ctrl_handle);
        }
        vTaskDelay(10);
    }
}

/**
 * @brief 机械臂控制任务: 等待 msg_receive 通知后执行两段绝对位置移动
 *        树莓派下发物理坐标（mm/°），直接转为脉冲后下发
 *
 * @param pvParameters Start parameters.
 */
void arm_ctrl(void *pvParameters) {
    UNUSED(pvParameters);

    raspi_serial_data_t command;
    static uint32_t last_sequence;
    bool first = true;
    int rounds = 0; /* 上位机传入的剩余轮次 */

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (raspi_serial_get_latest(&command) &&
            command.sequence != last_sequence) {
            last_sequence = command.sequence;

            /* 首次收到数据时从 command.a 读取总轮数 */
            if (first) {
                rounds = (int)command.a;
                first = false;
            }

            /* —— 第一段：绝对位置 (X, Y)，叠加相机偏移，R 轴不动 —— */
            uint32_t t1_x = ARM_X_MM_TO_PULSE(command.x + CAM_X_CORRECT);
            uint32_t t1_y = ARM_Y_MM_TO_PULSE(command.y + CAM_Y_CORRECT);

            arm_axis_move(1, t1_x, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            arm_axis_move(2, t1_y, 0);
            vTaskDelay(pdMS_TO_TICKS(20));

            arm_wait_axis_done(1, t1_x, 200, 5000);
            arm_wait_axis_done(2, t1_y, 200, 5000);

            /* —— 第二段：绝对位置 (X1, Y1)，R 轴相对移动，叠加相机偏移 —— */
            uint32_t t2_x = ARM_X_MM_TO_PULSE(command.x1 + CAM_X_CORRECT);
            uint32_t t2_y = ARM_Y_MM_TO_PULSE(command.y1 + CAM_Y_CORRECT);
            int32_t  t2_r = ARM_DEG_TO_PULSE_S(command.th);

            arm_axis_move(1, t2_x, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            arm_axis_move(2, t2_y, 0);
            vTaskDelay(pdMS_TO_TICKS(20));

            /* R 轴相对移动：先读取当前位置，计算预期绝对位置用于到位判断 */
            arm_update_position(3);
            uint32_t r_expected = (uint32_t)((int32_t)arm_get_position_pulse(3) + t2_r);
            arm_axis_rel_move(3, t2_r, 0);
            vTaskDelay(pdMS_TO_TICKS(20));

            arm_wait_axis_done(1, t2_x, 200, 5000);
            arm_wait_axis_done(2, t2_y, 200, 5000);
            arm_wait_axis_done(3, r_expected, 200, 5000);

            send_reply("OK\n");
            /* 本轮完成，轮次递减 */
            rounds--;
            if (rounds <= 0) {
                LED1_ON();
                first = true; /* 等待下一轮命令时重新读取 rounds */
            }
        }
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