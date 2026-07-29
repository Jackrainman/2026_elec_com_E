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

static TaskHandle_t task4_handle;
void task4(void *pvParameters);

static TaskHandle_t arm_test_handle;
void arm_test(void *pvParameters);

static TaskHandle_t msg_receive_handle;
void msg_receive(void *pvParameters);

static TaskHandle_t arm_ctrl_handle;
void arm_ctrl(void *pvParameters);

#define DEMO_MOTOR_SPEED_RPM  300U   /* 换向演示速度 (RPM) */
#define DEMO_MOTOR_TOGGLE_MS  2000U  /* 换向周期 (ms) */

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
    xTaskCreate(task4, "task4", 128, NULL, 2, &task4_handle);
    xTaskCreate(arm_test, "arm_test", 512, NULL, 3, &arm_test_handle);
    xTaskCreate(msg_receive, "msg_receive", 256, NULL, 3, &msg_receive_handle);
    xTaskCreate(arm_ctrl, "arm_ctrl", 512, NULL, 3, &arm_ctrl_handle);

    vTaskSuspend(task4_handle);
    //vTaskSuspend(arm_test_handle);
    vTaskSuspend(msg_receive_handle);
    vTaskSuspend(arm_ctrl_handle);

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
    LED1_ON();

    while (1) {
        LED0_TOGGLE();
        LED1_TOGGLE();
        vTaskDelay(1000);
    }
}

/**
 * @brief Task2: print running time and received data.
 *
 * @param pvParameters Start parameters.
 */
void task2(void *pvParameters) {
    UNUSED(pvParameters);

    uint8_t buf[20] = {0};

    while (1) {
        uint32_t len = uart_dmarx_read(&huart1, buf, sizeof(buf) - 1);
        if (len > 0) {
            buf[len] = '\0';
            uart_printf(&huart1, "Received: %s.\n", buf);
        } else {
            printf(
                "STM32F4xx FreeRTOS project template.Running time: %u ms. \n",
                xTaskGetTickCount());
        }
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

/**
 * @brief Arm test task: 按键控制机械臂动作
 *        X/Y/R 均使用相对移动，WKUP 回零用绝对位置
 *
 * KEY0: X/Y +5cm,  R +90°
 * KEY1: X/Y -3cm,  R -90°
 * KEY2: X/Y +10cm, R +270°
 * WKUP: X/Y 回零,  R -180°
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
                /* X/Y +5cm, R +90° */
                arm_axis_rel_move(1, (int32_t)ARM_X_MM_TO_PULSE(50), 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_rel_move(2, (int32_t)ARM_Y_MM_TO_PULSE(50), 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_rel_move(3, (int32_t)ARM_DEG_TO_PULSE(90), 100);
            } break;

            case KEY1_PRESS: {
                /* X/Y -3cm, R -90° */
                arm_axis_rel_move(1, -(int32_t)ARM_X_MM_TO_PULSE(30), 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_rel_move(2, -(int32_t)ARM_Y_MM_TO_PULSE(30), 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_rel_move(3, -(int32_t)ARM_DEG_TO_PULSE(90), 100);
            } break;

            case KEY2_PRESS: {
                /* X/Y +10cm, R +270° */
                arm_axis_rel_move(1, (int32_t)ARM_X_MM_TO_PULSE(100), 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_rel_move(2, (int32_t)ARM_Y_MM_TO_PULSE(100), 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_rel_move(3, (int32_t)ARM_DEG_TO_PULSE(270), 100);
            } break;

            case WKUP_PRESS: {
                /* X/Y 回零点, R -180° */
                arm_axis_move(1, 0, 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(2, 0, 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_rel_move(3, -(int32_t)ARM_DEG_TO_PULSE(180), 100);
            } break;

            default: break;
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

    while (1) {
        if (raspi_serial_get_latest(&command) &&
            command.sequence != last_sequence) {
            last_sequence = command.sequence;

            /* 通知 arm_ctrl 有新坐标 */
            xTaskNotifyGive(arm_ctrl_handle);
        }
        vTaskDelay(10);
    }
}

/**
 * @brief 机械臂控制任务: 等待 msg_receive 通知后执行三轴联动
 *        移动量 = 上次坐标 − 本次坐标，先读位置再发指令，用 arm_wait_axis_done 等到
 *
 * @param pvParameters Start parameters.
 */
void arm_ctrl(void *pvParameters) {
    UNUSED(pvParameters);

    raspi_serial_data_t command;
    raspi_serial_data_t last = {0};
    bool first = true;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (raspi_serial_get_latest(&command)) {
            if (first) {
                last = command;
                first = false;
                continue;
            }

            /* 增量 = 上次 − 本次 */
            int32_t dx  = ARM_X_MM_TO_PULSE_S(last.x  - command.x);
            int32_t dy  = ARM_Y_MM_TO_PULSE_S(last.y  - command.y);
            int32_t dr  = ARM_DEG_TO_PULSE_S(last.th - command.th);
            int32_t dx1 = ARM_X_MM_TO_PULSE_S(last.x1 - command.x1);
            int32_t dy1 = ARM_Y_MM_TO_PULSE_S(last.y1 - command.y1);
            int32_t da  = ARM_DEG_TO_PULSE_S(last.a  - command.a);

            last = command;

            /* —— 先读三轴当前位置（总线空闲时查询）—— */
            arm_update_position(1);
            vTaskDelay(pdMS_TO_TICKS(15));
            arm_update_position(2);
            vTaskDelay(pdMS_TO_TICKS(15));
            arm_update_position(3);
            vTaskDelay(pdMS_TO_TICKS(15));

            uint32_t cur1 = arm_get_position_pulse(1);
            uint32_t cur2 = arm_get_position_pulse(2);
            uint32_t cur3 = arm_get_position_pulse(3);

            /* 第一段目标 = 当前位置 + 增量 */
            uint32_t t1_x = (int32_t)cur1 + dx;
            uint32_t t1_y = (int32_t)cur2 + dy;
            uint32_t t1_r = (int32_t)cur3 + dr;

            /* —— 发送第一段相对移动 —— */
            arm_axis_rel_move(1, dx, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            arm_axis_rel_move(2, dy, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            arm_axis_rel_move(3, dr, 0);
            vTaskDelay(pdMS_TO_TICKS(20));

            /* 等到位 */
            arm_wait_axis_done(1, t1_x, 50, 5000);
            arm_wait_axis_done(2, t1_y, 50, 5000);
            arm_wait_axis_done(3, t1_r, 50, 5000);

            /* —— 再读位置，算第二段目标 —— */
            arm_update_position(1);
            vTaskDelay(pdMS_TO_TICKS(15));
            arm_update_position(2);
            vTaskDelay(pdMS_TO_TICKS(15));
            arm_update_position(3);
            vTaskDelay(pdMS_TO_TICKS(15));

            cur1 = arm_get_position_pulse(1);
            cur2 = arm_get_position_pulse(2);
            cur3 = arm_get_position_pulse(3);

            uint32_t t2_x = (int32_t)cur1 + dx1;
            uint32_t t2_y = (int32_t)cur2 + dy1;
            uint32_t t2_r = (int32_t)cur3 + da;

            /* —— 发送第二段相对移动 —— */
            arm_axis_rel_move(1, dx1, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            arm_axis_rel_move(2, dy1, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            arm_axis_rel_move(3, da, 0);
            vTaskDelay(pdMS_TO_TICKS(20));

            arm_wait_axis_done(1, t2_x, 50, 5000);
            arm_wait_axis_done(2, t2_y, 50, 5000);
            arm_wait_axis_done(3, t2_r, 50, 5000);
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
