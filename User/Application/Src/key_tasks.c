/**
 * @file    key_tasks.c
 * @author  Deadline039
 * @brief   舵机测试任务: 用 KEY 测试舵机动作,
 *          不用时在 start_task 中注释掉 key_tasks_init() 即可.
 * @version 1.0
 * @date    2026-07-31
 */

#include "includes.h"

static TaskHandle_t servo_test_handle;
static void servo_test(void *pvParameters);

/*****************************************************************************/

/**
 * @brief 创建舵机测试相关任务.
 */
void key_tasks_init(void) {
    arm_init(); /* 初始化舵机 */

    xTaskCreate(servo_test, "servo_test", 256, NULL, 2, &servo_test_handle);
}

/**
 * @brief 舵机测试任务: 按键步进调节舵机脉冲
 *
 * KEY0: 脉冲 +200us (角度增大)
 * KEY1: 脉冲 -200us (角度减小)
 * KEY2: 脉冲回中位 1500us
 * WKUP: 气泵点动 (开 500ms 后关)
 *
 * @param pvParameters Start parameters.
 */
static void servo_test(void *pvParameters) {
    UNUSED(pvParameters);

    key_press_t key;
    int16_t pulse = 1500; /* 起始中位 */

    while (1) {
        key = key_scan(0);

        switch (key) {
            case KEY0_PRESS: {
                SERVO_UP();
            } break;

            case KEY1_PRESS: {
                SERVO_DOWN();
            } break;

            case KEY2_PRESS: {
                pulse = 1500;
                servo_set_pulse_us(&servo, (uint16_t)pulse);
            } break;

            case WKUP_PRESS: {
                MAGNET_ON();
                vTaskDelay(pdMS_TO_TICKS(2000));
                MAGNET_OFF();
            } break;

            default:
                break;
        }

        vTaskDelay(10);
    }
}
