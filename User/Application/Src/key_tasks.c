/**
 * @file    key_tasks.c
 * @author  Deadline039
 * @brief   按键调试任务: 只有用 KEY 手动调试机械臂时才需要,
 *          不用时在 start_task 中注释掉 key_tasks_init() 即可.
 * @version 1.0
 * @date    2026-07-29
 */

#include "includes.h"

static TaskHandle_t arm_test_handle;
static void arm_test(void *pvParameters);

/*****************************************************************************/

/**
 * @brief 创建按键调试相关任务.
 *
 * @note arm_init() 重复调用无害(重新使能电机), 因此本组与 recv 组各自调用.
 */
void key_tasks_init(void) {
    arm_init();

    xTaskCreate(arm_test, "arm_test", 512, NULL, 3, &arm_test_handle);
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
static void arm_test(void *pvParameters) {
    UNUSED(pvParameters);

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

            default:
                break;
        }

        vTaskDelay(10);
    }
}
