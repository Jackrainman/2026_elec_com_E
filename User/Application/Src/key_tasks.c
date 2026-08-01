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
 * @note arm_init() 重复调用无害(重新使能电机并清零位置), 因此本组与 recv 组各自调用.
 */
void key_tasks_init(void) {
    arm_init();

    xTaskCreate(arm_test, "arm_test", 512, NULL, 3, &arm_test_handle);
}

/**
 * @brief Arm test task: 按键控制机械臂绝对运动
 *        各键对应固定的绝对目标位置，叠加相机坐标系偏移
 *
 * 实际动作 (以代码为准, 现场按需直接改数值):
 * KEY0: X=100mm,  Y=50mm,   R=0°
 * KEY1: X=-30mm,  Y=-30mm,  R=-90°
 * KEY2: X=100mm,  Y=100mm,  R=270°
 * WKUP: X=0mm,    Y=0mm,    R=-180°  (回零，叠加偏移)
 *
 * LED2 亮 = 电机离线 (arm_init 同步失败)
 *
 * @param pvParameters Start parameters.
 */
static void arm_test(void *pvParameters) {
    UNUSED(pvParameters);

    key_press_t key;

    if (!arm_is_ready()) {
        LED2_ON(); /* 有电机离线, 亮灯提示 */
    }

    while (1) {
        key = key_scan(0);
        switch (key) {
            case KEY0_PRESS: {
                /* 绝对位置: X=100mm, Y=50mm, R=0° */
                int32_t tx = ARM_X_MM_TO_PULSE_S(50.0f + CAM_X_CORRECT);
                int32_t ty = ARM_Y_MM_TO_PULSE_S(82.0f + CAM_Y_CORRECT);
                int32_t tr = ARM_DEG_TO_PULSE_S(0.0f);

                arm_axis_move(1, tx, 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(2, ty, 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(3, tr, 100);
            } break;

            case KEY1_PRESS: {
                /* 绝对位置: X=-30mm, Y=-30mm, R=-90° */
                int32_t tx = ARM_X_MM_TO_PULSE_S(0.0f + CAM_X_CORRECT);
                int32_t ty = ARM_Y_MM_TO_PULSE_S(0.0f + CAM_Y_CORRECT);
                int32_t tr = ARM_DEG_TO_PULSE_S(-90.0f);

                arm_axis_move(1, tx, 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(2, ty, 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(3, tr, 100);
            } break;

            case KEY2_PRESS: {
                /* 绝对位置: X=100mm, Y=100mm, R=270° */
                int32_t tx = ARM_X_MM_TO_PULSE_S(100.0f + CAM_X_CORRECT);
                int32_t ty = ARM_Y_MM_TO_PULSE_S(100.0f + CAM_Y_CORRECT);
                int32_t tr = ARM_DEG_TO_PULSE_S(270.0f);

                arm_axis_move(1, tx, 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(2, ty, 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(3, tr, 100);
            } break;

            case WKUP_PRESS: {
                /* 回零: X=0mm, Y=0mm, R=-180°（叠加偏移量） */
                int32_t tx = ARM_X_MM_TO_PULSE_S(0.0f + CAM_X_CORRECT);
                int32_t ty = ARM_Y_MM_TO_PULSE_S(50.0f + CAM_Y_CORRECT);
                int32_t tr = ARM_DEG_TO_PULSE_S(-180.0f);

                arm_axis_move(1, tx, 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(2, ty, 100);
                vTaskDelay(pdMS_TO_TICKS(10));
                arm_axis_move(3, tr, 100);
            } break;

            default:
                break;
        }

        vTaskDelay(10);
    }
}
