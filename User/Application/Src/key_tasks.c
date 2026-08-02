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
 * KEY1: 跑一遍完整两段搬运流程 (取料 -> 放料, 坐标在 case 内硬编码)
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
                /* 流程坐标 (相机坐标系, mm/°), 现场按需直接改数值 */
                float pick_x = 200.0f, pick_y = 300.0f;     /* 第一段: 取料点 */
                float place_x = 100.0f, place_y = 100.0f; /* 第二段: 放料点 */
                float place_th = 0.0f; /* 第二段: R 轴角度 */

                LED1_ON(); /* 流程忙指示, 跑完熄灭 */

                /* —— 第一段: 绝对位置 (取料点), 叠加相机偏移, R 轴同时回零 —— */
                int32_t t1_x = ARM_X_MM_TO_PULSE_S(pick_x + CAM_X_CORRECT);
                int32_t t1_y = -ARM_Y_MM_TO_PULSE_S(pick_y + CAM_Y_CORRECT);

                arm_axis_move(1, t1_x, 100);
                vTaskDelay(pdMS_TO_TICKS(20));
                arm_axis_move(2, t1_y, 100);
                vTaskDelay(pdMS_TO_TICKS(20));
                arm_axis_move(3, 0, 100); /* R 轴绝对回零 */
                vTaskDelay(pdMS_TO_TICKS(20));

                arm_wait_axis_done(1, t1_x, 200, 2000);
                arm_wait_axis_done(2, t1_y, 200, 2000);
                arm_wait_axis_done(3, 0, 100, 2000);

                /* 取料点到位: 下探吸料后抬起 */
                SERVO_DOWN();
                vTaskDelay(pdMS_TO_TICKS(1000));
                MAGNET_ON();
                vTaskDelay(pdMS_TO_TICKS(1000));
                SERVO_UP();
                vTaskDelay(pdMS_TO_TICKS(2000));

                /* —— 第二段: 绝对位置 (放料点 + th), 叠加相机偏移 —— */
                int32_t t2_x = ARM_X_MM_TO_PULSE_S(place_x + CAM_X_CORRECT);
                int32_t t2_y = -ARM_Y_MM_TO_PULSE_S(place_y + CAM_Y_CORRECT);
                int32_t t2_r =
                    ARM_DEG_TO_PULSE_S(place_th); // 这里需要确认转向是否正确

                arm_axis_move(1, t2_x, 100);
                vTaskDelay(pdMS_TO_TICKS(20));
                arm_axis_move(2, t2_y, 100);
                vTaskDelay(pdMS_TO_TICKS(20));
                /* 第一段已保证 R 在原点, th 直接作绝对目标 */
                arm_axis_move(3, t2_r, 100);
                vTaskDelay(pdMS_TO_TICKS(20));

                arm_wait_axis_done(1, t2_x, 200, 2000);
                arm_wait_axis_done(2, t2_y, 200, 2000);
                arm_wait_axis_done(3, t2_r, 100, 2000);

                /* 放料点到位: 松开放料 */
                // MAGNET_OFF();
                servo_set_pulse_us(&servo, 1700);
                vTaskDelay(pdMS_TO_TICKS(1000));

                LED1_OFF();
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
