/**
 * @file    recv_tasks.c
 * @author  Deadline039
 * @brief   上位机数据接收任务: 只有接树莓派串口指令时才需要,
 *          不用时在 start_task 中注释掉 recv_tasks_init() 即可.
 * @version 1.0
 * @date    2026-07-29
 */

#include "includes.h"

static TaskHandle_t msg_receive_handle;
static TaskHandle_t arm_ctrl_handle;

static void msg_receive(void *pvParameters);
static void arm_ctrl(void *pvParameters);

/*****************************************************************************/

/**
 * @brief 创建上位机数据接收相关任务.
 *
 * @note arm_init() 本组与 key 组各自调用.
 *       arm_ctrl 跑完全部轮次后会自挂起, 需要外部恢复才能继续.
 */
void recv_tasks_init(void) {
    raspi_serial_init(&huart1);
    arm_init(); /* arm_ctrl 依赖 */

    xTaskCreate(msg_receive, "msg_receive", 256, NULL, 3, &msg_receive_handle);
    xTaskCreate(arm_ctrl, "arm_ctrl", 512, NULL, 3, &arm_ctrl_handle);

    // vTaskSuspend(msg_receive_handle);
    // vTaskSuspend(arm_ctrl_handle);
}

/**
 * @brief 消息接收任务: 轮询树莓派串口坐标数据, 通知 arm_ctrl 处理
 *
 * @param pvParameters Start parameters.
 */
static void msg_receive(void *pvParameters) {
    UNUSED(pvParameters);

    raspi_serial_data_t command;
    static uint32_t last_sequence;
    static float last_x = 0;

    while (1) {
        if (raspi_serial_get_latest(&command) &&
            command.sequence != last_sequence && command.x != last_x) {
            last_sequence = command.sequence;
            last_x = command.x;
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
static void arm_ctrl(void *pvParameters) {
    UNUSED(pvParameters);

    raspi_serial_data_t command;
    bool first = true;
    int rounds = 0; /* 上位机传入的剩余轮次 */

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (raspi_serial_get_latest(&command)) {
            /* 首次收到数据时从 command.a 读取总轮数 */
            if (first) {
                rounds = (int)command.a;
                first = false;
            }

            /* —— 第一段：绝对位置 (X, Y)，叠加相机偏移，R 轴同时回原点 —— */
            int32_t t1_x = ARM_X_MM_TO_PULSE_S(command.x + CAM_X_CORRECT);
            int32_t t1_y = ARM_Y_MM_TO_PULSE_S(command.y + CAM_Y_CORRECT);

            arm_axis_move(1, t1_x, 100);
            vTaskDelay(pdMS_TO_TICKS(20));
            arm_axis_move(2, t1_y, 100);
            vTaskDelay(pdMS_TO_TICKS(20));
            arm_axis_move(
                3, 0, 100); /* R 轴绝对回零（移植自 main 的 last.th 逻辑） */
            vTaskDelay(pdMS_TO_TICKS(20));

            arm_wait_axis_done(1, t1_x, 100, 5000);
            arm_wait_axis_done(2, t1_y, 100, 5000);
            arm_wait_axis_done(3, 0, 100, 5000);

            /* 第一个坐标到位 → 开气泵，1s 后关 */
            SERVO_DOWN();
            {
                TickType_t xPumpTick = xTaskGetTickCount();
                while ((xTaskGetTickCount() - xPumpTick) <
                       pdMS_TO_TICKS(1000)) {
                    vTaskDelay(1);
                }
            }
            MAGNET_ON();
            {
                TickType_t xPumpTick = xTaskGetTickCount();
                while ((xTaskGetTickCount() - xPumpTick) <
                       pdMS_TO_TICKS(1000)) {
                    vTaskDelay(1);
                }
            }
            SERVO_UP();

            /* —— 第二段：绝对位置 (X1, Y1, th)，叠加相机偏移 —— */
            int32_t t2_x = ARM_X_MM_TO_PULSE_S(command.x1 + CAM_X_CORRECT);
            int32_t t2_y = ARM_Y_MM_TO_PULSE_S(command.y1 + CAM_Y_CORRECT);
            int32_t t2_r =
                ARM_DEG_TO_PULSE_S(command.th); // 这里需要确认转向是否正确

            arm_axis_move(1, t2_x, 100);
            vTaskDelay(pdMS_TO_TICKS(20));
            arm_axis_move(2, t2_y, 100);
            vTaskDelay(pdMS_TO_TICKS(20));
            /* 第一段已保证 R 在原点，th 直接作绝对目标 */
            arm_axis_move(3, t2_r, 100);
            vTaskDelay(pdMS_TO_TICKS(20));

            arm_wait_axis_done(1, t2_x, 100, 5000);
            arm_wait_axis_done(2, t2_y, 100, 5000);
            arm_wait_axis_done(3, t2_r, 100, 5000);

            /* 第二个坐标到位 → 开气泵，1s 后关 */
            servo_set_pulse_us(&servo, 1350);
            {
                TickType_t xPumpTick = xTaskGetTickCount();
                while ((xTaskGetTickCount() - xPumpTick) <
                       pdMS_TO_TICKS(1000)) {
                    vTaskDelay(1);
                }
            }
            MAGNET_OFF();
            {
                TickType_t xPumpTick = xTaskGetTickCount();
                while ((xTaskGetTickCount() - xPumpTick) <
                       pdMS_TO_TICKS(1000)) {
                    vTaskDelay(1);
                }
            }
            SERVO_UP();

            /* 本轮完成，轮次递减；全部完成后亮灯挂起 */
            send_reply("ok");
            rounds--;
            if (rounds <= 0) {
                LED1_ON();
                /* 回原点 */
                // arm_axis_move(1, 0, 100);
                // vTaskDelay(pdMS_TO_TICKS(20));
                // arm_axis_move(2, t1_x, 100);
                // vTaskDelay(pdMS_TO_TICKS(20));
                // arm_axis_move(3, 0, 100);
                // vTaskDelay(pdMS_TO_TICKS(1000));
                vTaskSuspend(NULL);
            }
        }
    }
}
