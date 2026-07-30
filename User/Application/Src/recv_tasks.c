/**
 * @file    recv_tasks.c
 * @author  Deadline039
 * @brief   上位机数据接收任务: 只有接树莓派串口指令时才需要,
 *          不用时在 start_task 中注释掉 recv_tasks_init() 即可.
 * @version 1.0
 * @date    2026-07-29
 */

#include "includes.h"

#define CAM_X_CORRECT   40.0f
#define CAM_Y_CORRECT   17.0f

static TaskHandle_t msg_receive_handle;
static TaskHandle_t arm_ctrl_handle;

static void msg_receive(void *pvParameters);
static void arm_ctrl(void *pvParameters);

/*****************************************************************************/

/**
 * @brief 创建上位机数据接收相关任务.
 *
 * @note arm_init() 重复调用无害(重新使能电机), 因此本组与 key 组各自调用.
 *       默认只跑 task2(收到指令回 OK, 演示/调试用);
 *       正式使用 msg_receive + arm_ctrl 联动流程时,
 *       请挂起 task2 并恢复下面两个任务, 避免重复应答.
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
            command.sequence != last_sequence &&
            command.x != last_x) {
            last_sequence = command.sequence;
            last_x = command.x;
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
static void arm_ctrl(void *pvParameters) {
    UNUSED(pvParameters);

    raspi_serial_data_t command;
    raspi_serial_data_t last = {0};
    bool first = true;
    bool first_move = true;
    int rounds = 0;  /* 上位机传入的轮次 */

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (raspi_serial_get_latest(&command)) {
            /* 首次收到数据时从 command.a 读取总轮数 */
            if (first) {
                rounds = (int)command.a;
                first = false;
            }

            if (first_move) {
                last.x1 = CAM_X_CORRECT;
                last.y1 = CAM_Y_CORRECT;
                first_move = false;
            }

            /* 第一段增量 = 本次 − 上次 */
            int32_t dx  = ARM_X_MM_TO_PULSE_S(command.x - last.x1);
            int32_t dy  = ARM_Y_MM_TO_PULSE_S(command.y - last.y1);
            int32_t dr  = ARM_DEG_TO_PULSE_S(command.th);

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

            /* —— 发送第一段相对移动（仅 XY）—— */
            arm_axis_rel_move(1, dx, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            arm_axis_rel_move(2, dy, 0);
            vTaskDelay(pdMS_TO_TICKS(20));

            /* 等到位 */
            arm_wait_axis_done(1, t1_x, 50, 5000);
            arm_wait_axis_done(2, t1_y, 50, 5000);

            /* 第一个坐标到位 → 开气泵，1s 后关 */
            PUMP_ON();
            {
                TickType_t xPumpTick = xTaskGetTickCount();
                while ((xTaskGetTickCount() - xPumpTick) < pdMS_TO_TICKS(2000)) {
                    vTaskDelay(1);
                }
            }
            PUMP_OFF();

            /* —— 再读位置，算第二段目标（XY + R）—— */
            arm_update_position(1);
            vTaskDelay(pdMS_TO_TICKS(15));
            arm_update_position(2);
            vTaskDelay(pdMS_TO_TICKS(15));
            arm_update_position(3);
            vTaskDelay(pdMS_TO_TICKS(15));

            cur1 = arm_get_position_pulse(1);
            cur2 = arm_get_position_pulse(2);
            cur3 = arm_get_position_pulse(3);

            /* 第二段增量 = x1−x, y1−y */
            int32_t dx1 = ARM_X_MM_TO_PULSE_S(command.x1 - command.x);
            int32_t dy1 = ARM_Y_MM_TO_PULSE_S(command.y1 - command.y);

            uint32_t t2_x = (int32_t)cur1 + dx1;
            uint32_t t2_y = (int32_t)cur2 + dy1;
            uint32_t t2_r = (int32_t)cur3 + dr;

            /* —— 发送第二段相对移动（XY + R）—— */
            arm_axis_rel_move(1, dx1, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            arm_axis_rel_move(2, dy1, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            arm_axis_rel_move(3, dr, 0);
            vTaskDelay(pdMS_TO_TICKS(20));

            arm_wait_axis_done(1, t2_x, 50, 5000);
            arm_wait_axis_done(2, t2_y, 50, 5000);
            arm_wait_axis_done(3, t2_r, 50, 5000);

            /* 第二个坐标到位 → 开气泵，1s 后关 */
            PUMP_ON();
            {
                TickType_t xPumpTick = xTaskGetTickCount();
                while ((xTaskGetTickCount() - xPumpTick) < pdMS_TO_TICKS(2000)) {
                    vTaskDelay(1);
                }
            }
            PUMP_OFF();

            /* 本轮完成，轮次递减；全部完成后亮灯挂起 */
            send_reply("ok");
            rounds--;
            if (rounds <= 0) {
                LED1_ON();
                vTaskSuspend(NULL);
            }
        }
    }
}
