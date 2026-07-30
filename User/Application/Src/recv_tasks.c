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
