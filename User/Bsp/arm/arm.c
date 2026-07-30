/**
 ****************************************************************************************************
 * @file        arm.c
 * @brief       三轴机械臂控制模块实现 (基于 Emm42 CAN 闭环步进电机)
 ****************************************************************************************************
 */

#include "arm.h"
#include "emm42_can/emm42_can.h"
#include "FreeRTOS.h"
#include "task.h"

/* ========================== 电机实例 ========================== */

static emm42_can_motor_t arm_motor[3];

/* 轴编号 → 数组索引 */
#define ARM_AXIS_IDX(axis)  ((axis) - 1)

/* ========================== API 实现 ========================== */

/**
 * @brief  初始化机械臂
 */
void arm_init(void)
{
    for (int i = 0; i < 3; i++) {
        uint8_t addr = (i == 0) ? ARM_MOTOR_X_ADDR :
                       (i == 1) ? ARM_MOTOR_Y_ADDR : ARM_MOTOR_R_ADDR;
        emm42_can_motor_init(&arm_motor[i], can1_selected, addr);
        emm42_can_en_control(&arm_motor[i], true, false);
    }
}

/**
 * @brief  单轴绝对移动（相对坐标零点）
 */
void arm_axis_move(uint8_t axis, uint32_t pulse, uint16_t speed)
{
    if (axis < 1 || axis > 3) return;

    if (speed == 0) speed = ARM_DEFAULT_SPEED;

    /* 绝对位置模式: mode=1 (相对坐标零点) */
    emm42_can_pos_control_pulse(&arm_motor[ARM_AXIS_IDX(axis)], ARM_DIR_CW,
                                speed, ARM_DEFAULT_ACC, pulse, 1, false);
}

/**
 * @brief  单轴相对移动
 */
void arm_axis_rel_move(uint8_t axis, int32_t pulse, uint16_t speed)
{
    if (axis < 1 || axis > 3) return;

    if (speed == 0) speed = ARM_DEFAULT_SPEED;

    uint8_t dir = (pulse >= 0) ? ARM_DIR_CW : ARM_DIR_CCW;
    uint32_t abs_pulse = (pulse >= 0) ? (uint32_t)pulse : (uint32_t)(-pulse);

    /* 相对位置模式: mode=2 (相对当前位置) */
    emm42_can_pos_control_pulse(&arm_motor[ARM_AXIS_IDX(axis)], dir, speed,
                                ARM_DEFAULT_ACC, abs_pulse, 2, false);
}

/**
 * @brief  估算单轴移动耗时
 */
uint32_t arm_est_move_ms(uint32_t pulse, uint16_t speed)
{
    if (speed == 0) speed = ARM_DEFAULT_SPEED;

    /* 圈数 = pulse / 3200, 时间 = 圈数 / (RPM / 60) */
    float sec = (float)pulse / (float)ARM_PULSE_PER_REV / ((float)speed / 60.0f);

    return (uint32_t)(sec * 1000.0f) + 300;
}

/**
 * @brief  查询并更新指定轴的实时位置
 */
void arm_update_position(uint8_t axis)
{
    if (axis < 1 || axis > 3) return;

    /* 发送读位置指令 */
    emm42_can_read_sys_params(&arm_motor[ARM_AXIS_IDX(axis)], EMM42_CAN_S_CPOS);
    /* 等待电机应答（CAN 传输与回调处理时间） */
    vTaskDelay(pdMS_TO_TICKS(5));
}

/**
 * @brief  等待指定轴运动到位（阻塞式，主动查询位置）
 */
bool arm_wait_axis_done(uint8_t axis, uint32_t target, uint32_t tolerance,
                        uint32_t timeout_ms)
{
    if (axis < 1 || axis > 3) return false;

    /* cur_pos 存的是角度, 转到脉冲域比较: pulse = degree / 360 * 3200 */
    uint32_t target_pulse = target;
    uint32_t tol_pulse    = tolerance;

    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = (timeout_ms > 0) ? pdMS_TO_TICKS(timeout_ms)
                                                : portMAX_DELAY;

    while (1) {
        arm_update_position(axis);

        float cur_deg = arm_motor[ARM_AXIS_IDX(axis)].cur_pos;
        uint32_t cur_pulse = (uint32_t)(cur_deg / 360.0f * (float)ARM_PULSE_PER_REV);

        int32_t diff = (int32_t)cur_pulse - (int32_t)target_pulse;
        if (diff < 0) diff = -diff;
        if ((uint32_t)diff <= tol_pulse) return true;

        /* 超时检查 */
        if (timeout_ms > 0) {
            if ((xTaskGetTickCount() - start) >= timeout_ticks) {
                return false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief  全轴急停
 */
void arm_emergency_stop(void)
{
    for (int i = 0; i < 3; i++) {
        emm42_can_stop_now(&arm_motor[i], false);
    }
}

/**
 * @brief  使能/失能全部电机
 */
void arm_enable_all(bool en)
{
    for (int i = 0; i < 3; i++) {
        emm42_can_en_control(&arm_motor[i], en, false);
    }
}

/**
 * @brief  读取指定轴当前位置（脉冲数）
 */
uint32_t arm_get_position_pulse(uint8_t axis)
{
    if (axis < 1 || axis > 3) return 0;
    float cur_deg = arm_motor[ARM_AXIS_IDX(axis)].cur_pos;
    return (uint32_t)(cur_deg / 360.0f * (float)ARM_PULSE_PER_REV);
}

/**
 * @brief  全部轴当前位置清零
 */
void arm_set_zero(void)
{
    for (int i = 0; i < 3; i++) {
        emm42_can_reset_curpos_to_zero(&arm_motor[i]);
    }
}
