/**
 ****************************************************************************************************
 * @file        arm.c
 * @brief       三轴机械臂控制模块实现 (基于正点原子 SMD 步进电机)
 ****************************************************************************************************
 */

#include "arm.h"
#include "../atk_smd/smd.h"
#include "FreeRTOS.h"
#include "task.h"

/* ========================== 电机实例 ========================== */

static smd_motor_t arm_motor[3];

/* 轴编号 → 数组索引 */
#define ARM_AXIS_IDX(axis)  ((axis) - 1)

/* 轴编号 → 电机地址 */
static const uint8_t arm_axis_addr[3] = {
    ARM_MOTOR_X_ADDR,
    ARM_MOTOR_Y_ADDR,
    ARM_MOTOR_R_ADDR,
};

/* ========================== API 实现 ========================== */

/**
 * @brief  初始化机械臂
 */
void arm_init(void)
{
    for (int i = 0; i < 3; i++) {
        smd_motor_init(&arm_motor[i], arm_axis_addr[i]);
        smd_set_mode(arm_axis_addr[i], 0);
        smd_motor_enable(arm_axis_addr[i], 0);
    }
}

/**
 * @brief  单轴绝对移动（相对坐标零点）
 */
void arm_axis_move(uint8_t axis, uint32_t pulse, uint16_t speed)
{
    if (axis < 1 || axis > 3) return;

    if (speed == 0) speed = ARM_DEFAULT_SPEED;

    smd_pos_mode(arm_axis_addr[ARM_AXIS_IDX(axis)], ARM_DIR_CW,
                 ARM_DEFAULT_ACC, speed, pulse);
}

/**
 * @brief  单轴相对移动
 */
void arm_axis_rel_move(uint8_t axis, int32_t pulse, uint16_t speed)
{
    if (axis < 1 || axis > 3) return;

    if (speed == 0) speed = ARM_DEFAULT_SPEED;

    uint8_t  dir      = (pulse >= 0) ? ARM_DIR_CW : ARM_DIR_CCW;
    uint32_t abs_pulse = (pulse >= 0) ? (uint32_t)pulse : (uint32_t)(-pulse);

    smd_pos_rel_mode(arm_axis_addr[ARM_AXIS_IDX(axis)], dir,
                     ARM_DEFAULT_ACC, speed, abs_pulse);
}

/**
 * @brief  估算单轴移动耗时
 */
uint32_t arm_est_move_ms(uint32_t pulse, uint16_t speed)
{
    if (speed == 0) speed = ARM_DEFAULT_SPEED;

    float sec = (float)pulse / (float)ARM_PULSE_PER_REV / ((float)speed / 60.0f);

    return (uint32_t)(sec * 1000.0f) + 300;
}

/**
 * @brief  查询并更新指定轴的实时位置
 */
void arm_update_position(uint8_t axis)
{
    if (axis < 1 || axis > 3) return;

    smd_read_pos(arm_axis_addr[ARM_AXIS_IDX(axis)]);
    vTaskDelay(pdMS_TO_TICKS(5));
}

/**
 * @brief  等待指定轴运动到位（阻塞式，主动查询位置）
 */
bool arm_wait_axis_done(uint8_t axis, uint32_t target, uint32_t tolerance,
                        uint32_t timeout_ms)
{
    if (axis < 1 || axis > 3) return false;

    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = (timeout_ms > 0) ? pdMS_TO_TICKS(timeout_ms)
                                                : portMAX_DELAY;

    while (1) {
        arm_update_position(axis);

        int32_t cur  = (int32_t)arm_motor[ARM_AXIS_IDX(axis)].real_pos;
        int32_t diff = (int32_t)cur - (int32_t)target;
        if (diff < 0) diff = -diff;
        if ((uint32_t)diff <= tolerance) return true;

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
        smd_stop_now(arm_axis_addr[i]);
    }
}

/**
 * @brief  使能/失能全部电机
 */
void arm_enable_all(bool en)
{
    for (int i = 0; i < 3; i++) {
        smd_motor_enable(arm_axis_addr[i], en ? 0 : 1);
    }
}

/**
 * @brief  使能/失能指定轴
 */
void arm_enable_axis(uint8_t axis, bool en)
{
    if (axis < 1 || axis > 3) return;
    smd_motor_enable(arm_axis_addr[ARM_AXIS_IDX(axis)], en ? 0 : 1);
}

/**
 * @brief  读取指定轴当前位置（脉冲数）
 */
uint32_t arm_get_position_pulse(uint8_t axis)
{
    if (axis < 1 || axis > 3) return 0;
    return (uint32_t)arm_motor[ARM_AXIS_IDX(axis)].real_pos;
}

/**
 * @brief  全部轴当前位置清零
 */
void arm_set_zero(void)
{
    for (int i = 0; i < 3; i++) {
        smd_angle_to_zero(arm_axis_addr[i]);
    }
}
