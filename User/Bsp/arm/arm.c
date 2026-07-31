/**
 ****************************************************************************************************
 * @file    arm.c
 * @author  xinglu
 * @brief   三轴机械臂控制模块实现 (基于 Emm42 闭环步进电机)
 * @version 1.0
 * @date    2026-07-31
 ****************************************************************************************************
 */

#include "arm.h"

servo_t servo;

/* ========================== 电机实例 ========================== */

static emm42_motor_t arm_motor[3];

/* 轴编号 → 数组索引 */
#define ARM_AXIS_IDX(axis) ((axis) - 1)

/* ========================== API 实现 ========================== */

/**
 * @brief  初始化机械臂
 */
void arm_init(void) {
    for (int i = 0; i < 3; i++) {
        emm42_motor_init(&arm_motor[i], &huart4, i + 1, RS485_RE1_GPIO_Port,
                         RS485_RE1_Pin);
        emm42_en_control(&arm_motor[i], true, false);
    }
    servo_init(&servo, &htim1, TIM_CHANNEL_1, 1200, 1500);
}

/**
 * @brief  单轴绝对移动（相对坐标零点）
 */
void arm_axis_move(uint8_t axis, int32_t pulse, uint16_t speed) {
    if (axis < 1 || axis > 3) {
        return;
    }

    if (speed == 0) {
        speed = ARM_DEFAULT_SPEED;
    }

    uint8_t dir = (pulse >= 0) ? ARM_DIR_CW : ARM_DIR_CCW;
    uint32_t abs_pulse =
        (pulse >= 0) ? (uint32_t)pulse : (uint32_t)(-(int64_t)pulse);

    /* 绝对位置模式: mode=1 (相对于坐标零点) */
    emm42_pos_control_pulse(&arm_motor[ARM_AXIS_IDX(axis)], dir, speed,
                            ARM_DEFAULT_ACC, abs_pulse, 1, false);
}

/**
 * @brief  单轴相对移动
 */
void arm_axis_rel_move(uint8_t axis, int32_t pulse, uint16_t speed) {
    if (axis < 1 || axis > 3) {
        return;
    }

    if (speed == 0) {
        speed = ARM_DEFAULT_SPEED;
    }

    uint8_t dir = (pulse >= 0) ? ARM_DIR_CW : ARM_DIR_CCW;
    uint32_t abs_pulse =
        (pulse >= 0) ? (uint32_t)pulse : (uint32_t)(-(int64_t)pulse);

    /* 相对位置模式: mode=2 (相对于当前位置) */
    emm42_pos_control_pulse(&arm_motor[ARM_AXIS_IDX(axis)], dir, speed,
                            ARM_DEFAULT_ACC, abs_pulse, 2, false);
}

/**
 * @brief  估算单轴移动耗时
 */
uint32_t arm_est_move_ms(uint32_t pulse, uint16_t speed) {
    if (speed == 0) {
        speed = ARM_DEFAULT_SPEED;
    }

    /* 圈数 = pulse / 3200, 时间 = 圈数 / (RPM / 60) */
    float sec =
        (float)pulse / (float)ARM_PULSE_PER_REV / ((float)speed / 60.0f);

    return (uint32_t)(sec * 1000.0f) + 300;
}

/**
 * @brief  读取一条 RS485 应答帧并解析
 * @return true=成功解析一条
 */
static bool arm_read_response(emm42_motor_t *motor) {
    uint8_t buf[32];
    uint32_t len = uart_dmarx_read(motor->huart, buf, sizeof(buf));
    if (len > 0) {
        return emm42_frame_process(buf, len);
    }
    return false;
}

/**
 * @brief  查询并更新指定轴的实时位置
 */
void arm_update_position(uint8_t axis) {
    if (axis < 1 || axis > 3) {
        return;
    }

    /* 发送查询位置指令 */
    emm42_read_sys_params(&arm_motor[ARM_AXIS_IDX(axis)], EMM42_S_CPOS);
    /* 等待电机应答（RS485 半双工 turnaround + 传输时间） */
    vTaskDelay(pdMS_TO_TICKS(5));

    /* 排空应答帧，确保读到本次查询的回复 */
    for (int i = 0; i < 5; i++) {
        if (!arm_read_response(&arm_motor[ARM_AXIS_IDX(axis)])) {
            break;
        }
    }
}

/**
 * @brief  等待指定轴运动到位（阻塞式，主动查询位置）
 */
bool arm_wait_axis_done(uint8_t axis, int32_t target, uint32_t tolerance,
                        uint32_t timeout_ms) {
    if (axis < 1 || axis > 3) {
        return false;
    }

    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks =
        (timeout_ms > 0) ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY;

    while (1) {
        arm_update_position(axis);

        float cur_deg = arm_motor[ARM_AXIS_IDX(axis)].cur_pos;
        int32_t cur_pulse =
            (int32_t)(cur_deg / 360.0f * (float)ARM_PULSE_PER_REV);

        int64_t diff = (int64_t)cur_pulse - (int64_t)target;
        if (diff < 0) {
            diff = -diff;
        }
        if ((uint64_t)diff <= (uint64_t)tolerance) {
            return true;
        }

        /* 超时检测 */
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
void arm_emergency_stop(void) {
    for (int i = 0; i < 3; i++) {
        emm42_stop_now(&arm_motor[i], false);
    }
}

/**
 * @brief  使能/失能全部电机
 */
void arm_enable_all(bool en) {
    for (int i = 0; i < 3; i++) {
        emm42_en_control(&arm_motor[i], en, false);
    }
}

/**
 * @brief  读取指定轴当前位置（脉冲数）
 */
int32_t arm_get_position_pulse(uint8_t axis) {
    if (axis < 1 || axis > 3) {
        return 0;
    }
    float cur_deg = arm_motor[ARM_AXIS_IDX(axis)].cur_pos;
    return (int32_t)(cur_deg / 360.0f * (float)ARM_PULSE_PER_REV);
}

/**
 * @brief  全部轴当前位置清零
 */
void arm_set_zero(void) {
    for (int i = 0; i < 3; i++) {
        emm42_reset_curpos_to_zero(&arm_motor[i]);
    }
}
