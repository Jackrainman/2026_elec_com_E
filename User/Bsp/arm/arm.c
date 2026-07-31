/**
 ****************************************************************************************************
 * @file    arm.c
 * @author  xinglu
 * @brief   三轴机械臂控制模块实现 (基于正点原子SMD步进电机)
 * @version 1.0
 * @date    2026-07-31
 ****************************************************************************************************
 */

#include "arm.h"

/* ========================== 舵机实例 ========================== */
servo_t servo;

/* ========================== 电机实例 ========================== */

static smd_motor_t arm_motor[3];

/* 各轴通信同步状态：true=初始化时该轴读回位置成功（电机在线） */
static volatile bool arm_axis_ready[3] = {false, false, false};

/* 轴编号 → 数组索引 */
#define ARM_AXIS_IDX(axis)  ((axis) - 1)

/* 轴编号 → 电机地址 */
static const uint8_t arm_axis_addr[3] = {
    ARM_MOTOR_X_ADDR,
    ARM_MOTOR_Y_ADDR,
    ARM_MOTOR_R_ADDR,
};

/* ========================== 内部函数 ========================== */

/**
 * @brief  读回指定轴位置并等待应答，确认电机通信在线
 * @note   模仿 2026R1 lift_wait_smd_sync(): 清 valid_mask -> 发读位置命令 ->
 *         超时轮询 SMD_MASK_REAL_POS 置位，失败重试
 * @param  axis  轴编号 (1~3)
 * @return true=同步成功(电机在线), false=超时(离线或无应答)
 */
static bool arm_sync_axis(uint8_t axis)
{
    smd_motor_t *motor = &arm_motor[ARM_AXIS_IDX(axis)];

    for (int retry = 0; retry < ARM_SYNC_RETRY_TIMES; retry++) {
        motor->valid_mask = 0;
        smd_read_pos(arm_axis_addr[ARM_AXIS_IDX(axis)]);

        TickType_t start = xTaskGetTickCount();
        while ((motor->valid_mask & SMD_MASK_REAL_POS) == 0U) {
            if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(ARM_SYNC_TIMEOUT_MS)) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if ((motor->valid_mask & SMD_MASK_REAL_POS) != 0U) {
            return true;
        }
    }

    return false;
}

/* ========================== API 实现 ========================== */

/**
 * @brief  初始化机械臂
 * @note   上电等待 1s 后，将三个电机当前位置设为零点；
 *         随后读回各轴位置确认电机在线（用 arm_is_ready() 查询结果）
 */
void arm_init(void)
{
    for (int i = 0; i < 3; i++) {
        if (smd_motor_init(&arm_motor[i], arm_axis_addr[i]) != 0) {
            continue; /* 软件资源初始化失败，arm_axis_ready 保持 false */
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
<<<<<<< HEAD
    servo_init(&servo, &htim1, TIM_CHANNEL_1, 2500, 500);
=======
>>>>>>> 5ab86b9f1c292ba4bdf24f500d0b2d5ef43d284c

    /* 等待电机上电稳定后，将当前位置清零作为坐标零点 */
    vTaskDelay(pdMS_TO_TICKS(1000));
    arm_set_zero();

    /* 模仿 2026R1：读回各轴位置并校验应答，确认电机在线 */
    for (int i = 0; i < 3; i++) {
        arm_axis_ready[i] = arm_sync_axis(i + 1);
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
        vTaskDelay(20);
    }
}

/**
 * @brief  查询机械臂是否初始化成功
 * @return true=三轴均通信同步成功, false=存在离线轴
 */
bool arm_is_ready(void)
{
    return arm_axis_ready[0] && arm_axis_ready[1] && arm_axis_ready[2];
}

/**
 * @brief  查询指定轴是否初始化成功（通信同步成功）
 * @param  axis  轴编号
 * @return true=该轴在线, false=离线或无应答
 */
bool arm_axis_is_ready(uint8_t axis)
{
    if (axis < 1 || axis > 3) return false;
    return arm_axis_ready[ARM_AXIS_IDX(axis)];
}
