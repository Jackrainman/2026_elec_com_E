/**
 ****************************************************************************************************
 * @file        arm.c
 * @brief       四电机滑轨机械臂控制模块实现
 * @note        依赖 smd 步进电机驱动层
 ****************************************************************************************************
 */

#include "arm.h"
#include "../atk_smd/smd.h"

/* ========================== 电机实例 ========================== */

static smd_motor_t arm_motor_x;
static smd_motor_t arm_motor_y;
static smd_motor_t arm_motor_r;

/* 电机实例指针数组，按轴编号索引 */
static smd_motor_t *const arm_motor_list[3] = {
    &arm_motor_x,
    &arm_motor_y,
    &arm_motor_r,
};

/* ========================== 内部辅助 ========================== */

/* 轴编号 → 电机地址映射 */
static const uint8_t arm_axis_addr[3] = {
    ARM_MOTOR_X_ADDR,
    ARM_MOTOR_Y_ADDR,
    ARM_MOTOR_R_ADDR,
};

/* 各轴方向：0=正向与坐标正方向一致, 1=反向 */
/* 用户根据实际安装方向修改 */
static const uint8_t arm_axis_positive_dir[3] = {
    ARM_DIR_CW,   /* X轴正方向 */
    ARM_DIR_CW,   /* Y轴正方向 */
    ARM_DIR_CW,   /* R轴正方向 */
};

/**
 * @brief  将物理量转为脉冲数
 */
static int32_t arm_pos_to_pulse(uint8_t axis, float pos)
{
    float pulse_per_unit;

    switch (axis) {
        case 1: pulse_per_unit = ARM_X_PULSE_PER_MM; break;
        case 2: pulse_per_unit = ARM_Y_PULSE_PER_MM; break;
        case 3: pulse_per_unit = ARM_R_PULSE_PER_DEG; break;
        default: return 0;
    }

    return (int32_t)(pos * pulse_per_unit);
}

/**
 * @brief  发送单轴绝对位置指令
 * @param  axis   轴编号 0~3
 * @param  pos    物理位置（mm或°）
 * @param  speed  RPM，0则用默认值
 * @param  dir    方向（ARM_DIR_CW / ARM_DIR_CCW）
 */
static void arm_axis_pos_cmd(uint8_t axis, float pos, uint16_t speed, uint8_t dir)
{
    uint8_t addr = arm_axis_addr[axis - 1];
    int32_t pulses = arm_pos_to_pulse(axis, pos);

    if (speed == 0) {
        speed = ARM_DEFAULT_SPEED;
    }

    smd_pos_mode(addr, dir, ARM_DEFAULT_ACC, speed, (uint32_t)pulses);
}

/**
 * @brief  发送单轴相对位置指令
 */
static void arm_axis_rel_cmd(uint8_t axis, float delta, uint16_t speed)
{
    uint8_t addr = arm_axis_addr[axis - 1];
    uint8_t dir = (delta >= 0) ? arm_axis_positive_dir[axis - 1]
                               : (uint8_t)(!arm_axis_positive_dir[axis - 1]);
    int32_t pulses = arm_pos_to_pulse(axis, (delta >= 0) ? delta : -delta);

    if (speed == 0) {
        speed = ARM_DEFAULT_SPEED;
    }

    smd_pos_rel_mode(addr, dir, ARM_DEFAULT_ACC, speed, (uint32_t)pulses);
}

/* ========================== API 实现 ========================== */

/**
 * @brief  初始化机械臂
 */
void arm_init(void)
{
    /* 注册电机到SMD驱动 */
    smd_motor_init(&arm_motor_x, ARM_MOTOR_X_ADDR);
    smd_motor_init(&arm_motor_y, ARM_MOTOR_Y_ADDR);
    smd_motor_init(&arm_motor_r, ARM_MOTOR_R_ADDR);

    /* 设置全部电机为通信位置模式（模式0） */
    smd_set_mode(ARM_MOTOR_X_ADDR, 0);
    smd_set_mode(ARM_MOTOR_Y_ADDR, 0);
    smd_set_mode(ARM_MOTOR_R_ADDR, 0);

    /* 使能全部电机 */
    smd_motor_enable(ARM_MOTOR_X_ADDR, 0);
    smd_motor_enable(ARM_MOTOR_Y_ADDR, 0);
    smd_motor_enable(ARM_MOTOR_R_ADDR, 0);
}

/**
 * @brief  XY两轴移动到绝对位置
 */
void arm_move_to(float x_mm, float y_mm, uint16_t speed)
{
    uint8_t addr_x = arm_axis_addr[0];
    uint8_t addr_y = arm_axis_addr[1];

    if (speed == 0) {
        speed = ARM_DEFAULT_SPEED;
    }

    /* 两轴同时发送指令（电机自行执行，无需等待） */
    smd_pos_mode(addr_x, arm_axis_positive_dir[0], ARM_DEFAULT_ACC, speed,
                 (uint32_t)arm_pos_to_pulse(1, x_mm));
    smd_pos_mode(addr_y, arm_axis_positive_dir[1], ARM_DEFAULT_ACC, speed,
                 (uint32_t)arm_pos_to_pulse(2, y_mm));
}

/**
 * @brief  XYR三轴移动到绝对位置
 */
void arm_move_to_all(float x_mm, float y_mm, float r_deg, uint16_t speed)
{
    arm_move_to(x_mm, y_mm, speed);

    smd_pos_mode(arm_axis_addr[2], arm_axis_positive_dir[2], ARM_DEFAULT_ACC,
                 (speed == 0) ? ARM_DEFAULT_SPEED : speed,
                 (uint32_t)arm_pos_to_pulse(3, r_deg));
}

/**
 * @brief  单轴绝对移动
 */
void arm_axis_move(uint8_t axis, float pos, uint16_t speed)
{
    if (axis < 1 || axis > 3) return;
    arm_axis_pos_cmd(axis, pos, speed, arm_axis_positive_dir[axis - 1]);
}

/**
 * @brief  单轴相对移动
 */
void arm_axis_rel_move(uint8_t axis, float delta, uint16_t speed)
{
    if (axis < 1 || axis > 3) return;
    arm_axis_rel_cmd(axis, delta, speed);
}

/**
 * @brief  R轴自转（相对角度）
 */
void arm_rotate(float deg, uint16_t speed)
{
    arm_axis_rel_cmd(3, deg, speed);
}

/**
 * @brief  全轴急停
 */
void arm_emergency_stop(void)
{
    smd_stop_now(ARM_MOTOR_X_ADDR);
    smd_stop_now(ARM_MOTOR_Y_ADDR);
    smd_stop_now(ARM_MOTOR_R_ADDR);
}

/**
 * @brief  使能/失能全部电机
 * @param  en  0=使能, 1=失能
 */
void arm_enable_all(uint8_t en)
{
    smd_motor_enable(ARM_MOTOR_X_ADDR, en);
    smd_motor_enable(ARM_MOTOR_Y_ADDR, en);
    smd_motor_enable(ARM_MOTOR_R_ADDR, en);
}

/**
 * @brief  使能/失能指定轴
 */
void arm_enable_axis(uint8_t axis, uint8_t en)
{
    if (axis < 1 || axis > 3) return;
    smd_motor_enable(arm_axis_addr[axis - 1], en);
}

/**
 * @brief  全部轴当前位置清零
 */
void arm_set_zero(void)
{
    smd_angle_to_zero(ARM_MOTOR_X_ADDR);
    smd_angle_to_zero(ARM_MOTOR_Y_ADDR);
    smd_angle_to_zero(ARM_MOTOR_R_ADDR);
}

/**
 * @brief  查询指定轴是否到位
 */
uint8_t arm_is_arrived(uint8_t axis)
{
    if (axis < 1 || axis > 3) return 0;

    uint8_t addr = arm_axis_addr[axis - 1];

    /* 发送到位查询指令 */
    smd_read_arrived_sta(addr);

    /* 直接读取自身持有的电机实例 */
    return arm_motor_list[axis - 1]->arrived_sta;
}
