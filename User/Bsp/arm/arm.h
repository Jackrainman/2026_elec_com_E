/**
 ****************************************************************************************************
 * @file        arm.h
 * @brief       三电机滑轨机械臂控制模块
 *              - X轴 / Y轴 平面直线运动
 *              - R轴 末端自转
 *              - 基于正点原子 SMD 步进电机驱动
 * @note        轴编号约定：
 *              1: X轴（左右）, 2: Y轴（前后）, 3: R轴（自转）
 ****************************************************************************************************
 */

#ifndef __ARM_H
#define __ARM_H

#include "stdint.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== 电机地址配置 ========================== */
#define ARM_MOTOR_X_ADDR    1   /* X轴电机地址 */
#define ARM_MOTOR_Y_ADDR    2   /* Y轴电机地址 */
#define ARM_MOTOR_R_ADDR    3   /* R轴（自转）电机地址 */

/* ========================== 机械参数 ========================== */
/* 51200 脉冲 = 1 转（电机细分设置） */
#define ARM_PULSE_PER_REV   51200

/* 各轴导程 / 传动比：脉冲当量 = pulses / mm */
/* 用户按实际丝杆导程、同步轮减速比修改以下宏 */
#define ARM_X_PULSE_PER_MM  512.0f   /* X轴：每毫米对应脉冲数 */
#define ARM_Y_PULSE_PER_MM  512.0f   /* Y轴：每毫米对应脉冲数 */
#define ARM_R_PULSE_PER_DEG 142.22f  /* R轴：每度对应脉冲数 (51200/360) */

/* ========================== 默认运动参数 ========================== */
#define ARM_DEFAULT_SPEED   60      /* 默认转速 RPM */
#define ARM_DEFAULT_ACC     1       /* 默认加速度档位 */
#define ARM_HOMING_SPEED    30      /* 回零转速 RPM */

/* 方向定义（与电机驱动一致） */
#define ARM_DIR_CW   0   /* 正转 */
#define ARM_DIR_CCW  1   /* 反转 */

/* ========================== 软限位（单位：脉冲） ========================== */
/* 设为 0 则不启用限位检查 */
#define ARM_X_LIMIT_MIN  0
#define ARM_X_LIMIT_MAX  0
#define ARM_Y_LIMIT_MIN  0
#define ARM_Y_LIMIT_MAX  0

/* ========================== API ========================== */

/**
 * @brief  初始化机械臂（使能全部电机并设为位置模式）
 * @note   需先调用 smd_motor_init() 为每个电机注册 smd_motor_t
 */
void arm_init(void);

/**
 * @brief  控制 XY 两轴移动到绝对位置（mm），R轴不动
 * @param  x_mm    X轴目标位置（mm）
 * @param  y_mm    Y轴目标位置（mm）
 * @param  speed   转速（RPM），填0则使用默认值
 */
void arm_move_to(float x_mm, float y_mm, uint16_t speed);

/**
 * @brief  控制 XYR 三轴移动到绝对位置
 * @param  x_mm    X轴目标位置（mm）
 * @param  y_mm    Y轴目标位置（mm）
 * @param  r_deg   R轴目标角度（°）
 * @param  speed   转速（RPM），填0则使用默认值
 */
void arm_move_to_all(float x_mm, float y_mm, float r_deg, uint16_t speed);

/**
 * @brief  单轴移动到绝对位置（mm 或 °）
 * @param  axis    轴编号：1=X, 2=Y, 3=R
 * @param  pos     目标位置（X/Y为mm，R为°）
 * @param  speed   转速（RPM），填0则使用默认值
 */
void arm_axis_move(uint8_t axis, float pos, uint16_t speed);

/**
 * @brief  单轴相对移动
 * @param  axis    轴编号
 * @param  delta   增量（X/Y为mm，R为°）
 * @param  speed   转速（RPM），填0则使用默认值
 */
void arm_axis_rel_move(uint8_t axis, float delta, uint16_t speed);

/**
 * @brief  查询指定轴是否到位
 * @param  axis    轴编号：1=X, 2=Y, 3=R
 * @retval 0=未到位, 1=已到位
 * @note   会触发一次查询指令，建议调用间隔 >5ms
 */
uint8_t arm_is_arrived(uint8_t axis);

/**
 * @brief  R轴自转指定角度（相对）
 * @param  deg     旋转角度（°），正为CW，负为CCW
 * @param  speed   转速（RPM），填0则使用默认值
 */
void arm_rotate(float deg, uint16_t speed);

/**
 * @brief  急停（全部电机立即刹车）
 */
void arm_emergency_stop(void);

/**
 * @brief  使能 / 失能全部电机
 * @param  en  0=使能, 1=失能
 */
void arm_enable_all(uint8_t en);

/**
 * @brief  使能 / 失能指定轴
 */
void arm_enable_axis(uint8_t axis, uint8_t en);

/**
 * @brief  将全部轴当前位置清零（设为原点）
 */
void arm_set_zero(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_H */
