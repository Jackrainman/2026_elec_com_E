/**
 ****************************************************************************************************
 * @file        arm.h
 * @brief       三轴机械臂控制模块 (基于 Emm42 闭环步进电机)
 *              - X轴 / Y轴 平面运动
 *              - R轴 末端自转
 * @note        轴编号约定：
 *              1: X轴, 2: Y轴, 3: R轴（自转）
 *              所有电机共用一路 RS485 总线 (UART4)
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

/* ========================== 默认运动参数 ========================== */
#define ARM_DEFAULT_SPEED   120     /* 默认转速 RPM */
#define ARM_DEFAULT_ACC     5       /* 默认加速度档位 */

/* 细分: 16细分 = 3200 脉冲/圈 */
#define ARM_PULSE_PER_REV   3200U

/* ========================== 单位换算 ========================== */
#define ARM_MM_PER_REV      10.0f   /*!< 丝杆导程 mm/圈 (按实际修改) */

/* 距离 → 脉冲 */
#define ARM_MM_TO_PULSE(mm)  ((uint32_t)((float)(mm) / ARM_MM_PER_REV * (float)ARM_PULSE_PER_REV))
/* 角度 → 脉冲 */
#define ARM_DEG_TO_PULSE(deg) ((uint32_t)((float)(deg) / 360.0f * (float)ARM_PULSE_PER_REV))

/* 方向定义（与 emm42 一致: 0=CW, 1=CCW） */
#define ARM_DIR_CW   0
#define ARM_DIR_CCW  1

/* ========================== API ========================== */

/**
 * @brief  初始化机械臂（使能全部电机）
 * @note   所有电机挂载在 UART4 RS485 总线上
 */
void arm_init(void);

/**
 * @brief  单轴绝对移动（相对坐标零点）
 * @param  axis   轴编号：1=X, 2=Y, 3=R
 * @param  pulse  目标脉冲数 (16细分, 3200脉冲/圈)
 * @param  speed  转速（RPM），填0则使用默认值
 */
void arm_axis_move(uint8_t axis, uint32_t pulse, uint16_t speed);

/**
 * @brief  单轴相对移动
 * @param  axis   轴编号
 * @param  pulse  相对脉冲数 (正 CW / 负 CCW)
 * @param  speed  转速（RPM），填0则使用默认值
 */
void arm_axis_rel_move(uint8_t axis, int32_t pulse, uint16_t speed);

/**
 * @brief  估算单轴移动耗时
 * @param  pulse  移动脉冲数
 * @param  speed  转速（RPM）
 * @return 预估耗时（ms），已含加减速余量
 */
uint32_t arm_est_move_ms(uint32_t pulse, uint16_t speed);

/**
 * @brief  查询并更新指定轴的实时位置（阻塞约 5ms）
 * @note   通过 RS485 主动查询电机编码器位置
 * @param  axis  轴编号
 */
void arm_update_position(uint8_t axis);

/**
 * @brief  等待指定轴运动到位（阻塞式，主动查询位置）
 * @param  axis       轴编号
 * @param  target     目标脉冲数
 * @param  tolerance  到位容差（脉冲数），建议 50 ~ 200
 * @param  timeout_ms 超时（ms），填 0 则永不超时
 * @return true=到位, false=超时
 */
bool arm_wait_axis_done(uint8_t axis, uint32_t target, uint32_t tolerance,
                        uint32_t timeout_ms);

/**
 * @brief  急停（全部电机立即刹车）
 */
void arm_emergency_stop(void);

/**
 * @brief  使能 / 失能全部电机
 * @param  en  true=锁轴使能, false=松轴
 */
void arm_enable_all(bool en);

/**
 * @brief  将全部轴当前位置清零（设为坐标原点）
 */
void arm_set_zero(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_H */
