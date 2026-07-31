/**
 ****************************************************************************************************
 * @file    arm.h
 * @author  xinglu
 * @brief   三轴机械臂控制模块 (基于正点原子SMD步进电机)
 *          - X轴 / Y轴 平面运动
 *          - R轴 末端自转
 * @version 1.0
 * @date    2026-07-31
 * @note    轴编号约定：
 *          1: X轴, 2: Y轴, 3: R轴（自转）
 *          所有电机共用一路 RS485 总线 (UART4)
 ****************************************************************************************************
 */

#ifndef __ARM_H
#define __ARM_H

#include "stdint.h"
#include "stdbool.h"
#include "servo/servo.h"
#include "atk_smd/smd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 舵机 */
extern servo_t servo;
#define SERVO_UP()   servo_set_state(&servo, SERVO_OPEN)
#define SERVO_DOWN() servo_set_state(&servo, SERVO_CLOSE)

/* ========================== 电机地址配置 ========================== */
#define ARM_MOTOR_X_ADDR    1   /* X轴电机地址 */
#define ARM_MOTOR_Y_ADDR    2   /* Y轴电机地址 */
#define ARM_MOTOR_R_ADDR    3   /* R轴（自转）电机地址 */

/* ========================== 默认运动参数 ========================== */
#define ARM_DEFAULT_SPEED   60      /* 默认转速 RPM */
#define ARM_DEFAULT_ACC     1       /* 默认加速度档位 */

/* 初始化同步确认参数（模仿 2026R1） */
#define ARM_SYNC_RETRY_TIMES  3    /* 单轴同步重试次数 */
#define ARM_SYNC_TIMEOUT_MS   50   /* 单次等待应答超时 (ms) */

/* 细分: 51200 脉冲/圈（按实际电机细分设置修改） */
#define ARM_PULSE_PER_REV   51200U

/* ========================== 单位换算 ========================== */
#define ARM_X_MM_PER_REV    40.0f   /*!< X轴丝杆导程 mm/圈  */
#define ARM_Y_MM_PER_REV    40.0f   /*!< Y轴丝杆导程 mm/圈  */
#define ARM_DEG_PER_REV     360.0f  /*!< 角度 °/圈         */

/* 距离 → 脉冲（无符号，绝对位置用） */
#define ARM_X_MM_TO_PULSE(mm)  ((uint32_t)((float)(mm) / ARM_X_MM_PER_REV * (float)ARM_PULSE_PER_REV))
#define ARM_Y_MM_TO_PULSE(mm)  ((uint32_t)((float)(mm) / ARM_Y_MM_PER_REV * (float)ARM_PULSE_PER_REV))
/* 角度 → 脉冲（无符号） */
#define ARM_DEG_TO_PULSE(deg)  ((uint32_t)((float)(deg) / ARM_DEG_PER_REV * (float)ARM_PULSE_PER_REV))

/* 距离 → 脉冲（有符号，相对位置用） */
#define ARM_X_MM_TO_PULSE_S(mm)  ((int32_t)((float)(mm) / ARM_X_MM_PER_REV * (float)ARM_PULSE_PER_REV))
#define ARM_Y_MM_TO_PULSE_S(mm)  ((int32_t)((float)(mm) / ARM_Y_MM_PER_REV * (float)ARM_PULSE_PER_REV))
/* 角度 → 脉冲（有符号） */
#define ARM_DEG_TO_PULSE_S(deg)  ((int32_t)((float)(deg) / ARM_DEG_PER_REV * (float)ARM_PULSE_PER_REV))

/* ========================== 相机坐标系偏移 ========================== */
#define CAM_X_CORRECT (-13.0f)  /*!< X轴初始坐标系偏移 (mm) */
#define CAM_Y_CORRECT (-32.0f)  /*!< Y轴初始坐标系偏移 (mm) */

/* 方向定义（与 SMD 驱动一致: 0=CW, 1=CCW） */
#define ARM_DIR_CW   0
#define ARM_DIR_CCW  1

/* ========================== API ========================== */

/**
 * @brief  初始化机械臂（使能全部电机并设为位置模式）
 * @note   上电等待 1s 后，将三个电机当前位置设为零点；
 *         随后读回各轴位置确认电机在线（用 arm_is_ready() 查询结果）
 */
void arm_init(void);

/**
 * @brief  查询机械臂是否初始化成功
 * @note   模仿 2026R1：初始化时读回各轴位置并校验应答，
 *         三轴均同步成功才返回 true
 * @return true=三轴均在线, false=存在离线轴
 */
bool arm_is_ready(void);

/**
 * @brief  查询指定轴是否初始化成功（通信同步成功）
 * @param  axis  轴编号：1=X, 2=Y, 3=R
 * @return true=该轴在线, false=离线或无应答
 */
bool arm_axis_is_ready(uint8_t axis);

/**
 * @brief  单轴绝对移动（相对坐标零点）
 * @param  axis   轴编号：1=X, 2=Y, 3=R
 * @param  pulse  目标脉冲数
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
 * @note   通过串口主动查询电机位置
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
 * @brief  使能 / 失能指定轴
 */
void arm_enable_axis(uint8_t axis, bool en);

/**
 * @brief  读取指定轴当前位置（脉冲数）
 * @note   需先调用 arm_update_position() 刷新位置
 * @param  axis  轴编号
 * @return 当前位置脉冲数
 */
uint32_t arm_get_position_pulse(uint8_t axis);

/**
 * @brief  将全部轴当前位置清零（设为坐标原点）
 */
void arm_set_zero(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_H */
