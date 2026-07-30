/**
 * @file    emm42.h
 * @author  Jackrainman
 * @brief   ZDT X42S Emm42 闭环步进电机驱动 (RS485)
 * @version 1.0
 * @date    2026-07-29
 *
 ******************************************************************************
 *    Date    | Version |   Author    | Version Info
 * -----------+---------+-------------+----------------------------------------
 * 2026-07-29 |   1.0   | Jackrainman | 初版, 对齐 X42S 二代手册 V1.0.3
 */

#ifndef __EMM42_H
#define __EMM42_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <cubemx.h>

#include <stdbool.h>

/* 是否使用 X 固件换算 (0: Emm 固件, 位置 x360/65536, 转速 RPM;
 *                     1: X 固件, 位置 /10, 转速 0.1RPM) */
#define EMM42_USE_X_FIRMWARE 0

/* 最多支持电机数量 (注册表按地址索引)，少了省空间，最高支持 0-255 */
#define EMM42_MOTOR_NUM_MAX 10

#define EMM42_MASK_SPEED        (1UL << 0) /*!< 转速有更新 */
#define EMM42_MASK_REAL_POS     (1UL << 1) /*!< 实时位置有更新 */
#define EMM42_MASK_POS_ERR      (1UL << 2) /*!< 位置误差有更新 */
#define EMM42_MASK_ORG_STATUS   (1UL << 3) /*!< 回零状态标志有更新 */
#define EMM42_MASK_MOTOR_STATUS (1UL << 4) /*!< 电机状态标志有更新 */
#define EMM42_MASK_ORG_PARAMS   (1UL << 5) /*!< 回零参数有更新 */

/* 多电机命令子命令最大字节数 (手册 5.3.1 节) */
#define EMM42_MULTI_CMD_MAX 128

/* 回零状态标志位 (功能码 0x3B, 手册 5.4.4 节) */
#define EMM42_ORG_ENC_RDY  0x01 /*!< 编码器就绪 */
#define EMM42_ORG_CAL_RDY  0x02 /*!< 校准表就绪 */
#define EMM42_ORG_RUNNING  0x04 /*!< Org_SF, 正在回零 */
#define EMM42_ORG_FAIL     0x08 /*!< Org_CF, 回零失败 */
#define EMM42_ORG_OTP      0x10 /*!< 过热保护触发 */
#define EMM42_ORG_OCP      0x20 /*!< 过流保护触发 */

/* 电机状态标志位 (功能码 0x3A, 手册 5.5.15 节) */
#define EMM42_ST_EN         0x01 /*!< 已使能 */
#define EMM42_ST_IN_POS     0x02 /*!< 位置到达 */
#define EMM42_ST_STALL      0x04 /*!< 堵转标志 */
#define EMM42_ST_STALL_PROT 0x08 /*!< 堵转保护触发 */
#define EMM42_ST_LIMIT_L    0x10 /*!< 左限位输入高电平 */
#define EMM42_ST_LIMIT_R    0x20 /*!< 右限位输入高电平 */
#define EMM42_ST_POWER_LOSS 0x80 /*!< 掉电标志 */

/**
 * @brief 读系统参数功能码, 对齐 X42S 二代手册 5.5 节
 */
typedef enum {
    EMM42_S_VER      = 0x1F, /*!< 固件/硬件版本 */
    EMM42_S_PHA_RL   = 0x20, /*!< 相电阻和相电感 */
    EMM42_S_VBUS     = 0x24, /*!< 总线电压 */
    EMM42_S_CBUS     = 0x26, /*!< 总线电流 */
    EMM42_S_CPHA     = 0x27, /*!< 相电流 */
    EMM42_S_ENCL     = 0x31, /*!< 线性化编码器值 */
    EMM42_S_CLKI     = 0x32, /*!< 输入脉冲数 */
    EMM42_S_TPOS     = 0x33, /*!< 电机目标位置 */
    EMM42_S_SPOS     = 0x34, /*!< 实时设定的目标位置 */
    EMM42_S_VEL      = 0x35, /*!< 电机实时转速 */
    EMM42_S_CPOS     = 0x36, /*!< 电机实时位置 */
    EMM42_S_PERR     = 0x37, /*!< 电机位置误差 */
    EMM42_S_VBAT     = 0x38, /*!< 电池电压 (Y42) */
    EMM42_S_TEMP     = 0x39, /*!< 驱动温度 */
    EMM42_S_FLAG     = 0x3A, /*!< 电机状态标志 */
    EMM42_S_ORG      = 0x3B, /*!< 回零状态标志 */
    EMM42_S_ORG_FLAG = 0x3C, /*!< 回零状态 + 电机状态标志 */
    EMM42_S_PIN      = 0x3D  /*!< 引脚 IO 电平状态 */
} emm42_sys_params_t;

/**
 * @brief 回零参数, 对齐 X42S 二代手册 5.4.5/5.4.6 节
 */
typedef struct {
    uint8_t mode;        /*!< 回零模式, 0 单圈就近 / 1 单圈方向 /
                              2 无限位碰撞 / 3 限位 / 4 回到坐标零点 /
                              5 回到上次掉电位置 */
    uint8_t dir;         /*!< 回零方向, 0 = CW, 1 = CCW */
    uint16_t vel;        /*!< 回零速度, 0 - 3000 (RPM) */
    uint32_t timeout_ms; /*!< 回零超时时间, 毫秒 */
    uint16_t detect_vel; /*!< 碰撞回零检测转速, RPM */
    uint16_t detect_ma;  /*!< 碰撞回零检测电流, mA */
    uint16_t detect_ms;  /*!< 碰撞回零检测时间, 毫秒 */
    uint8_t auto_origin; /*!< 上电自动触发回零, 0 不使能 / 1 使能 */
} emm42_origin_params_t;

/**
 * @brief 电机句柄, 配置与状态合一
 */
typedef struct {
    UART_HandleTypeDef *huart; /*!< 所在 RS485 总线的串口 */
    uint8_t addr;              /*!< 电机地址 1-255, 0 为广播地址 */

    GPIO_TypeDef *rs485_port; /*!< RS485 RE 方向脚端口 */
    uint16_t rs485_pin;       /*!< RS485 RE 方向脚引脚 */

    float cur_pos; /*!< 实时位置, 度 */
    float cur_vel; /*!< 实时转速, RPM */
    float pos_err; /*!< 位置误差, 度 */

    uint8_t org_status;   /*!< 回零状态标志, 见 EMM42_ORG_* */
    uint8_t motor_status; /*!< 电机状态标志, 见 EMM42_ST_* */

    emm42_origin_params_t origin_params; /*!< 回零参数 */

    uint32_t valid_mask; /*!< 字段更新掩码, 见 EMM42_MASK_* */
} emm42_motor_t;

/* ------------------------------------------------------------------
 * 常用控制: 日常使用只需这一组
 * ------------------------------------------------------------------ */
uint8_t emm42_motor_init(emm42_motor_t *motor, UART_HandleTypeDef *huart,
                         uint8_t addr, GPIO_TypeDef *rs485_port,
                         uint16_t rs485_pin);
uint8_t emm42_motor_deinit(emm42_motor_t *motor);

void emm42_en_control(emm42_motor_t *motor, bool state, bool snF);
void emm42_vel_control(emm42_motor_t *motor, uint8_t dir, uint16_t vel,
                       uint8_t acc, bool snF);
void emm42_pos_control(emm42_motor_t *motor, uint8_t dir, uint16_t vel,
                       uint8_t acc, float degree, uint8_t mode, bool snF);
void emm42_pos_control_pulse(emm42_motor_t *motor, uint8_t dir, uint16_t vel,
                             uint8_t acc, uint32_t pulse, uint8_t mode,
                             bool snF);
void emm42_stop_now(emm42_motor_t *motor, bool snF);
void emm42_reset_curpos_to_zero(emm42_motor_t *motor);

bool emm42_frame_process(const uint8_t *buf, uint16_t len);

/* ------------------------------------------------------------------
 * 状态读取与上报
 * ------------------------------------------------------------------ */
void emm42_read_sys_params(emm42_motor_t *motor, emm42_sys_params_t param);
void emm42_set_auto_report(emm42_motor_t *motor, emm42_sys_params_t param,
                           uint16_t ms);

/* ------------------------------------------------------------------
 * 回零
 * ------------------------------------------------------------------ */
void emm42_origin_set_zero(emm42_motor_t *motor, bool store);
void emm42_origin_trigger_return(emm42_motor_t *motor, uint8_t o_mode,
                                 bool snF);
void emm42_origin_interrupt(emm42_motor_t *motor);
void emm42_origin_read_params(emm42_motor_t *motor);
void emm42_origin_set_params(emm42_motor_t *motor,
                             const emm42_origin_params_t *params, bool store);

/* ------------------------------------------------------------------
 * 多机
 * ------------------------------------------------------------------ */
void emm42_sync_motion(emm42_motor_t *motor);
void emm42_multi_cmd(emm42_motor_t *motor, const uint8_t *cmds, uint16_t len);

/* ------------------------------------------------------------------
 * 调试与产线: 校准/保护/出厂/改地址, 正常使用不需要
 * ------------------------------------------------------------------ */
void emm42_reset_motor(emm42_motor_t *motor);
void emm42_calibrate_encoder(emm42_motor_t *motor);
void emm42_release_protection(emm42_motor_t *motor);
void emm42_restore_factory(emm42_motor_t *motor);
void emm42_set_addr(emm42_motor_t *motor, uint8_t new_addr, bool store);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __EMM42_H */
