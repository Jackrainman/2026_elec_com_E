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

#define EMM42_MASK_SPEED    (1UL << 0) /*!< 转速有更新 */
#define EMM42_MASK_REAL_POS (1UL << 1) /*!< 实时位置有更新 */

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
    EMM42_S_ORG_FLAG = 0x3C, /*!< 回零状态 + 电机状态标志 */
    EMM42_S_PIN      = 0x3D  /*!< 引脚 IO 电平状态 */
} emm42_sys_params_t;

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

    uint32_t valid_mask; /*!< 字段更新掩码, 见 EMM42_MASK_* */
} emm42_motor_t;

uint8_t emm42_motor_init(emm42_motor_t *motor, UART_HandleTypeDef *huart,
                         uint8_t addr, GPIO_TypeDef *rs485_port,
                         uint16_t rs485_pin);
uint8_t emm42_motor_deinit(emm42_motor_t *motor);

void emm42_reset_motor(emm42_motor_t *motor);
void emm42_reset_curpos_to_zero(emm42_motor_t *motor);
void emm42_en_control(emm42_motor_t *motor, bool state, bool snF);
void emm42_vel_control(emm42_motor_t *motor, uint8_t dir, uint16_t vel,
                       uint8_t acc, bool snF);
void emm42_pos_control(emm42_motor_t *motor, uint8_t dir, uint16_t vel,
                       uint8_t acc, float degree, uint8_t mode, bool snF);
void emm42_pos_control_pulse(emm42_motor_t *motor, uint8_t dir, uint16_t vel,
                             uint8_t acc, uint32_t pulse, uint8_t mode, bool snF);
void emm42_stop_now(emm42_motor_t *motor, bool snF);
void emm42_origin_trigger_return(emm42_motor_t *motor, uint8_t o_mode,
                                 bool snF);

void emm42_read_sys_params(emm42_motor_t *motor, emm42_sys_params_t param);
void emm42_set_auto_report(emm42_motor_t *motor, emm42_sys_params_t param,
                           uint16_t ms);

bool emm42_frame_process(const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __EMM42_H */
