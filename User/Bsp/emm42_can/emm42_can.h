/**
 * @file    emm42_can.h
 * @author  Jackrainman
 * @brief   ZDT X42S Emm42 闭环步进电机驱动 (CAN)
 * @version 1.0
 * @date    2026-07-29
 *
 ******************************************************************************
 *    Date    | Version |   Author    | Version Info
 * -----------+---------+-------------+----------------------------------------
 * 2026-07-29 |   1.0   | Jackrainman | 初版, 对齐 X42S 二代手册 V1.0.3 4.2 节
 */

#ifndef __EMM42_CAN_H
#define __EMM42_CAN_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <cubemx.h>

#include <stdbool.h>

/* 是否使用 X 固件换算 (0: Emm 固件, 位置 x360/65536, 转速 RPM;
 *                     1: X 固件, 位置 /10, 转速 0.1RPM) */
#define EMM42_CAN_USE_X_FIRMWARE 0

/* 通讯校验字节, 出厂默认固定 0x6B (手册 4.2.1 节),
 * 若电机端改为 XOR/CRC8 校验需同步修改发送与接收校验 */
#define EMM42_CAN_CHECK_BYTE    0x6B

/* 最多支持电机数量, 地址范围 0 - 255, 0 为广播地址 */
#define EMM42_CAN_MOTOR_NUM_MAX 255

/* 电机整步步数/圈 (1.8° 电机为 200), 用于角度-脉冲换算 */
#define EMM42_CAN_STEPS_PER_REV 200

/* 出厂默认细分 (16 细分 = 3200 脉冲/圈) */
#define EMM42_CAN_DEFAULT_MICROSTEP 16

#define EMM42_CAN_MASK_SPEED        (1UL << 0) /*!< 转速有更新 */
#define EMM42_CAN_MASK_REAL_POS     (1UL << 1) /*!< 实时位置有更新 */
#define EMM42_CAN_MASK_POS_ERR      (1UL << 2) /*!< 位置误差有更新 */
#define EMM42_CAN_MASK_ORG_STATUS   (1UL << 3) /*!< 回零状态标志有更新 */
#define EMM42_CAN_MASK_MOTOR_STATUS (1UL << 4) /*!< 电机状态标志有更新 */

/* 回零状态标志位 (功能码 0x3B, 手册 5.4.4 节) */
#define EMM42_CAN_ORG_ENC_RDY  0x01 /*!< 编码器就绪 */
#define EMM42_CAN_ORG_CAL_RDY  0x02 /*!< 校准表就绪 */
#define EMM42_CAN_ORG_RUNNING  0x04 /*!< Org_SF, 正在回零 */
#define EMM42_CAN_ORG_FAIL     0x08 /*!< Org_CF, 回零失败 */
#define EMM42_CAN_ORG_OTP      0x10 /*!< 过热保护触发 */
#define EMM42_CAN_ORG_OCP      0x20 /*!< 过流保护触发 */

/* 电机状态标志位 (功能码 0x3A, 手册 5.5.15 节) */
#define EMM42_CAN_ST_EN         0x01 /*!< 已使能 */
#define EMM42_CAN_ST_IN_POS     0x02 /*!< 位置到达 */
#define EMM42_CAN_ST_STALL      0x04 /*!< 堵转标志 */
#define EMM42_CAN_ST_STALL_PROT 0x08 /*!< 堵转保护触发 */
#define EMM42_CAN_ST_LIMIT_L    0x10 /*!< 左限位输入高电平 */
#define EMM42_CAN_ST_LIMIT_R    0x20 /*!< 右限位输入高电平 */
#define EMM42_CAN_ST_POWER_LOSS 0x80 /*!< 掉电标志 */

/* CAN 扩展帧 ID 掩码: ID = (Addr << 8) | Packet, 单帧 Packet 为 0 */
#define EMM42_CAN_ID(addr)      ((uint32_t)(addr) << 8)

/**
 * @brief 命令应答状态码, 对齐手册 4.2.2 节
 */
typedef enum {
    EMM42_CAN_ACK_OK        = 0x02, /*!< 接收的命令正确 */
    EMM42_CAN_ACK_AT_ZERO   = 0x12, /*!< 已在零点 */
    EMM42_CAN_ACK_DONE      = 0x9F, /*!< 动作执行完成 (电机主动返回) */
    EMM42_CAN_ACK_PARAM_ERR = 0xE2, /*!< 命令参数错误 */
    EMM42_CAN_ACK_FRAME_ERR = 0xEE  /*!< 命令格式错误 */
} emm42_can_ack_t;

/**
 * @brief 读系统参数功能码, 对齐 X42S 二代手册 5.5 节
 * @note  读取类命令均可由 `emm42_can_read_sys_params` 发送;
 *        返回超过 8 字节的参数 (0x21/0x22 等) 暂不解析, 见 README
 */
typedef enum {
    EMM42_CAN_S_PROT_TH  = 0x13, /*!< 过热过流保护检测阈值 */
    EMM42_CAN_S_HB_TIME  = 0x16, /*!< 心跳保护功能时间 */
    EMM42_CAN_S_OPTION   = 0x1A, /*!< 选项参数状态 */
    EMM42_CAN_S_VER      = 0x1F, /*!< 固件/硬件版本 */
    EMM42_CAN_S_PHA_RL   = 0x20, /*!< 相电阻和相电感 */
    EMM42_CAN_S_PID      = 0x21, /*!< PID 参数 (Emm) */
    EMM42_CAN_S_ORG_PAR  = 0x22, /*!< 回零参数 */
    EMM42_CAN_S_ILIMIT   = 0x23, /*!< 积分限幅/刚性系数 */
    EMM42_CAN_S_VBUS     = 0x24, /*!< 总线电压 */
    EMM42_CAN_S_CBUS     = 0x26, /*!< 总线电流 */
    EMM42_CAN_S_CPHA     = 0x27, /*!< 相电流 */
    EMM42_CAN_S_ENCL     = 0x31, /*!< 线性化编码器值 */
    EMM42_CAN_S_CLKI     = 0x32, /*!< 输入脉冲数 */
    EMM42_CAN_S_TPOS     = 0x33, /*!< 电机目标位置 */
    EMM42_CAN_S_SPOS     = 0x34, /*!< 实时设定的目标位置 */
    EMM42_CAN_S_VEL      = 0x35, /*!< 电机实时转速 */
    EMM42_CAN_S_CPOS     = 0x36, /*!< 电机实时位置 */
    EMM42_CAN_S_PERR     = 0x37, /*!< 电机位置误差 */
    EMM42_CAN_S_VBAT     = 0x38, /*!< 电池电压 (Y42) */
    EMM42_CAN_S_TEMP     = 0x39, /*!< 驱动温度 */
    EMM42_CAN_S_FLAG     = 0x3A, /*!< 电机状态标志 */
    EMM42_CAN_S_ORG_ST   = 0x3B, /*!< 回零状态标志 */
    EMM42_CAN_S_ORG_FLAG = 0x3C, /*!< 回零状态 + 电机状态标志 */
    EMM42_CAN_S_PIN      = 0x3D, /*!< 引脚 IO 电平状态 */
    EMM42_CAN_S_ORG_RA   = 0x3F, /*!< 碰撞回零返回角度 */
    EMM42_CAN_S_WINDOW   = 0x41  /*!< 位置到达窗口 */
} emm42_can_sys_params_t;

/**
 * @brief 回零参数, 对齐手册 5.4.6 节
 */
typedef struct {
    uint8_t mode;        /*!< 回零模式 0 - 5 */
    uint8_t dir;         /*!< 回零方向, 0 = CW, 1 = CCW */
    uint16_t vel;        /*!< 回零速度, 0 - 3000 (RPM) */
    uint32_t timeout_ms; /*!< 回零超时时间, ms */
    uint16_t bump_vel;   /*!< 碰撞检测转速, 0 - 3000 (RPM) */
    uint16_t bump_ma;    /*!< 碰撞检测电流, 0 - 5000 (mA) */
    uint16_t bump_ms;    /*!< 碰撞检测时间, ms */
    uint8_t o_pot_en;    /*!< 上电自动回零, 0 关 / 1 开 */
} emm42_can_origin_params_t;

/**
 * @brief DMX512 协议参数, 对齐手册 5.6.19 节
 */
typedef struct {
    uint16_t total_channels; /*!< 总通道数 1 - 64 */
    uint8_t motor_channels;  /*!< 每电机通道数 1 / 2 */
    uint8_t motion_mode;     /*!< 运动模式, 0 相对 / 1 绝对 */
    uint16_t vel;            /*!< 单通道速度, 1 - 3000 (RPM) */
    uint16_t acc;            /*!< 加速度 1 - 65535 */
    uint16_t vel_step;       /*!< 双通道速度步长 */
    uint32_t move_step;      /*!< 双通道运动步长 */
} emm42_can_dmx_params_t;

/**
 * @brief 驱动配置参数 (Emm), 对齐手册 5.8.6 节
 */
typedef struct {
    uint8_t motor_type;   /*!< 电机类型 0x19 = 1.8° / 0x32 = 0.9° */
    uint8_t pulse_mux;    /*!< 脉冲端口复用 0 - 4 */
    uint8_t comm_mux;     /*!< 通讯端口复用 0 - 4, 3 = CAN */
    uint8_t en_level;     /*!< En 有效电平 0 = L, 1 = H, 2 = Hold */
    uint8_t dir_level;    /*!< Dir 有效电平 0 = CW, 1 = CCW */
    uint8_t microstep;    /*!< 细分 0 - 255, 0 = 256 细分 */
    uint8_t interp;       /*!< 细分插补 0 / 1 */
    uint16_t open_ma;     /*!< 开环模式工作电流, mA */
    uint16_t closed_ma;   /*!< 闭环堵转最大电流, mA */
    uint16_t max_voltage; /*!< 闭环最大输出电压, (0 - 5000) * 4mV */
    uint8_t uart_baud;    /*!< 串口波特率档位 0 - 8, 5 = 115200 */
    uint8_t can_baud;     /*!< CAN 速率档位 0 - 9, 7 = 500K */
    uint8_t checksum;     /*!< 通讯校验 0 - 4, 0 = 固定 6B */
    uint8_t ack_mode;     /*!< 控制命令应答方式 0 - 4 */
    uint8_t stall_en;     /*!< 堵转保护 0 关 / 1 使能 */
    uint16_t stall_vel;   /*!< 堵转检测转速, RPM */
    uint16_t stall_ma;    /*!< 堵转检测电流, mA */
    uint16_t stall_ms;    /*!< 堵转检测时间, ms */
    uint16_t arrive_win;  /*!< 位置到达窗口 */
} emm42_can_config_t;

/**
 * @brief 电机句柄, 配置与状态合一
 */
typedef struct {
    can_selected_t can_select; /*!< 选择 CAN 通信 */
    uint8_t addr;              /*!< 电机地址 1-255, 0 为广播地址 */

    uint16_t microstep; /*!< 当前细分 (1-256), 供角度-脉冲换算,
                             由 `emm42_can_set_microstep` 同步更新 */

    uint8_t ack_status; /*!< 最近一次命令应答状态, 见 `emm42_can_ack_t` */

    float cur_pos; /*!< 实时位置, 度 */
    float cur_vel; /*!< 实时转速, RPM */
    float pos_err; /*!< 位置误差, 度 */

    uint8_t org_status;   /*!< 回零状态标志, 见 EMM42_CAN_ORG_* */
    uint8_t motor_status; /*!< 电机状态标志, 见 EMM42_CAN_ST_* */

    uint32_t valid_mask; /*!< 字段更新掩码, 见 EMM42_CAN_MASK_* */
} emm42_can_motor_t;

uint8_t emm42_can_motor_init(emm42_can_motor_t *motor,
                             can_selected_t can_select, uint8_t addr);
uint8_t emm42_can_motor_deinit(emm42_can_motor_t *motor);

/* ======================== 触发动作命令 (手册 5.2 节) ======================== */

void emm42_can_calibrate_encoder(emm42_can_motor_t *motor);
void emm42_can_reset_motor(emm42_can_motor_t *motor);
void emm42_can_reset_curpos_to_zero(emm42_can_motor_t *motor);
void emm42_can_release_protect(emm42_can_motor_t *motor);
void emm42_can_factory_reset(emm42_can_motor_t *motor);

/* ======================== 运动控制命令 (手册 5.3 节) ======================== */

void emm42_can_en_control(emm42_can_motor_t *motor, bool state, bool snF);
void emm42_can_vel_control(emm42_can_motor_t *motor, uint8_t dir, uint16_t vel,
                           uint8_t acc, bool snF);
void emm42_can_pos_control(emm42_can_motor_t *motor, uint8_t dir, uint16_t vel,
                           uint8_t acc, float degree, uint8_t mode, bool snF);
void emm42_can_pos_control_pulse(emm42_can_motor_t *motor, uint8_t dir,
                                 uint16_t vel, uint8_t acc, uint32_t pulse,
                                 uint8_t mode, bool snF);
void emm42_can_stop_now(emm42_can_motor_t *motor, bool snF);
void emm42_can_sync_motion(emm42_can_motor_t *motor);
void emm42_can_send_frame(emm42_can_motor_t *motor, const uint8_t *buf,
                          uint16_t len);

/* ======================== 原点回零命令 (手册 5.4 节) ======================== */

void emm42_can_origin_set_single_turn_zero(emm42_can_motor_t *motor,
                                           bool store);
void emm42_can_origin_trigger_return(emm42_can_motor_t *motor, uint8_t o_mode,
                                     bool snF);
void emm42_can_origin_abort(emm42_can_motor_t *motor);
void emm42_can_origin_set_params(emm42_can_motor_t *motor, bool store,
                                 const emm42_can_origin_params_t *params);

/* ======================== 读取系统参数 (手册 5.5 节) ======================== */

void emm42_can_read_sys_params(emm42_can_motor_t *motor,
                               emm42_can_sys_params_t param);
void emm42_can_set_auto_report(emm42_can_motor_t *motor,
                               emm42_can_sys_params_t param, uint16_t ms);

/* ======================== 读写驱动参数 (手册 5.6 节) ======================== */

void emm42_can_set_id(emm42_can_motor_t *motor, bool store, uint8_t new_id);
void emm42_can_set_microstep(emm42_can_motor_t *motor, bool store,
                             uint8_t microstep);
void emm42_can_set_power_loss_flag(emm42_can_motor_t *motor, uint8_t flag);
void emm42_can_set_motor_type(emm42_can_motor_t *motor, bool store,
                              uint8_t type);
void emm42_can_set_firmware(emm42_can_motor_t *motor, bool store, uint8_t fw);
void emm42_can_set_ctrl_mode(emm42_can_motor_t *motor, bool store,
                             bool closed_loop);
void emm42_can_set_direction(emm42_can_motor_t *motor, bool store,
                             uint8_t dir);
void emm42_can_set_button_lock(emm42_can_motor_t *motor, bool store, bool lock);
void emm42_can_set_speed_scale(emm42_can_motor_t *motor, bool store, bool en);
void emm42_can_set_open_current(emm42_can_motor_t *motor, bool store,
                                uint16_t ma);
void emm42_can_set_closed_current(emm42_can_motor_t *motor, bool store,
                                  uint16_t ma);
void emm42_can_set_pid(emm42_can_motor_t *motor, bool store, uint32_t kp,
                       uint32_t ki, uint32_t kd);
void emm42_can_read_dmx_params(emm42_can_motor_t *motor);
void emm42_can_set_dmx_params(emm42_can_motor_t *motor, bool store,
                              const emm42_can_dmx_params_t *params);
void emm42_can_set_arrive_window(emm42_can_motor_t *motor, bool store,
                                 uint16_t window);
void emm42_can_set_protect_threshold(emm42_can_motor_t *motor, bool store,
                                     uint16_t temp, uint16_t ma, uint16_t ms);
void emm42_can_set_heartbeat(emm42_can_motor_t *motor, bool store,
                             uint32_t ms);
void emm42_can_set_integral_limit(emm42_can_motor_t *motor, bool store,
                                  uint32_t value);
void emm42_can_set_origin_return_angle(emm42_can_motor_t *motor, bool store,
                                       uint16_t angle);
void emm42_can_broadcast_read_id(can_selected_t can_select);
void emm42_can_set_param_lock(emm42_can_motor_t *motor, bool store,
                              uint8_t level);

/* ================== 上电自动运行 (手册 5.7 节, Emm 固件) ================== */

void emm42_can_set_auto_run(emm42_can_motor_t *motor, bool store, uint8_t dir,
                            uint16_t vel, uint8_t acc, bool en_ctrl);

/* ================== 读取/修改所有驱动参数 (手册 5.8 节, Emm 固件) ================== */

void emm42_can_read_system_status(emm42_can_motor_t *motor);
void emm42_can_read_config(emm42_can_motor_t *motor);
void emm42_can_write_config(emm42_can_motor_t *motor, bool store,
                            const emm42_can_config_t *config);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __EMM42_CAN_H */
