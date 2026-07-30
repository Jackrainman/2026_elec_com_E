/**
 * @file    emm42_can.c
 * @author  Jackrainman
 * @brief   ZDT X42S Emm42 闭环步进电机驱动 (CAN)
 * @version 1.0
 * @date    2026-07-29
 */

#include "emm42_can.h"

#include "can_list/can_list.h"

/**
 * @brief 发送一条命令, 大于 8 字节时自动拆包 (手册 4.2.1 节)
 *
 * 扩展帧 ID = (Addr << 8) | Packet, Packet 从 0 开始计数;
 * 数据内容为功能码 + 命令数据 + 校验码, 不再包含地址字节.
 *
 * @param motor 电机句柄
 * @param buf 命令数据 (功能码开头, 校验码结尾)
 * @param len 数据长度
 */
static void send_command(emm42_can_motor_t *motor, const uint8_t *buf,
                         uint16_t len) {
    if (motor == NULL || buf == NULL || len == 0) {
        return;
    }

    uint16_t sent = 0;
    uint8_t packet = 0;

    do {
        uint8_t chunk = (len - sent > 8) ? 8 : (uint8_t)(len - sent);
        can_send_message(motor->can_select, CAN_ID_EXT,
                         EMM42_CAN_ID(motor->addr) | packet, chunk,
                         &buf[sent]);
        sent += chunk;
        ++packet;
    } while (sent < len);
}

/**
 * @brief CAN 收到消息中断回调
 *
 * 电机返回格式: 功能码 + 返回数据 + 校验码 (手册 4.2.2 节),
 * 单帧即可放下, 不做多包重组.
 *
 * @param node_obj 节点数据 (电机句柄)
 * @param can_rx_header CAN 消息头
 * @param can_msg CAN 消息
 */
static void can_callback(void *node_obj, can_rx_header_t *can_rx_header,
                         uint8_t *can_msg) {
    if (node_obj == NULL) {
        return;
    }

    emm42_can_motor_t *motor = (emm42_can_motor_t *)node_obj;

    if (can_rx_header->id_type != CAN_ID_EXT) {
        return;
    }

    uint8_t len = can_rx_header->data_length;

    if (len < 3 || can_msg[len - 1] != EMM42_CAN_CHECK_BYTE) {
        return;
    }

    switch (can_msg[0]) {
        case EMM42_CAN_S_CPOS: {
            /* 读取电机实时位置返回: 符号 + 4 字节位置 */
            if (len != 7) {
                return;
            }

            uint32_t pos_raw = ((uint32_t)can_msg[2] << 24) |
                               ((uint32_t)can_msg[3] << 16) |
                               ((uint32_t)can_msg[4] << 8) |
                               ((uint32_t)can_msg[5]);
            int32_t pos_signed =
                (can_msg[1] == 0x01) ? -(int32_t)pos_raw : (int32_t)pos_raw;

#if (EMM42_CAN_USE_X_FIRMWARE == 1)
            motor->cur_pos = (float)pos_signed / 10.0f;
#else
            motor->cur_pos = (float)pos_signed * 360.0f / 65536.0f;
#endif /* EMM42_CAN_USE_X_FIRMWARE == 1 */
            motor->valid_mask |= EMM42_CAN_MASK_REAL_POS;
        } break;

        case EMM42_CAN_S_VEL: {
            /* 读取电机实时转速返回: 符号 + 2 字节转速 */
            if (len != 5) {
                return;
            }

            uint16_t vel_raw =
                ((uint16_t)can_msg[2] << 8) | ((uint16_t)can_msg[3]);

#if (EMM42_CAN_USE_X_FIRMWARE == 1)
            float vel = (float)vel_raw / 10.0f;
#else
            float vel = (float)vel_raw;
#endif /* EMM42_CAN_USE_X_FIRMWARE == 1 */
            motor->cur_vel = (can_msg[1] == 0x01) ? -vel : vel;
            motor->valid_mask |= EMM42_CAN_MASK_SPEED;
        } break;

        case EMM42_CAN_S_PERR: {
            /* 读取电机位置误差返回: 符号 + 4 字节误差 (手册 5.5.14 节) */
            if (len != 7) {
                return;
            }

            uint32_t err_raw = ((uint32_t)can_msg[2] << 24) |
                               ((uint32_t)can_msg[3] << 16) |
                               ((uint32_t)can_msg[4] << 8) |
                               ((uint32_t)can_msg[5]);
            int32_t err_signed =
                (can_msg[1] == 0x01) ? -(int32_t)err_raw : (int32_t)err_raw;

#if (EMM42_CAN_USE_X_FIRMWARE == 1)
            motor->pos_err = (float)err_signed / 100.0f;
#else
            motor->pos_err = (float)err_signed * 360.0f / 65536.0f;
#endif /* EMM42_CAN_USE_X_FIRMWARE == 1 */
            motor->valid_mask |= EMM42_CAN_MASK_POS_ERR;
        } break;

        case EMM42_CAN_S_FLAG: {
            /* 读取电机状态标志返回: 1 字节标志 (手册 5.5.15 节) */
            if (len != 3) {
                return;
            }

            motor->motor_status = can_msg[1];
            motor->valid_mask |= EMM42_CAN_MASK_MOTOR_STATUS;
        } break;

        case EMM42_CAN_S_ORG_ST: {
            /* 读取回零状态标志返回: 1 字节标志 (手册 5.4.4 节) */
            if (len != 3) {
                return;
            }

            motor->org_status = can_msg[1];
            motor->valid_mask |= EMM42_CAN_MASK_ORG_STATUS;
        } break;

        case EMM42_CAN_S_ORG_FLAG: {
            /* 读取回零状态标志 + 电机状态标志返回 (手册 5.5.16 节) */
            if (len != 4) {
                return;
            }

            motor->org_status = can_msg[1];
            motor->motor_status = can_msg[2];
            motor->valid_mask |=
                EMM42_CAN_MASK_ORG_STATUS | EMM42_CAN_MASK_MOTOR_STATUS;
        } break;

        default: {
            /* 其余为命令应答: 功能码 + 状态码 + 校验码 (手册 4.2.2 节),
             * 02 正确 / 12 已在零点 / E2 参数错误 / EE 格式错误 /
             * 9F 动作完成 */
            motor->ack_status = can_msg[1];
        } break;
    }
}

/**
 * @brief 初始化电机
 *
 * @param motor 电机句柄
 * @param can_select 选择哪一个 CAN 来通信
 * @param addr 电机地址, 必须小于 `EMM42_CAN_MOTOR_NUM_MAX`
 * @return 初始化状态:
 * @retval - 0: 成功
 * @retval - 1: 指针为空
 * @retval - 2: 地址超限
 * @retval - 3: 添加 CAN 接收表错误
 */
uint8_t emm42_can_motor_init(emm42_can_motor_t *motor,
                             can_selected_t can_select, uint8_t addr) {
    if (motor == NULL) {
        return 1;
    }

    if (addr >= EMM42_CAN_MOTOR_NUM_MAX) {
        return 2;
    }

    motor->can_select = can_select;
    motor->addr = addr;
    motor->microstep = EMM42_CAN_DEFAULT_MICROSTEP;
    motor->ack_status = 0;
    motor->cur_pos = 0.0f;
    motor->cur_vel = 0.0f;
    motor->pos_err = 0.0f;
    motor->org_status = 0;
    motor->motor_status = 0;
    motor->valid_mask = 0;

    if (can_list_add_new_node(can_select, (void *)motor, EMM42_CAN_ID(addr),
                              0x1FFFFFFF, CAN_ID_EXT, can_callback) != 0) {
        return 3;
    }

    return 0;
}

/**
 * @brief 反初始化电机
 *
 * @param motor 电机句柄
 * @return 反初始化状态:
 * @retval - 0: 成功
 * @retval - 1: `motor`为空
 * @retval - 2: 移除出错
 */
uint8_t emm42_can_motor_deinit(emm42_can_motor_t *motor) {
    if (motor == NULL) {
        return 1;
    }

    if (can_list_del_node_by_id(motor->can_select, CAN_ID_EXT,
                                EMM42_CAN_ID(motor->addr)) != 0) {
        return 2;
    }

    return 0;
}

/**
 * @brief 重启电机
 *
 * @param motor 电机句柄
 */
void emm42_can_reset_motor(emm42_can_motor_t *motor) {
    uint8_t cmd[3] = {0x08, 0x97, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 将当前位置角度清零
 *
 * @param motor 电机句柄
 */
void emm42_can_reset_curpos_to_zero(emm42_can_motor_t *motor) {
    uint8_t cmd[3] = {0x0A, 0x6D, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 电机使能控制
 *
 * @param motor 电机句柄
 * @param state 使能状态, false 松轴 / true 锁轴
 * @param snF 同步标志, false 立即执行 / true 缓存等待多机同步
 */
void emm42_can_en_control(emm42_can_motor_t *motor, bool state, bool snF) {
    /* 注意: 手册 V1.0.3 辅助码为 0xAB, 此处临时改为 0xF3 做通信测试 */
    uint8_t cmd[5] = {0xF3, 0xF3, (uint8_t)state, (uint8_t)snF,
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 速度模式控制
 *
 * @param motor 电机句柄
 * @param dir 方向, 0 = CW, 1 = CCW
 * @param vel 速度, 0 - 3000 (RPM)
 * @param acc 加速度档位, 0 - 255, 0 为直接启动
 * @param snF 同步标志, false 立即执行 / true 缓存等待多机同步
 */
void emm42_can_vel_control(emm42_can_motor_t *motor, uint8_t dir, uint16_t vel,
                           uint8_t acc, bool snF) {
    uint8_t cmd[7] = {0xF6,
                      dir,
                      (uint8_t)(vel >> 8),
                      (uint8_t)(vel >> 0),
                      acc,
                      (uint8_t)snF,
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 位置模式控制
 *
 * @param motor 电机句柄
 * @param dir 方向, 0 = CW, 1 = CCW
 * @param vel 速度, 0 - 3000 (RPM)
 * @param acc 加速度档位, 0 - 255, 0 为直接启动
 * @param degree 目标角度, 度 (按句柄 `microstep` 细分换算脉冲)
 * @param mode 运动模式, 0 = 相对上一输入目标位置,
 *             1 = 相对坐标零点绝对位置, 2 = 相对当前实时位置
 * @param snF 同步标志, false 立即执行 / true 缓存等待多机同步
 */
void emm42_can_pos_control(emm42_can_motor_t *motor, uint8_t dir, uint16_t vel,
                           uint8_t acc, float degree, uint8_t mode, bool snF) {
    if (motor == NULL) {
        return;
    }

    float pulse_per_rev =
        (float)EMM42_CAN_STEPS_PER_REV * (float)motor->microstep;
    uint32_t clk = (uint32_t)(degree / 360.0f * pulse_per_rev);

    uint8_t cmd[12] = {0xFD,
                       dir,
                       (uint8_t)(vel >> 8),
                       (uint8_t)(vel >> 0),
                       acc,
                       (uint8_t)(clk >> 24),
                       (uint8_t)(clk >> 16),
                       (uint8_t)(clk >> 8),
                       (uint8_t)(clk >> 0),
                       mode,
                       (uint8_t)snF,
                       EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 位置模式控制 (脉冲直传)
 *
 * @param motor 电机句柄
 * @param dir 方向, 0 = CW, 1 = CCW
 * @param vel 速度, 0 - 3000 (RPM)
 * @param acc 加速度档位, 0 - 255, 0 为直接启动
 * @param pulse 目标脉冲数 (16细分下 3200 脉冲/圈)
 * @param mode 运动模式, 0 = 相对上一输入目标位置,
 *             1 = 相对坐标零点绝对位置, 2 = 相对当前实时位置
 * @param snF 同步标志, false 立即执行 / true 缓存等待多机同步
 */
void emm42_can_pos_control_pulse(emm42_can_motor_t *motor, uint8_t dir,
                                 uint16_t vel, uint8_t acc, uint32_t pulse,
                                 uint8_t mode, bool snF) {
    uint8_t cmd[12] = {0xFD,
                       dir,
                       (uint8_t)(vel >> 8),
                       (uint8_t)(vel >> 0),
                       acc,
                       (uint8_t)(pulse >> 24),
                       (uint8_t)(pulse >> 16),
                       (uint8_t)(pulse >> 8),
                       (uint8_t)(pulse >> 0),
                       mode,
                       (uint8_t)snF,
                       EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 立即停止
 *
 * @param motor 电机句柄
 * @param snF 同步标志, false 立即执行 / true 缓存等待多机同步
 */
void emm42_can_stop_now(emm42_can_motor_t *motor, bool snF) {
    uint8_t cmd[4] = {0xFE, 0x98, (uint8_t)snF, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 触发回零
 *
 * @param motor 电机句柄
 * @param o_mode 回零模式, 0 - 5, 见手册 5.4.2 节
 * @param snF 同步标志, false 立即执行 / true 缓存等待多机同步
 */
void emm42_can_origin_trigger_return(emm42_can_motor_t *motor, uint8_t o_mode,
                                     bool snF) {
    uint8_t cmd[4] = {0x9A, o_mode, (uint8_t)snF, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 读取系统参数
 *
 * 实时位置/转速/位置误差/状态标志的返回由回调自动更新到电机句柄,
 * 其余参数的返回内容需在回调中按需扩展解析.
 *
 * @param motor 电机句柄
 * @param param 参数功能码, 见 `emm42_can_sys_params_t`
 */
void emm42_can_read_sys_params(emm42_can_motor_t *motor,
                               emm42_can_sys_params_t param) {
    uint8_t cmd[2] = {(uint8_t)param, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 定时返回信息, 电机按周期主动上报参数 (手册 5.5.1 节)
 *
 * @param motor 电机句柄
 * @param param 参数功能码, 见 `emm42_can_sys_params_t`
 * @param ms 定时时间, 毫秒; 0 为停止返回
 */
void emm42_can_set_auto_report(emm42_can_motor_t *motor,
                               emm42_can_sys_params_t param, uint16_t ms) {
    uint8_t cmd[6] = {0x11,
                      0x18,
                      (uint8_t)param,
                      (uint8_t)(ms >> 8),
                      (uint8_t)(ms >> 0),
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/* ======================== 触发动作命令 (手册 5.2 节) ======================== */

/**
 * @brief 触发编码器校准
 *
 * 闭环模式下电机缓转正一圈再反一圈做线性化校准, 校准前需空载.
 *
 * @param motor 电机句柄
 */
void emm42_can_calibrate_encoder(emm42_can_motor_t *motor) {
    uint8_t cmd[3] = {0x06, 0x45, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 解除堵转/过热/过流保护
 *
 * @param motor 电机句柄
 */
void emm42_can_release_protect(emm42_can_motor_t *motor) {
    uint8_t cmd[3] = {0x0E, 0x52, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 恢复出厂设置
 *
 * 恢复后需断电重上电, 并空载重新校准编码器.
 *
 * @param motor 电机句柄
 */
void emm42_can_factory_reset(emm42_can_motor_t *motor) {
    uint8_t cmd[3] = {0x0F, 0x5F, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/* ======================== 运动控制命令 (手册 5.3 节) ======================== */

/**
 * @brief 触发多机同步运动
 *
 * 所有之前收到同步标志 snF = true (缓存命令) 的电机同步开始执行,
 * 通常用广播地址 0 的句柄调用.
 *
 * @param motor 电机句柄
 */
void emm42_can_sync_motion(emm42_can_motor_t *motor) {
    uint8_t cmd[3] = {0xFF, 0x66, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 发送一条原始命令帧
 *
 * 数据内容为功能码 + 命令数据 + 校验码 (不含地址),
 * 大于 8 字节自动拆包; 可用于组多电机命令 (0xAA, 手册 5.3.1 节).
 *
 * @param motor 电机句柄
 * @param buf 命令数据
 * @param len 数据长度
 */
void emm42_can_send_frame(emm42_can_motor_t *motor, const uint8_t *buf,
                          uint16_t len) {
    send_command(motor, buf, len);
}

/* ======================== 原点回零命令 (手册 5.4 节) ======================== */

/**
 * @brief 设置单圈回零的零点位置
 *
 * @param motor 电机句柄
 * @param store 是否存储, false 不存 / true 存储 (掉电不丢失)
 */
void emm42_can_origin_set_single_turn_zero(emm42_can_motor_t *motor,
                                           bool store) {
    uint8_t cmd[4] = {0x93, 0x88, (uint8_t)store, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 强制中断并退出回零操作
 *
 * @param motor 电机句柄
 */
void emm42_can_origin_abort(emm42_can_motor_t *motor) {
    uint8_t cmd[3] = {0x9C, 0x48, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改回零参数
 *
 * @param motor 电机句柄
 * @param store 是否存储, false 不存 / true 存储 (掉电不丢失)
 * @param params 回零参数, 见 `emm42_can_origin_params_t`
 */
void emm42_can_origin_set_params(emm42_can_motor_t *motor, bool store,
                                 const emm42_can_origin_params_t *params) {
    if (params == NULL) {
        return;
    }

    uint8_t cmd[19] = {0x4C,
                       0xAE,
                       (uint8_t)store,
                       params->mode,
                       params->dir,
                       (uint8_t)(params->vel >> 8),
                       (uint8_t)(params->vel >> 0),
                       (uint8_t)(params->timeout_ms >> 24),
                       (uint8_t)(params->timeout_ms >> 16),
                       (uint8_t)(params->timeout_ms >> 8),
                       (uint8_t)(params->timeout_ms >> 0),
                       (uint8_t)(params->bump_vel >> 8),
                       (uint8_t)(params->bump_vel >> 0),
                       (uint8_t)(params->bump_ma >> 8),
                       (uint8_t)(params->bump_ma >> 0),
                       (uint8_t)(params->bump_ms >> 8),
                       (uint8_t)(params->bump_ms >> 0),
                       params->o_pot_en,
                       EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/* ======================== 读写驱动参数 (手册 5.6 节) ======================== */

/**
 * @brief 修改电机 ID/地址
 *
 * @note 修改后电机应答 ID 随之改变, 需先 `emm42_can_motor_deinit`
 *       再用新地址重新 `emm42_can_motor_init`.
 *
 * @param motor 电机句柄
 * @param store 是否存储, false 不存 / true 存储 (掉电不丢失)
 * @param new_id 新地址 1 - 255
 */
void emm42_can_set_id(emm42_can_motor_t *motor, bool store, uint8_t new_id) {
    uint8_t cmd[5] = {0xAE, 0x4B, (uint8_t)store, new_id,
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改细分值
 *
 * 同步更新句柄 `microstep` 字段, `emm42_can_pos_control` 按新细分换算.
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param microstep 细分 1 - 255, 0 = 256 细分
 */
void emm42_can_set_microstep(emm42_can_motor_t *motor, bool store,
                             uint8_t microstep) {
    if (motor == NULL) {
        return;
    }

    motor->microstep = (microstep == 0) ? 256 : microstep;

    uint8_t cmd[5] = {0x84, 0x8A, (uint8_t)store, microstep,
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改掉电标志
 *
 * @param motor 电机句柄
 * @param flag 掉电标志 0 / 1, 写 0 后若发生掉电再上会恢复 1
 */
void emm42_can_set_power_loss_flag(emm42_can_motor_t *motor, uint8_t flag) {
    uint8_t cmd[3] = {0x50, flag, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改电机类型
 *
 * @note 修改后需重新空载校准编码器.
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param type 电机类型 0x19 = 1.8° / 0x32 = 0.9°
 */
void emm42_can_set_motor_type(emm42_can_motor_t *motor, bool store,
                              uint8_t type) {
    uint8_t cmd[5] = {0xD7, 0x35, (uint8_t)store, type,
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改固件类型
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param fw 固件类型, 0 = X 固件, 1 = Emm 固件, 2 = Emm 狂暴模式
 */
void emm42_can_set_firmware(emm42_can_motor_t *motor, bool store, uint8_t fw) {
    uint8_t cmd[5] = {0xD5, 0x69, (uint8_t)store, fw, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改开环/闭环控制模式
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param closed_loop false 开环 / true 闭环
 */
void emm42_can_set_ctrl_mode(emm42_can_motor_t *motor, bool store,
                             bool closed_loop) {
    uint8_t cmd[5] = {0x46, 0xA6, (uint8_t)store, (uint8_t)closed_loop,
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改电机运动正方向
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param dir 正方向, 0 = CW, 1 = CCW
 */
void emm42_can_set_direction(emm42_can_motor_t *motor, bool store,
                             uint8_t dir) {
    uint8_t cmd[5] = {0xD4, 0x60, (uint8_t)store, dir, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改锁定按键功能
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param lock false 解锁 / true 锁定 (锁定后除恢复出厂外按键无效)
 */
void emm42_can_set_button_lock(emm42_can_motor_t *motor, bool store,
                               bool lock) {
    uint8_t cmd[5] = {0xD0, 0xB3, (uint8_t)store, (uint8_t)lock,
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改命令速度值是否缩小 10 倍输入 (Emm)
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param en false 关闭 / true 使能 (使能后速度精确到 0.1 RPM)
 */
void emm42_can_set_speed_scale(emm42_can_motor_t *motor, bool store, bool en) {
    uint8_t cmd[5] = {0x4F, 0x71, (uint8_t)store, (uint8_t)en,
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改开环模式工作电流
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param ma 工作电流, 0 - 5000 (mA)
 */
void emm42_can_set_open_current(emm42_can_motor_t *motor, bool store,
                                uint16_t ma) {
    uint8_t cmd[6] = {0x44,
                      0x33,
                      (uint8_t)store,
                      (uint8_t)(ma >> 8),
                      (uint8_t)(ma >> 0),
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改闭环模式最大电流 (Emm 固件下为堵转最大电流)
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param ma 最大电流, 0 - 5000 (mA)
 */
void emm42_can_set_closed_current(emm42_can_motor_t *motor, bool store,
                                  uint16_t ma) {
    uint8_t cmd[6] = {0x45,
                      0x66,
                      (uint8_t)store,
                      (uint8_t)(ma >> 8),
                      (uint8_t)(ma >> 0),
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改 PID 参数 (Emm)
 *
 * @note 发送 16 字节, 自动拆包.
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void emm42_can_set_pid(emm42_can_motor_t *motor, bool store, uint32_t kp,
                       uint32_t ki, uint32_t kd) {
    uint8_t cmd[16] = {0x4A,
                       0xC3,
                       (uint8_t)store,
                       (uint8_t)(kp >> 24),
                       (uint8_t)(kp >> 16),
                       (uint8_t)(kp >> 8),
                       (uint8_t)(kp >> 0),
                       (uint8_t)(ki >> 24),
                       (uint8_t)(ki >> 16),
                       (uint8_t)(ki >> 8),
                       (uint8_t)(ki >> 0),
                       (uint8_t)(kd >> 24),
                       (uint8_t)(kd >> 16),
                       (uint8_t)(kd >> 8),
                       (uint8_t)(kd >> 0),
                       EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 读取 DMX512 协议参数
 *
 * @note 返回 17 字节, 多包返回暂不重组解析.
 *
 * @param motor 电机句柄
 */
void emm42_can_read_dmx_params(emm42_can_motor_t *motor) {
    uint8_t cmd[3] = {0x49, 0x78, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改 DMX512 协议参数
 *
 * @note 发送 18 字节, 自动拆包.
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param params DMX512 参数, 见 `emm42_can_dmx_params_t`
 */
void emm42_can_set_dmx_params(emm42_can_motor_t *motor, bool store,
                              const emm42_can_dmx_params_t *params) {
    if (params == NULL) {
        return;
    }

    uint8_t cmd[18] = {0xD9,
                       0x90,
                       (uint8_t)store,
                       (uint8_t)(params->total_channels >> 8),
                       (uint8_t)(params->total_channels >> 0),
                       params->motor_channels,
                       params->motion_mode,
                       (uint8_t)(params->vel >> 8),
                       (uint8_t)(params->vel >> 0),
                       (uint8_t)(params->acc >> 8),
                       (uint8_t)(params->acc >> 0),
                       (uint8_t)(params->vel_step >> 8),
                       (uint8_t)(params->vel_step >> 0),
                       (uint8_t)(params->move_step >> 24),
                       (uint8_t)(params->move_step >> 16),
                       (uint8_t)(params->move_step >> 8),
                       (uint8_t)(params->move_step >> 0),
                       EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改位置到达窗口
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param window 位置到达窗口 (缩小 10 倍, 8 = 0.8°)
 */
void emm42_can_set_arrive_window(emm42_can_motor_t *motor, bool store,
                                 uint16_t window) {
    uint8_t cmd[6] = {0xD1,
                      0x07,
                      (uint8_t)store,
                      (uint8_t)(window >> 8),
                      (uint8_t)(window >> 0),
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改过热过流保护检测阈值
 *
 * @note 发送 10 字节, 自动拆包.
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param temp 过热保护阈值, ℃
 * @param ma 过流保护阈值, mA
 * @param ms 检测时间, ms
 */
void emm42_can_set_protect_threshold(emm42_can_motor_t *motor, bool store,
                                     uint16_t temp, uint16_t ma,
                                     uint16_t ms) {
    uint8_t cmd[10] = {0xD3,
                       0x56,
                       (uint8_t)store,
                       (uint8_t)(temp >> 8),
                       (uint8_t)(temp >> 0),
                       (uint8_t)(ma >> 8),
                       (uint8_t)(ma >> 0),
                       (uint8_t)(ms >> 8),
                       (uint8_t)(ms >> 0),
                       EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改心跳保护功能时间
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param ms 心跳时间, ms; 0 为关闭, 超时未收到正确命令则急停
 */
void emm42_can_set_heartbeat(emm42_can_motor_t *motor, bool store,
                             uint32_t ms) {
    uint8_t cmd[8] = {0x68,
                      0x38,
                      (uint8_t)store,
                      (uint8_t)(ms >> 24),
                      (uint8_t)(ms >> 16),
                      (uint8_t)(ms >> 8),
                      (uint8_t)(ms >> 0),
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改积分限幅/刚性系数
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param value 积分限幅 (Emm, 默认 65535); 值过大电机会震颤
 */
void emm42_can_set_integral_limit(emm42_can_motor_t *motor, bool store,
                                  uint32_t value) {
    uint8_t cmd[8] = {0x4B,
                      0x57,
                      (uint8_t)store,
                      (uint8_t)(value >> 24),
                      (uint8_t)(value >> 16),
                      (uint8_t)(value >> 8),
                      (uint8_t)(value >> 0),
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改碰撞回零返回角度
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param angle 返回角度, 单位 0.1°; 0 = 按电流检测返回
 */
void emm42_can_set_origin_return_angle(emm42_can_motor_t *motor, bool store,
                                       uint16_t angle) {
    uint8_t cmd[6] = {0x5C,
                      0xAC,
                      (uint8_t)store,
                      (uint8_t)(angle >> 8),
                      (uint8_t)(angle >> 0),
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 广播读取 ID 地址
 *
 * 用于忘记地址时单独接线查询, 应答功能码 0x15,
 * 地址落在句柄的 `ack_status` 字段 (需已注册任意地址的句柄接收).
 *
 * @param can_select 选择哪一个 CAN 发送
 */
void emm42_can_broadcast_read_id(can_selected_t can_select) {
    uint8_t cmd[2] = {0x15, EMM42_CAN_CHECK_BYTE};
    can_send_message(can_select, CAN_ID_EXT, EMM42_CAN_ID(0), sizeof(cmd),
                     cmd);
}

/**
 * @brief 修改锁定修改参数功能
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param level 锁定等级 0 - 3, 0 解锁 / 1 禁改通讯参数 /
 *              2, 3 禁改所有参数与触发校准
 */
void emm42_can_set_param_lock(emm42_can_motor_t *motor, bool store,
                              uint8_t level) {
    uint8_t cmd[5] = {0xD6, 0x4B, (uint8_t)store, level,
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/* ================== 上电自动运行 (手册 5.7 节, Emm 固件) ================== */

/**
 * @brief 存储一组速度参数, 上电自动运行 (Emm)
 *
 * @note 发送 9 字节, 自动拆包.
 *
 * @param motor 电机句柄
 * @param store false 清除 / true 存储
 * @param dir 方向, 0 = CW, 1 = CCW
 * @param vel 速度, 0 - 3000 (RPM)
 * @param acc 加速度档位, 0 - 255
 * @param en_ctrl En 引脚控制启停, false 关 / true 开
 */
void emm42_can_set_auto_run(emm42_can_motor_t *motor, bool store, uint8_t dir,
                            uint16_t vel, uint8_t acc, bool en_ctrl) {
    uint8_t cmd[9] = {0xF7,
                      0x1C,
                      (uint8_t)store,
                      dir,
                      (uint8_t)(vel >> 8),
                      (uint8_t)(vel >> 0),
                      acc,
                      (uint8_t)en_ctrl,
                      EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/* ================== 读取/修改所有驱动参数 (手册 5.8 节, Emm 固件) ================== */

/**
 * @brief 读取系统状态参数 (Emm)
 *
 * @note 返回 31 字节, 多包返回暂不重组解析.
 *
 * @param motor 电机句柄
 */
void emm42_can_read_system_status(emm42_can_motor_t *motor) {
    uint8_t cmd[3] = {0x43, 0x7A, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 读取驱动配置参数 (Emm)
 *
 * @note 返回 33 字节, 多包返回暂不重组解析.
 *
 * @param motor 电机句柄
 */
void emm42_can_read_config(emm42_can_motor_t *motor) {
    uint8_t cmd[3] = {0x42, 0x6C, EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}

/**
 * @brief 修改驱动配置参数 (Emm)
 *
 * @note 发送 32 字节, 自动拆包.
 *
 * @param motor 电机句柄
 * @param store 是否存储
 * @param config 驱动配置参数, 见 `emm42_can_config_t`
 */
void emm42_can_write_config(emm42_can_motor_t *motor, bool store,
                            const emm42_can_config_t *config) {
    if (config == NULL) {
        return;
    }

    uint8_t cmd[32] = {0x48,
                       0xD1,
                       (uint8_t)store,
                       config->motor_type,
                       config->pulse_mux,
                       config->comm_mux,
                       config->en_level,
                       config->dir_level,
                       config->microstep,
                       config->interp,
                       0x00,
                       (uint8_t)(config->open_ma >> 8),
                       (uint8_t)(config->open_ma >> 0),
                       (uint8_t)(config->closed_ma >> 8),
                       (uint8_t)(config->closed_ma >> 0),
                       (uint8_t)(config->max_voltage >> 8),
                       (uint8_t)(config->max_voltage >> 0),
                       config->uart_baud,
                       config->can_baud,
                       0x00,
                       config->checksum,
                       config->ack_mode,
                       config->stall_en,
                       (uint8_t)(config->stall_vel >> 8),
                       (uint8_t)(config->stall_vel >> 0),
                       (uint8_t)(config->stall_ma >> 8),
                       (uint8_t)(config->stall_ma >> 0),
                       (uint8_t)(config->stall_ms >> 8),
                       (uint8_t)(config->stall_ms >> 0),
                       (uint8_t)(config->arrive_win >> 8),
                       (uint8_t)(config->arrive_win >> 0),
                       EMM42_CAN_CHECK_BYTE};
    send_command(motor, cmd, sizeof(cmd));
}
