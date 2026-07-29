/**
 * @file    emm42.c
 * @author  Jackrainman
 * @brief   ZDT X42S Emm42 闭环步进电机驱动 (RS485)
 * @version 1.0
 * @date    2026-07-29
 */

#include "emm42.h"

/* 电机注册表, 按地址索引 */
static emm42_motor_t *emm42_motor_list[EMM42_MOTOR_NUM_MAX] = {NULL};

/**
 * @brief RS485 进入发送模式
 *
 * @param motor 电机句柄
 */
static void rs485_tx(emm42_motor_t *motor) {
    HAL_GPIO_WritePin(motor->rs485_port, motor->rs485_pin, GPIO_PIN_SET);
}

/**
 * @brief RS485 进入接收模式
 *
 * @param motor 电机句柄
 */
static void rs485_rx(emm42_motor_t *motor) {
    HAL_GPIO_WritePin(motor->rs485_port, motor->rs485_pin, GPIO_PIN_RESET);
}

/**
 * @brief 发送一帧数据 (阻塞发送, 完成后切回接收模式)
 *
 * @param motor 电机句柄
 * @param buf 数据帧
 * @param len 帧长度
 */
static void send_frame(emm42_motor_t *motor, const uint8_t *buf, uint16_t len) {
    rs485_tx(motor);
    HAL_UART_Transmit(motor->huart, (uint8_t *)buf, len, 100);
    rs485_rx(motor);
}

/**
 * @brief 初始化电机
 *
 * @param motor 电机句柄
 * @param huart 所在 RS485 总线的串口
 * @param addr 电机地址, 必须小于 `EMM42_MOTOR_NUM_MAX`
 * @param rs485_port RS485 RE 方向脚端口
 * @param rs485_pin RS485 RE 方向脚引脚
 * @return 初始化状态:
 * @retval - 0: 成功
 * @retval - 1: 指针为空
 * @retval - 2: 地址超限
 */
uint8_t emm42_motor_init(emm42_motor_t *motor, UART_HandleTypeDef *huart,
                         uint8_t addr, GPIO_TypeDef *rs485_port,
                         uint16_t rs485_pin) {
    if (motor == NULL || huart == NULL || rs485_port == NULL) {
        return 1;
    }

    if (addr >= EMM42_MOTOR_NUM_MAX) {
        return 2;
    }

    motor->huart = huart;
    motor->addr = addr;
    motor->rs485_port = rs485_port;
    motor->rs485_pin = rs485_pin;
    motor->cur_pos = 0.0f;
    motor->cur_vel = 0.0f;
    motor->valid_mask = 0;

    emm42_motor_list[addr] = motor;

    return 0;
}

/**
 * @brief 反初始化电机
 *
 * @param motor 电机句柄
 * @return 反初始化状态:
 * @retval - 0: 成功
 * @retval - 1: `motor`为空
 */
uint8_t emm42_motor_deinit(emm42_motor_t *motor) {
    if (motor == NULL) {
        return 1;
    }

    if (motor->addr < EMM42_MOTOR_NUM_MAX &&
        emm42_motor_list[motor->addr] == motor) {
        emm42_motor_list[motor->addr] = NULL;
    }

    return 0;
}

/**
 * @brief 重启电机
 *
 * @param motor 电机句柄
 */
void emm42_reset_motor(emm42_motor_t *motor) {
    uint8_t cmd[4] = {motor->addr, 0x08, 0x97, 0x6B};
    send_frame(motor, cmd, sizeof(cmd));
}

/**
 * @brief 将当前位置角度清零
 *
 * @param motor 电机句柄
 */
void emm42_reset_curpos_to_zero(emm42_motor_t *motor) {
    uint8_t cmd[4] = {motor->addr, 0x0A, 0x6D, 0x6B};
    send_frame(motor, cmd, sizeof(cmd));
}

/**
 * @brief 电机使能控制
 *
 * @param motor 电机句柄
 * @param state 使能状态, false 松轴 / true 锁轴
 * @param snF 同步标志, false 立即执行 / true 缓存等待多机同步
 */
void emm42_en_control(emm42_motor_t *motor, bool state, bool snF) {
    uint8_t cmd[6] = {motor->addr, 0xF3, 0xAB, (uint8_t)state, (uint8_t)snF,
                      0x6B};
    send_frame(motor, cmd, sizeof(cmd));
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
void emm42_vel_control(emm42_motor_t *motor, uint8_t dir, uint16_t vel,
                       uint8_t acc, bool snF) {
    uint8_t cmd[8] = {motor->addr,
                      0xF6,
                      dir,
                      (uint8_t)(vel >> 8),
                      (uint8_t)(vel >> 0),
                      acc,
                      (uint8_t)snF,
                      0x6B};
    send_frame(motor, cmd, sizeof(cmd));
}

/**
 * @brief 位置模式控制
 *
 * @param motor 电机句柄
 * @param dir 方向, 0 = CW, 1 = CCW
 * @param vel 速度, 0 - 3000 (RPM)
 * @param acc 加速度档位, 0 - 255, 0 为直接启动
 * @param degree 目标角度, 度 (按 16 细分 3200 脉冲/圈换算)
 * @param mode 运动模式, 0 = 相对上一输入目标位置,
 *             1 = 相对坐标零点绝对位置, 2 = 相对当前实时位置
 * @param snF 同步标志, false 立即执行 / true 缓存等待多机同步
 */
void emm42_pos_control(emm42_motor_t *motor, uint8_t dir, uint16_t vel,
                       uint8_t acc, float degree, uint8_t mode, bool snF) {
    uint32_t clk = (uint32_t)(degree / 360.0f * 3200.0f);

    uint8_t cmd[13] = {motor->addr,
                       0xFD,
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
                       0x6B};
    send_frame(motor, cmd, sizeof(cmd));
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
void emm42_pos_control_pulse(emm42_motor_t *motor, uint8_t dir, uint16_t vel,
                             uint8_t acc, uint32_t pulse, uint8_t mode, bool snF) {
    uint8_t cmd[13] = {motor->addr,
                       0xFD,
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
                       0x6B};
    send_frame(motor, cmd, sizeof(cmd));
}

/**
 * @brief 立即停止
 *
 * @param motor 电机句柄
 * @param snF 同步标志, false 立即执行 / true 缓存等待多机同步
 */
void emm42_stop_now(emm42_motor_t *motor, bool snF) {
    uint8_t cmd[5] = {motor->addr, 0xFE, 0x98, (uint8_t)snF, 0x6B};
    send_frame(motor, cmd, sizeof(cmd));
}

/**
 * @brief 触发回零
 *
 * @param motor 电机句柄
 * @param o_mode 回零模式, 0 - 5, 见手册 5.4.2 节
 * @param snF 同步标志, false 立即执行 / true 缓存等待多机同步
 */
void emm42_origin_trigger_return(emm42_motor_t *motor, uint8_t o_mode,
                                 bool snF) {
    uint8_t cmd[5] = {motor->addr, 0x9A, o_mode, (uint8_t)snF, 0x6B};
    send_frame(motor, cmd, sizeof(cmd));
}

/**
 * @brief 读取系统参数
 *
 * @param motor 电机句柄
 * @param param 参数功能码, 见 `emm42_sys_params_t`
 */
void emm42_read_sys_params(emm42_motor_t *motor, emm42_sys_params_t param) {
    uint8_t cmd[3] = {motor->addr, (uint8_t)param, 0x6B};
    send_frame(motor, cmd, sizeof(cmd));
}

/**
 * @brief 定时返回信息, 电机按周期主动上报参数 (手册 5.5.1 节)
 *
 * @param motor 电机句柄
 * @param param 参数功能码, 见 `emm42_sys_params_t`
 * @param ms 定时时间, 毫秒; 0 为停止返回
 */
void emm42_set_auto_report(emm42_motor_t *motor, emm42_sys_params_t param,
                           uint16_t ms) {
    uint8_t cmd[7] = {motor->addr,
                      0x11,
                      0x18,
                      (uint8_t)param,
                      (uint8_t)(ms >> 8),
                      (uint8_t)(ms >> 0),
                      0x6B};
    send_frame(motor, cmd, sizeof(cmd));
}

/**
 * @brief 应答帧解析, 按地址更新注册电机的状态
 *
 * 在应用任务中对收到的数据调用, 例如 `uart_dmarx_read()` 读到一帧后.
 *
 * @param buf 一帧数据 (以 0x6B 结尾)
 * @param len 帧长度
 * @return 解析结果:
 * @retval - true: 帧合法
 * @retval - false: 帧非法或无对应电机
 */
bool emm42_frame_process(const uint8_t *buf, uint16_t len) {
    if (buf == NULL || len < 3 || buf[len - 1] != 0x6B) {
        return false;
    }

    uint8_t addr = buf[0];
    uint8_t func = buf[1];

    if (addr >= EMM42_MOTOR_NUM_MAX || emm42_motor_list[addr] == NULL) {
        return false;
    }

    emm42_motor_t *motor = emm42_motor_list[addr];

    switch (func) {
        case 0x36: {
            /* 读取电机实时位置返回: 符号 + 4 字节位置 */
            if (len != 8) {
                return false;
            }

            uint32_t pos_raw = ((uint32_t)buf[3] << 24) |
                               ((uint32_t)buf[4] << 16) |
                               ((uint32_t)buf[5] << 8) | ((uint32_t)buf[6]);
            int32_t pos_signed =
                (buf[2] == 0x01) ? -(int32_t)pos_raw : (int32_t)pos_raw;

#if (EMM42_USE_X_FIRMWARE == 1)
            motor->cur_pos = (float)pos_signed / 10.0f;
#else
            motor->cur_pos = (float)pos_signed * 360.0f / 65536.0f;
#endif /* EMM42_USE_X_FIRMWARE == 1 */
            motor->valid_mask |= EMM42_MASK_REAL_POS;
        } break;

        case 0x35: {
            /* 读取电机实时转速返回: 符号 + 2 字节转速 */
            if (len != 6) {
                return false;
            }

            uint16_t vel_raw = ((uint16_t)buf[3] << 8) | ((uint16_t)buf[4]);

#if (EMM42_USE_X_FIRMWARE == 1)
            float vel = (float)vel_raw / 10.0f;
#else
            float vel = (float)vel_raw;
#endif /* EMM42_USE_X_FIRMWARE == 1 */
            motor->cur_vel = (buf[2] == 0x01) ? -vel : vel;
            motor->valid_mask |= EMM42_MASK_SPEED;
        } break;

        default: {
            /* 其余为命令应答, 检查返回码 (手册第 4 章):
             * 02 正确 / 12 已在零点 / E2 参数错误 / EE 格式错误 /
             * 9F 动作完成 */
            if (len < 4) {
                return false;
            }
        } break;
    }

    return true;
}
