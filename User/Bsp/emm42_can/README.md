# ZDT X42S Emm42 闭环步进电机驱动 (CAN)

对齐《ZDT_X42S 第二代闭环步进电机用户手册 V1.0.3》第 4.2 节 CAN 通讯协议：

- 固定使用**扩展帧**，ID = `(Addr << 8) | Packet`，`Packet` 从 0 开始计数；
- 数据内容为 `功能码 + 命令数据 + 校验码`，不再包含地址字节（地址在 ID 中）；
- 校验码出厂默认固定 `0x6B`（`EMM42_CAN_CHECK_BYTE`）；
- 大于 8 字节的命令（如位置模式 0xFD）自动拆包发送；
- 电机返回为单帧（查询返回均 ≤ 8 字节），回调中直接解析，不做多包重组。

电机端需将通讯端口复用为 CAN（`CAN1_MAP`），并保证 CAN 速率与 FDCAN 外设配置一致（默认 500K）。

# 依赖

- `can_list`（CAN 收发 + 按 ID 分派回调）

# 使用

1. 将 `emm42_can.c` 添加到工程的 `Bsp` 分组中
2. 在 `bsp.h` 中包含 `emm42_can/emm42_can.h`
3. 若电机刷的是 X 固件，将 `EMM42_CAN_USE_X_FIRMWARE` 置 1 切换换算方式
4. `can_list` 在 RTOS 模式（`CAN_LIST_USE_RTOS = 1`）下，电机初始化（注册回调）必须在调度器启动后进行

# API

## 结构体对象

`emm42_can_motor_t` 包括电机配置与实时状态：

```
typedef struct {
    can_selected_t can_select; /*!< 选择 CAN 通信 */
    uint8_t addr;              /*!< 电机地址 1-255, 0 为广播地址 */

    uint8_t ack_status; /*!< 最近一次命令应答状态 (02 正确 / 9F 完成 / E2 / EE) */

    float cur_pos; /*!< 实时位置, 度 */
    float cur_vel; /*!< 实时转速, RPM */
    float pos_err; /*!< 位置误差, 度 */

    uint8_t org_status;   /*!< 回零状态标志, 见 EMM42_CAN_ORG_* */
    uint8_t motor_status; /*!< 电机状态标志, 见 EMM42_CAN_ST_* */

    uint32_t valid_mask; /*!< 字段更新掩码, 见 EMM42_CAN_MASK_* */
} emm42_can_motor_t;
```

## 函数方法

- `emm42_can_motor_init` / `emm42_can_motor_deinit` 初始化/反初始化电机（自动注册 can_list 回调）

### 触发动作（手册 5.2 节）

- `emm42_can_calibrate_encoder` 触发编码器校准
- `emm42_can_reset_motor` 重启电机
- `emm42_can_reset_curpos_to_zero` 当前位置清零
- `emm42_can_release_protect` 解除堵转/过热/过流保护
- `emm42_can_factory_reset` 恢复出厂设置

### 运动控制（手册 5.3 节）

- `emm42_can_en_control` 使能控制（松轴/锁轴）
- `emm42_can_vel_control` 速度模式
- `emm42_can_pos_control` 位置模式（按角度）
- `emm42_can_pos_control_pulse` 位置模式（脉冲直传）
- `emm42_can_stop_now` 立即停止
- `emm42_can_sync_motion` 触发多机同步运动
- `emm42_can_send_frame` 发送原始命令帧（可组 0xAA 多电机命令）

### 原点回零（手册 5.4 节）

- `emm42_can_origin_set_single_turn_zero` 设置单圈回零的零点位置
- `emm42_can_origin_trigger_return` 触发回零
- `emm42_can_origin_abort` 强制中断并退出回零
- `emm42_can_origin_set_params` 修改回零参数（结构体 `emm42_can_origin_params_t`）

### 读取系统参数（手册 5.5 节）

- `emm42_can_read_sys_params` 读取系统参数（枚举 `emm42_can_sys_params_t`，实时位置/转速返回自动更新到句柄）
- `emm42_can_set_auto_report` 定时主动上报参数

### 读写驱动参数（手册 5.6 节）

- `emm42_can_set_id` 修改电机地址（改后需 deinit 再以新地址重新 init）
- `emm42_can_set_microstep` 修改细分值
- `emm42_can_set_power_loss_flag` 修改掉电标志
- `emm42_can_set_motor_type` / `emm42_can_set_firmware` / `emm42_can_set_ctrl_mode` / `emm42_can_set_direction` 电机类型/固件类型/开闭环/正方向
- `emm42_can_set_button_lock` / `emm42_can_set_param_lock` 锁定按键/锁定修改参数
- `emm42_can_set_speed_scale` 命令速度值缩小 10 倍输入（Emm）
- `emm42_can_set_open_current` / `emm42_can_set_closed_current` 开环工作电流/闭环堵转最大电流
- `emm42_can_set_pid` 修改 PID 参数（Emm）
- `emm42_can_read_dmx_params` / `emm42_can_set_dmx_params` DMX512 参数（结构体 `emm42_can_dmx_params_t`）
- `emm42_can_set_arrive_window` 位置到达窗口
- `emm42_can_set_protect_threshold` 过热过流保护检测阈值
- `emm42_can_set_heartbeat` 心跳保护功能时间
- `emm42_can_set_integral_limit` 积分限幅/刚性系数
- `emm42_can_set_origin_return_angle` 碰撞回零返回角度
- `emm42_can_broadcast_read_id` 广播读取 ID 地址

### 上电自动运行 / 所有驱动参数（手册 5.7/5.8 节，Emm）

- `emm42_can_set_auto_run` 存储一组速度参数，上电自动运行
- `emm42_can_read_system_status` 读取系统状态参数
- `emm42_can_read_config` / `emm42_can_write_config` 读取/修改驱动配置参数（结构体 `emm42_can_config_t`）

**已知限制**：电机返回超过 8 字节时会拆包（Packet > 0），当前回调只处理单帧返回，
因此 PID（0x21）、回零参数（0x22）、DMX 参数（0x49）、系统状态（0x43）、驱动配置（0x42）
等长返回暂不重组解析；对应写入命令不受影响（发送侧已自动拆包）。
实时位置（0x36）、转速（0x35）、位置误差（0x37）、电机状态标志（0x3A）、
回零状态标志（0x3B）、回零+电机状态（0x3C）的返回均为单帧，回调已自动解析到句柄。

# 示例

```
/* 声明电机句柄 */
static emm42_can_motor_t motor_x;

void some_task(void *args) {
    /* 电机初始化, 地址 1, 使用 CAN1 通信 (调度器启动后调用) */
    emm42_can_motor_init(&motor_x, can1_selected, 1);

    /* 锁轴 */
    emm42_can_en_control(&motor_x, true, false);

    /* 位置模式: CW, 400RPM, 加速度 10, 相对当前位置转 90 度 */
    emm42_can_pos_control(&motor_x, 0, 400, 10, 90.0f, 2, false);

    while (1) {
        /* 轮询实时位置 */
        emm42_can_read_sys_params(&motor_x, EMM42_CAN_S_CPOS);
        vTaskDelay(20);

        if (motor_x.valid_mask & EMM42_CAN_MASK_REAL_POS) {
            motor_x.valid_mask &= ~EMM42_CAN_MASK_REAL_POS;
            /* 使用 motor_x.cur_pos ... */
        }
    }
}
```
