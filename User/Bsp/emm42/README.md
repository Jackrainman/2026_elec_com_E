# ZDT X42S Emm42 闭环步进电机驱动 (RS485)

协议见同目录手册 `ZDT_X42S第二代闭环步进电机用户手册V1.0.3_251224.pdf`。

# 依赖

- `usart_ex`（接收侧 DMA 读取）
- CubeMX 配置好 RS485 串口与 RE 方向脚

# 使用

日常只需四个环节：`emm42_motor_init` → `emm42_en_control` →
`emm42_vel_control` / `emm42_pos_control`，接收侧用 `emm42_frame_process`
解析应答。其余 API 按需取用，分组见下。

1. 将 `emm42.c` 添加到工程的 `Bsp` 分组中，在 `bsp.h` 中包含 `emm42.h`
2. 声明电机句柄并初始化（句柄内含串口、地址、RE 脚）
3. 在 `usart_ex.h` 中使能对应串口的 `UARTx_RX_DMA`
4. 应用任务中用 `uart_dmarx_read()` 读取数据后调用 `emm42_frame_process()` 解析，
   电机状态更新到句柄的 `cur_pos` / `cur_vel`，并置位 `valid_mask`
5. 若使用 X 固件，将 `EMM42_USE_X_FIRMWARE` 置 1

# API

## 常用控制

- `emm42_motor_init` / `emm42_motor_deinit` 初始化 / 反初始化电机
- `emm42_en_control` 使能控制
- `emm42_vel_control` 速度模式
- `emm42_pos_control` 位置模式（`mode`: 0 相对上一目标 / 1 相对零点绝对 / 2 相对当前位置）
- `emm42_stop_now` 立即停止
- `emm42_reset_curpos_to_zero` 当前位置清零
- `emm42_frame_process` 应答帧解析（实时位置/转速/位置误差/回零状态/
  电机状态/回零参数更新到句柄，并置位 `valid_mask`）

## 状态读取与上报

- `emm42_read_sys_params` 读取系统参数
- `emm42_set_auto_report` 定时返回信息（电机周期主动上报，`ms = 0` 停止）

## 回零

- `emm42_origin_set_zero` 设置单圈回零的零点位置
- `emm42_origin_trigger_return` 触发回零
- `emm42_origin_interrupt` 强制中断并退出回零
- `emm42_origin_read_params` / `emm42_origin_set_params` 读取 / 修改回零参数

## 多机

- `emm42_sync_motion` 触发多机同步运动（广播地址下发）
- `emm42_multi_cmd` 多电机命令（一帧打包多条子命令广播下发）

## 调试与产线

- `emm42_reset_motor` 重启电机
- `emm42_calibrate_encoder` 触发编码器校准
- `emm42_release_protection` 解除堵转/过热/过流保护
- `emm42_restore_factory` 恢复出厂设置
- `emm42_set_addr` 修改电机 ID/地址

# 示例

```c
/* 声明句柄并初始化: UART4 总线, 地址 1, RE 脚 */
emm42_motor_t motor1;
emm42_motor_init(&motor1, &huart4, 1, RS485_RE1_GPIO_Port, RS485_RE1_Pin);

/* 使能 + 速度模式 300RPM */
emm42_en_control(&motor1, true, false);
emm42_vel_control(&motor1, 0, 300, 0, false);

/* 周期上报实时位置与转速, 每 10ms 一次 */
emm42_set_auto_report(&motor1, EMM42_S_CPOS, 10);
emm42_set_auto_report(&motor1, EMM42_S_VEL, 10);

/* 接收任务中解析 */
uint8_t buf[64];
uint32_t len = uart_dmarx_read(&huart4, buf, sizeof(buf));
if (len > 0) {
    emm42_frame_process(buf, len);
    if (motor1.valid_mask & EMM42_MASK_REAL_POS) {
        motor1.valid_mask &= ~EMM42_MASK_REAL_POS;
        /* 使用 motor1.cur_pos */
    }
}
```
