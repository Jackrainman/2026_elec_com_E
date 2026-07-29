# 正点原子 步进电机驱动器 通信协议

基于正点原子步进电机驱动器的串口/ CAN 通信协议驱动，支持位置、速度、力矩等多种控制模式。

# 依赖

- `smd_usart` (串口通信) 或 `smd_can` (CAN 通信，可选)
- FreeRTOS (用于接收任务和队列)
- HAL 库 (UART/ CAN 外设)

# 使用

1. 将头文件复制到 `User/Bsp/Inc/` 中，源文件复制到 `User/Bsp/Src` 中
2. 将 `smd.c` 和 `smd_usart.c` (或 `smd_can.c`) 添加到工程的 `Bsp` 分组中
3. 在 `bsp.h` 中包含 `smd.h`
4. 在 `smd.h` 中通过 `COMM_TYPE` 宏选择通信方式（0=串口，1=CAN）
5. 初始化串口：调用 `smd_usart_recv_init()` 创建接收任务和队列
6. 初始化电机：使用 `smd_motor_init()` 创建电机句柄
7. 在 UART 接收回调中调用 `smd_uart_rx_cplt_callback()` (已在 `smd_usart.c` 中通过 `HAL_UARTEx_RxEventCallback` 自动调用)

# API

## 结构体对象

`smd_motor_t` 包括电机的各种状态参数，如位置、速度、电压等。使用前需要定义电机的句柄以接收这些参数。
```
/**
 * @brief 步进电机结构体
 * @note  valid_mask 用于标记"本次解析有更新"的字段（1=本次有更新，0=本次未更新）
 *
 *        bit[0]  : info        文本消息缓存有更新
 *        bit[1]  : last_error  错误码有更新
 *        bit[2]  : pulse_cnt   累计脉冲数有更新
 *        bit[3]  : pos_err     位置误差有更新
 *        bit[4]  : bus_volt    总线电压有更新
 *        bit[5]  : speed_rpm   转速有更新
 *        bit[6]  : real_pos    实时位置有更新
 *        bit[7]  : target_pos  目标位置有更新
 *        bit[8]  : motor_sta   电机状态有更新
 *        bit[9]  : enable_sta  使能状态有更新
 *        bit[10] : arrived_sta 到位状态有更新
 *        bit[11] : clog_flag   堵转标志有更新
 */
typedef struct {
    uint8_t slave_addr;  /* 电机地址同id */
    int16_t speed_rpm;   /* 电机转速 */
    int32_t real_pos;    /* 电机位置 */
    int32_t target_pos;  /* 目标位置 */
    int32_t pos_err;     /* 位置误差 */
    int32_t pulse_cnt;   /* 累计脉冲数 */
    float bus_volt;      /* 总线电压 */
    uint8_t motor_sta;   /* 电机状态 */
    uint8_t enable_sta;  /* 使能状态 */
    uint8_t arrived_sta; /* 到位状态 */
    uint8_t clog_flag;   /* 堵转标志 */
    uint8_t last_error;  /* 最后一次错误码 */
    uint8_t *info;       /* 电机信息缓冲区 */
    uint32_t valid_mask; /* 有效字段掩码 */
} smd_motor_t;
```

## 函数方法

### 初始化与控制
- `smd_motor_init` 初始化电机句柄，分配信息缓冲区
- `smd_motor_enable` 电机使能控制
- `smd_stop_now` 立即停止（刹车）
- `smd_clear_sta` 清除状态（堵转、刹车，失能）

### 运动控制
- `smd_pos_mode` 绝对位置模式控制
- `smd_pos_rel_mode` 相对位置模式控制
- `smd_speed_mode` 速度模式控制
- `smd_torque_mode` 力矩模式控制
- `smd_angle_to_zero` 将当前位置清零

### 参数读取
- `smd_read_pos` 读取电机实时位置
- `smd_read_rotate_speed` 读电机实时转速
- `smd_read_vol` 读取总线电压
- `smd_read_motor_sta` 读取电机运行状态
- `smd_read_sys_params` 读取系统参数（批量读取所有状态）

### 参数设置
- `smd_set_pos_pid` 设置位置环 PID
- `smd_set_speed_pid` 设置速度环 PID
- `smd_set_mode` 设置工作模式
- `smd_set_step` 设置细分

### 帧处理
- `serial_frame_process` 解析接收到的数据帧，更新电机状态
- `smd_checksum` 计算校验和

# 示例

```c
#include "smd.h"
#include "smd_usart.h"

smd_motor_t motor_lift;

int main(void) {
    bsp_init();

    /* 初始化串口接收 */
    smd_usart_recv_init();

    /* 初始化电机，地址为 1 */
    smd_motor_init(&motor_lift, 1);

    /* 使能电机 */
    smd_motor_enable(1, 0);  /* 0=使能, 1=失能 */

    /* 读取系统参数 */
    smd_read_sys_params(1);

    HAL_Delay(1000);  /* 等待应答 */

    /* 绝对位置模式：方向0，加速度1，速度100 RPM，目标位置51200（1圈） */
    smd_pos_mode(1, 0, 1, 100, 51200);

    while (1) {
        /* 检查位置是否有更新 */
        if (motor_lift.valid_mask & SMD_MASK_REAL_POS) {
            printf("实时位置: %ld\n", motor_lift.real_pos);
        }

        /* 检查是否到位 */
        if (motor_lift.valid_mask & SMD_MASK_ARRIVED_STA) {
            if (motor_lift.arrived_sta == 1) {
                printf("已到位\n");
            }
        }

        /* 检查是否有错误 */
        if (motor_lift.valid_mask & SMD_MASK_INFO) {
            printf("电机信息: %s\n", (char *)motor_lift.info);
        }

        HAL_Delay(100);
    }
}
```

# 注意事项

1. **通信方式选择**：通过 `smd.h` 中的 `COMM_TYPE` 宏选择串口（0）或 CAN（1）通信
2. **RS485 控制**：使用串口通信时，需要正确配置 RS485_RE 引脚来控制发送/接收模式
3. **接收任务**：`smd_usart_recv_init()` 会创建一个 FreeRTOS 任务来处理接收数据，需要确保系统已启动调度器
4. **valid_mask**：每次接收应答后，`valid_mask` 会更新，使用时应检查对应位是否置位
5. **地址范围**：电机地址（slave_addr）最大支持 10 个，通过 `MOTOR_NUM_MAX` 宏定义
6. **位置单位**：51200 脉冲对应 1 圈（360 度）

注：valid_mask 在 SMD 回包被接收任务解析之后才变，用于标记“本次解析有更新”的字段（1=本次有更新，0=本次未更新），valid_mask 的置位只有在收到 SMD 回包并进入 serial_frame_process() 之后才会发生，别的地方不会凭空把它变成有效位
