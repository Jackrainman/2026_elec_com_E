/**
 * @file raspi_serial.c
 * @author meiwenhuaqingnian
 * @brief 简略的树莓派串口通信协议
 * @version 0.1
 * @date 2026-07-29
 * 
 * 
 */
#include "raspi_serial.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "usart_ex/usart_ex.h"

#define READ_SIZE       64U
#define TASK_STACK      256U
#define TASK_PRIORITY   2U
#define POLL_TIME_MS    2U
#define VALUE_COUNT     6U

static UART_HandleTypeDef *raspi_uart;
static TaskHandle_t task_handle;

static raspi_serial_data_t latest_data = {0};
static bool data_available = false;

/* 分离转化数据 */
static bool parse_frame(char *frame, float value[VALUE_COUNT]) {
    char *position = frame;

    for (uint32_t i = 0U; i < VALUE_COUNT; ++i) {
        char *end;
        value[i] = strtof(position, &end);

        if ((end == position) || !isfinite(value[i])) {
            return false;
        }

        position = end;

        if ((i + 1U) < VALUE_COUNT) {
            if (*position != ',') {
                return false;
            }
            ++position;
        }
    }

    return *position == '\0';
}

/* 保存数值 */
static void save_data(const float value[VALUE_COUNT]) {
    taskENTER_CRITICAL();

    latest_data.x = -value[0];
    latest_data.y = value[1];
    latest_data.th = value[2];
    latest_data.x1 = -value[3];
    latest_data.y1 = value[4];
    latest_data.a = value[5];
    ++latest_data.sequence;
    data_available = true;

    taskEXIT_CRITICAL();
}

/* 接收任务 */
static void receive_task(void *argument) {
    uint8_t read_buffer[READ_SIZE];
    char frame[RASPI_SERIAL_FRAME_SIZE];
    size_t length = 0U;
    bool discard = false;

    (void)argument;

    while (1) {
        uint32_t count = uart_dmarx_read(raspi_uart, read_buffer,
                                         sizeof(read_buffer));

        if (count == 0U) {
            vTaskDelay(pdMS_TO_TICKS(POLL_TIME_MS));
            continue;
        }

        for (uint32_t i = 0U; i < count; ++i) {
            char received = (char)read_buffer[i];

            if (received == '\r') {
                continue;
            }

            if (received == '\n') {
                float value[VALUE_COUNT];
                frame[length] = '\0';

                if (!discard && (length > 0U) &&
                    parse_frame(frame, value)) {
                    save_data(value);
                }

                length = 0U;
                discard = false;
                continue;
            }

            if (discard) {
                continue;
            }

            if (length < (sizeof(frame) - 1U)) {
                frame[length++] = received;
            } else {
                length = 0U;
                discard = true;
            }
        }
    }
}

/* 分配串口，创建任务 */
bool raspi_serial_init(UART_HandleTypeDef *huart) {
    if ((huart == NULL) || (huart->hdmarx == NULL) ||
        (task_handle != NULL)) {
        return false;
    }

    raspi_uart = huart;

    if (xTaskCreate(receive_task, "raspi_serial", TASK_STACK, NULL,
                    TASK_PRIORITY, &task_handle) != pdPASS) {
        raspi_uart = NULL;
        task_handle = NULL;
        return false;
    }

    return true;
}


/* 读取数据 */
bool raspi_serial_get_latest(raspi_serial_data_t *data) {
    bool available;

    if (data == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    *data = latest_data;
    available = data_available;
    taskEXIT_CRITICAL();

    return available;
}

/* 反馈数据 */
void send_reply(const char *reply)
{
    uart_dmatx_write(raspi_uart, reply, strlen(reply));
    uart_dmatx_send(raspi_uart);
}
