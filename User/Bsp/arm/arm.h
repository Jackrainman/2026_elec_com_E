/**
 ****************************************************************************************************
 * @file    arm.h
 * @author  xinglu
 * @brief   �����е�ۿ���ģ�� (��������ԭ��SMD�������)
 *          - X�� / Y�� ƽ���˶�
 *          - R�� ĩ����ת
 * @version 1.0
 * @date    2026-07-31
 * @note    ����Լ����
 *          1: X��, 2: Y��, 3: R�ᣨ��ת��
 *          ���е������һ· RS485 ���� (UART4)
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

/* ��� */
extern servo_t servo;
#define SERVO_DOWN()         servo_set_state(&servo, SERVO_OPEN)
#define SERVO_UP()           servo_set_state(&servo, SERVO_CLOSE)

/* ========================== �����ַ���� ========================== */
#define ARM_MOTOR_X_ADDR     1 /* X������ַ */
#define ARM_MOTOR_Y_ADDR     2 /* Y������ַ */
#define ARM_MOTOR_R_ADDR     3 /* R�ᣨ��ת�������ַ */

/* ========================== Ĭ���˶����� ========================== */
#define ARM_DEFAULT_SPEED    60  /* Ĭ��ת�� RPM */
#define ARM_DEFAULT_ACC      0 /* Ĭ�ϼ��ٶ� */

/* ��ʼ��ͬ��ȷ�ϲ�����ģ�� 2026R1�� */
#define ARM_SYNC_RETRY_TIMES 3  /* ����ͬ�����Դ��� */
#define ARM_SYNC_TIMEOUT_MS  50 /* ���εȴ�Ӧ��ʱ (ms) */

/* �����ȷ�ϲ�����ģ�� 2026R1 lift_send_smd_pos_cmd�� */
#define ARM_ACK_RETRY_TIMES  3   /* ������������Դ��� */
#define ARM_ACK_TIMEOUT_MS   100 /* ���εȴ�����Ӧ��ʱ (ms) */

/* ϸ��: 51200 ����/Ȧ����ʵ�ʵ��ϸ�������޸ģ� */
#define ARM_PULSE_PER_REV    51200U

/* ========================== ��λ���� ========================== */
#define ARM_X_MM_PER_REV     40.0f  /*!< X��˿�˵��� mm/Ȧ  */
#define ARM_Y_MM_PER_REV     40.0f  /*!< Y��˿�˵��� mm/Ȧ  */
#define ARM_DEG_PER_REV      360.0f /*!< �Ƕ� ��/Ȧ         */

/* ���� �� ���壨�޷��ţ�����λ���ã� */
#define ARM_X_MM_TO_PULSE(mm)                                                  \
    ((uint32_t)((float)(mm) / ARM_X_MM_PER_REV * (float)ARM_PULSE_PER_REV))
#define ARM_Y_MM_TO_PULSE(mm)                                                  \
    ((uint32_t)((float)(mm) / ARM_Y_MM_PER_REV * (float)ARM_PULSE_PER_REV))
/* �Ƕ� �� ���壨�޷��ţ� */
#define ARM_DEG_TO_PULSE(deg)                                                  \
    ((uint32_t)((float)(deg) / ARM_DEG_PER_REV * (float)ARM_PULSE_PER_REV))

/* ���� �� ���壨�з��ţ����λ���ã� */
#define ARM_X_MM_TO_PULSE_S(mm)                                                \
    ((int32_t)((float)(mm) / ARM_X_MM_PER_REV * (float)ARM_PULSE_PER_REV))
#define ARM_Y_MM_TO_PULSE_S(mm)                                                \
    ((int32_t)((float)(mm) / ARM_Y_MM_PER_REV * (float)ARM_PULSE_PER_REV))
/* �Ƕ� �� ���壨�з��ţ� */
#define ARM_DEG_TO_PULSE_S(deg)                                                \
    ((int32_t)((float)(deg) / ARM_DEG_PER_REV * (float)ARM_PULSE_PER_REV))

/* ========================== 姜姜小组 ========================== */
#define CAM_X_CORRECT (13.0f) /*!< X���ʼ����ϵƫ�� (mm) */ // ����
#define CAM_Y_CORRECT (-217.0f) /*!< Y���ʼ����ϵƫ�� (mm) */ // ����

// /* ========================== 志玉小组 ========================== */
// #define CAM_X_CORRECT (40.0f) /*!< X���ʼ����ϵƫ�� (mm) */ // ����
// #define CAM_Y_CORRECT (-230.0f) /*!< Y���ʼ����ϵƫ�� (mm) */ // ����

// #define CAM_X_CORRECT (0.0f) /*!< X���ʼ����ϵƫ�� (mm) */ // ����
// #define CAM_Y_CORRECT (0.0f) /*!< Y���ʼ����ϵƫ�� (mm) */ // ����

/* �����壨�� SMD ����һ��: 0=CW, 1=CCW�� */
#define ARM_DIR_CW    0
#define ARM_DIR_CCW   1

/* ========================== API ========================== */

/**
 * @brief  ��ʼ����е�ۣ�ʹ��ȫ���������Ϊλ��ģʽ��
 * @note   �ϵ�ȴ� 1s �󣬽����������ǰλ����Ϊ��㣻
 *         �����ظ���λ��ȷ�ϵ�����ߣ��� arm_is_ready() ��ѯ�����
 */
void arm_init(void);

/**
 * @brief  ��ѯ��е���Ƿ��ʼ���ɹ�
 * @note   ģ�� 2026R1����ʼ��ʱ���ظ���λ�ò�У��Ӧ��
 *         �����ͬ���ɹ��ŷ��� true
 * @return true=���������, false=����������
 */
bool arm_is_ready(void);

/**
 * @brief  ��ѯָ�����Ƿ��ʼ���ɹ���ͨ��ͬ���ɹ���
 * @param  axis  ���ţ�1=X, 2=Y, 3=R
 * @return true=��������, false=���߻���Ӧ��
 */
bool arm_axis_is_ready(uint8_t axis);

/**
 * @brief  ��������ƶ������������㣩
 * @note   ����ʽ�����ͺ�ȴ����Ӧ��ʧ���Զ����� ARM_ACK_RETRY_TIMES ��
 * @param  axis   ���ţ�1=X, 2=Y, 3=R
 * @param  pulse  Ŀ�������� (�� CW / �� CCW)
 * @param  speed  ת�٣�RPM������0��ʹ��Ĭ��ֵ
 */
void arm_axis_move(uint8_t axis, int32_t pulse, uint16_t speed);

/**
 * @brief  ��������ƶ�
 * @note   ����ʽ�����ͺ�ȴ����Ӧ��ʧ���Զ����� ARM_ACK_RETRY_TIMES ��
 * @param  axis   ����
 * @param  pulse  ��������� (�� CW / �� CCW)
 * @param  speed  ת�٣�RPM������0��ʹ��Ĭ��ֵ
 */
void arm_axis_rel_move(uint8_t axis, int32_t pulse, uint16_t speed);

/**
 * @brief  ���㵥���ƶ���ʱ
 * @param  pulse  �ƶ�������
 * @param  speed  ת�٣�RPM��
 * @return Ԥ����ʱ��ms�����Ѻ��Ӽ�������
 */
uint32_t arm_est_move_ms(uint32_t pulse, uint16_t speed);

/**
 * @brief  ��ѯ������ָ�����ʵʱλ�ã�����Լ 5ms��
 * @note   ͨ������������ѯ���λ��
 * @param  axis  ����
 */
void arm_update_position(uint8_t axis);

/**
 * @brief  �ȴ�ָ�����˶���λ������ʽ��������ѯλ�ã�
 * @param  axis       ����
 * @param  target     Ŀ�������� (�� CW / �� CCW)
 * @param  tolerance  ��λ�ݲ�������������� 50 ~ 200
 * @param  timeout_ms ��ʱ��ms������ 0 ��������ʱ
 * @return true=��λ, false=��ʱ
 */
bool arm_wait_axis_done(uint8_t axis, int32_t target, uint32_t tolerance,
                        uint32_t timeout_ms);

/**
 * @brief  ��ͣ��ȫ���������ɲ����
 */
void arm_emergency_stop(void);

/**
 * @brief  ʹ�� / ʧ��ȫ�����
 * @param  en  true=����ʹ��, false=����
 */
void arm_enable_all(bool en);

/**
 * @brief  ʹ�� / ʧ��ָ����
 */
void arm_enable_axis(uint8_t axis, bool en);

/**
 * @brief  ��ȡָ���ᵱǰλ�ã���������
 * @note   ���ȵ��� arm_update_position() ˢ��λ��
 * @param  axis  ����
 * @return ��ǰλ��������
 */
int32_t arm_get_position_pulse(uint8_t axis);

/**
 * @brief  ��ȫ���ᵱǰλ�����㣨��Ϊ����ԭ�㣩
 */
void arm_set_zero(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_H */
