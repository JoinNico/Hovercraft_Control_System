/**
 * @file    decision.c
 * @brief   决策执行模块（重构版）
 *
 * 主要变更：
 *   1. SMARTCAR_STATUS  →  SmartCarStatus（与 decision.h 保持一致）
 *   2. IMG_ERROR 直接来自 imageplus_refactored.h 的宏（原含义不变）
 *   3. IMG_VAR   直接来自 identify.h 透传的宏（用于 straight_acceleration）
 *   4. road_type 通过 get_road_type() 获取，与 identify.c 解耦
 *   5. PID 指针操作保持不变，angle_pid 仍由外部提供
 */

#include "decision.h"

/* ===== PID 参数指针（指向外部 angle_pid 结构体字段） ===== */
static float *angle_kp = &(angle_pid.Kp);
static float *angle_ki = &(angle_pid.Ki);
static float *angle_kd = &(angle_pid.Kd);

/* ===== 运行状态单例 ===== */
SmartCarStatus smartcar_status;

/*---------------------------------------------------------------------------
 * 直道自适应加速
 *
 * 速度模型（二次曲线）：
 *   x      = clamp(|IMG_VAR / thresh|, 0, 1)
 *   speed  = (g_SpeedMin - g_SpeedMax) * x² + g_SpeedMax
 *
 * x=0（完全直线）→ peak_speed = ratio * global_speed
 * x=1（弯曲达阈值）→ global_speed（基础速度）
 *---------------------------------------------------------------------------*/
float straight_acceleration(int global_speed, float ratio, float thresh)
{
    if (thresh == 0.0f)
        return (float)global_speed;

    float g_SpeedMax = ratio * (float)global_speed; /* 直线最高速 */
    float g_SpeedMin = (float)global_speed;          /* 基础速度 */

    float x = fabsf(IMG_VAR / thresh);
    x = (x > 1.0f) ? 1.0f : x;

    float a = g_SpeedMin - g_SpeedMax;
    float c = g_SpeedMax;

    return a * x * x + c;
}

/*---------------------------------------------------------------------------
 * 决策主函数
 *
 * 根据 get_road_type() 返回的当前路况枚举，设置：
 *   - motor_on / motor_close_circle（使能标志）
 *   - global_speed（期望速度）
 *   - angle_pid 的 Kp / Ki / Kd
 *   - side_motor_flag（侧向电机标志，仅 BEND 段动态调整）
 *---------------------------------------------------------------------------*/
void decision(void)
{
    switch (get_road_type())
    {
        /* ── 长直道：提速 20%，PID 基础参数 ── */
        case LONG_STRAIGHT:
            smartcar_status.motor_on           = 1;
            smartcar_status.global_speed       = smartcar_param.global_speed * 1.2f;
            smartcar_status.motor_close_circle = 1;
            *angle_kp = ANGLE_KP;
            *angle_ki = ANGLE_KI;
            *angle_kd = ANGLE_KD;
            break;

        /* ── 普通弯道：偏差过大时开启侧向电机辅助转向 ── */
        case BEND:
            smartcar_status.motor_on = 1;
            smartcar_param.side_motor_flag =
                (IMG_ERROR > 10.0f || IMG_ERROR < -10.0f) ? 1 : 0;
            smartcar_status.global_speed       = smartcar_param.global_speed;
            smartcar_status.motor_close_circle = 1;
            *angle_kp = ANGLE_KP;
            *angle_ki = ANGLE_KI;
            *angle_kd = ANGLE_KD;
            break;

        /* ── 环中行驶：PID 微调，关闭侧向电机 ── */
        case CIRCLE_IN:
            smartcar_status.motor_on           = 1;
            smartcar_param.side_motor_flag     = 0;
            smartcar_status.global_speed       = smartcar_param.global_speed;
            smartcar_status.motor_close_circle = 1;
            *angle_kp = ANGLE_KP * 1.0f;
            *angle_ki = ANGLE_KI * 0.5f;
            *angle_kd = ANGLE_KD * 0.5f;
            break;

        /* ── 圆环各过渡阶段：降速 + 开侧向电机 + PID 减弱 ── */
        case CIRCLE_FORECAST:
        case CIRCLE_READY_ENTRY:
        case CIRCLE_ENTRY:
        case CIRCLE_READY_EXIT:
        case CIRCLE_EXIT:
            smartcar_status.motor_on           = 1;
            smartcar_param.side_motor_flag     = 1;
            smartcar_status.global_speed       = 0.6f;
            smartcar_status.motor_close_circle = 1;
            *angle_kp = ANGLE_KP * 0.8f;
            *angle_ki = ANGLE_KI * 0.7f;
            *angle_kd = ANGLE_KD * 0.7f;
            break;

        /* ── 路障直行段：加强 PID + 减速 ── */
        case BLOCK_IN:
            smartcar_status.motor_on           = 1;
            smartcar_status.global_speed       = 0.5f;
            smartcar_status.motor_close_circle = 1;
            *angle_kp = ANGLE_KP * 1.2f;
            *angle_ki = ANGLE_KI * 1.2f;
            *angle_kd = ANGLE_KD * 1.2f;
            break;

        /* ── 终止线冲线段：恢复基础速度 ── */
        case FINISH:
            smartcar_status.motor_on           = 1;
            smartcar_status.global_speed       = smartcar_param.global_speed;
            smartcar_status.motor_close_circle = 1;
            *angle_kp = ANGLE_KP;
            *angle_ki = ANGLE_KI;
            *angle_kd = ANGLE_KD;
            break;

        /* ── 停车：关驱动，清零目标角速度 ── */
        case FINISH_STOP:
            smartcar_status.motor_on               = 0;
            smartcar_status.global_speed           = 0.0f;
            smartcar_status.motor_close_circle     = 1;
            smartcar_status.goal_angular_velocity  = 0.0f;
            *angle_kp = ANGLE_KP;
            *angle_ki = ANGLE_KI;
            *angle_kd = ANGLE_KD;
            break;

        default:
            break;
    }
}
