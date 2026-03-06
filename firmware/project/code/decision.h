/**
 * @file    decision.h
 * @brief   决策执行模块（重构版）
 *
 * 依赖 identify.h（RoadType 枚举）与 imageplus_refactored.h（IMG_* 宏）。
 */

#ifndef _DECISION_H
#define _DECISION_H

#include "zf_common_headfile.h"

/* ===== 方向常量 ===== */
#define STEER_DIR           1

/* ===== 速度比率 ===== */
#define SLOW_SPEED_RATIO    0.8f  /* 慢速（如上坡）比率 */
#define FAST_SPEED_RATIO    1.1f  /* 快速（如下坡）比率 */

/* ===== 智能车运行状态结构体 ===== */
typedef struct {
    float         goal_angular_velocity; /* 目标角速度 */
    int           goal_PWM;              /* 目标驱动 PWM */
    int           goal_side_PWM;         /* 目标侧向 PWM */
    unsigned char motor_on;             /* 驱动使能 */
    unsigned char motor_close_circle;   /* 闭环使能 */
    float         global_speed;         /* 全局速度设定值 */
    int           base_PWM;             /* 基础 PWM */
    float         inner_target;         /* 内圈目标值 */
    int           left_duty;            /* 左轮占空比 */
    int           right_duty;           /* 右轮占空比 */
    int           brushless_duty;       /* 无刷电机占空比 */
} SmartCarStatus;

extern SmartCarStatus smartcar_status;

/* ===== 公开接口 ===== */

/**
 * @brief 根据当前道路类型设置速度与 PID 参数
 *
 * 调用方须先执行 analyze_road()，保证 road_type 已更新。
 */
void decision(void);

/**
 * @brief 直道加速辅助函数
 *
 * @param global_speed  基础速度
 * @param ratio         最大速度倍率（peak_speed = ratio * global_speed）
 * @param thresh        方差阈值；为 0 时直接返回 global_speed
 * @return float        当前帧期望速度
 *
 * 速度曲线：variance=0 时取 peak_speed，variance=thresh 时取 global_speed，
 * 中间按二次曲线插值。
 */
float straight_acceleration(int global_speed, float ratio, float thresh);

#endif /* _DECISION_H */
