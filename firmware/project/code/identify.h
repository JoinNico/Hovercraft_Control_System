/**
 * @file    identify.h
 * @brief   道路类型识别模块（重构版）
 *
 * 依赖 imageplus_refactored.h 的新接口：
 *   - g_img.loss.left_cnt / right_cnt    取代原 left_lose / right_lose
 *   - g_img.circle_forecast.fore_dir     取代原 circle_forecast.circle_fore_dir
 *   - g_img.circle_proc_complete         取代原 circle_image_process_complete
 *   - g_img.circle_entry_active          取代原 circle_entry_process
 *   - g_img.circle_mending_active        取代原 in_circle_mending
 *   - g_img.circle_exit_detect           取代原 circle_exit_detect
 *   - g_img.circle_exit_active           取代原 circle_exit_process
 *   - g_img.barn_exist                   取代原 barn_exist_flag
 *   - g_img.is_cross                     取代原 is_cross
 *   - g_img.modify_err                   取代原 modify_err
 *   - g_img.variance                     取代原 variance（通过 IMG_VAR）
 *   - IMG_LAST_LINE / IMG_ERROR / IMG_VAR 宏
 *   - image_circle_activate() / image_circle_reset()
 *   - image_block_reset() / image_search_block()
 */

#ifndef _IDENTIFY_H
#define _IDENTIFY_H

#include "zf_common_headfile.h"

/* ===== 跨模块常量 ===== */
#define CROSS_LENGTH    45

/* ===== 道路类型枚举 ===== */
typedef enum Road_Type
{
    BEND               = 0,

    SSTART             = 10,
    START_ENTRY        = 11,
    START_IN           = 12,
    START_EXIT1        = 13,
    START_EXIT2        = 14,

    CIRCLE             = 20,
    CIRCLE_FORECAST    = 21,
    CIRCLE_READY_ENTRY = 22,
    CIRCLE_ENTRY       = 23,
    CIRCLE_IN          = 24,
    CIRCLE_READY_EXIT  = 25,
    CIRCLE_EXIT        = 26,

    BLOCK              = 30,
    BLOCK_ENTRY1       = 31,
    BLOCK_ENTRY2       = 32,
    BLOCK_IN           = 33,
    BLOCK_EXIT1        = 34,
    BLOCK_EXIT2        = 35,
    BLOCK_EXIT3        = 36,

    BARN               = 40,
    BARN_IMG_ENTRY     = 42,
    BARN_IN            = 44,
    BARN_EXIT          = 45,
    BARN_MAG_FORWARD   = 46,
    BARN_MAG_AFTER_FORWARD_BACK = 47,
    BARN_MAG_BACK_BACK = 48,

    CROSS              = 50,

    RAMP               = 60,
    RAMP_UP            = 61,
    RAMP_DOWN          = 62,

    OUT_BOUND          = 70,

    LONG_STRAIGHT      = 80,
    FORK_ENTRY         = 81,
    FORK_IN            = 82,
    FORK_EXIT          = 83,

    FINISH             = 90,
    FINISH_IN          = 92,
    FINISH_STOP        = 93,
    FINISH_MAG_STOP    = 94,

    BREAK              = 100,
    BREAK_IN           = 101,
    BREAK_READY_OUT    = 102,
} RoadType;

/* ===== 互斥锁枚举（防止多路段同时响应） ===== */
typedef enum ele_lock_ty
{
    _BEND      = 0,
    _SSTART    = 1,
    _CIRCLE    = 2,
    _BLOCK     = 3,
    _CROSS     = 4,
    _RAMP      = 5,
    _BARN      = 6,
    _OUT_BOUND = 7,
    _FORK      = 8,
    _FINISH    = 9,
    _BREAK     = 10,
} EleLock;

/* ===== 对外状态变量 ===== */
extern int   road_type;         /* 当前帧道路类型（RoadType） */
extern float circle_modify_err; /* 圆环段转向修正量 */
extern int   ele_lock;          /* 当前互斥锁（EleLock） */

/* ===== 圆环预判标志（供 imageplus 内部访问） ===== */
extern int left_circle_forecast_flag;
extern int right_circle_forecast_flag;

/* ===== 便捷宏（对接新 g_img） ===== */
/** 中线方差，用于长直道检测 */
#define IMG_VAR         (g_img.variance)
/** 左边界丢失行数 */
#define IMG_LEFT_LOSE   (g_img.loss.left_cnt)
/** 右边界丢失行数 */
#define IMG_RIGHT_LOSE  (g_img.loss.right_cnt)
/** 圆环预判方向（RoadDir 枚举） */
#define IMG_CIRCLE_FORE_DIR  (g_img.circle_forecast.fore_dir)

/* ===== 公开接口 ===== */
void  analyze_road(void);

static inline int   get_road_type(void)          { return road_type; }
static inline float get_circle_modify_err(void)  { return circle_modify_err; }

int   get_lock(void);
void  set_lock(int lock);
void  clear_lock(void);
int   length_init(void);

#endif /* _IDENTIFY_H */
