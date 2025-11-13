#ifndef _IDENTIFY_H
#define _IDENTIFY_H

#define CROSS_LENGTH        45

#define NONE    0
#define SMALL_CIRCLE    1
#define BIG_CIRCLE  2
#include "zf_common_headfile.h"

enum Road_Type
{
    BEND              = 0,
    SSTART            = 10,
    START_ENTRY       = 11, // 出库开始
    START_IN          = 12, // 转弯
    START_EXIT1       = 13, // 直行
    START_EXIT2       = 14, // 循迹
    CIRCLE            = 20,
    CIRCLE_FORECAST   = 21,
    CIRCLE_READY_ENTRY= 22,
    CIRCLE_ENTRY      = 23,
    CIRCLE_IN         = 24,
    CIRCLE_READY_EXIT = 25,
    CIRCLE_EXIT       = 26,
    BLOCK             = 30,
    BLOCK_ENTRY1      = 31,  // 路障开始转弯
    BLOCK_ENTRY2      = 32,  // 路障开始回正
    BLOCK_IN          = 33,  // 路障直行
    BLOCK_EXIT1       = 34,  // 路障结束转弯
    BLOCK_EXIT2       = 35,  // 路障结束回正
    BLOCK_EXIT3       = 36,  // 回正后循迹
    BARN              = 40,
//    BARN_MAG_BACK     = 41, //后退一段
    BARN_IMG_ENTRY    = 42,
//    BARN_MAG_ENTRY    = 43, //前行一段
    BARN_IN           = 44, //转弯
    BARN_EXIT         = 45, //直行
    BARN_MAG_FORWARD  = 46, //前行
    BARN_MAG_AFTER_FORWARD_BACK    = 47,//后退
    BARN_MAG_BACK_BACK    = 48,//后退
    CROSS             = 50,
    RAMP              = 60,
    RAMP_UP           = 61,
    RAMP_DOWN         = 62,
    OUT_BOUND         = 70,
    LONG_STRAIGHT     = 80,
    FORK_ENTRY        = 81,
    FORK_IN           = 82,
    FORK_EXIT         = 83,
    FINISH            = 90,
    FINISH_IN         = 92,
    FINISH_STOP       = 93,
    FINISH_MAG_STOP   = 94,
    BREAK             = 100,
    BREAK_IN          = 101, //最短断路行驶距离 电磁循迹
    BREAK_READY_OUT   = 102, //检测到有路退出
};

enum ele_lock_ty
{
        _BEND = 0,
        _SSTART = 1,
        _CIRCLE = 2,
        _BLOCK = 3,
        _CROSS = 4,
        _RAMP = 5,
        _BARN = 6,
        _OUT_BOUND = 7,
        _FORK = 8,
        _FINISH = 9,
        _BREAK = 10
};

extern int road_type;
extern float circle_modify_err;
extern int start_line;
extern float finish_line_modify_err;
extern int ele_lock;
extern int ramp_status;
extern uint8 circle_times;
extern uint8 fork_times ;
extern  int ramp_status;

extern int left_circle_forecast_flag;
extern int right_circle_forecast_flag;
int detect_out_bound(void);
void set_out_bound(void);
int detect_long_straight_road(void);
void analyze_road(void);
static inline int get_road_type(void)   {return road_type;}
static inline float get_circle_modify_err(void) {return circle_modify_err;}
int get_lock(void);
int length_init(void);

#endif
