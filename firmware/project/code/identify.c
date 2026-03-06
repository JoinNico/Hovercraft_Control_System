/**
 * @file    identify.c
 * @brief   道路类型识别模块（重构版）
 *
 * 主要变更：
 *   1. 所有原散列全局变量（left_lose / right_lose / barn_exist_flag /
 *      circle_image_process_complete / circle_entry_process /
 *      in_circle_mending / circle_exit_detect / circle_exit_process /
 *      is_cross / modify_err）均通过 g_img 结构体字段访问。
 *   2. image_circle_activate() / image_circle_reset() /
 *      image_block_reset() / image_search_block()
 *      取代原 set_process_circle_flag() / clear_process_circle_flag() /
 *      clear_process_block_flag() / search_block()。
 *   3. 圆环方向判断从裸数字 0/1/-1 改为 ROAD_DIR_LEFT/RIGHT/NONE 枚举。
 *   4. 距离、锁、参数等非图像接口保持不变。
 */

#include "identify.h"

/* ===== 模块状态 ===== */
int   road_type      = BEND;
int   last_road_type = BEND;
float circle_modify_err = 0.0f;
int   cross_state    = 0;
int   ele_lock       = _BEND;

/* 圆环预判连续帧计数 */
int left_circle_forecast_flag  = 0;
int right_circle_forecast_flag = 0;

/* ===== 里程计数（由外部 length_init 初始化） ===== */
//static int length_circle1 = 0;
//static int length_circle2 = 0;
//static int length_barn    = 0;
//static int length_block   = 0;
//static int length_break   = 0;

/* ===== 圆环参数 ===== */
#define CIRCLE_NUM                  1
#define CIRCLE_DIR(n)               smartcar_param.circle_dir[(n)]
#define CIRCLE_FORECAST_DISTANCE    0.6f
#define IN_CIRCLE_DISTANCE          2.5f
#define CIRCLE_READY_EXIT_DISTANCE  0.5f
#define CIRCLE_EXIT_ANGLE           (-10)

/* ===== 车库参数 ===== */
#define STOPLINE_DIS_CTRL       25.0f
#define STOPLINE_IN_DISTANCE    1.0f

/* ===== 路障参数 ===== */
#define IN_BLOCK_DISTANCE   0.7f
#define BLOCK_DIS_CTRL      1.0f

/*===========================================================================
 * 互斥锁
 *=========================================================================*/
int get_lock(void)  { return ele_lock; }
void set_lock(int lock)  { ele_lock = lock; }
void clear_lock(void)    { ele_lock = _BEND; }

/*===========================================================================
 * 里程初始化
 *=========================================================================*/
int length_init(void)
{
//    circle_times = smartcar_param.circle_num;
    smartcar_param.circle_num =
        smartcar_param.circle_num + smartcar_param.circle_dir[1];
    return 0;
}

/*===========================================================================
 * 圆环识别状态机
 *
 * 图像接口变更对照：
 *   旧                              新
 *   circle_forecast.circle_fore_dir  IMG_CIRCLE_FORE_DIR（值类型：RoadDir）
 *   left_lose                        IMG_LEFT_LOSE
 *   right_lose                       IMG_RIGHT_LOSE
 *   circle_image_process_complete    g_img.circle_proc_complete
 *   circle_entry_process             g_img.circle_entry_active
 *   in_circle_mending                g_img.circle_mending_active
 *   circle_exit_process              g_img.circle_exit_active
 *   circle_exit_detect               g_img.circle_exit_detect
 *   set_process_circle_flag(dir)     image_circle_activate(dir)
 *   clear_process_circle_flag()      image_circle_reset()
 *   modify_err                       g_img.modify_err
 *=========================================================================*/
int detect_circle_image(void)
{
    static int   circle_cnt     = 0;
    static int   circle_flag    = 0;
    static float circle_dis_rec = 0.0f;

    /* 当前圆环方向：ROAD_DIR_LEFT(0) 或 ROAD_DIR_RIGHT(1) */
    RoadDir circle_dir = (RoadDir)CIRCLE_DIR(circle_cnt);

    /* 互斥锁检查 */
    if (get_lock() != _BEND && get_lock() != _CIRCLE)
        return 0;

    switch (circle_flag)
    {
        /* ── 待机：连续帧预判确认 ── */
        case 0:
            /* 左环预判 */
            if (IMG_CIRCLE_FORE_DIR == ROAD_DIR_LEFT &&
                circle_dir == ROAD_DIR_LEFT &&
                left_circle_forecast_flag <= 0)
            {
                left_circle_forecast_flag--;
                if (left_circle_forecast_flag < -1)
                {
                    left_circle_forecast_flag = 1;
                    circle_dis_rec = get_distance();
                    circle_flag    = CIRCLE_FORECAST;
                    set_lock(_CIRCLE);
                }
            }
            else if (left_circle_forecast_flag <= 0)
            {
                left_circle_forecast_flag = 0;
            }

            /* 右环预判 */
            if (IMG_CIRCLE_FORE_DIR == ROAD_DIR_RIGHT &&
                circle_dir == ROAD_DIR_RIGHT &&
                right_circle_forecast_flag <= 0)
            {
                right_circle_forecast_flag--;
                if (right_circle_forecast_flag < -1)
                {
                    right_circle_forecast_flag = 1;
                    circle_dis_rec = get_distance();
                    circle_flag    = CIRCLE_FORECAST;
                    set_lock(_CIRCLE);
                }
            }
            else if (right_circle_forecast_flag <= 0)
            {
                right_circle_forecast_flag = 0;
            }
            break;

        /* ── 预判阶段：激活图像补线，等待行驶一段距离 ── */
        case CIRCLE_FORECAST:
            image_circle_activate((int)circle_dir); /* 取代 set_process_circle_flag */
            if ((get_distance() - circle_dis_rec) > CIRCLE_FORECAST_DISTANCE &&
                (left_circle_forecast_flag == 1 || right_circle_forecast_flag == 1))
            {
                circle_flag = CIRCLE_READY_ENTRY;
                left_circle_forecast_flag  = 0;
                right_circle_forecast_flag = 0;
            }
            break;

        /* ── 准入阶段：等待单侧边界大量丢失（即到达入口） ── */
        case CIRCLE_READY_ENTRY:
            if ((IMG_LEFT_LOSE  > 10 && IMG_RIGHT_LOSE < 5  && circle_dir == ROAD_DIR_LEFT) ||
                (IMG_RIGHT_LOSE > 10 && IMG_LEFT_LOSE  < 5  && circle_dir == ROAD_DIR_RIGHT))
            {
                circle_flag  = CIRCLE_ENTRY;
                circle_dis_rec = get_distance();
                g_img.circle_entry_active = 1; /* 取代 circle_entry_process = 1 */
            }
            break;

        /* ── 入环阶段：等待图像补线完成 ── */
        case CIRCLE_ENTRY:
            if (g_img.circle_proc_complete == 1) /* 取代 circle_image_process_complete */
            {
                circle_flag = CIRCLE_IN;
                circle_dis_rec = get_distance();
                g_img.circle_entry_active  = 0;  /* 取代 circle_entry_process = 0 */
                g_img.circle_mending_active = 1; /* 取代 in_circle_mending = 1 */
            }
            break;

        /* ── 环中阶段：行驶足够距离且边界恢复后转出环 ── */
        case CIRCLE_IN:
            if (IMG_LEFT_LOSE > 15 &&
                (get_distance() - circle_dis_rec) > IN_CIRCLE_DISTANCE)
            {
                circle_flag  = CIRCLE_EXIT;
                circle_dis_rec = get_distance();
                g_img.circle_mending_active = 0; /* 取代 in_circle_mending = 0 */
                g_img.circle_exit_active    = 1; /* 取代 circle_exit_process = 1 */
            }
            break;

        /* ── 准出阶段（保留状态，当前逻辑已跳过） ── */
        case CIRCLE_READY_EXIT:
            if ((get_distance() - circle_dis_rec) > CIRCLE_READY_EXIT_DISTANCE)
            {
                circle_flag  = CIRCLE_EXIT;
                circle_dis_rec = get_distance();
                g_img.circle_exit_detect = 0;
                g_img.circle_exit_active = 1;
            }
            break;

        /* ── 出环阶段：单侧丢失数恢复后退出 ── */
        case CIRCLE_EXIT:
            if ((IMG_LEFT_LOSE  < 10 && circle_dir == ROAD_DIR_LEFT) ||
                (IMG_RIGHT_LOSE < 10 && circle_dir == ROAD_DIR_RIGHT))
            {
                circle_flag = 0;
                circle_cnt++;
                if (circle_cnt >= CIRCLE_NUM)
                    circle_cnt = 0;

                image_circle_reset();               /* 取代 clear_process_circle_flag() */
                g_img.circle_exit_active = 0;
                clear_lock();
            }
            break;

        default:
            break;
    }

    /* ── 计算转向修正量 ── */
    circle_modify_err = g_img.modify_err; /* 取代 modify_err 全局变量 */

    if (circle_flag == CIRCLE_READY_EXIT)
    {
        circle_modify_err = (circle_dir == ROAD_DIR_LEFT)
                            ?  CIRCLE_EXIT_ANGLE
                            : -CIRCLE_EXIT_ANGLE;
    }

    return circle_flag;
}

/*===========================================================================
 * 入库（终止线）识别
 *
 * 图像接口变更：
 *   barn_exist_flag  →  g_img.barn_exist
 *=========================================================================*/
int detect_stop(void)
{
    static int   barn_flag    = 0;
    static float barn_dis_rec = 0.0f;

    if (get_distance() < STOPLINE_DIS_CTRL)
        return 0;
    if (get_lock() != _BEND && get_lock() != _BARN)
        return 0;

    switch (barn_flag)
    {
        case 0:
            if (g_img.barn_exist == 1) /* 取代 barn_exist_flag */
            {
                set_lock(_BARN);
                barn_flag    = FINISH;
                barn_dis_rec = get_distance();
            }
            break;

        case FINISH:
            if ((get_distance() - barn_dis_rec) > STOPLINE_IN_DISTANCE)
                barn_flag = FINISH_STOP;
            break;

        case FINISH_STOP:
            break;

        default:
            break;
    }

    return barn_flag;
}

/*===========================================================================
 * 路障识别
 *
 * 图像接口变更：
 *   search_block()          →  image_search_block()
 *   clear_process_block_flag() →  image_block_reset()
 *=========================================================================*/
int detect_block(void)
{
    static int   block_flag            = 0;
    static int   block_num             = 1;
    static int   block_pre_flag        = 0;
    static float block_pre_flag_point  = 0.0f;

    if ((get_lock() != _BEND && get_lock() != _BLOCK) ||
        get_distance() < BLOCK_DIS_CTRL)
        return 0;

    if (block_num != 0)
        block_pre_flag = image_search_block(); /* 取代 search_block() */

    if (block_pre_flag)
    {
        block_pre_flag        = 0;
        block_flag            = BLOCK_IN;
        block_num--;
        block_pre_flag_point  = get_distance();
        set_lock(_BLOCK);
    }

    if (block_flag == BLOCK_IN)
    {
        if ((get_distance() - block_pre_flag_point) > IN_BLOCK_DISTANCE)
        {
            block_flag = 0;
            image_block_reset(); /* 取代 clear_process_block_flag() */
            clear_lock();
        }
    }

    return block_flag;
}

/*===========================================================================
 * 长直道识别
 *
 * 图像接口变更：
 *   get_last_line()   →  IMG_LAST_LINE
 *   get_variance()    →  IMG_VAR
 *=========================================================================*/
int detect_long_straight_road(void)
{
    if (get_lock() != _BEND)
        return 0;

    const int threshold = 16;
    if (IMG_LAST_LINE < 40 && IMG_VAR < threshold)
        return LONG_STRAIGHT;

    return 0;
}

/*===========================================================================
 * 十字路口识别
 *
 * 图像接口变更：
 *   is_cross  →  g_img.is_cross
 *=========================================================================*/
int detect_cross(void)
{
    if (get_lock() != _BEND && get_lock() != _CROSS)
        return 0;

    if (g_img.is_cross == 1) /* 取代 is_cross */
    {
        cross_state = CROSS;
        set_lock(_CROSS);
    }
    else
    {
        cross_state = 0;
        clear_lock();
    }

    return cross_state;
}

/*===========================================================================
 * 主路况分析入口
 *=========================================================================*/
void analyze_road(void)
{
    int ret;

    last_road_type = road_type;
    road_type      = BEND;

    if (0 != (ret = detect_circle_image())) { road_type = ret; return; }
    if (0 != (ret = detect_stop()))         { road_type = ret; return; }
    if (0 != (ret = detect_cross()))        { road_type = ret; return; }
    if (0 != (ret = detect_block()))        { road_type = ret; return; }
}
