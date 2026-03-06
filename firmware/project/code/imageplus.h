#ifndef IMAGEPLUS_REFACTORED_H
#define IMAGEPLUS_REFACTORED_H

#include "zf_common_headfile.h"

/* ===== 图像尺寸与分辨率参数 ===== */
#define IMAGE_HEIGHT    60
#define IMAGE_WIDTH     80
#define MID_LINE_VAL    40      /* 图像水平中心列 */
#define SHIELD_LINE     18      /* 入环后屏蔽的顶部行数 */
#define ROW_RAR         2       /* 行方向压缩比 */
#define COL_RAR         2       /* 列方向压缩比 */

/* ===== 权重维度（模糊控制档位数） ===== */
#define WEIGHT_DIM      4

/* ===== 二值化算法选择 ===== */
typedef enum {
    BIN_ALGO_OTSU = 0,          /* 大津法全局阈值 */
    BIN_ALGO_ANYWAY_1,          /* 固定阈值方案1 */
    BIN_ALGO_ANYWAY_2,          /* 固定阈值方案2 */
    BIN_ALGO_SOBEL_AUTO,        /* Sobel自适应阈值 */
} BinAlgorithm;

/* ===== 特殊道路类型处理方向 ===== */
typedef enum {
    ROAD_DIR_LEFT  = 0,         /* 左侧 */
    ROAD_DIR_RIGHT = 1,         /* 右侧 */
    ROAD_DIR_NONE  = -1,        /* 无效/未检测到 */
} RoadDir;

/* ===== 拐点（极值点）信息 ===== */
typedef struct {
    uint8 valid;                /* 1=有效拐点 */
    uint8 row;
    uint8 col;
} Inflexion;

/* ===== 单条边界（左/右）的梯度方差 ===== */
typedef struct {
    float left;                 /* 左边界梯度方差 */
    float right;                /* 右边界梯度方差 */
} EdgeGradientVar;

/* ===== 圆形赛道预判结果 ===== */
typedef struct {
    RoadDir fore_dir;           /* 预判方向：LEFT/RIGHT/NONE */
    float   left_grad_var;      /* 左边梯度方差（辅助判断） */
    float   right_grad_var;     /* 右边梯度方差（辅助判断） */
} CircleForecast;

/* ===== 逐行扫描结果（每帧核心输出） ===== */
typedef struct {
    int left_edge [IMAGE_HEIGHT + 1];   /* 各行左边界列坐标，0表示丢失 */
    int right_edge[IMAGE_HEIGHT + 1];   /* 各行右边界列坐标，IMAGE_WIDTH表示丢失 */
    int mid_line  [IMAGE_HEIGHT + 1];   /* 各行中线列坐标 */
    int jump_cnt  [IMAGE_HEIGHT + 1];   /* 各行跳变点数量（用于终止线检测） */
} LaneLines;

/* ===== 边界丢失统计 ===== */
typedef struct {
    int left_cnt;               /* 仅左边界丢失的行数 */
    int right_cnt;              /* 仅右边界丢失的行数 */
    int both_cnt;               /* 两侧同时丢失的行数 */
    int left_start_row;         /* 左边界首次丢失行 */
    int right_start_row;        /* 右边界首次丢失行 */
} EdgeLoss;

/* ===== 突变点统计（用于圆/交叉检测） ===== */
typedef struct {
    int road_width_break_row;   /* 道路宽度突变起始行 */
    int road_width_break_cnt;   /* 道路宽度突变持续行数 */
    int left_break_row;         /* 左边界突变起始行 */
    int left_break_cnt;         /* 左边界突变持续行数 */
    int right_break_row;        /* 右边界突变起始行 */
    int right_break_cnt;        /* 右边界突变持续行数 */
    Inflexion left_inflexion;   /* 左边界拐点 */
    Inflexion right_inflexion;  /* 右边界拐点 */
} BreakStats;

/* ===== 单帧图像分析的完整状态 ===== */
typedef struct {
    /* 原始/二值图像 */
    unsigned char binary [IMAGE_HEIGHT][IMAGE_WIDTH];
    unsigned char composite[IMAGE_HEIGHT][IMAGE_WIDTH]; /* 叠加可视化结果 */

    /* 扫描结果 */
    LaneLines     lanes;

    /* 质量统计 */
    EdgeLoss      loss;
    BreakStats    breaks;

    /* 前瞻 */
    int  lookahead_row;         /* 前瞻所在行（last_line） */
    int  lookahead_col;         /* 前瞻所在列（longest_col） */
    int  available_rows;        /* 有效行数 */

    /* 曲率与误差 */
    float curvity;              /* 中线曲率 */
    float image_error;          /* 加权中线偏差 */
    float modify_err;           /* 单点拟合偏差 */
    float variance;             /* 中线方差 */

    /* 图像质量 */
    unsigned char dyn_thresh;   /* 动态二值化阈值 */
    int fps;

    /* 特殊路段标志 */
    CircleForecast circle_forecast;
    int barn_exist;             /* 车库检测结果 */
    int barn_break_row;         /* 车库突变行 */
    int finish_line_row;        /* 终止线所在行 */
    int is_cross;               /* 交叉路口标志 */

    /* 环形赛道处理状态 */
    int circle_process_dir;     /* -1=未激活，0=左环，1=右环 */
    int circle_entry_active;    /* 入环处理标志 */
    int circle_exit_detect;     /* 出环检测标志 */
    int circle_exit_active;     /* 出环处理标志 */
    int circle_mending_active;  /* 环内补线标志 */
    int circle_proc_complete;   /* 处理完成标志 */
    int circle_prehandle_lock;
    int left_circle_prehandle;
    int right_circle_prehandle;
    int cross_handle;

    /* 障碍物（避障）处理状态 */
    int block_dir;              /* 0=无，1=左障碍，2=右障碍 */
} ImageState;

/* ===== 全局单例（声明） ===== */
extern ImageState g_img;

/* ===== 只读参考表（声明） ===== */
extern const int   g_road_width[IMAGE_HEIGHT + 1];
extern const float g_row_weight[WEIGHT_DIM][IMAGE_HEIGHT];

/* ===== 公开接口 ===== */

/** 每帧主入口：二值化→扫边→计算偏差→特殊路段处理 */
void image_analyze(uint8_t (*img)[MT9V03X_H][MT9V03X_W]);

/** 复位环形处理标志 */
void image_circle_reset(void);

/** 激活环形处理（dir: 0=左环, 1=右环） */
void image_circle_activate(int dir);

/** 复位障碍物处理标志 */
void image_block_reset(void);

/** 车库检测（dir: 0=从左入库, 1=从右入库; valid: 是否实际修正边界） */
int  image_search_barn(int dir, int valid);

/** 障碍物检测 */
int  image_search_block(void);

/** 交叉路口检测与边界修正 */
void image_search_cross(void);

/** 停止线检测 */
int  image_search_stopline(void);

/** 分叉口检测 */
int  image_search_fork(void);

/** 圆形赛道预判 */
CircleForecast image_search_circle(void);

/** 全列扫描全黑列数（用于圆出口判断） */
int  image_col_scan_all_black(int start_row);

/** 获取边界梯度方差 */
EdgeGradientVar image_get_edge_grad_var(int end_row, int min_valid_row);

/** 叠加可视化标注到 binary 图（调试用） */
void image_draw_overlay(void);

/** 出环判断（底行全黑像素计数） */
int  image_circle_exit_judge(void);

/* ===== 便捷访问宏（取代原来的 inline getter/setter） ===== */
#define IMG_LAST_LINE       (g_img.lookahead_row)
#define IMG_AVAIL_LINES     (g_img.available_rows)
#define IMG_CURVITY         (g_img.curvity)
#define IMG_ERROR           (g_img.image_error)
#define IMG_LEFT(row)       (g_img.lanes.left_edge[row])
#define IMG_RIGHT(row)      (g_img.lanes.right_edge[row])
#define IMG_MID(row)        (g_img.lanes.mid_line[row])
#define IMG_DYN_THR         (g_img.dyn_thresh)
#endif /* IMAGEPLUS_REFACTORED_H */
