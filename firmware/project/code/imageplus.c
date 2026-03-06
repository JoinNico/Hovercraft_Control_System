#include "imageplus.h"


/* ===== 像素常量 ===== */
#ifndef WHITE
#define WHITE 0xFF
#endif
#ifndef BLACK
#define BLACK 0x00
#endif

/* ===== 全局单例 ===== */
ImageState g_img;

/* ===== 只读参考表 ===== */
const int g_road_width[IMAGE_HEIGHT + 1] = {
    0, 0, 0, 0, 14, 16, 16, 18, 18, 20,
    20, 20, 22, 22, 24, 24, 24, 26, 26, 28,
    28, 30, 30, 31, 32, 32, 34, 34, 35, 36,
    37, 38, 38, 39, 40, 41, 42, 43, 43, 44,
    45, 46, 47, 48, 49, 49, 50, 51, 52, 53,
    53, 56, 57, 58, 59, 59, 60, 61, 62, 63,
};

/**
 * 行权重表（WEIGHT_DIM × IMAGE_HEIGHT）
 * 维度0: 近处权重  维度1: 中距权重  维度2: 远处权重  维度3: 超远权重
 */
const float g_row_weight[WEIGHT_DIM][IMAGE_HEIGHT] = {
    /* 近处权重 */
    {
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
        0.5f,0.5f,0.5f,0.5f,0.5f,1.0f,1.5f,2.0f,2.5f,2.8f,
        3.0f,3.5f,4.2f,5.0f,5.8f,6.5f,6.3f,6.0f,5.8f,5.5f,
        5.0f,4.8f,4.5f,4.0f,3.5f,3.0f,2.5f,2.0f,1.5f,1.0f
    },
    /* 中距权重 */
    {
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.1f,0.2f,0.2f,0.3f,
        0.3f,0.3f,0.5f,0.5f,0.5f,0.9f,1.2f,1.8f,2.2f,2.3f,
        2.8f,3.0f,3.4f,4.0f,4.5f,5.0f,5.5f,6.2f,6.8f,6.5f,
        5.9f,5.3f,4.8f,4.5f,4.2f,4.0f,3.8f,3.5f,3.1f,2.8f,
        2.5f,2.1f,1.8f,1.4f,1.1f,1.0f,0.8f,0.6f,0.4f,0.2f
    },
    /* 远处权重 */
    {
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
        0.5f,0.5f,0.5f,1.0f,1.0f,1.4f,1.4f,1.5f,1.9f,2.0f,
        2.5f,2.5f,2.5f,3.0f,3.5f,3.9f,4.2f,4.8f,5.8f,6.3f,
        6.8f,8.0f,8.5f,8.1f,8.5f,8.0f,7.5f,7.2f,6.8f,6.3f,
        5.9f,5.3f,4.8f,4.5f,4.2f,4.0f,3.8f,3.5f,3.1f,2.8f,
        2.5f,2.1f,1.8f,1.4f,1.1f,1.0f,0.8f,0.6f,0.4f,0.2f
    },
    /* 超远权重 */
    {
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
        0.5f,0.5f,0.5f,1.0f,1.0f,1.4f,1.4f,1.5f,1.9f,2.0f,
        2.5f,2.5f,2.5f,4.0f,5.5f,6.9f,8.2f,8.8f,9.8f,9.3f,
        9.8f,9.5f,9.2f,8.8f,8.5f,8.0f,7.5f,7.0f,6.5f,5.3f,
        4.9f,4.3f,3.8f,3.5f,3.2f,3.0f,3.8f,3.5f,3.1f,2.8f,
        2.5f,2.1f,1.8f,1.4f,1.1f,1.0f,0.8f,0.6f,0.4f,0.2f
    }
};

/* ===== 模块私有变量 ===== */

/** 上一帧底行搜索起点（帧间延续） */
static int s_search_start = IMAGE_WIDTH / 2;

/** 中线历史（用于入环补线保留） */
static int s_mid_line_keep[IMAGE_HEIGHT + 1];

/** 辅助边界（barn/circle 内部重新扫描，不直接修改全局边界） */
static int s_tmp_left [IMAGE_HEIGHT];
static int s_tmp_right[IMAGE_HEIGHT];

/* ===== 私有函数前向声明 ===== */
static void        state_reset(void);
static void        preprocess_image(BinAlgorithm algo, uint8_t img[MT9V03X_H][MT9V03X_W]);
static void        scan_border_and_midline(void);
static void        calc_curvity(void);
static void        calc_variance(void);
static void        calc_weighted_error(void);
static void        scan_break_points(int start, int end);
static void        process_circle(int dir);
static void        process_block(int dir);
static unsigned char otsu_threshold(unsigned char img[IMAGE_HEIGHT][IMAGE_WIDTH]);
static void        sobel_auto_threshold(unsigned char img_in[IMAGE_HEIGHT][IMAGE_WIDTH],
                                        unsigned char img_out[IMAGE_HEIGHT][IMAGE_WIDTH]);
static void        linear_regression(int x[], int y[], int start, int end,
                                     float *k, float *b);
static float       edge_variance_from_regression(int edges[], int last_line);
static void        scan_black_pixels(int row, int *left_cnt, int *right_cnt);

/* 模糊化隶属度函数 */
static float fuzzify_left (float x, float x0, float x1);
static float fuzzify_mid  (float x, float x0, float xm, float x1);
static float fuzzify_right(float x, float x0, float x1);
static void  get_fuzzy_weights(float speed, float w[WEIGHT_DIM]);

/* ================================================================
 * 公开接口实现
 * ================================================================ */

/**
 * @brief 每帧图像处理主流程
 */
void image_analyze(uint8_t (*img)[MT9V03X_H][MT9V03X_W])
{
    state_reset();
    preprocess_image(BIN_ALGO_OTSU, *img);
    scan_border_and_midline();
    calc_curvity();
    calc_variance();

    int scan_end = IMG_LAST_LINE < 20 ? 20 : IMG_LAST_LINE;
    scan_break_points(55, scan_end);

    g_img.barn_exist      = image_search_barn(0, 0);
    g_img.circle_forecast = image_search_circle();

    if (g_img.circle_process_dir != ROAD_DIR_NONE)
        process_circle(g_img.circle_process_dir);

    if (g_img.block_dir != 0)
        process_block(g_img.block_dir);

    image_search_cross();
    calc_weighted_error();

    /* 单摄像头修正误差 */
    {
        float mid_val = 0.0f, result = 0.0f, wsum = 0.0f;
        for (int i = IMAGE_HEIGHT - 1; i > IMG_LAST_LINE; i--) {
            result += g_row_weight[1][i] * IMG_MID(i);
            wsum   += g_row_weight[1][i];
        }
        result = (wsum != 0.0f) ? (result / wsum) : MID_LINE_VAL;
        g_img.modify_err = result - MID_LINE_VAL;
    }

    image_draw_overlay();
}

void image_circle_activate(int dir)
{
    g_img.circle_process_dir  = dir;
    g_img.circle_proc_complete = 0;
}

void image_circle_reset(void)
{
    g_img.circle_process_dir = ROAD_DIR_NONE;
}

void image_block_reset(void)
{
    g_img.block_dir = 0;
}

/* ================================================================
 * 二值化
 * ================================================================ */

static void preprocess_image(BinAlgorithm algo, uint8_t img[MT9V03X_H][MT9V03X_W])
{
    unsigned char buf[IMAGE_HEIGHT][IMAGE_WIDTH];
    int m = 0, n = 0;

    /* 按压缩比采样原始图像 */
    for (int i = 0; i < MT9V03X_H; i += ROW_RAR) {
        n = 0;
        for (int j = 0; j < MT9V03X_W; j += COL_RAR)
            buf[m][n++] = img[i][j];
        m++;
    }

    if (algo == BIN_ALGO_SOBEL_AUTO) {
        sobel_auto_threshold(buf, g_img.binary);
        return;
    }

    /* OTSU 全局阈值 */
    g_img.dyn_thresh = otsu_threshold(buf);

    for (int i = 0; i < IMAGE_HEIGHT; i++)
        for (int j = 0; j < IMAGE_WIDTH; j++)
            g_img.binary[i][j] = (buf[i][j] > g_img.dyn_thresh) ? WHITE : BLACK;

    /* 入环时屏蔽顶部干扰区域 */
    if (get_road_type() == CIRCLE_IN) {
        for (int i = 0; i < SHIELD_LINE; i++)
            memset(g_img.binary[i], BLACK, IMAGE_WIDTH);
    }
}

/* ================================================================
 * 大津法阈值
 * ================================================================ */
static unsigned char otsu_threshold(unsigned char img[IMAGE_HEIGHT][IMAGE_WIDTH])
{
    /* ── 1. 建整数直方图 ── */
    uint16_t hist[256];
    memset(hist, 0, sizeof(hist));

    for (int i = 0; i < IMAGE_HEIGHT; i++)
        for (int j = 0; j < IMAGE_WIDTH; j++)
            hist[img[i][j]]++;          /* 最大值 4800，uint16 足够 */

    /* ── 2. 全局灰度加权总和 ── */
    const int32_t N = IMAGE_HEIGHT * IMAGE_WIDTH; /* 4800，编译期常量 */
    int32_t sum_total = 0;
    for (int i = 0; i < 256; i++)
        sum_total += (int32_t)i * hist[i]; /* max 1,224,000 → int32 ✓ */

    /* ── 3. 遍历阈值，最大化类间方差 ── */
    int32_t count0 = 0;
    int32_t sum0   = 0;
    int64_t best   = 0;
    int     thresh = 0;

    for (int i = 0; i < 256; i++) {
        count0 += hist[i];
        sum0   += (int32_t)i * hist[i];

        int32_t count1 = N - count0;
        if (count0 == 0 || count1 == 0) continue; /* 跳过边界无效段 */

        /*
         * t = sum0 * N - sum_total * count0
                  * 最大约 5.87e9，用 int64 承接，再右移 2 位压入 int32
                  * 以便平方后仍在 int64 范围内
         */
        int32_t ts = (int32_t)(
            ((int64_t)sum0 * N - (int64_t)sum_total * count0) >> 2
        );

        /* val = ts² / (count0 × count1)，全程 int64，无浮点 */
        int64_t val = (int64_t)ts * ts / ((int64_t)count0 * count1);

        if (val > best) {
            best   = val;
            thresh = i;
        }
    }

    return (unsigned char)thresh;

//    float hist[256] = {0};
//    unsigned int N = IMAGE_HEIGHT * IMAGE_WIDTH;
//    for (int i = 0; i < IMAGE_HEIGHT; i++)
//        for (int j = 0; j < IMAGE_WIDTH; j++)
//            hist[img[i][j]]++;
//
//    for (int i = 0; i < 256; i++)
//        hist[i] /= N;
//    float avg = 0.0f;
//    for (int i = 1; i < 256; i++)
//        avg += i * hist[i];
//    int   threshold    = 0;
//    float max_variance = 0.0f;
//    float w = 0.0f, u = 0.0f;
//    for (int i = 0; i < 256; i++) {
//        w += hist[i];
//        u += i * hist[i];
//        float t  = avg * w - u;
//        float dw = w * (1.0f - w);
//        if (dw < 1e-6f) continue;
//        float var = t * t / dw;
//        if (var > max_variance) {
//            max_variance = var;
//            threshold    = i;
//        }
//    }
//    return (unsigned char)threshold;
}

/* ================================================================
 * Sobel 自适应阈值二值化
 * ================================================================ */
static void sobel_auto_threshold(unsigned char in[IMAGE_HEIGHT][IMAGE_WIDTH],
                                 unsigned char out[IMAGE_HEIGHT][IMAGE_WIDTH])
{
    for (int i = 1; i < IMAGE_HEIGHT - 1; i++) {
        for (int j = 1; j < IMAGE_WIDTH - 1; j++) {
            short g[4];
            /* 4方向 Sobel */
            g[0] = -(short)in[i-1][j-1]+(short)in[i-1][j+1]
                   -(short)in[i  ][j-1]+(short)in[i  ][j+1]
                   -(short)in[i+1][j-1]+(short)in[i+1][j+1];
            g[1] = -(short)in[i-1][j-1]+(short)in[i+1][j-1]
                   -(short)in[i-1][j  ]+(short)in[i+1][j  ]
                   -(short)in[i-1][j+1]+(short)in[i+1][j+1];
            g[2] = -(short)in[i-1][j  ]+(short)in[i  ][j-1]
                   -(short)in[i  ][j+1]+(short)in[i+1][j  ]
                   -(short)in[i-1][j+1]+(short)in[i+1][j-1];
            g[3] = -(short)in[i-1][j  ]+(short)in[i  ][j+1]
                   -(short)in[i  ][j-1]+(short)in[i+1][j  ]
                   -(short)in[i-1][j-1]+(short)in[i+1][j+1];

            short gmax = 0;
            for (int k = 0; k < 4; k++) {
                short a = g[k] < 0 ? -g[k] : g[k];
                if (a > gmax) gmax = a;
            }

            /* 邻域像素和作为自适应分母 */
            short sum =
                (short)in[i-1][j-1]+(short)in[i-1][j]+(short)in[i-1][j+1]+
                (short)in[i  ][j-1]+(short)in[i  ][j]+(short)in[i  ][j+1]+
                (short)in[i+1][j-1]+(short)in[i+1][j]+(short)in[i+1][j+1];

            out[i][j] = (gmax > sum / 12.0f) ? BLACK : WHITE;
        }
    }
}

/* ================================================================
 * 边界扫描 & 中线计算
 * ================================================================ */
static void scan_border_and_midline(void)
{
    LaneLines  *L = &g_img.lanes;
    EdgeLoss   *E = &g_img.loss;

    /* ── 第一阶段：底部10行，以图像中心为起点全行扫描 ── */
    for (int i = IMAGE_HEIGHT - 1; i > IMAGE_HEIGHT - 10; i--) {
        int j_start = (i == IMAGE_HEIGHT - 1) ? s_search_start : L->mid_line[i + 1];
        j_start = (j_start < 1) ? 1 : (j_start > IMAGE_WIDTH - 2 ? IMAGE_WIDTH - 2 : j_start);

        /* 搜索左边界 */
        L->left_edge[i] = 0;
        for (int j = j_start; j >= 1; j--) {
            if (g_img.binary[i][j-1] == BLACK && g_img.binary[i][j] == WHITE) {
                L->left_edge[i] = j - 1;
                break;
            }
        }

        /* 搜索右边界 */
        L->right_edge[i] = IMAGE_WIDTH;
        j_start = (i == IMAGE_HEIGHT - 1) ? s_search_start : L->mid_line[i + 1];
        j_start = (j_start < 1) ? 1 : (j_start > IMAGE_WIDTH - 2 ? IMAGE_WIDTH - 2 : j_start);
        for (int j = j_start; j <= IMAGE_WIDTH - 2; j++) {
            if (g_img.binary[i][j] == WHITE && g_img.binary[i][j+1] == BLACK) {
                L->right_edge[i] = j + 1;
                break;
            }
        }

        /* 计算中线 */
        int lf = L->left_edge[i], rf = L->right_edge[i];
        int lp = L->left_edge[i+1], rp = L->right_edge[i+1];

        if (lf != 0 && rf != IMAGE_WIDTH) {
            L->mid_line[i] = (lf + rf) / 2;
        } else if (lf == 0 && rf != IMAGE_WIDTH) {
            E->left_cnt++;
            if (E->left_start_row == 0) E->left_start_row = i;
            L->mid_line[i] = ((rf - lf) >= (rp - lp + 1))
                             ? L->mid_line[i+1]
                             : (rf - g_road_width[i] / 2);
        } else if (lf != 0 && rf == IMAGE_WIDTH) {
            E->right_cnt++;
            if (E->right_start_row == 0) E->right_start_row = i;
            L->mid_line[i] = ((rf - lf) >= (rp - lp + 1))
                             ? L->mid_line[i+1]
                             : (lf + g_road_width[i] / 2);
        } else {
            E->both_cnt++;
            L->mid_line[i] = (i == IMAGE_HEIGHT - 1) ? IMAGE_WIDTH / 2 : L->mid_line[i+1];
        }

        /* 更新下一帧底行搜索起点 */
        if (i == IMAGE_HEIGHT - 1) {
            int m = L->mid_line[i];
            s_search_start = (m <= 12) ? 12 : (m >= IMAGE_WIDTH - 12 ? IMAGE_WIDTH - 12 : m);
        }
    }

    /* ── 第二阶段：从第10行向上边缘追踪 ── */
    for (int i = IMAGE_HEIGHT - 10; i > 2; i--) {
        int lp = L->left_edge[i+1], rp = L->right_edge[i+1];
        int j, jj;

        if (lp != 0 && rp != IMAGE_WIDTH) {
            /* 上行两侧均有效：窄窗口追踪 */
            j  = (lp + 3 >= IMAGE_WIDTH - 2) ? IMAGE_WIDTH - 2 : lp + 3;
            jj = (lp - 5 <= 1)              ? 1               : lp - 5;
            L->left_edge[i] = 0;
            for (; j >= jj; j--) {
                if (g_img.binary[i][j-1] == BLACK && g_img.binary[i][j] == WHITE) {
                    L->left_edge[i] = j; break;
                }
            }
            j  = (rp - 3 <= 1)              ? 1               : rp - 3;
            jj = (rp + 5 >= IMAGE_WIDTH - 2) ? IMAGE_WIDTH - 2 : rp + 5;
            L->right_edge[i] = IMAGE_WIDTH;
            for (; j <= jj; j++) {
                if (g_img.binary[i][j] == WHITE && g_img.binary[i][j+1] == BLACK) {
                    L->right_edge[i] = j; break;
                }
            }
        } else if (lp != 0 && rp == IMAGE_WIDTH) {
            /* 仅左侧有效 */
            j  = (lp + 10 >= IMAGE_WIDTH - 2) ? IMAGE_WIDTH - 2 : lp + 10;
            jj = (lp - 5  <= 1)               ? 1               : lp - 5;
            L->left_edge[i] = 0;
            for (; j >= jj; j--) {
                if (g_img.binary[i][j-1] == BLACK && g_img.binary[i][j] == WHITE) {
                    L->left_edge[i] = j; break;
                }
            }
            /* 右侧全行扫描 */
            L->right_edge[i] = IMAGE_WIDTH;
            j = L->mid_line[i+1];
            j = (j >= IMAGE_WIDTH - 2) ? IMAGE_WIDTH - 2 : j;
            for (; j <= IMAGE_WIDTH - 2; j++) {
                if (g_img.binary[i][j] == WHITE && g_img.binary[i][j+1] == BLACK) {
                    L->right_edge[i] = j; break;
                }
            }
        } else if (lp == 0 && rp != IMAGE_WIDTH) {
            /* 仅右侧有效 */
            j  = (rp - 10 <= 1)               ? 1               : rp - 10;
            jj = (rp + 5  >= IMAGE_WIDTH - 2)  ? IMAGE_WIDTH - 2 : rp + 5;
            L->right_edge[i] = IMAGE_WIDTH;
            for (; j <= jj; j++) {
                if (g_img.binary[i][j] == WHITE && g_img.binary[i][j+1] == BLACK) {
                    L->right_edge[i] = j; break;
                }
            }
            /* 左侧全行扫描 */
            L->left_edge[i] = 0;
            j = L->mid_line[i+1];
            j = (j <= 1) ? 1 : j;
            for (; j >= 1; j--) {
                if (g_img.binary[i][j-1] == BLACK && g_img.binary[i][j] == WHITE) {
                    L->left_edge[i] = j; break;
                }
            }
        } else {
            /* 两侧均丢失：全行双向扫描 */
            j = L->mid_line[i+1];
            L->left_edge[i] = 0;
            for (int jj2 = j; jj2 >= 1; jj2--) {
                if (g_img.binary[i][jj2-1] == BLACK && g_img.binary[i][jj2] == WHITE) {
                    L->left_edge[i] = jj2; break;
                }
            }
            L->right_edge[i] = IMAGE_WIDTH;
            for (int jj2 = j; jj2 <= IMAGE_WIDTH - 2; jj2++) {
                if (g_img.binary[i][jj2] == WHITE && g_img.binary[i][jj2+1] == BLACK) {
                    L->right_edge[i] = jj2; break;
                }
            }
        }

        /* 中线计算（同第一阶段逻辑） */
        int lf = L->left_edge[i], rf = L->right_edge[i];
        int lp2 = L->left_edge[i+1], rp2 = L->right_edge[i+1];

        if ((rf - lf) >= (rp2 - lp2 + 1)) {
            L->mid_line[i] = L->mid_line[i+1];
        } else if (lf != 0 && rf != IMAGE_WIDTH) {
            L->mid_line[i] = (lf + rf) / 2;
        } else if (lf != 0 && rf == IMAGE_WIDTH) {
            E->right_cnt++;
            if (E->right_start_row == 0) E->right_start_row = i;
            L->mid_line[i] = (lp2 != 0)
                             ? (L->mid_line[i+1] + lf - lp2)
                             : (lf + g_road_width[i] / 2);
        } else if (lf == 0 && rf != IMAGE_WIDTH) {
            E->left_cnt++;
            if (E->left_start_row == 0) E->left_start_row = i;
            L->mid_line[i] = (rp2 != IMAGE_WIDTH)
                             ? (L->mid_line[i+1] + rf - rp2)
                             : (rf - g_road_width[i] / 2);
        } else {
            E->both_cnt++;
            L->mid_line[i] = L->mid_line[i+1];
        }

        /* 检测前瞻终止条件 */
        if (i <= 4) {
            g_img.lookahead_row  = i;
            g_img.available_rows = IMAGE_HEIGHT - 1 - i;
            break;
        }

        int m = L->mid_line[i];
        m = (m < 4) ? 4 : (m > IMAGE_WIDTH - 4 ? IMAGE_WIDTH - 4 : m);

        int exceed_edge = (lf != 0 && lf >= IMAGE_WIDTH - 4) ||
                          (rf != IMAGE_WIDTH && rf <= 4)      ||
                          ((i >= 3) && g_img.binary[i-1][m  ] == BLACK
                                    && g_img.binary[i-1][m-1] == BLACK
                                    && g_img.binary[i-1][m+1] == BLACK);
        if (exceed_edge) {
            g_img.lookahead_row  = i;
            g_img.available_rows = IMAGE_HEIGHT - i;
            break;
        }
    }
}

/* ================================================================
 * 突变点扫描
 * ================================================================ */
static void scan_break_points(int start, int end)
{
    BreakStats *B = &g_img.breaks;
    LaneLines  *L = &g_img.lanes;

    int rw_recover = 0, le_recover = 0, re_recover = 0;

    for (int row = start; row > end + 1; row--) {
        int w_cur  = L->right_edge[row]   - L->left_edge[row];
        int w_next = L->right_edge[row+1] - L->left_edge[row+1];
        int w_bp   = (B->road_width_break_cnt > 0)
                     ? (L->right_edge[B->road_width_break_row] - L->left_edge[B->road_width_break_row])
                     : 0;

        /* 道路宽度突变 */
        if (w_cur > w_next + 5 || (B->road_width_break_cnt > 0 && w_cur > w_bp + 5)) {
            if (B->road_width_break_cnt == 0)
                B->road_width_break_row = row + 1;
            B->road_width_break_cnt++;
        } else if (B->road_width_break_cnt > 0 && w_cur < L->right_edge[B->road_width_break_row]) {
            rw_recover++;
        }

        /* 左边界突变 */
        if (L->left_edge[row] - L->left_edge[row+1] < -5 ||
            (B->left_break_cnt > 0 && L->left_edge[row] - L->left_edge[B->left_break_row] < 0)) {
            if (B->left_break_cnt == 0)
                B->left_break_row = row + 1;
            B->left_break_cnt++;
        } else if (B->left_break_cnt > 0 && L->left_edge[row] - L->left_edge[B->left_break_row] > 0) {
            le_recover++;
        }

        /* 右边界突变 */
        if (L->right_edge[row] - L->right_edge[row+1] > 5 ||
            (B->right_break_cnt > 0 && L->right_edge[row] - L->right_edge[B->right_break_row] > 0)) {
            if (B->right_break_cnt == 0)
                B->right_break_row = row + 1;
            B->right_break_cnt++;
        } else if (B->right_break_cnt > 0 && L->right_edge[row] - L->right_edge[B->right_break_row] < 0) {
            re_recover++;
        }
    }

    /* 拐点检测 */
    for (int i = start - 1; i > end + 1; i--) {
        int dl  = L->left_edge[i]  - L->left_edge[i+1];
        int dl2 = L->left_edge[i-1] - L->left_edge[i];
        if (dl >= -1 && dl <= 2 && dl2 <= -12) {
            B->left_inflexion.valid = 1;
            B->left_inflexion.row   = i;
            B->left_inflexion.col   = L->left_edge[i];
        }

        int dr  = L->right_edge[i]  - L->right_edge[i+1];
        int dr2 = L->right_edge[i-1] - L->right_edge[i];
        if (dr <= 1 && dr >= -2 && dr2 >= 12) {
            B->right_inflexion.valid = 1;
            B->right_inflexion.row   = i;
            B->right_inflexion.col   = L->right_edge[i];
        }
    }
    (void)rw_recover; (void)le_recover; (void)re_recover;
}

/* ================================================================
 * 圆形赛道预判
 * ================================================================ */
CircleForecast image_search_circle(void)
{
    CircleForecast  ret;
    EdgeGradientVar egv = image_get_edge_grad_var(IMG_LAST_LINE + 2, 45);
    BreakStats     *B   = &g_img.breaks;

    ret.left_grad_var  = egv.left;
    ret.right_grad_var = egv.right;

    /* 左环判断 */
    if (B->road_width_break_row > 30 && B->road_width_break_cnt > 7 &&
        B->left_break_row > 30 &&
        (B->left_break_cnt - B->right_break_cnt) > 7 &&
        g_img.left_circle_prehandle <= 0 &&
        B->right_break_cnt < 3 && egv.right < 0.4f)
    {
        ret.fore_dir = ROAD_DIR_LEFT;
    }
    /* 右环判断 */
    else if (B->road_width_break_row > 30 && B->road_width_break_cnt > 7 &&
             B->right_break_row > 30 &&
             (B->right_break_cnt - B->left_break_cnt) > 7 &&
             B->left_break_cnt <= 0 &&
             B->left_break_cnt < 3 && egv.left < 0.4f)
    {
        ret.fore_dir = ROAD_DIR_RIGHT;
    }
    else {
        ret.fore_dir = ROAD_DIR_NONE;
    }

    return ret;
}

/* ================================================================
 * 边界梯度方差
 * ================================================================ */
EdgeGradientVar image_get_edge_grad_var(int end_row, int min_valid_row)
{
    LaneLines       *L   = &g_img.lanes;
    EdgeGradientVar  ret = {10.0f, 10.0f};

    /* 右边界 */
    {
        int start = 40;
        for (int row = IMAGE_HEIGHT - 2; row > 40; row--) {
            if (L->right_edge[row] < IMAGE_WIDTH - 1) { start = row; break; }
        }
        float grad = 0.0f; int cnt = 0;
        for (int row = start; row >= end_row; row--) {
            grad += (L->right_edge[row] - L->right_edge[row+1]);
            cnt++;
        }
        if (cnt >= 10 && start > min_valid_row) {
            grad /= cnt;
            float var = 0.0f;
            for (int row = start; row >= end_row; row--) {
                float d = (float)(L->right_edge[row] - L->right_edge[row+1]) - grad;
                var += d * d;
            }
            ret.right = var / cnt;
        }
    }

    /* 左边界 */
    {
        int start = 40;
        for (int row = IMAGE_HEIGHT - 2; row > 40; row--) {
            if (L->left_edge[row] > 0) { start = row; break; }
        }
        float grad = 0.0f; int cnt = 0;
        for (int row = start; row >= end_row; row--) {
            grad += (L->left_edge[row] - L->left_edge[row+1]);
            cnt++;
        }
        if (cnt >= 10 && start > min_valid_row) {
            grad /= cnt;
            float var = 0.0f;
            for (int row = start; row >= end_row; row--) {
                float d = (float)(L->left_edge[row] - L->left_edge[row+1]) - grad;
                var += d * d;
            }
            ret.left = var / cnt;
        }
    }

    return ret;
}

/* ================================================================
 * 环形赛道图像修正
 * ================================================================ */
static void process_circle(int dir)
{
    LaneLines *L = &g_img.lanes;

    /* ---------- 左环 ---------- */
    if (dir == ROAD_DIR_LEFT) {
        if (g_img.circle_entry_active == 1) {
            int bp_row = 0, bp_col = 0;
            for (int row = IMAGE_HEIGHT - 1; row > 18; row--) {
                if (IMAGE_HEIGHT - 1 == row) {
                    L->right_edge[row] = IMAGE_WIDTH - 1;
                    for (int col = IMAGE_WIDTH / 2; col < IMAGE_WIDTH - 1; col++) {
                        if (g_img.binary[row][col] == WHITE && g_img.binary[row][col+1] == BLACK) {
                            L->right_edge[row] = col; break;
                        }
                    }
                } else {
                    L->right_edge[row] = IMAGE_WIDTH - 1;
                    for (int col = L->right_edge[row+1] - 3; col < IMAGE_WIDTH - 1; col++) {
                        if (g_img.binary[row][col] == WHITE && g_img.binary[row][col+1] == BLACK) {
                            L->right_edge[row] = col; break;
                        }
                    }
                }
                L->left_edge[row] = 0;
                for (int col = L->right_edge[row]; col > 0; col--) {
                    if (g_img.binary[row][col-1] == BLACK && g_img.binary[row][col] == WHITE) {
                        L->left_edge[row] = col; break;
                    }
                }
                if ((L->right_edge[row] - L->left_edge[row] < 8 && row <= 30) || row <= 20) {
                    g_img.lookahead_row = row;
                    /* 反向找突变点并插值修正 */
                    for (int j = row + 2; j < IMAGE_HEIGHT - 1; j++) {
                        if (-L->left_edge[j+1] + L->left_edge[j] > 6 &&
                            L->right_edge[j] - L->left_edge[j] > 5 &&
                            L->right_edge[j] - L->left_edge[j] < 50)
                        {
                            bp_row = j; bp_col = L->left_edge[j];
                            int mp = j + 30;
                            if (mp > IMAGE_HEIGHT - 1) mp = IMAGE_HEIGHT - 1;
                            float k = (float)(L->right_edge[mp] - bp_col) / (mp - j);
                            for (int ii = j; ii <= mp; ii++)
                                L->right_edge[ii] = (int)(bp_col + k * (ii - j));
                            /* 继续向上补线 */
                            for (int ii = j; ii >= 0; ii--) {
                                L->right_edge[ii] = IMAGE_WIDTH - 1;
                                int zs = (L->right_edge[ii+1] - 10 > 2) ? L->right_edge[ii+1] - 10 : 2;
                                int ze = L->right_edge[ii+1] + 2;
                                for (int z = zs; z < ze; z++) {
                                    if (g_img.binary[ii][z-1] == WHITE && g_img.binary[ii][z] == BLACK) {
                                        L->right_edge[ii] = z; break;
                                    }
                                }
                                if (L->right_edge[ii] - L->right_edge[ii+1] > 1) {
                                    g_img.lookahead_row = ii;
                                    for (int a = ii; a <= j + 1; a++) {
                                        L->left_edge[a] = 0;
                                        for (int b = L->right_edge[a]; b > 0; b--) {
                                            if (g_img.binary[a][b-1] == BLACK && g_img.binary[a][b] == WHITE) {
                                                L->left_edge[a] = b; break;
                                            }
                                        }
                                    }
                                    break;
                                }
                            }
                            /* 重新计算中线 */
                            for (int a = IMAGE_HEIGHT - 1; a > g_img.lookahead_row; a--) {
                                L->mid_line[a] = (L->left_edge[a] > 0)
                                                ? (L->right_edge[a] + L->left_edge[a]) / 2
                                                : (L->right_edge[a] - g_road_width[a] / 2);
                            }
                            break;
                        }
                    }
                    break;
                }
            }
            if (IMAGE_HEIGHT - 8 < bp_row || bp_col > IMAGE_WIDTH - 10)
                g_img.circle_proc_complete = 1;
        }

        if (g_img.circle_mending_active == 1) {
            int bp_row = 0;
            for (int row = IMAGE_HEIGHT - 1; row > g_img.lookahead_row; row--) {
                /* 重新找左/右边界（从左向右找左边界，再找右边界） */
                if (IMAGE_HEIGHT - 1 == row) {
                    L->left_edge[row] = 0;
                    for (int col = IMAGE_WIDTH / 2; col > 0; col--) {
                        if (g_img.binary[row][col] == WHITE && g_img.binary[row][col-1] == BLACK) {
                            L->left_edge[row] = col; break;
                        }
                    }
                } else {
                    L->left_edge[row] = 0;
                    for (int col = L->left_edge[row+1] + 5; col > 0; col--) {
                        if (g_img.binary[row][col] == WHITE && g_img.binary[row][col-1] == BLACK) {
                            L->left_edge[row] = col; break;
                        }
                    }
                }
                L->right_edge[row] = IMAGE_WIDTH - 1;
                for (int col = L->left_edge[row]; col < IMAGE_WIDTH - 1; col++) {
                    if (g_img.binary[row][col+1] == BLACK && g_img.binary[row][col] == WHITE) {
                        L->right_edge[row] = col; break;
                    }
                }
                if (bp_row == 0 && L->right_edge[row] - L->right_edge[row+1] > 1)
                    bp_row = row;
            }
            if (bp_row != 0) {
                int mp = g_img.lookahead_row;
                float k = (float)(0 - L->right_edge[bp_row]) / (mp - bp_row);
                for (int i = bp_row; i >= mp; i--)
                    L->right_edge[i] = (int)(L->right_edge[bp_row] + k * (i - bp_row));
                for (int a = IMAGE_HEIGHT - 1; a > g_img.lookahead_row; a--) {
                    L->mid_line[a] = L->right_edge[a] - g_road_width[a] / 2;
                    if (bp_row > 45) s_mid_line_keep[a] = L->mid_line[a];
                }
            } else {
                for (int a = IMAGE_HEIGHT - 1; a > g_img.lookahead_row; a--) {
                    L->mid_line[a] = (L->right_edge[a] < IMAGE_WIDTH - 5)
                                    ? (L->right_edge[a] + L->left_edge[a]) / 2
                                    : s_mid_line_keep[a];
                }
            }
        }

        if (g_img.circle_exit_detect == 1) {
            for (int row = g_img.lookahead_row; row < IMAGE_HEIGHT - 1; row++) {
                if (L->left_edge[row] - L->left_edge[row+1] > 3) {
                    g_img.breaks.left_break_row = row;
                    break;
                }
            }
        }
        if (g_img.circle_exit_active == 1) {
            for (int row = IMAGE_HEIGHT - 1; row > g_img.lookahead_row; row--) {
                L->mid_line[row]  = L->right_edge[row] - g_road_width[row] / 2;
                L->left_edge[row] = L->right_edge[row] - g_road_width[row];
            }
        }
    }

    /* ---------- 右环（对称逻辑，左右互换） ---------- */
    else if (dir == ROAD_DIR_RIGHT) {
        if (g_img.circle_entry_active == 1) {
            int bp_row = 0, bp_col = 0;
            for (int row = IMAGE_HEIGHT - 1; row > 18; row--) {
                if (IMAGE_HEIGHT - 1 == row) {
                    L->left_edge[row] = 0;
                    for (int col = IMAGE_WIDTH / 2; col > 0; col--) {
                        if (g_img.binary[row][col] == WHITE && g_img.binary[row][col-1] == BLACK) {
                            L->left_edge[row] = col; break;
                        }
                    }
                } else {
                    L->left_edge[row] = 0;
                    for (int col = L->left_edge[row+1] + 3; col > 0; col--) {
                        if (g_img.binary[row][col] == WHITE && g_img.binary[row][col-1] == BLACK) {
                            L->left_edge[row] = col; break;
                        }
                    }
                }
                L->right_edge[row] = IMAGE_WIDTH - 1;
                for (int col = L->left_edge[row]; col < IMAGE_WIDTH - 1; col++) {
                    if (g_img.binary[row][col] == WHITE && g_img.binary[row][col+1] == BLACK) {
                        L->right_edge[row] = col; break;
                    }
                }
                if ((L->right_edge[row] - L->left_edge[row] < 8 && row <= 30) || row <= 20) {
                    g_img.lookahead_row = row;
                    for (int j = row + 2; j < IMAGE_HEIGHT - 1; j++) {
                        if (L->right_edge[j] - L->right_edge[j+1] > 6 &&
                            L->right_edge[j] - L->left_edge[j] > 5 &&
                            L->right_edge[j] - L->left_edge[j] < 50)
                        {
                            bp_row = j; bp_col = L->right_edge[j];
                            int mp = j + 30;
                            if (mp > IMAGE_HEIGHT - 1) mp = IMAGE_HEIGHT - 1;
                            float k = (float)(L->left_edge[mp] - bp_col) / (mp - j);
                            for (int ii = j; ii <= mp; ii++)
                                L->left_edge[ii] = (int)(bp_col + k * (ii - j));
                            for (int a = IMAGE_HEIGHT - 1; a > g_img.lookahead_row; a--) {
                                L->mid_line[a] = (L->right_edge[a] < IMAGE_WIDTH - 5)
                                                ? (L->right_edge[a] + L->left_edge[a]) / 2
                                                : (L->left_edge[a] + g_road_width[a] / 2);
                            }
                            break;
                        }
                    }
                    break;
                }
            }
            if (IMAGE_HEIGHT - 8 < bp_row || bp_col < 10)
                g_img.circle_proc_complete = 1;
        }

        if (g_img.circle_mending_active == 1) {
            int bp_row = 0;
            for (int row = IMAGE_HEIGHT - 1; row > g_img.lookahead_row; row--) {
                if (IMAGE_HEIGHT - 1 == row) {
                    L->right_edge[row] = IMAGE_WIDTH - 1;
                    for (int col = IMAGE_WIDTH / 2; col < IMAGE_WIDTH - 1; col++) {
                        if (g_img.binary[row][col] == WHITE && g_img.binary[row][col+1] == BLACK) {
                            L->right_edge[row] = col; break;
                        }
                    }
                } else {
                    L->right_edge[row] = IMAGE_WIDTH - 1;
                    for (int col = L->right_edge[row+1] - 5; col < IMAGE_WIDTH - 1; col++) {
                        if (g_img.binary[row][col] == WHITE && g_img.binary[row][col+1] == BLACK) {
                            L->right_edge[row] = col; break;
                        }
                    }
                }
                L->left_edge[row] = 0;
                for (int col = L->right_edge[row]; col > 0; col--) {
                    if (g_img.binary[row][col-1] == BLACK && g_img.binary[row][col] == WHITE) {
                        L->left_edge[row] = col; break;
                    }
                }
                if (bp_row == 0 && L->left_edge[row] - L->left_edge[row+1] > 1)
                    bp_row = row;
            }
            if (bp_row != 0) {
                int mp = g_img.lookahead_row;
                float k = (float)(0 - L->left_edge[bp_row]) / (mp - bp_row);
                for (int i = bp_row; i >= mp; i--)
                    L->left_edge[i] = (int)(L->left_edge[bp_row] + k * (i - bp_row));
                for (int a = IMAGE_HEIGHT - 1; a > g_img.lookahead_row; a--) {
                    L->mid_line[a] = L->left_edge[a] + g_road_width[a] / 2;
                    if (bp_row > 45) s_mid_line_keep[a] = L->mid_line[a];
                }
            } else {
                for (int a = IMAGE_HEIGHT - 1; a > g_img.lookahead_row; a--) {
                    L->mid_line[a] = (L->left_edge[a] > 5)
                                    ? (L->right_edge[a] + L->left_edge[a]) / 2
                                    : s_mid_line_keep[a];
                }
            }
        }

        if (g_img.circle_exit_detect == 1) {
            for (int row = g_img.lookahead_row; row < IMAGE_HEIGHT - 1; row++) {
                if (L->right_edge[row] - L->right_edge[row+1] < -3) {
                    g_img.breaks.right_break_row = row;
                    break;
                }
            }
        }
        if (g_img.circle_exit_active == 1) {
            for (int row = IMAGE_HEIGHT - 1; row > g_img.lookahead_row; row--) {
                L->mid_line[row]   = L->left_edge[row] + g_road_width[row] / 2;
                L->right_edge[row] = L->left_edge[row] + g_road_width[row];
            }
        }
    }
}

/* ================================================================
 * 交叉路口检测与修正
 * ================================================================ */
void image_search_cross(void)
{
    LaneLines *L = &g_img.lanes;
    int lbp = 0, rbp = 0;
    g_img.is_cross = 0;

    for (int row = g_img.lookahead_row + 1; row < IMAGE_HEIGHT - 10; row++) {
        if (L->left_edge[row]  - L->left_edge[row+1]  >  4) lbp = row;
        if (L->right_edge[row] - L->right_edge[row+1] < -4) rbp = row;
        if (lbp != 0 && rbp != 0 && (int)fabs((float)(rbp - lbp)) < 3) {
            g_img.is_cross = 1;
            for (int r = IMAGE_HEIGHT - 1; r > g_img.lookahead_row; r--) {
                L->right_edge[r] = 40 + g_road_width[r] / 2;
                L->left_edge[r]  = 40 - g_road_width[r] / 2;
                L->mid_line[r]   = 40;
            }
        }
    }
}

/* ================================================================
 * 障碍物（避障）扫描辅助
 * ================================================================ */
static void scan_black_pixels(int row, int *left_cnt, int *right_cnt)
{
    *left_cnt  = 0;
    *right_cnt = 0;
    for (int i = IMAGE_WIDTH / 2; i > 5; i--)
        if (g_img.binary[row][i] == BLACK) (*left_cnt)++;
    for (int j = IMAGE_WIDTH / 2; j < IMAGE_WIDTH - 5; j++)
        if (g_img.binary[row][j] == BLACK) (*right_cnt)++;
}

int image_search_block(void)
{
    LaneLines *L   = &g_img.lanes;
    int bz  = 30; /* 参考行 */
    int w0  = abs(L->right_edge[bz-10] - L->left_edge[bz-10]);
    int w1  = abs(L->right_edge[bz]    - L->left_edge[bz]);
    int w2  = abs(L->right_edge[bz+10] - L->left_edge[bz+10]);

    if (abs(w1 - w0) < 15 || abs(w2 - w1) > 20) {
        int lc = 0, rc = 0;
        scan_black_pixels(35, &lc, &rc);
        if (lc + rc > 32) {
            int total_lose = g_img.loss.left_cnt + g_img.loss.right_cnt;
            if (total_lose < 33 && lc > rc) g_img.block_dir = 1;
            if (total_lose < 33 && rc > lc) g_img.block_dir = 2;
        }
    }
    return g_img.block_dir;
}

static void process_block(int dir)
{
    LaneLines *L = &g_img.lanes;
    int offset = (dir == 1) ? +30 : -30;
    for (int row = IMAGE_HEIGHT - 1; row > 0; row--)
        L->mid_line[row] += offset;
}

/* ================================================================
 * 车库检测
 * ================================================================ */
int image_search_barn(int dir, int valid)
{
    LaneLines *L      = &g_img.lanes;
    int        is_barn = 0;

    g_img.barn_break_row = 0;

    if (dir == ROAD_DIR_LEFT) {
        /* 重新扫描右边界 */
        for (int row = IMAGE_HEIGHT - 1; row > 18; row--) {
            s_tmp_right[row] = IMAGE_WIDTH - 1;
            if (valid) L->right_edge[row] = IMAGE_WIDTH - 1;
            if (row == IMAGE_HEIGHT - 1) {
                for (int col = IMAGE_WIDTH / 2; col < IMAGE_WIDTH - 1; col++) {
                    if (g_img.binary[row][col] == WHITE && g_img.binary[row][col+1] == BLACK) {
                        s_tmp_right[row] = col;
                        if (valid) L->right_edge[row] = col;
                        break;
                    }
                }
            } else {
                int cs = (s_tmp_right[row+1] + 5 > IMAGE_WIDTH - 2)
                         ? (s_tmp_right[row+1] + 5) : (IMAGE_WIDTH - 2);
                int ce = (s_tmp_right[row+1] - 8 > 0) ? (s_tmp_right[row+1] - 8) : 0;
                for (int col = cs; col > ce; col--) {
                    if (g_img.binary[row][col] == WHITE && g_img.binary[row][col+1] == BLACK) {
                        s_tmp_right[row] = col;
                        if (valid) L->right_edge[row] = col;
                        break;
                    }
                }
            }
        }
        /* 重新扫描左边界并统计跳变点 */
        for (int row = IMAGE_HEIGHT - 1; row > 18; row--) {
            s_tmp_left[row] = 0;
            if (valid) L->left_edge[row] = 0;
            L->jump_cnt[row] = 0;
            if (row == IMAGE_HEIGHT - 1) {
                for (int col = IMAGE_WIDTH / 2; col > 0; col--) {
                    if (g_img.binary[row][col] == WHITE && g_img.binary[row][col-1] == BLACK) {
                        s_tmp_left[row] = col;
                        if (valid) L->left_edge[row] = col;
                        break;
                    }
                }
            } else {
                int ref = (s_tmp_right[row] > IMAGE_WIDTH - 8) ? (IMAGE_WIDTH - 8) : s_tmp_right[row];
                for (int col = ref; col > 5; col--) {
                    if (g_img.binary[row][col] != g_img.binary[row][col-1])
                        L->jump_cnt[row]++;
                    if (g_img.binary[row][col+6]==WHITE && g_img.binary[row][col+5]==WHITE &&
                        g_img.binary[row][col+4]==WHITE && g_img.binary[row][col+3]==WHITE &&
                        g_img.binary[row][col+2]==WHITE && g_img.binary[row][col+1]==WHITE &&
                        g_img.binary[row][col  ]==WHITE && g_img.binary[row][col-1]==BLACK &&
                        g_img.binary[row][col-2]==BLACK && g_img.binary[row][col-3]==BLACK &&
                        g_img.binary[row][col-4]==BLACK && g_img.binary[row][col-5]==BLACK)
                    {
                        s_tmp_left[row] = col;
                        if (valid) L->left_edge[row] = col;
                        break;
                    }
                }
            }
        }
        int finish_cnt = 0;
        int start_row  = (g_img.lookahead_row > 30) ? g_img.lookahead_row : 30;
        for (int i = start_row; i < IMAGE_HEIGHT - 1; i++) {
            if (L->jump_cnt[i] > 9) {
                finish_cnt++;
                g_img.finish_line_row = i;
            }
        }
        if (finish_cnt >= 3 || valid)
            is_barn = 1;
    }

    else if (dir == ROAD_DIR_RIGHT) {
        /* 对称逻辑：先扫左边界，再扫右边界 */
        for (int row = IMAGE_HEIGHT - 1; row > 18; row--) {
            s_tmp_left[row] = 0;
            if (valid) L->left_edge[row] = 0;
            if (row == IMAGE_HEIGHT - 1) {
                for (int col = IMAGE_WIDTH / 2; col > 0; col--) {
                    if (g_img.binary[row][col] == WHITE && g_img.binary[row][col-1] == BLACK) {
                        s_tmp_left[row] = col;
                        if (valid) L->left_edge[row] = col;
                        break;
                    }
                }
            } else {
                int cs = (s_tmp_left[row+1] - 5 > 1) ? (s_tmp_left[row+1] - 5) : 1;
                int ce = (s_tmp_left[row+1] + 8 < IMAGE_WIDTH - 1) ? (s_tmp_left[row+1] + 8) : (IMAGE_WIDTH - 1);
                for (int col = cs; col < ce; col++) {
                    if (g_img.binary[row][col] == WHITE && g_img.binary[row][col-1] == BLACK) {
                        s_tmp_left[row] = col;
                        if (valid) L->left_edge[row] = col;
                        break;
                    }
                }
            }
        }
        for (int row = IMAGE_HEIGHT - 1; row > 18; row--) {
            s_tmp_right[row] = IMAGE_WIDTH - 1;
            if (valid) L->right_edge[row] = IMAGE_WIDTH - 1;
            L->jump_cnt[row] = 0;
            if (row == IMAGE_HEIGHT - 10) {
                for (int col = IMAGE_WIDTH / 2; col < IMAGE_WIDTH - 1; col++) {
                    if (g_img.binary[row][col] == WHITE && g_img.binary[row][col+1] == BLACK) {
                        s_tmp_right[row] = col;
                        if (valid) L->right_edge[row] = col;
                        break;
                    }
                }
            } else {
                int ref = (s_tmp_left[row] < 7) ? 7 : s_tmp_left[row];
                for (int col = ref; col < IMAGE_WIDTH - 6; col++) {
                    if (g_img.binary[row][col] != g_img.binary[row][col+1])
                        L->jump_cnt[row]++;
                    if (g_img.binary[row][col-6]==WHITE && g_img.binary[row][col-5]==WHITE &&
                        g_img.binary[row][col-4]==WHITE && g_img.binary[row][col-3]==WHITE &&
                        g_img.binary[row][col-2]==WHITE && g_img.binary[row][col-1]==WHITE &&
                        g_img.binary[row][col  ]==WHITE && g_img.binary[row][col+1]==BLACK &&
                        g_img.binary[row][col+2]==BLACK && g_img.binary[row][col+3]==BLACK &&
                        g_img.binary[row][col+4]==BLACK && g_img.binary[row][col+5]==BLACK)
                    {
                        s_tmp_right[row] = col;
                        if (valid) L->right_edge[row] = col;
                        break;
                    }
                }
            }
        }
        int finish_cnt = 0;
        int start_row  = (g_img.lookahead_row > 30) ? g_img.lookahead_row : 30;
        for (int i = start_row; i < IMAGE_HEIGHT - 1; i++) {
            if (L->jump_cnt[i] > 9) {
                finish_cnt++;
                g_img.finish_line_row = i;
            }
        }
        if (finish_cnt >= 3 || valid)
            is_barn = 1;
    }

    return is_barn;
}

/* ================================================================
 * 分叉口检测
 * ================================================================ */
int image_search_fork(void)
{
    LaneLines *L = &g_img.lanes;
    if (g_img.loss.both_cnt != 0 &&
        (g_img.loss.left_cnt > 30 || g_img.loss.right_cnt > 30))
        return 0;

    int lpt = 0, rpt = 0, fl = 0, fr = 0;
    for (int i = IMAGE_HEIGHT - 10; i > g_img.lookahead_row; i--) {
        if (L->left_edge[i] < L->left_edge[i+1] && L->left_edge[i] > 5) {
            lpt++;
            if (lpt == 1) fl = i;
        } else { lpt = 0; }

        if (L->right_edge[i] > L->right_edge[i+1] && L->right_edge[i] <= IMAGE_WIDTH - 5) {
            rpt++;
            if (rpt == 1) fr = i;
        } else { rpt = 0; }

        if (lpt == 4) break;
    }
    for (int i = IMAGE_HEIGHT - 10; i > g_img.lookahead_row; i--) {
        if (L->right_edge[i] > L->right_edge[i+1] && L->right_edge[i] <= IMAGE_WIDTH - 5) {
            rpt++;
            if (rpt == 1) fr = i;
        } else { rpt = 0; }
        if (rpt == 4) break;
    }

    if (lpt < 4 || rpt < 4 || abs(fr - fl) > 8) return 0;
    return 1;
}

/* ================================================================
 * 停止线检测
 * ================================================================ */
int image_search_stopline(void)
{
    /* 简化：检测底行黑色像素占比 */
    int black = 0;
    for (int i = 0; i < IMAGE_WIDTH; i++)
        if (g_img.binary[IMAGE_HEIGHT - 1][i] == BLACK) black++;
    return (black > IMAGE_WIDTH * 3 / 4) ? 1 : 0;
}

/* ================================================================
 * 列全黑扫描（出环辅助）
 * ================================================================ */
int image_col_scan_all_black(int start_row)
{
    int cnt = 0;
    for (int i = 0; i < IMAGE_WIDTH; i++) {
        int j;
        for (j = start_row; j < IMAGE_HEIGHT; j++) {
            if (g_img.binary[j][i] == WHITE) break;
        }
        if (j == IMAGE_HEIGHT) cnt++;
    }
    return cnt;
}

/* ================================================================
 * 出环判断（底行全黑像素 > 75）
 * ================================================================ */
int image_circle_exit_judge(void)
{
    int cnt = 0;
    for (int i = 0; i < IMAGE_WIDTH; i++)
        if (g_img.binary[IMAGE_HEIGHT - 1][i] == BLACK) cnt++;
    return (cnt > 75) ? 1 : 0;
}

/* ================================================================
 * 曲率计算
 * ================================================================ */
static void calc_curvity(void)
{
    float temp = 0.0f;
    for (int i = IMAGE_HEIGHT - 10; i > g_img.lookahead_row + 1; i--)
        temp += IMG_MID(i-1) - IMG_MID(i);

    if (g_img.available_rows > 2) {
        temp /= g_img.available_rows;
        temp  = (temp >  1.5f) ?  1.5f : temp;
        temp  = (temp < -1.5f) ? -1.5f : temp;
        g_img.curvity = temp;
    } else {
        g_img.curvity = 1.0f;
    }
}

/* ================================================================
 * 中线方差计算
 * ================================================================ */
static void calc_variance(void)
{
    float sum = 0.0f;
    for (int i = IMAGE_HEIGHT - 1; i > g_img.lookahead_row; i--) {
        float d = IMG_MID(i) - MID_LINE_VAL;
        sum += d * d;
    }
    g_img.variance = (g_img.lookahead_row < 50)
                    ? (sum / (IMAGE_HEIGHT - g_img.lookahead_row - 1))
                    : 1000.0f;
}

/* ================================================================
 * 模糊加权偏差计算（主要控制量）
 * ================================================================ */
static float fuzzify_left (float x, float x0, float x1)
{
    if (x <= x0) return 1.0f;
    if (x > x1)  return 0.0f;
    return (x1 - x) / (x1 - x0);
}

static float fuzzify_mid(float x, float x0, float xm, float x1)
{
    if (x <= x0 || x > x1) return 0.0f;
    if (x <= xm) return (x - x0) / (xm - x0);
    return (x1 - x) / (x1 - xm);
}

static float fuzzify_right(float x, float x0, float x1)
{
    if (x <= x0) return 0.0f;
    if (x >= x1) return 1.0f;
    return (x - x0) / (x1 - x0);
}

static void get_fuzzy_weights(float speed, float w[WEIGHT_DIM])
{
    w[0] = fuzzify_left (speed, 0.5f, 1.0f);
    w[1] = fuzzify_mid  (speed, 0.5f, 1.0f, 1.5f);
    w[2] = fuzzify_mid  (speed, 1.0f, 1.5f, 2.0f);
    w[3] = fuzzify_right(speed, 1.5f, 2.0f);
}

static void calc_weighted_error(void)
{
    float w[WEIGHT_DIM] = {0};
    float result[WEIGHT_DIM] = {0};
    float wsum[WEIGHT_DIM]   = {0};

    get_fuzzy_weights(filter_speed, w);

    for (int n = 0; n < WEIGHT_DIM; n++) {
        for (int i = IMAGE_HEIGHT - 1; i > g_img.lookahead_row; i--) {
            result[n] += g_row_weight[n][i] * IMG_MID(i);
            wsum[n]   += g_row_weight[n][i];
        }
        result[n] = (wsum[n] != 0.0f) ? (result[n] / wsum[n]) : MID_LINE_VAL;
    }

    float mid_val = 0.0f;
    for (int n = 0; n < WEIGHT_DIM; n++)
        mid_val += w[n] * result[n];

    g_img.image_error = mid_val - MID_LINE_VAL;
}

/* ================================================================
 * 线性回归（用于边界方差辅助）
 * ================================================================ */
static void linear_regression(int x[], int y[], int start, int end, float *k, float *b)
{
    if (end <= start) return;
    float t1 = 0, t2 = 0, t3 = 0, t4 = 0;
    int   sz = end - start;
    for (int i = start; i < end; i++) {
        t1 += (float)x[i] * x[i];
        t2 += x[i];
        t3 += (float)x[i] * y[i];
        t4 += y[i];
    }
    float denom = t1 * sz - t2 * t2;
    if (denom == 0.0f) return;
    *k = (t3 * sz - t2 * t4) / denom;
    *b = (t1 * t4 - t2 * t3) / denom;
}

static float edge_variance_from_regression(int edges[], int last_line)
{
    int idx[IMAGE_HEIGHT];
    for (int i = 0; i < IMAGE_HEIGHT; i++) idx[i] = i;

    int end = IMAGE_HEIGHT;
    int tmp = edges[IMAGE_HEIGHT - 1];
    for (int i = IMAGE_HEIGHT - 1; i > last_line; i--) {
        if (edges[i] != tmp) { end = i; break; }
    }

    float k = 0.0f, b = 0.0f;
    linear_regression(idx, edges, last_line + 1, end, &k, &b);

    float sum = 0.0f;
    for (int i = last_line + 1; i < IMAGE_HEIGHT; i++) {
        float d = edges[i] - (k * i + b);
        sum += d * d;
    }
    return sum / (IMAGE_HEIGHT - last_line - 1);
}

/* ================================================================
 * 可视化叠加（调试用，将中线/边界标记到 binary 图）
 * ================================================================ */
void image_draw_overlay(void)
{
    LaneLines *L = &g_img.lanes;
    int ll = g_img.lookahead_row;

    for (int i = IMAGE_HEIGHT - 1; i > ll; i--) {
        int m  = L->mid_line[i];
        int lf = L->left_edge[i]  + 2;
        int rf = L->right_edge[i] - 2;

        m  = CLAMP(m,  1, IMAGE_WIDTH - 2);
        lf = CLAMP(lf, 1, IMAGE_WIDTH - 2);
        rf = CLAMP(rf, 1, IMAGE_WIDTH - 2);

        g_img.binary[i][m]  = BLACK;
        g_img.binary[i][lf] = BLACK;
        g_img.binary[i][rf] = BLACK;
    }

    /* 在误差位置画3×3小方块 */
    int mc = (int)g_img.image_error + MID_LINE_VAL;
    mc = (mc < 1) ? 1 : (mc > IMAGE_WIDTH - 2 ? IMAGE_WIDTH - 2 : mc);
    for (int r = 40; r <= 42; r++)
        for (int c = mc - 1; c <= mc + 1; c++)
            g_img.binary[r][c] = BLACK;

    /* 前瞻行画横线 */
    memset(g_img.binary[ll], BLACK, IMAGE_WIDTH);
}

/* ================================================================
 * 状态复位（每帧开始调用）
 * ================================================================ */
static void state_reset(void)
{
    LaneLines  *L = &g_img.lanes;
    EdgeLoss   *E = &g_img.loss;
    BreakStats *B = &g_img.breaks;

    E->left_cnt      = 0;
    E->right_cnt     = 0;
    E->both_cnt      = 0;
    E->left_start_row  = 0;
    E->right_start_row = 0;

    g_img.lookahead_row  = 0;
    g_img.available_rows = 0;

//    g_road_width[IMAGE_HEIGHT] = IMAGE_WIDTH; /* 末尾哨兵 */

    for (int i = 0; i <= IMAGE_HEIGHT; i++) {
        L->left_edge[i]  = 0;
        L->right_edge[i] = IMAGE_WIDTH;
        L->mid_line[i]   = IMAGE_WIDTH / 2;
        L->jump_cnt[i]   = 0;
    }

    B->road_width_break_row = 0; B->road_width_break_cnt = 0;
    B->left_break_row       = 0; B->left_break_cnt       = 0;
    B->right_break_row      = 0; B->right_break_cnt      = 0;

    B->left_inflexion.valid  = 0;
    B->left_inflexion.row    = 0;
    B->left_inflexion.col    = 0;
    B->right_inflexion.valid = 0;
    B->right_inflexion.row   = 0;
    B->right_inflexion.col   = IMAGE_WIDTH;
}
