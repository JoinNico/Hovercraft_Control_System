#ifndef _IMAGE_H
#define _IMAGE_H

#include "zf_device_mt9v03x_dvp.h"
#include "zf_device_ips200.h"
#include "zf_common_font.h"
#include "zf_common_headfile.h"

#define IMAGE_HEIGHT 60 //120
#define IMAGE_WIDTH  80 //160
#define MID_LINE_VAL 40 //80
#define SHIELD_LINE  18 //36

//1就是不压缩，2就是压缩一倍
#define ROW_RAR 1 //行向压缩一倍
#define COL_RAR 2 //列向压缩一倍 //1

#define BARN_DIR    smartcar_param.barn_dir//1

typedef enum
{
    OSTU ,
    ANYWAY_Algorithm_1 ,
    ANYWAY_Algorithm_2 ,
    SobelAutoThreshold ,
}Bin_Image_Algorithm_Config;

typedef struct Inflexion//拐点
{
    uint8 flag;
    uint8 row;
    uint8 col;
}Inflexion;

typedef struct {
    float left_edge;
    float right_edge;
}gradient_variance_ty;

typedef struct {
    int circle_fore_dir;
    float left_edge_grad_var;
    float right_edge_grad_var;
}circle_forecast_ty;

extern int road_width[IMAGE_HEIGHT + 1];
extern int last_line;//前瞻所在行
extern int longest_col;//前瞻所在列
extern int available_line;//可用行数
extern float curvity;
extern float image_error;
extern float modify_err ;
extern int break_cnt;
extern int left_edge[IMAGE_HEIGHT + 1];
extern int right_edge[IMAGE_HEIGHT + 1];
extern int middle_line[IMAGE_HEIGHT + 1];
extern unsigned char binary_image[IMAGE_HEIGHT][IMAGE_WIDTH];
extern unsigned char composite_image[IMAGE_HEIGHT][IMAGE_WIDTH];
extern unsigned char dynamic_thresh;
extern int left_lose;
extern int right_lose;
extern int all_lose;
extern int jump_point_num[IMAGE_HEIGHT + 1];
extern int break_point[2];
extern int circle_image_process_complete;
extern int barn_break_point;
extern int circle_prehandle_lock;
extern int right_circle_prehandle;
extern int left_circle_prehandle;
extern int cross_handle;
extern int road_width_break_point;
extern int road_width_break_cnt;
extern int right_edge_break_point;
extern int right_edge_break_cnt;
extern int left_edge_break_point;
extern int left_edge_break_cnt;
extern int circle_entry_process;
extern int circle_exit_detect;
extern int circle_exit_process;
extern int in_circle_mending;
extern circle_forecast_ty circle_forecast;
extern int barn_exist_flag;
extern float curvity_r;
extern float curvity_l;
extern int fps;
extern int finish_line_row;
extern Inflexion left_inflexion;
extern Inflexion right_inflexion;
extern int is_cross;
extern int block_in_process;

static inline void set_last_line(int x)         {last_line = x;}
static inline void set_available_line(int x)    {available_line = x;}
static inline void set_curvity(float x)             {curvity = x;}
static inline int get_last_line(void)               {return last_line;}
static inline int get_available_line(void)      {return available_line;}
static inline float get_curvity(void)               {return curvity;}
static inline float get_image_error(void)           {return image_error;}
static inline float get_image_middle_line(void) {return image_error;}
static inline int get_break_cnt(void)   {return break_cnt;}
float get_image_error_single(void);
void analyze_image(void);
void clear_process_circle_flag(void);
void set_process_circle_flag(int dir);
float get_variance();
int col_scan_all_white(int start_row);
gradient_variance_ty get_edge_gradient_variance(int end, int min_valid_row);
int search_barn(int dir, int valid);
int search_block(void);
void search_cross(void);
void clear_process_block_flag(void);
int search_stopline(void);
void check_cheku(uint8 start_point, uint8 end_point ,uint8 qiangdu);
int search_fork(void);
circle_forecast_ty search_circle(void);
void calc_image_error(void);
void complex_image(void);
int Circle_Exit_Judgement(void);
#endif
