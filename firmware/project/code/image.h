#ifndef _IMAGE_H_
#define _IMAGE_H_

#include "zf_device_mt9v03x_dvp.h"
#include "zf_device_ips200.h"
#include "zf_common_font.h"
#include "zf_common_headfile.h"

#define IMAGEH  MT9V03X_W//#define IMAGEH  MT9V034_IMAGEH   /* 摄像头采集高度 */
#define IMAGEW  MT9V03X_H//#define IMAGEW  MT9V034_IMAGEW   /* 摄像头采集宽度 */

#define LCDH    120  /* TFT显示高度（用户使用）高度 */
#define LCDW    160  /* TFT显示宽度（用户使用）宽度 */
#define MID_LINE_VAL    (LCDW/2)                //固定中线值
#define MAIN_RUN_LINE   (LCDH/2)                //主跑行

#define bin_jump_num    1//跳过的点数
#define BOEDER_MAX  LCDW-2 //边界最大值
#define BORDER_MIN  1   //边界最小值

//typedef enum
//{
//    OSTU ,
//    ANYWAY_Algorithm_1 ,
//    ANYWAY_Algorithm_2 ,
//    SobelAutoThreshold ,
//}Bin_Image_Algorithm_Config;

extern unsigned char Image_Use[LCDH][LCDW];/** 压缩后之后用于存放屏幕显示数据  */
extern unsigned char Bin_Image[LCDH][LCDW];/** 二值化后用于OLED显示的数据 */
extern uint8 l_border[LCDH];//左线数组
extern uint8 r_border[LCDH];//右线数组
extern uint8 center_line[LCDH];//中线数组
extern uint8 middle_line;//主跑行中线值

void analyze_image(void);
void image_show(void);
#endif
