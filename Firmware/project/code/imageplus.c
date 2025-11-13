#include "imageplus.h"

#ifdef WHITE
#undef WHITE
#endif
#define WHITE 0xff
//#define WHITE 1

#ifdef BLACK
#undef BLACK
#endif
#define BLACK 0

//int road_width[IMAGE_HEIGHT + 1] = {
//        80, 80, 80, 80, 21, 21, 21, 22, 22, 23,
//
//        23, 24, 24, 25, 25, 26, 26, 26, 27, 28,
//
//        28, 28, 29, 30, 30, 30, 30, 31, 32, 32,
//
//        32, 33, 34, 34, 34, 34, 35, 36, 36, 36,
//
//        37, 38, 38, 38, 39, 39, 40, 40, 41, 41,
//
//        42, 44, 45, 45, 45, 46, 47, 47, 47, 47,
//};

int road_width[IMAGE_HEIGHT + 1] = {
        0, 0, 0, 0, 14, 16, 16, 18, 18, 20,

        20, 20, 22, 22, 24, 24, 24, 26, 26, 28,

        28, 30, 30, 31, 32, 32, 34, 34, 35, 36,

        37, 38, 38, 39, 40, 41, 42, 43, 43, 44,

        45, 46, 47, 48, 49, 49, 50, 51, 52, 53,

        53, 56, 57, 58, 59, 59, 60, 61, 62, 63,
};
#define DIMENSION 4

float image_row_weight[DIMENSION][IMAGE_HEIGHT] = {
        //近处权重
        {
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//9
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//19
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//29
                0.5, 0.5, 0.5, 0.5, 0.5, 1.0, 1.5, 2.0, 2.5, 2.8,//39
                3.0, 3.5, 4.2, 5.0, 5.8, 6.5, 6.3, 6.0, 5.8, 5.5,//49
                5.0, 4.8, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0 //59
        },
        //中处权重
        {
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//9
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.2, 0.3,//19
                0.3, 0.3, 0.5, 0.5, 0.5, 0.9, 1.2, 1.8, 2.2, 2.3,//29
                2.8, 3.0, 3.4, 4.0, 4.5, 5.0, 5.5, 6.2, 6.8, 6.5,//39
                5.9, 5.3, 4.8, 4.5, 4.2, 4.0, 3.8, 3.5, 3.1, 2.8,//49
                2.5, 2.1, 1.8, 1.4, 1.1, 1.0, 0.8, 0.6, 0.4, 0.2 //59
        },
        //远处权重
        {
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//9
                0.5, 0.5, 0.5, 1.0, 1.0, 1.4, 1.4, 1.5, 1.9, 2.0,//19
                2.5, 2.5, 2.5, 3.0, 3.5, 3.9, 4.2, 4.8, 5.8, 6.3,//29
                6.8, 8.0, 8.5, 8.1, 8.5, 8.0, 7.5, 7.2, 6.8, 6.3,//39
                5.9, 5.3, 4.8, 4.5, 4.2, 4.0, 3.8, 3.5, 3.1, 2.8,//49
                2.5, 2.1, 1.8, 1.4, 1.1, 1.0, 0.8, 0.6, 0.4, 0.2 //59
        },
        //超远处权重
        {
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//9
                0.5, 0.5, 0.5, 1.0, 1.0, 1.4, 1.4, 1.5, 1.9, 2.0,//19
                2.5, 2.5, 2.5, 4.0, 5.5, 6.9, 8.2, 8.8, 9.8, 9.3,//29
                9.8, 9.5, 9.2, 8.8, 8.5, 8.0, 7.5, 7.0, 6.5, 5.3,//39
                4.9, 4.3, 3.8, 3.5, 3.2, 3.0, 3.8, 3.5, 3.1, 2.8,//49
                2.5, 2.1, 1.8, 1.4, 1.1, 1.0, 0.8, 0.6, 0.4, 0.2 //59
        }
};


int fps = 0;
unsigned char binary_image[IMAGE_HEIGHT][IMAGE_WIDTH];
int left_edge[IMAGE_HEIGHT + 1];
int right_edge[IMAGE_HEIGHT + 1];
int middle_line[IMAGE_HEIGHT + 1];
int middle_line_keep[IMAGE_HEIGHT + 1];
int jump_point_num[IMAGE_HEIGHT + 1];
int search_start = IMAGE_WIDTH / 2;//搜索起点
int last_line = 0;//前瞻所在行
int longest_col = 0;//前瞻所在列
int available_line = 0;//可用行数
int left_lose = 0;//左边丢线数量
int right_lose = 0;//右边丢线数量
int all_lose = 0;//两边同时丢线 数量
int left_lose_start = 0;//记录左边丢线的开始行
int right_lose_start = 0;//记录右边边丢线的开始行
int white_start = 0;//全白开始点
int white_num = 0;//白线数量
int break_cnt = 0;//突变点计数
float curvity = 0;
float image_error = 0;
float modify_err = 0;
unsigned char dynamic_thresh;
Inflexion left_inflexion;//左拐点
Inflexion right_inflexion;//右拐点
int road_width_break_point = 0;
int road_width_break_cnt = 0;
int right_edge_break_point = 0;
int right_edge_break_cnt = 0;
int left_edge_break_point = 0;
int left_edge_break_cnt = 0;

unsigned char
Otsu1D(unsigned char img_2d[IMAGE_HEIGHT][IMAGE_WIDTH], unsigned short height, unsigned short width)//一维大津法，计算全局阈值
{
    float Histogram[256] = {0};//建立一维灰度直方图,并初始化变量
    uint32 N = height * width;//像素的总数

    for (int i = 0; i < height; i++)//矩阵的行数
    {
        for (int j = 0; j < width; j++)//矩阵的列数
        {
            unsigned char gray_val = img_2d[i][j];//获取当前灰度值
            Histogram[gray_val]++;//记录（i,j）的数量
        }
    }
    for (int i = 0; i < 256; i++)
    {
        Histogram[i] /= N;//归一化的每一个二元组的概率分布
    }

    float avg_val = 0.0;

    for (int i = 1; i < 256; i++)
    {
        avg_val += i * Histogram[i];
    }


    int threshold = 0; //阈值
    float max_variance = 0.0;
    float w = 0.0, u = 0.0;

    for (int i = 0; i < 256; i++)
    {
        w += Histogram[i];
        u += i * Histogram[i];

        float t = avg_val * w - u;
        float variance = t * t / (w * (1 - w));

        if (variance > max_variance)
        {
            max_variance = variance;
            threshold = i;
        }
    }
    return threshold;
}

/*!
 * @brief    基于soble边沿检测算子的一种自动阈值边沿检测
 * @param    imageIn    输入数组
 *           imageOut   输出数组      保存的二值化后的边沿信息
 * @return
 * @note
 * @example
 * @date     2020/5/15
 */
void lq_sobelAutoThreshold(unsigned char imageIn[IMAGE_HEIGHT][IMAGE_WIDTH],
                           unsigned char imageOut[IMAGE_HEIGHT][IMAGE_WIDTH])
{
    /** 卷积核大小 */
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = IMAGE_WIDTH - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = IMAGE_HEIGHT - KERNEL_SIZE / 2;
    short i, j, k;
    short temp[4];
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            /* 计算不同方向梯度幅值  */
            temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
                      - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]                      // {-1, 0, 1},
                      - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];             // {-1, 0, 1}};

            temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
                      - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]                      // { 0,  0,  0},
                      - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];             // { 1,  1,  1}};

            temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]             //  0, -1, -1
                      - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]                      //  1,  0, -1
                      - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];             //  1,  1,  0

            temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]             // -1, -1,  0
                      - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]                      // -1,  0,  1
                      - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];             //  0,  1,  1

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
            temp[2] = abs(temp[2]);
            temp[3] = abs(temp[3]);

            /* 找出梯度幅值最大值  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
            temp[3] = (short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
                      + (short) imageIn[i][j - 1] + (short) imageIn[i][j] + (short) imageIn[i][j + 1]
                      + (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];

            if (temp[0] > temp[3] / 12.0f)
            {
                imageOut[i][j] = BLACK;
            }
            else
            {
                imageOut[i][j] = WHITE;
            }
        }
    }
}

void preprocess_image(unsigned char mode)
{
    int m = 0, n = 0;
    unsigned char image_use[IMAGE_HEIGHT][IMAGE_WIDTH];
    //压缩图像
    for (int i = 0; i < MT9V03X_H; i += ROW_RAR)
    {
        n = 0;
        for (int j = 0; j < MT9V03X_W; j += COL_RAR)
        {
            image_use[m][n] = mt9v03x_image[i][j];
            n++;
        }
        m++;
    }
    if (mode == OSTU)
    {
        dynamic_thresh = Otsu1D(image_use, IMAGE_HEIGHT, IMAGE_WIDTH);
    }
    else if (mode == SobelAutoThreshold)
    {
        lq_sobelAutoThreshold(image_use, binary_image);  //动态调节阈值
        return;
    }

    for (int i = 0; i < IMAGE_HEIGHT; i++)
    {
        for (int j = 0; j < IMAGE_WIDTH; j++)
        {
            if (image_use[i][j] > dynamic_thresh)
            {
                binary_image[i][j] = WHITE;
            }
            else
            {
                binary_image[i][j] = BLACK;
            }
        }
    }

    if (get_road_type() == CIRCLE_IN)
    {
        //去除上行多余图像
        for (int i = 0; i < SHIELD_LINE; i++)
        {
            for (int j = 0; j < IMAGE_WIDTH; j++)
            {
                binary_image[i][j] = BLACK;
            }
        }
    }


}

void search_border_line_and_Mid_line(void)
{
    //printf("image_analyze start\n\r");
    int break_flag = 0;
    int break_point[2] = {0, 59};
    for (int i = IMAGE_HEIGHT - 1; i > IMAGE_HEIGHT - 10; i--)
    {
        int j = 0;
        //search left border
        if (i == IMAGE_HEIGHT - 1)//首行就以图像中心作为扫描起点
        {
            j = search_start;//IMAGE_WIDTH / 2
        }
        else
        {
            j = middle_line[i + 1];//否则就以上一行中点的位置作为本行扫描起点
        }
        if (j <= 1)
        {
            j = 1;
        }
        if (j >= IMAGE_WIDTH - 2)
        {
            j = IMAGE_WIDTH - 2;
        }
        while (j >= 1)
        {
            if (binary_image[i][j - 1] == BLACK && binary_image[i][j] == WHITE && binary_image[i][j + 1] == WHITE)
            {
                left_edge[i] = j - 1;
                break;
            }
            j--;
        }

        //search right border
        if (i == IMAGE_HEIGHT - 1) //再找右边界
        {
            j = search_start;//如果首行，从图像中心开始搜寻
        }
        else
        {
            j = middle_line[i + 1];//否则从上一行中心位置开始搜寻
        }
        if (j >= IMAGE_WIDTH - 2)//j >= IMAGE_WIDTH - 3有效范围内搜寻右线
        {
            j = IMAGE_WIDTH - 2;
        }
        if (j <= 1)
        {
            j = 1;
        }
        while (j <= IMAGE_WIDTH - 2)
        {
            if (binary_image[i][j - 1] == WHITE && binary_image[i][j] == WHITE && binary_image[i][j + 1] == BLACK)
            {
                right_edge[i] = j + 1;
                break;
            }
            j++;
        }


        //calculate middle line
        if (left_edge[i] != 0 && right_edge[i] != IMAGE_WIDTH)//中线判断，没有丢线
        {
            middle_line[i] = (left_edge[i] + right_edge[i]) / 2;
        }
        else if (left_edge[i] == 0 && right_edge[i] != IMAGE_WIDTH)//丢了左线
        {
            left_lose++;//记录只有左线丢的数量
            if (left_lose_start == 0)
                left_lose_start = i;
            if ((right_edge[i] - left_edge[i]) >= (right_edge[i + 1] - left_edge[i + 1] + 1))//突变
            {
                middle_line[i] = middle_line[i + 1];//用上一行的中点
            }
            else
            {
                middle_line[i] = right_edge[i] - road_width[i] / 2;//正常的话就用半宽补
            }
        }
        else if (left_edge[i] != 0 && right_edge[i] == IMAGE_WIDTH)//丢了右线
        {
            right_lose++;//记录只有右线丢的数量
            if (right_lose_start == 0)
                right_lose_start = i;
            if ((right_edge[i] - left_edge[i]) >= (right_edge[i + 1] - left_edge[i + 1] + 1))//突变
            {
                middle_line[i] = middle_line[i + 1];//用上一行的中点
            }
            else
            {
                middle_line[i] = left_edge[i] + road_width[i] / 2;//正常的话就用半宽补
            }
        }
        else if (left_edge[i] == 0 && right_edge[i] == IMAGE_WIDTH)//两边都丢了的话
        {
            all_lose++;
            if (i == IMAGE_HEIGHT - 1)//如果是首行就以图像中心作为中点
            {
                middle_line[i] = IMAGE_WIDTH / 2;
            }
            else
            {
                middle_line[i] = middle_line[i + 1];//如果不是首行就用上一行的中线作为本行中点
            }
        }
        if (middle_line[IMAGE_HEIGHT - 1] >= IMAGE_WIDTH - 12)
        {
            search_start = IMAGE_WIDTH - 12;
        }
        else if (middle_line[IMAGE_HEIGHT - 1] <= 12)
        {
            search_start = 12;
        }
        else
        {
            search_start = middle_line[IMAGE_HEIGHT - 1];//记录本帧图像第59行的中线值，作为下一幅图像的59行扫描起始点
        }
        //printf("%d %d %d\n\r", left_edge[i], right_edge[i], middle_line[i]);
    }
    //printf("image_analyze\n\r");
    for (int i = IMAGE_HEIGHT - 10; i > 2; i--)
    {
        if (left_edge[i + 1] != 0 && right_edge[i + 1] != IMAGE_WIDTH) //上一行两边都找到 启用边沿扫描
        {
            int j = ((left_edge[i + 1] + 3) >= IMAGE_WIDTH - 2) ? IMAGE_WIDTH - 2 : (left_edge[i + 1] +
                                                                                     3);//int j = ((left_edge[i + 1] + 10) >= IMAGE_WIDTH - 6) ? IMAGE_WIDTH - 6 : (left_edge[i + 1] + 10);//先找左边界
            int jj = ((left_edge[i + 1] - 5) <= 1) ? 1 : (left_edge[i + 1] - 5);
            while (j >= jj)
            {
                if (binary_image[i][j - 1] == BLACK && binary_image[i][j] == WHITE)//找黑白跳变
                {
                    left_edge[i] = j;
                    break;
                }
                j--;
            }
            j = ((right_edge[i + 1] - 3) <= 1) ? 1 : (right_edge[i + 1] -
                                                      3);//j = ((right_edge[i + 1] - 10) <= 5) ? 5 : (right_edge[i + 1] - 10);//再找右边界
            jj = ((right_edge[i + 1] + 5) >= IMAGE_WIDTH - 2) ? IMAGE_WIDTH - 2 : (right_edge[i + 1] + 5);
            while (j <= jj)
            {
                if (binary_image[i][j] == WHITE && binary_image[i][j + 1] == BLACK)//找白黑跳变
                {
                    right_edge[i] = j;
                    break;
                }
                j++;
            }
        }
        else if (left_edge[i + 1] != 0 && right_edge[i + 1] == IMAGE_WIDTH)//上一行只找到左边界
        {
            int j = ((left_edge[i + 1] + 10) >= IMAGE_WIDTH - 2) ? IMAGE_WIDTH - 2 : (left_edge[i + 1] + 10);//左边界用边沿扫描
            int jj = ((left_edge[i + 1] - 5) <= 1) ? 1 : (left_edge[i + 1] - 5);
            while (j >= jj)
            {
                if (binary_image[i][j - 1] == BLACK && binary_image[i][j] == WHITE)//找黑白
                {
                    left_edge[i] = j;
                    break;
                }
                j--;
            }
            j = middle_line[i + 1];//上一行丢了右边界用全行扫描
            if (j >= IMAGE_WIDTH - 2)
            {
                j = IMAGE_WIDTH - 2;
            }
            while (j <= IMAGE_WIDTH - 2)
            {
                if (binary_image[i][j] == WHITE && binary_image[i][j + 1] == BLACK)
                {
                    right_edge[i] = j;
                    break;
                }
                j++;
            }
        }
        else if (left_edge[i + 1] == 0 && right_edge[i + 1] != IMAGE_WIDTH)//上一行只找到右边界
        {
            int j = ((right_edge[i + 1] - 10) <= 1) ? 1 : (right_edge[i + 1] - 10);//边缘追踪找右边界
            int jj = ((right_edge[i + 1] + 5) >= IMAGE_WIDTH - 2) ? IMAGE_WIDTH - 2 : (right_edge[i + 1] + 5);
            while (j <= jj)
            {
                if (binary_image[i][j] == WHITE && binary_image[i][j + 1] == BLACK)//找白黑
                {
                    right_edge[i] = j;
                    break;
                }
                j++;
            }
            j = middle_line[i + 1];//全行扫描找左边界
            if (j <= 1)
            {
                j = 1;
            }
            while (j >= 1)
            {
                if (binary_image[i][j - 1] == BLACK && binary_image[i][j] == WHITE)
                {
                    left_edge[i] = j;
                    break;
                }
                j--;
            }
        }
        else if (left_edge[i + 1] == 0 && right_edge[i + 1] == IMAGE_WIDTH)//上一行没找到边界，可能是十字或者环形
        {
            int j = middle_line[i + 1];//全行找左边界
            while (j >= 1)
            {
                if (binary_image[i][j - 1] == BLACK && binary_image[i][j] == WHITE)
                {
                    left_edge[i] = j;
                    break;
                }
                j--;
            }
            j = middle_line[i + 1];//全行找右边界
            while (j <= IMAGE_WIDTH - 2)
            {
                if (binary_image[i][j] == WHITE && binary_image[i][j + 1] == BLACK)
                {
                    right_edge[i] = j;
                    break;
                }
                j++;
            }
        }
        if ((right_edge[i] - left_edge[i]) >= (right_edge[i + 1] - left_edge[i + 1] + 1))//不满足畸变
        {
            middle_line[i] = middle_line[i + 1];//用上一行
        }
        else
        {

            if (left_edge[i] != 0 && right_edge[i] != IMAGE_WIDTH)
            {
                middle_line[i] = (left_edge[i] + right_edge[i]) / 2;
            }
            else if (left_edge[i] != 0 && right_edge[i] == IMAGE_WIDTH)//只找到左
            {
                right_lose++;
                if (right_lose_start == 0)
                    right_lose_start = i;
                if (left_edge[i + 1] != 0)//上一行找到了，用偏移量
                {
                    middle_line[i] = middle_line[i + 1] + (left_edge[i] - left_edge[i + 1]);
                }
                else//否则就用赛道宽度一半补
                {
                    middle_line[i] = left_edge[i] + road_width[i] / 2;
                }
            }
            else if (left_edge[i] == 0 && right_edge[i] != IMAGE_WIDTH)//只找到右
            {
                left_lose++;
                if (left_lose_start == 0)
                    left_lose_start = i;
                if (right_edge[i + 1] != IMAGE_WIDTH)//上一行找到了，用偏移量
                {
                    middle_line[i] = middle_line[i + 1] + (right_edge[i] - right_edge[i + 1]);
                }
                else//否则就用赛道宽度一半补
                {
                    middle_line[i] = right_edge[i] - road_width[i] / 2;
                }
            }
            else if (left_edge[i] == 0 && right_edge[i] == IMAGE_WIDTH)//两边丢线
            {
                white_num++;
                all_lose++;
                if (white_num == 1)
                {
                    white_start = i;
                }
                middle_line[i] = middle_line[i + 1];//用上一行的
            }
        }
        if (i <= 4)
        {
            set_last_line(i);
            set_available_line(IMAGE_HEIGHT - 1 - i);
            break;
        }
        int m = middle_line[i];
        if (m < 4)
        {
            m = 4;
        }
        if (m > IMAGE_WIDTH - 4)
        {
            m = IMAGE_WIDTH - 4;
        }
        if ((left_edge[i] != 0 && left_edge[i] >= IMAGE_WIDTH - 4) ||
            (right_edge[i] != IMAGE_WIDTH && right_edge[i] <= 4) ||
            ((i >= 3) && (binary_image[i - 1][m] == BLACK) && (binary_image[i - 1][m - 1] == BLACK) &&
             (binary_image[i - 1][m + 1] == BLACK))) //最后一行
        {
            set_last_line(i);//i);//最后一行，动态前瞻
            set_available_line(IMAGE_HEIGHT - i);//有效行数
            break;
        }
    }
}

//计算边界斜率的方差，从IMAGE_HEIGHT - 2算到end
gradient_variance_ty get_edge_gradient_variance(int end, int min_valid_row)
{
    float gradient_l = 0;
    float curvity_tmp_l = 0;
    int gradient_cnt_l = 0;
    int start_row_l = 40;
    float gradient_r = 0;
    float curvity_tmp_r = 0;
    int gradient_cnt_r = 0;
    int start_row_r = 40;
    gradient_variance_ty ret;

    for (int row = IMAGE_HEIGHT - 2; row > 40; row--)
    {
        if (right_edge[row] < 79)
        {
            start_row_r = row;
            break;
        }
    }
    for (int row = start_row_r; row >= end; row--)
    {
        gradient_r += (right_edge[row] - right_edge[row + 1]);
        gradient_cnt_r++;
    }
    if (gradient_cnt_r >= 10 && start_row_r > min_valid_row)
    {
        gradient_r /= (float) gradient_cnt_r;
        for (int row = start_row_r; row >= end; row--)
        {
            curvity_tmp_r += pow((right_edge[row] - right_edge[row + 1] - gradient_r), 2);
        }
        curvity_tmp_r /= (float) gradient_cnt_r;

        //first_order_lpf(&curvity_r, curvity_tmp_r, 50, 200);
        ret.right_edge = curvity_tmp_r;
    }
    else
    {
        ret.right_edge = 10;
    }

    for (int row = IMAGE_HEIGHT - 2; row > 40; row--)
    {
        if (left_edge[row] > 0)
        {
            start_row_l = row;
            break;
        }
    }
    for (int row = start_row_l; row >= end; row--)
    {
        gradient_l += (left_edge[row] - left_edge[row + 1]);
        gradient_cnt_l++;
    }
    if (gradient_cnt_l >= 10 && start_row_l > min_valid_row)
    {
        gradient_l /= (float) gradient_cnt_l;
        for (int row = start_row_l; row >= end; row--)
        {
            curvity_tmp_l += pow((left_edge[row] - left_edge[row + 1] - gradient_l), 2);
        }
        curvity_tmp_l /= (float) gradient_cnt_l;

        //first_order_lpf(&curvity_l, curvity_tmp_l, 50, 200);
        ret.left_edge = curvity_tmp_l;
    }
    else
    {
        ret.left_edge = 10;
    }

    return ret;
}

int right_circle_prehandle = 0;
int left_circle_prehandle = 0;
int cross_handle = 0;
float circle_prehandle_dis = 0;
circle_forecast_ty circle_forecast;

circle_forecast_ty search_circle(void)
{
    circle_forecast_ty ret;
    gradient_variance_ty edge_grad_var = get_edge_gradient_variance(get_last_line() + 2, 45);
    ret.left_edge_grad_var = edge_grad_var.left_edge;
    ret.right_edge_grad_var = edge_grad_var.right_edge;
//        printf("g:%.2f,%.2f\n\r", edge_grad_var.left_edge, edge_grad_var.right_edge);

//        printf("rwbp:%d\r\n",road_width_break_point);
//        printf("rwbc:%d\r\n",road_width_break_cnt);
//
//        printf("rebp:%d\r\n",right_edge_break_point);
//        printf("rebc:%d\r\n",right_edge_break_cnt);
//
//        printf("lebc:%d\r\n",left_edge_break_cnt);
//        printf("rebc-lebc:%d\r\n",right_edge_break_cnt - left_edge_break_cnt);
//
//        printf("egv_l:%.2f\r\n",edge_grad_var.left_edge);


    //左环补线
    if (road_width_break_point > 30 && road_width_break_cnt > 7 && left_edge_break_point > 30
        && left_edge_break_cnt - right_edge_break_cnt > 7 && left_circle_prehandle <= 0
        && right_edge_break_cnt < 3 && edge_grad_var.right_edge < 0.4)
    {
        ret.circle_fore_dir = 0;
    }
        //右环补线
    else if (road_width_break_point > 30 && road_width_break_cnt > 7 && right_edge_break_point > 30
             && right_edge_break_cnt - left_edge_break_cnt > 7 && left_edge_break_cnt <= 0
             && left_edge_break_cnt < 3 && edge_grad_var.left_edge < 0.4)
    {
        ret.circle_fore_dir = 1;
    }
    else
    {
        ret.circle_fore_dir = -1;
    }

    return ret;
}

int circle_image_process_complete = 0;
int break_point[2];
//int circle_forecast_process = 0;
int circle_entry_process = 0;
int in_circle_mending = 0;
int circle_exit_detect = 0;
int circle_exit_process = 0;

void process_circle_in_image(int dir)
{
    //左环
    if (0 == dir)
    {
        if (1 == left_circle_forecast_flag)
        {
            for (int row = IMAGE_HEIGHT - 1; row > get_last_line(); row--)
            {
                middle_line[row] = right_edge[row] - road_width[row] / 2;
                left_edge[row] = right_edge[row] - road_width[row];
            }

        }
        if (circle_entry_process == 1)
        {
            break_point[1] = 0;
            break_point[0] = 0;
            for (int row = IMAGE_HEIGHT - 1; row > 18; row--)
            {
                //重新寻找右线
                if (IMAGE_HEIGHT - 1 == row)
                {
                    right_edge[row] = IMAGE_WIDTH - 1;
                    for (int col = IMAGE_WIDTH / 2; col < IMAGE_WIDTH - 1; col++)
                    {
                        if (WHITE == binary_image[row][col] && BLACK == binary_image[row][col + 1])
                        {
                            right_edge[row] = col;
                            break;
                        }
                    }
                }
                else
                {
                    right_edge[row] = IMAGE_WIDTH - 1;
                    for (int col = right_edge[row + 1] - 3; col < IMAGE_WIDTH - 1; col++)
                    {
                        if (WHITE == binary_image[row][col] && BLACK == binary_image[row][col + 1])
                        {
                            right_edge[row] = col;
                            break;
                        }
                    }
                }
                //根据右线向左寻找左线
                left_edge[row] = 0;
                for (int col = right_edge[row]; col > 0; col--)
                {
                    if (BLACK == binary_image[row][col - 1] && WHITE == binary_image[row][col])
                    {
                        left_edge[row] = col;
                        break;
                    }
                }
                //printf("%d:%d:%d\n\r", row, left_edge[row], right_edge[row]);
                //根据左右宽度计算左后一行
                if ((right_edge[row] - left_edge[row] < 8 && row <= 30) || row <= 20)
                {
                    set_last_line(row);
                    //printf("------%d\n\r", row);
                    //反向寻找突变点
                    for (int j = row + 2; j < IMAGE_HEIGHT - 1; j++)
                    {
                        //printf("--%d:%d\n\r", j, left_edge[j]);
                        if (-left_edge[j + 1] + left_edge[j] > 6 && right_edge[j] - left_edge[j] > 5 &&
                            right_edge[j] - left_edge[j] < 50)
                        {
                            break_point[0] = j;
                            break_point[1] = left_edge[j];
                            int modify_point = j + 30;
                            if (modify_point > IMAGE_HEIGHT - 1)
                                modify_point = IMAGE_HEIGHT - 1;
                            if (modify_point < 0)
                                modify_point = 0;
                            float k = (float) (right_edge[modify_point] - left_edge[j]) / (modify_point - j);
                            //修补突变点下方的右边界
                            for (int i = j; i <= modify_point; i++)
                            {
                                right_edge[i] = (int) (left_edge[j] + k * (i - j));
                            }
                            //修补突变点上方右边界
                            for (int i = j; i >= 0; i--)
                            {
                                right_edge[i] = IMAGE_WIDTH - 1;
                                for (int z = ((right_edge[i + 1] - 10) > 2 ? (right_edge[i + 1] - 10) : 2);
                                     z < right_edge[i + 1] + 2; z++)
                                {
                                    if (WHITE == binary_image[i][z - 1] && BLACK == binary_image[i][z])
                                    {
                                        right_edge[i] = z;
                                        break;
                                    }
                                }
                                if (right_edge[i] - right_edge[i + 1] > 1)
                                {
                                    set_last_line(i);
                                    //再次重新寻左边界
                                    for (int a = i; a <= j + 1; a++)
                                    {
                                        left_edge[a] = 0;
                                        for (int b = right_edge[a]; b > 0; b--)
                                        {
                                            if (BLACK == binary_image[a][b - 1] && WHITE == binary_image[a][b])
                                            {
                                                left_edge[a] = b;
                                                break;
                                            }
                                        }
                                        //if(BLACK == binary_image[a - 1][left_edge[a]])
                                        //  break;
                                    }
                                    break;
                                }
                            }
                            //printf("break:%d k:%f\n\r", j, k);
                            //计算中线
                            for (int a = IMAGE_HEIGHT - 1; a > get_last_line(); a--)
                            {
                                if (left_edge[a] > 0)
                                    middle_line[a] = (right_edge[a] + left_edge[a]) / 2;
                                else
                                {
                                    //半宽补线
                                    middle_line[a] = right_edge[a] - road_width[a] / 2;
                                    //斜率补线
                                    //middle_line[a] = middle_line[a + 1] + (right_edge[a] - right_edge[a + 1]);
                                }
                            }

                            break;
                        }
                    }
                    break;
                }
            }
            if (IMAGE_HEIGHT - 8 < break_point[0] ||
                break_point[1] > IMAGE_WIDTH - 10/*|| (break_point[1] > 74 && break_point[0] > 0)*/)
                circle_image_process_complete = 1;
            //printf("%d,%d\n\r", break_point[0], break_point[1]);
        }
        if (in_circle_mending == 1)//左环
        {
            break_point[1] = 0;
            break_point[0] = 0;
            //修补环出口部分，并检测环出口
            for (int row = IMAGE_HEIGHT - 1; row > get_last_line(); row--)
            {
                //重新寻找左线
                if (IMAGE_HEIGHT - 1 == row)
                {
                    left_edge[row] = 0;
                    for (int col = IMAGE_WIDTH / 2; col > 0; col--)
                    {
                        if (WHITE == binary_image[row][col] && BLACK == binary_image[row][col - 1])
                        {
                            left_edge[row] = col;
                            break;
                        }
                    }
                }
                else
                {
                    left_edge[row] = 0;
                    for (int col = left_edge[row + 1] + 5; col > 0; col--)
                    {
                        if (WHITE == binary_image[row][col] && BLACK == binary_image[row][col - 1])
                        {
                            left_edge[row] = col;
                            break;
                        }
                    }
                }
                //根据左线向右寻找右线
                right_edge[row] = IMAGE_WIDTH - 1;
                for (int col = left_edge[row]; col < IMAGE_WIDTH - 1; col++)
                {
                    if (BLACK == binary_image[row][col + 1] && WHITE == binary_image[row][col])
                    {
                        right_edge[row] = col;
                        break;
                    }
                }
                if (break_point[0] == 0 && right_edge[row] - right_edge[row + 1] > 1)
                {
                    break_point[0] = row;
                    break_point[1] = right_edge[row];
                }
            }
            if (break_point[0] != 0)
            {
                int modify_point = get_last_line();
                float k = (float) (0 - right_edge[(break_point[0])]) / (modify_point - (break_point[0]));
                //修补突变点上方右边界
                for (int i = break_point[0]; i >= modify_point; i--)
                {
                    right_edge[i] = (int) (right_edge[(break_point[0])] + k * (i - (break_point[0])));
                }
                for (int a = IMAGE_HEIGHT - 1; a > get_last_line(); a--)
                {
                    //if(right_edge[a] < 75)
                    //   middle_line[a] = right_edge[a] - road_width[a] / 2;
                    //middle_line[a] = middle_line[a + 1] + (right_edge[a] - right_edge[a + 1]);
                    //       middle_line[a] = (right_edge[a] + left_edge[a]) / 2;
                    //else
                    // {

                    //半宽补线
                    middle_line[a] = right_edge[a] - road_width[a] / 2;
                    //斜率补线
                    //middle_line[a] = middle_line[a + 1] + (right_edge[a] - right_edge[a + 1]);
                    if (break_point[0] > 45)
                    {
                        middle_line_keep[a] = middle_line[a];
                    }
                    // }

                }
            }
            else
            {
                for (int a = IMAGE_HEIGHT - 1; a > get_last_line(); a--)
                {
                    if (right_edge[a] < 75)
                        //middle_line[a] = right_edge[a] - road_width[a] / 2;
                        middle_line[a] = (right_edge[a] + left_edge[a]) / 2;
                    else
                    {
                        //半宽补线
                        //middle_line[a] = right_edge[a] - road_width[a] / 2;
                        //斜率补线
                        //middle_line[a] = middle_line[a + 1] + (right_edge[a] - right_edge[a + 1]);
                        middle_line[a] = middle_line_keep[a];
                    }
                }
            }
        }
        if (circle_exit_detect == 1)
        {
            for (int row = get_last_line(); row < IMAGE_HEIGHT - 1; row++)
            {
                if (left_edge[row] - left_edge[row + 1] > 3)
                {
                    break_point[0] = row;
                    break_point[1] = left_edge[row];
                    break;
                }
            }
        }
        if (circle_exit_process == 1)
        {
            for (int row = IMAGE_HEIGHT - 1; row > get_last_line(); row--)
            {
                middle_line[row] = right_edge[row] - road_width[row] / 2;
                left_edge[row] = right_edge[row] - road_width[row];
            }
        }
    }
    else if (1 == dir)//右环
    {
        if (1 == right_circle_forecast_flag)
        {
            for (int row = IMAGE_HEIGHT - 1; row > get_last_line(); row--)
            {
                middle_line[row] = left_edge[row] + road_width[row] / 2;
                right_edge[row] = left_edge[row] + road_width[row];
            }
        }

        if (circle_entry_process == 1)
        {
            break_point[0] = 0;
            break_point[1] = IMAGE_WIDTH;
            for (int row = IMAGE_HEIGHT - 1; row > 18; row--)
            {
                //重新寻找左线
                if (IMAGE_HEIGHT - 1 == row)
                {
                    left_edge[row] = 0;
                    for (int col = IMAGE_WIDTH / 2; col > 0; col--)
                    {
                        if (WHITE == binary_image[row][col] && BLACK == binary_image[row][col - 1])
                        {
                            left_edge[row] = col;
                            break;
                        }
                    }
                }
                else
                {
                    left_edge[row] = 0;
                    for (int col = left_edge[row + 1] + 3; col > 0; col--)
                    {
                        if (WHITE == binary_image[row][col] && BLACK == binary_image[row][col - 1])
                        {
                            left_edge[row] = col;
                            break;
                        }
                    }
                }
                //根据左线向右寻找右线
                right_edge[row] = IMAGE_WIDTH - 1;
                for (int col = left_edge[row]; col < IMAGE_WIDTH - 1; col++)
                {
                    if (BLACK == binary_image[row][col + 1] && WHITE == binary_image[row][col])
                    {
                        right_edge[row] = col;
                        break;
                    }
                }
                if ((right_edge[row] - left_edge[row] < 8 && row <= 30) || row <= 20)
                {
                    set_last_line(row);

                    //printf("------%d\n\r", row);
                    //反向寻找突变点
                    for (int j = row + 2; j < IMAGE_HEIGHT - 1; j++)
                    {
                        //printf("--%d:%d\n\r", j, left_edge[j]);
                        if (right_edge[j + 1] - right_edge[j] > 6 && right_edge[j] - left_edge[j] > 5 &&
                            right_edge[j] - left_edge[j] < 50)
                        {
                            break_point[0] = j;
                            break_point[1] = right_edge[j];
                            int modify_point = j + 30;
                            if (modify_point > IMAGE_HEIGHT - 1)
                                modify_point = IMAGE_HEIGHT - 1;
                            if (modify_point < 0)
                                modify_point = 0;
                            float k = (float) (left_edge[modify_point] - right_edge[j]) / (modify_point - j);
                            //修补突变点下方的左边界
                            for (int i = j; i <= modify_point; i++)
                            {
                                left_edge[i] = (int) (right_edge[j] + k * (i - j));
                            }
                            //修补突变点上方左边界
                            for (int i = j; i >= 0; i--)
                            {
                                left_edge[i] = 0;
                                for (int z = ((left_edge[i + 1] + 10) < IMAGE_WIDTH - 2 ? (left_edge[i + 1] + 10) :
                                              IMAGE_WIDTH - 2); z > left_edge[i + 1] - 2; z--)
                                {
                                    if (WHITE == binary_image[i][z + 1] && BLACK == binary_image[i][z])
                                    {
                                        left_edge[i] = z;
                                        break;
                                    }
                                }
                                if (left_edge[i] - left_edge[i + 1] < -1)
                                {
                                    set_last_line(i);
                                    //再次重新寻右边界
                                    for (int a = i; a <= j + 1; a++)
                                    {
                                        right_edge[a] = IMAGE_WIDTH - 1;
                                        for (int b = left_edge[a]; b < IMAGE_WIDTH - 1; b++)
                                        {
                                            if (BLACK == binary_image[a][b + 1] && WHITE == binary_image[a][b])
                                            {
                                                right_edge[a] = b;
                                                break;
                                            }
                                        }
                                        //if(BLACK == binary_image[a - 1][left_edge[a]])
                                        //  break;
                                    }
                                    break;
                                }
                            }
                            //printf("break:%d k:%f\n\r", j, k);
                            //计算中线
                            for (int a = IMAGE_HEIGHT - 1; a > get_last_line(); a--)
                            {
                                if (right_edge[a] < IMAGE_WIDTH - 1)
                                    middle_line[a] = (right_edge[a] + left_edge[a]) / 2;
                                else
                                {
                                    //半宽补线
                                    middle_line[a] = left_edge[a] + road_width[a] / 2;
                                    //斜率补线
                                    //middle_line[a] = middle_line[a + 1] + (left_edge[a] - left_edge[a + 1]);
                                }
                            }
                            break;
                        }
                    }
                    break;
                }
            }
            //printf("%d,%d\n\r", break_point[0], break_point[1]);
            if (IMAGE_HEIGHT - 8 < break_point[0] ||
                break_point[1] < 10/*|| (break_point[1] < 6 && break_point[0] > 30)*/)
                circle_image_process_complete = 1;
        }
        if (in_circle_mending == 1)
        {
            break_point[1] = IMAGE_WIDTH;
            break_point[0] = 0;
            //修补环出口部分，并检测环出口
            for (int row = IMAGE_HEIGHT - 1; row > get_last_line(); row--)
            {
                //重新寻找右线
                if (IMAGE_HEIGHT - 1 == row)
                {
                    right_edge[row] = IMAGE_WIDTH - 1;
                    for (int col = IMAGE_WIDTH / 2; col < IMAGE_WIDTH - 1; col++)
                    {
                        if (WHITE == binary_image[row][col] && BLACK == binary_image[row][col + 1])
                        {
                            right_edge[row] = col;
                            break;
                        }
                    }
                }
                else
                {
                    right_edge[row] = IMAGE_WIDTH - 1;
                    for (int col = right_edge[row + 1] - 5; col < IMAGE_WIDTH - 1; col++)
                    {
                        if (WHITE == binary_image[row][col] && BLACK == binary_image[row][col + 1])
                        {
                            right_edge[row] = col;
                            break;
                        }
                    }
                }
                //根据右线向左寻找左线
                left_edge[row] = 0;
                for (int col = right_edge[row]; col > 0; col--)
                {
                    if (BLACK == binary_image[row][col - 1] && WHITE == binary_image[row][col])
                    {
                        left_edge[row] = col;
                        break;
                    }
                }

                if (break_point[0] == 0 && left_edge[row] - left_edge[row + 1] < -1)
                {
                    break_point[0] = row;
                    break_point[1] = left_edge[row];
                }
            }
            if (break_point[0] != 0)
            {
                int modify_point = get_last_line();
                float k = (float) (IMAGE_WIDTH - 1 - left_edge[break_point[0]]) / (modify_point - (break_point[0]));
                //修补突变点上方右边界
                for (int i = break_point[0]; i >= modify_point; i--)
                {
                    left_edge[i] = (int) (left_edge[break_point[0]] + k * (i - (break_point[0])));
                }
//                  float k = (float)(IMAGE_WIDTH - 1 - left_edge[break_point[0] + 5]) / (modify_point - (break_point[0] + 5));
//                  //修补突变点上方右边界
//                  for(int i = break_point[0] + 5; i >= modify_point; i--)
//                  {
//                          left_edge[i] = (int)(left_edge[break_point[0] + 5] + k * (i - (break_point[0] + 5)));
//                  }
//                  float k = (float)(IMAGE_WIDTH - 1 - left_edge[break_point[0] + 10]) / (modify_point - (break_point[0] + 10));
//                  //修补突变点上方右边界
//                  for(int i = break_point[0] + 10; i >= modify_point; i--)
//                  {
//                          left_edge[i] = (int)(left_edge[break_point[0] + 10] + k * (i - (break_point[0] + 10)));
//                  }
                for (int a = IMAGE_HEIGHT - 1; a > get_last_line(); a--)
                {
                    // if(right_edge[a] < IMAGE_WIDTH - 1)
                    //       middle_line[a] = (right_edge[a] + left_edge[a]) / 2;
                    //else
                    //{
                    //半宽补线
                    //middle_line[a] = left_edge[a] - road_width[a] / 2;
                    //斜率补线
                    middle_line[a] = middle_line[a + 1] + (left_edge[a] - left_edge[a + 1]);
                    if (break_point[0] > 45)
                    {
                        //middle_line_keep[a] = middle_line[a]-30;
                        middle_line_keep[a] = middle_line[a];
                    }
                    //}
                }
            }
            else
            {
                for (int a = IMAGE_HEIGHT - 1; a > get_last_line(); a--)
                {
                    if (left_edge[a] > 5)
                        //middle_line[a] = left_edge[a] - road_width[a] / 2;
                        middle_line[a] = (right_edge[a] + left_edge[a]) / 2;
                        //middle_line[a] = middle_line[a + 1] + (left_edge[a] - left_edge[a + 1]);
                    else
                    {
                        //半宽补线
                        //middle_line[a] = left_edge[a] - road_width[a] / 2;
                        //斜率补线
                        //middle_line[a] = middle_line[a + 1] + (left_edge[a] - left_edge[a + 1]);
                        middle_line[a] = middle_line_keep[a];
                    }

                }
            }
        }
        if (circle_exit_detect == 1)
        {
            for (int row = get_last_line(); row < IMAGE_HEIGHT - 1; row++)
            {
                if (right_edge[row] - right_edge[row + 1] < -3)
                {
                    break_point[0] = row;
                    break_point[1] = left_edge[row];
                    break;
                }
            }
        }
        if (circle_exit_process == 1)
        {
            for (int row = IMAGE_HEIGHT - 1; row > get_last_line(); row--)
            {
                middle_line[row] = left_edge[row] + road_width[row] / 2;
                right_edge[row] = left_edge[row] + road_width[row];
            }
        }
    }
}


int is_cross = 0;
void search_cross(void)
{
    int left_break_point = 0;
    int right_break_point = 0;
    is_cross = 0;
    for (int row = get_last_line() + 1; row < IMAGE_HEIGHT - 10; row++)
    {
        if (left_edge[row] - left_edge[row + 1] > 4)
        {
            left_break_point = row;
        }
        if (right_edge[row] - right_edge[row + 1] < -4)
        {
            right_break_point = row;
        }
        if (fabs(right_break_point - left_break_point) < 3 && left_break_point != 0 && right_break_point != 0)
        {
            is_cross = 1;
            for (int row = IMAGE_HEIGHT - 1; row > get_last_line(); row--)
            {
                right_edge[row] = 40 + road_width[row]/2;
                left_edge[row] = 40 - road_width[row]/2;
                middle_line[row] = (left_edge[row] + right_edge[row]) / 2;
            }

        }
    }
}

int road_width_recover_cnt = 0;
int left_edge_recover_cnt = 0;
int right_edge_recover_cnt = 0;
Inflexion left_inflexion;
Inflexion right_inflexion;

void search_break(int start, int end)
{
    road_width_recover_cnt = 0;
    left_edge_recover_cnt = 0;
    right_edge_recover_cnt = 0;

    //寻找宽度突变与边界突变
    for (int row = start; row > end + 1; row--)
    {
        if (right_edge[row] - left_edge[row] > right_edge[row + 1] - left_edge[row + 1] + 5 ||
            (road_width_break_cnt > 0 && right_edge[row] - left_edge[row] >
                                         right_edge[road_width_break_point] - left_edge[road_width_break_point] + 5))
        {
            if (road_width_break_cnt == 0)
            {
                road_width_break_point = row + 1;
                road_width_break_cnt++;
            }
            else
                road_width_break_cnt++;
        }
        else if (road_width_break_cnt > 0 && right_edge[row] - left_edge[row] < right_edge[road_width_break_point])
        {
            road_width_recover_cnt++;
        }

        if (left_edge[row] - left_edge[row + 1] < -5 ||
            (left_edge_break_cnt > 0 && left_edge[row] - left_edge[left_edge_break_point] < 0))
        {
            if (left_edge_break_cnt == 0)
            {
                left_edge_break_point = row + 1;
                left_edge_break_cnt++;
            }
            else
                left_edge_break_cnt++;
        }
        else if (left_edge_break_cnt > 0 && left_edge[row] - left_edge[left_edge_break_point] > 0)
        {
            left_edge_recover_cnt++;
        }

        if (right_edge[row] - right_edge[row + 1] > 5 ||
            (right_edge_break_cnt > 0 && right_edge[row] - right_edge[right_edge_break_point] > 0))
        {
            if (right_edge_break_cnt == 0)
            {
                right_edge_break_point = row + 1;
                right_edge_break_cnt++;
            }
            else
                right_edge_break_cnt++;
        }
        else if (right_edge_break_cnt > 0 && right_edge[row] - right_edge[right_edge_break_point] < 0)
        {
            right_edge_recover_cnt++;
        }
    }

    for (int i = start - 1; i > end + 1; i--)
    {
        //通过极值点来求解
        if (left_edge[i] - left_edge[i + 1] >= -1 && left_edge[i] - left_edge[i + 1] <= 2 &&
            left_edge[i - 1] - left_edge[i] <= -12)
        {
            left_inflexion.flag = 1;
            left_inflexion.row = i;
            left_inflexion.col = left_edge[i];
        }
        if (right_edge[i] - right_edge[i + 1] <= 1 && right_edge[i] - right_edge[i + 1] >= -2 &&
            right_edge[i - 1] - right_edge[i] >= 12)
        {
            right_inflexion.flag = 1;
            right_inflexion.row = i;
            right_inflexion.col = right_edge[i];
        }
//         printf("s:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n\r",
//                left_inflexion.row, left_inflexion.col, right_inflexion.row, right_inflexion.col,
//                 road_width_break_cnt, road_width_break_point, road_width_recover_cnt, left_edge_break_cnt,
//                 left_edge_break_point, left_edge_recover_cnt,
//                 right_edge_break_cnt, right_edge_break_point, right_edge_recover_cnt);
    }
//    printf("s:%d,%d,%d,%d\n\r",left_inflexion.row, left_inflexion.col, right_inflexion.row, right_inflexion.col);
//     printf("road:%d,%d,%d,\r\n right: %d,%d,%d,\r\n left:%d,%d,%d\r\n", road_width_break_point, road_width_break_cnt, road_width_recover_cnt,
//                                                  right_edge_break_point, right_edge_break_cnt, right_edge_recover_cnt,
//                                                  left_edge_break_point, left_edge_break_cnt, left_edge_recover_cnt);
}

void search_jump_point(int start, int end)
{
    /*for(int i = start; i < end; i++) {
            jump_point_num[i] = 0;
            for(int j = left_edge[i]; j < right_edge[i]; j++) {
                    if(binary_image[i][j] - binary_image[i][j + 1] != 0) {
                            jump_point_num[i]++;
                    }
            }
            printf("%d:%d\n\r", i, jump_point_num[i]);
    }*/
    for (int i = start; i < end; i++)
    {
        jump_point_num[i] = 0;
        for (int j = 15; j < 65; j++)
        {
            if (binary_image[i][j] - binary_image[i][j + 1] != 0)
                jump_point_num[i]++;
        }
        //printf("%d:%d\n\r", i, jump_point_num[i]);
    }
}

void Regression(int x[], int y[], int start, int end, float *k, float *b)
{
    float t1 = 0, t2 = 0, t3 = 0, t4 = 0;
    if (end <= start)
        return;
    int size = end - start;
    for (uint8 i = start; i < end; i++)
    {
        t1 += x[i] * x[i];
        t2 += x[i];
        t3 += x[i] * y[i];
        t4 += y[i];
    }
    *k = (t3 * size - t2 * t4) / (t1 * size - t2 * t2);
    *b = (t1 * t4 - t2 * t3) / (t1 * size - t2 * t2);
}

float getVarianceOfEdge(int Edges[], int lastLine)
{

    float variance = 0.0f;
    int indexs[IMAGE_HEIGHT] = {0};
    for (int i = 0; i < IMAGE_HEIGHT; i++)
    {
        indexs[i] = i;
    }
    float k = 0.0f;
    float b = 0.0f;

    int end = IMAGE_HEIGHT;

    int tmp = Edges[IMAGE_HEIGHT - 1];
    for (int i = IMAGE_HEIGHT - 1; i > lastLine; i--)
    {
        if (Edges[i] != tmp)
        {
            end = i;
        }
    }

    Regression(indexs, Edges, lastLine + 1, end, &k, &b);

    float sum = 0.0f;

    for (int i = lastLine + 1; i < IMAGE_HEIGHT; i++)
    {
        sum += (Edges[i] - (k * i + b)) * (Edges[i] - (k * i + b));
    }

    variance = sum / (IMAGE_HEIGHT - lastLine - 1);

    //printf();

    return variance;
}

void calc_middle_line_curvity(void)//计算中线曲率
{
    float temp = 0;
    for (int i = IMAGE_HEIGHT - 10; i > get_last_line() + 1; i--)
    {
        temp += middle_line[i - 1] - middle_line[i];
    }
    if (get_available_line() > 2)
    {
        temp = temp / get_available_line();
        temp = temp > 1.5f ? 1.5f : temp;
        temp = temp < -1.5f ? -1.5f : temp;
        set_curvity(temp);
    }
    else
    {
        set_curvity(1);
    }

}

float variance = 0.0f;

void set_variance(float var)
{
    variance = var;
}

float get_variance()
{
    return variance;
}

void calc_middleline_variance() //计算方差
{
    float var = 0.0f;
    float var_sum = 0.0f;

    variance = 1000;

    for (int i = IMAGE_HEIGHT - 1; i > get_last_line(); i--)
    {
        var_sum += (middle_line[i] - MID_LINE_VAL) * (middle_line[i] - MID_LINE_VAL);
    }
    if (get_last_line() < 50)
    {
        var = var_sum / (IMAGE_HEIGHT - get_last_line() - 1);
        set_variance(var);
    }
    else
    {
        set_variance(1000);
    }
    //printf("v:%f\n\r", variance);
}

float Fuzzify_Left(float x, float X_start, float X_end)//模糊化，隶属度计算，适用于左边
{
    if (x <= X_start)
        return 1;
    else if ((X_start < x) && (x <= X_end))
        return (X_end - x) / (X_end - X_start);
    else if (x > X_end)
        return 0;
    return 0;
}

float Fuzzify_Mid(float x, float X_start, float X_mid, float X_end)//模糊化，隶属度计算，适用于中间
{
    if (x <= X_start)
        return 0;
    else if ((X_start < x) && (x <= X_mid))
        return (x - X_start) / (X_mid - X_start);
    else if ((X_mid < x) && (x <= X_end))
        return (X_end - x) / (X_end - X_mid);
    else if (x > X_end)
        return 0;
    return 0;
}

float Fuzzify_Right(float x, float X_start, float X_end)//模糊化，隶属度计算，适用于右边
{
    if (x <= X_start)
        return 0;
    else if ((X_start < x) && (x < X_end))
        return (x - X_start) / (X_end - X_start);
    else if (x >= X_end)
        return 1;
    return 1;
}

void GetWeightMembership(float real_speed, float w[DIMENSION])//求模糊隶属度,用速度作输入
{
#define LOWER_LIMIT 0.5f
#define MIDDLE_LIMIT 1.0f
#define UPPER_LIMIT 1.5f
#define SUPER_LIMIT 2.0f
    //real_speed 是真实速度，我们认为140以下是低速，240是中速，340以上是高速（可自行修改）
    w[0] = Fuzzify_Left(real_speed, LOWER_LIMIT, MIDDLE_LIMIT);
    w[1] = Fuzzify_Mid(real_speed, LOWER_LIMIT, MIDDLE_LIMIT, UPPER_LIMIT);
    w[2] = Fuzzify_Mid(real_speed, MIDDLE_LIMIT, UPPER_LIMIT, SUPER_LIMIT);
    w[3] = Fuzzify_Right(real_speed, UPPER_LIMIT, SUPER_LIMIT);
}


//void calc_image_error(void)
//{
//  float result = 0.0f;
//  float weight_sum = 0.0f;
//  for(unsigned char i = IMAGE_HEIGHT - 10; i > get_last_line(); i--) {
//      result += image_row_weight[i] * middle_line[i];
//      weight_sum += image_row_weight[i];
//  }
//  if(weight_sum != 0)
//      result = result / weight_sum;
//  else
//      result = MID_LINE_VAL;
//  result -= MID_LINE_VAL;
//  image_error = result;
//}

void calc_image_error(void)//摄像头拟合中线
{
    float mid_val = 0.0f;
    float result[DIMENSION] = {0};
    float weight_sum[DIMENSION] = {0};
    float w[DIMENSION] = {0};

#if 1
    GetWeightMembership(wheel_speed, w);//获取w1 w2 w3
#else
    w[0] = 0.0f;
    w[1] = 0.0f;
    w[2] = 0.0f;
    w[3] = 1.0f;
#endif

    for (int n = 0; n < DIMENSION; n++)
    {
        for (uint8 i = IMAGE_HEIGHT - 1; i > get_last_line(); i--)
        {
            result[n] += image_row_weight[n][i] * middle_line[i];
            weight_sum[n] += image_row_weight[n][i];
        }
        if (weight_sum[n] != 0)
        {
            result[n] = result[n] / weight_sum[n];
        }
        else
        {
            result[n] = MID_LINE_VAL;
        }
        mid_val += w[n] * result[n];
    }
    mid_val -= MID_LINE_VAL;
    image_error = mid_val;

}

void variables_init(void)
{
    left_lose = 0;
    right_lose = 0;
    all_lose = 0;
    white_num = 0;
    break_cnt = 0;

    left_lose_start = 0;//记录左边丢线的开始行
    right_lose_start = 0;//记录右边边丢线的开始行
    white_start = 0;//全白开始点
    road_width[IMAGE_HEIGHT] = IMAGE_WIDTH;

    for (int i = 0; i < IMAGE_HEIGHT + 1; i++)
    {//赋初值
        left_edge[i] = 0;
        right_edge[i] = IMAGE_WIDTH;
        middle_line[i] = IMAGE_WIDTH / 2;
        jump_point_num[i] = 0;
    }
    left_inflexion.flag = 0;
    left_inflexion.row = 0;
    left_inflexion.col = 0;
    right_inflexion.flag = 0;
    right_inflexion.row = 0;
    right_inflexion.col = IMAGE_WIDTH;
    road_width_break_point = 0;
    road_width_break_cnt = 0;
    right_edge_break_point = 0;
    right_edge_break_cnt = 0;
    left_edge_break_point = 0;
    left_edge_break_cnt = 0;
    left_inflexion.flag = 0;
    left_inflexion.col = 0;
    left_inflexion.row = 0;
    right_inflexion.flag = 0;
    right_inflexion.col = 0;
    right_inflexion.row = 0;
}

unsigned char composite_image[IMAGE_HEIGHT][IMAGE_WIDTH];

void complex_image(void)
{
#if 0
    for(int i = 0; i < IMAGE_HEIGHT; i++)
            for(int j = 0; j < IMAGE_WIDTH; j++)
                    composite_image[i][j] = 0xff;

    for(int i = IMAGE_HEIGHT - 1; i > get_last_line(); i--)
    {
            int mid_line = middle_line[i];
            mid_line = mid_line > IMAGE_WIDTH - 1 ? IMAGE_WIDTH - 1 : mid_line;
            mid_line = mid_line < 0 ? 0 : mid_line;
            //composite_image[i][mid_line] = 0;

            int left_line = left_edge[i];
            left_line = left_line > IMAGE_WIDTH - 1 ? IMAGE_WIDTH - 1 : left_line;
            left_line = left_line < 0 ? 0 : left_line;
            composite_image[i][left_line] = 0;

            int right_line = right_edge[i];
            right_line = right_line > IMAGE_WIDTH - 1 ? IMAGE_WIDTH - 1 : right_line;
            right_line = right_line < 0 ? 0 : right_line;
            composite_image[i][right_line] = 0;
    }
    float camera_midline = get_image_middle_line();
    float error = camera_midline - MID_LINE_VAL;
    int last_line = get_last_line();
    int available_line = get_available_line();
    int mid_line = (int)get_image_middle_line();
    mid_line = mid_line > IMAGE_WIDTH - 2 ? IMAGE_WIDTH - 2 : mid_line;
    mid_line = mid_line < 1 ? 1 : mid_line;
    /*composite_image[50][mid_line - 1] = 0;composite_image[50][mid_line] = 0;composite_image[50][mid_line + 1] = 0;
    composite_image[51][mid_line - 1] = 0;composite_image[51][mid_line] = 0;composite_image[51][mid_line + 1] = 0;
    composite_image[52][mid_line - 1] = 0;composite_image[52][mid_line] = 0;composite_image[52][mid_line + 1] = 0;*/
    for(int i = 0; i < IMAGE_WIDTH; i++)
    {
            for(int j = 0; j <= last_line; j++)
            composite_image[j][i] = 0;
    }
#else
    for (int i = IMAGE_HEIGHT - 1; i > get_last_line(); i--)
    {
        int mid_line = middle_line[i];
        mid_line = mid_line > IMAGE_WIDTH - 2 ? IMAGE_WIDTH - 2 : mid_line;
        mid_line = mid_line < 1 ? 1 : mid_line;
        binary_image[i][mid_line] = 0;

        int left_line = left_edge[i] + 2;
        left_line = left_line > IMAGE_WIDTH - 2 ? IMAGE_WIDTH - 2 : left_line;
        left_line = left_line < 1 ? 1 : left_line;
        binary_image[i][left_line] = 0;

        int right_line = right_edge[i] - 2;
        right_line = right_line > IMAGE_WIDTH - 2 ? IMAGE_WIDTH - 2 : right_line;
        right_line = right_line < 1 ? 1 : right_line;
        binary_image[i][right_line] = 0;
    }
    float camera_midline = get_image_middle_line() + IMAGE_WIDTH / 2;
    float error = camera_midline - MID_LINE_VAL;
    int last_line = get_last_line();
    int available_line = get_available_line();
    int mid_line = (int) get_image_middle_line();
    mid_line = mid_line > IMAGE_WIDTH - 2 ? IMAGE_WIDTH - 2 : mid_line;
    mid_line = mid_line < 1 ? 1 : mid_line;
    binary_image[40][mid_line - 1] = 0;
    binary_image[40][mid_line] = 0;
    binary_image[40][mid_line + 1] = 0;
    binary_image[41][mid_line - 1] = 0;
    binary_image[41][mid_line] = 0;
    binary_image[41][mid_line + 1] = 0;
    binary_image[42][mid_line - 1] = 0;
    binary_image[42][mid_line] = 0;
    binary_image[42][mid_line + 1] = 0;
    for (int i = 0; i < IMAGE_WIDTH; i++)
    {
        //for(int j = 0; j <= last_line; j++)
        binary_image[last_line][i] = 0;
    }
    /*for(int i = 0; i < IMAGE_WIDTH; i++)
    {
            for(int j = IMAGE_HEIGHT - 10; j <= IMAGE_HEIGHT; j++)
            binary_image[j][i] = 0;
    }*/
//        for (int a=0;a<60;a++){
//            ips200_draw_point(road_width[a],a,RGB565_RED);
//        }


#endif
}

//void send_binary_image(void)
//{
//    uart_putchar(UART_1,0x00);
//    uart_putchar(UART_1,0xff);//发送命令
//    uart_putchar(UART_1,0x01);
//    uart_putchar(UART_1,0x01);
//    uart_putbuff(UART_1, (uint8 *)mt9v03x_image, 60 * 160);  //发送图像
//    printf("%6.2f", image_error);
//    printf("%03d", break_cnt);
//}

void calculate_road_width()
{
//        static int flag = 0;
    //if(flag++ != 100)
    //{
    //  return;
    //}
    printf("road_width:\n\r");
    for (int i = 0; i < IMAGE_HEIGHT; i++)
    {
        if (i % 10 == 0)
        {
            printf("\n\rrow:%d:", i / 10);
        }
        printf("%d, ", right_edge[i] - left_edge[i]);
    }
    printf("end:\n\r");
}

int _right_edge[IMAGE_HEIGHT];
int _left_edge[IMAGE_HEIGHT];
int barn_break_point = 0;
int barn_exist_flag = 0;
int finish_line_row = 0;

int search_barn(int dir, int valid)
{
        barn_break_point = 0;
        int is_barn = 0;
        if(0 == dir)//左边入库
        {
                //寻右边界
                for(int row = IMAGE_HEIGHT - 1; row > 18; row--)
                {
                        _right_edge[row] = IMAGE_WIDTH - 1;
                        if(valid)
                                right_edge[row] = IMAGE_WIDTH - 1;
                        if(IMAGE_HEIGHT - 1 == row)
                        {
                                for(int col = IMAGE_WIDTH / 2; col < IMAGE_WIDTH - 1; col++)
                                {
                                        if(WHITE == binary_image[row][col] && BLACK == binary_image[row][col + 1])
                                        {
                                                _right_edge[row] = col;
                                                if(valid)
                                                        right_edge[row] = col;
                                                break;
                                        }
                                }
                        }
                        else
                        {
                                for(int col = (_right_edge[row + 1] + 5 > IMAGE_WIDTH - 2 ? (_right_edge[row + 1] + 5) : (IMAGE_WIDTH - 2)); col > (_right_edge[row + 1] - 8 > 0 ? (_right_edge[row + 1] - 8) : 0); col--)
                                {
                                        if(WHITE == binary_image[row][col] && BLACK == binary_image[row][col + 1])
                                        {
                                            _right_edge[row] = col;
                                            if(valid)
                                                        right_edge[row] = col;
                                            break;
                                        }
                                }
                        }
                }
                //寻左边界
                for(int row = IMAGE_HEIGHT - 1; row > 18; row--)
                {
                        _left_edge[row] = 0;
                        if(valid)
                            left_edge[row] = 0;
                        jump_point_num[row] = 0;
                        if(IMAGE_HEIGHT - 1 == row)
                        {
                                for(int col = IMAGE_WIDTH / 2; col > 0; col--)
                                {
                                        if(WHITE == binary_image[row][col] && BLACK == binary_image[row][col - 1])
                                        {
                                                _left_edge[row] = col;
                                                if(valid)
                                                        left_edge[row] = col;
                                                break;
                                        }
                                }
                        }
                        else
                        {
                          for(int col = (_right_edge[row] > IMAGE_WIDTH - 8 ? (IMAGE_WIDTH - 8) : _right_edge[row]); col > 5; col--)
                                {
                                        if(binary_image[row][col] != binary_image[row][col - 1])
                                                jump_point_num[row]++;
                                        if(WHITE == binary_image[row][col + 6] && WHITE == binary_image[row][col + 5] && WHITE == binary_image[row][col + 4]
                                           && WHITE == binary_image[row][col + 3] && WHITE == binary_image[row][col + 2]
                                           && WHITE == binary_image[row][col + 1] && WHITE == binary_image[row][col] && BLACK == binary_image[row][col - 1]
                                             && BLACK == binary_image[row][col - 2] && BLACK == binary_image[row][col - 3] && BLACK == binary_image[row][col - 4]
                                               && BLACK == binary_image[row][col - 5])
                                        {
                                                _left_edge[row] = col;
                                                if(valid)
                                                        left_edge[row] = col;
                                                break;
                                        }
                                }
                        }
                }
                int finish_line = 0;
                finish_line_row = 0;
                for(int i = (get_last_line() > 30 ? get_last_line() : 30); i < IMAGE_HEIGHT - 1; i++)
                {
                       // printf("%d:%d\n\r", i, jump_point_num[i]);
                        if(jump_point_num[i] > 9)
                        {
                                finish_line++;
                                finish_line_row = i;
//                                printf("%d:%d\n\r", i, jump_point_num[i]);
                        }
                }
                if(finish_line >= 3 || valid)
                {
                        is_barn = 1;
                        //找左边界突变点
                        /*
                        for(int i = 27; i < IMAGE_HEIGHT - 1; i++)
                        {
                                //printf("%d:%d\n\r", i, _right_edge[i]);
                                if(_left_edge[i] - _left_edge[i + 1] > 6)
                                {
                                        if(valid)
                                            set_last_line(i);
                                        barn_break_point = i;
                                        break;
                                }
                        }

                        if(break_point != 0 && valid)
                        {
                                int modify_point = barn_break_point + 40;
                                if(modify_point > IMAGE_HEIGHT - 1)
                                        modify_point = IMAGE_HEIGHT - 1;
                                float k = (float)(80 - (left_edge[barn_break_point])) / (modify_point - barn_break_point);
                                //修补突变点下方的左边界
                                for(int i = barn_break_point; i <= modify_point; i++)
                                {
                                        right_edge[i] = left_edge[barn_break_point] + k * (i - barn_break_point);
                                }
                                middle_line[IMAGE_HEIGHT - 1] = 40;
                                if(barn_break_point < 31)
                                {
                                        for(int i = IMAGE_HEIGHT - 2; i > barn_break_point; i--)
                                        {
                                                if(left_edge[i] == 0)
                                                        middle_line[i] = middle_line[i + 1] + (right_edge[i] - right_edge[i + 1]);
                                                else
                                                        middle_line[i] = (right_edge[i] + left_edge[i]) / 2;
                                        }
                                }
                                else if(barn_break_point >= 31 && barn_break_point <= 42)
                                {
                                        for(int i = IMAGE_HEIGHT - 2; i > barn_break_point; i--)
                                        {
                                                middle_line[i] = middle_line[i + 1] + (right_edge[i] - right_edge[i + 1]);
                                        }
                                }
                                else
                                {
                                        for(int i = IMAGE_HEIGHT - 2; i > barn_break_point; i--)
                                        {
                                                middle_line[i] = 0;
                                        }
                                }

                        }*/
                }

        }
        else if(1 == dir)
        {
                //寻左边界
                for(int row = IMAGE_HEIGHT - 1; row > 18; row--)
                {
                        _left_edge[row] = 0;
                        if(valid)
                                left_edge[row] = 0;
                        if(IMAGE_HEIGHT - 1 == row)
                        {
                                for(int col = IMAGE_WIDTH / 2; col > 0; col--)
                                {
                                        if(WHITE == binary_image[row][col] && BLACK == binary_image[row][col - 1])
                                        {
                                                _left_edge[row] = col;
                                                if(valid)
                                                        left_edge[row] = col;
                                                break;
                                        }
                                }
                        }
                        else
                        {
                                for(int col = (_left_edge[row + 1] - 5 > 1 ? (_left_edge[row + 1] - 5) : 1); col < (_left_edge[row + 1] + 5 < IMAGE_WIDTH - 1 ? (_left_edge[row + 1] + 8) : (IMAGE_WIDTH - 1)); col++)
                                {
                                        if(WHITE == binary_image[row][col] && BLACK == binary_image[row][col - 1])
                                        {
                                                _left_edge[row] = col;
                                                if(valid)
                                                        left_edge[row] = col;
                                                break;
                                        }
                                }
                        }
                }
                //寻右边界
                for(int row = IMAGE_HEIGHT - 1; row > 18; row--)
                {
                        _right_edge[row] = IMAGE_WIDTH - 1;
                        if(valid)
                          right_edge[row] = IMAGE_WIDTH - 1;
                        jump_point_num[row] = 0;
                        if(IMAGE_HEIGHT - 10 == row)
                        {
                                for(int col = IMAGE_WIDTH / 2; col < IMAGE_WIDTH - 1; col++)
                                {
                                        if(WHITE == binary_image[row][col] && BLACK == binary_image[row][col + 1])
                                        {
                                                _right_edge[row] = col;
                                                if(valid)
                                                        right_edge[row] = col;
                                                break;
                                        }
                                }
                        }
                        else
                        {
                                for(int col = _left_edge[row] < 7 ? 7 : _left_edge[row]; col < IMAGE_WIDTH - 6; col++)
                                {
                                        if(binary_image[row][col] != binary_image[row][col + 1])
                                                jump_point_num[row]++;
                                        if(WHITE == binary_image[row][col - 6] && WHITE == binary_image[row][col - 5] && WHITE == binary_image[row][col - 4]
                                           && WHITE == binary_image[row][col - 3] && WHITE == binary_image[row][col - 2]
                                           && WHITE == binary_image[row][col - 1] && WHITE == binary_image[row][col] && BLACK == binary_image[row][col + 1]
                                             && BLACK == binary_image[row][col + 2] && BLACK == binary_image[row][col + 3] && BLACK == binary_image[row][col + 4]
                                               && BLACK == binary_image[row][col + 5])
                                        {
                                                _right_edge[row] = col;
                                                if(valid)
                                                        right_edge[row] = col;
                                                break;
                                        }
                                }
                        }
                }
                int finish_line = 0;
                finish_line_row = 0;
                for(int i = (get_last_line() > 30 ? get_last_line() : 30); i < IMAGE_HEIGHT - 1; i++)
                {
                        //printf("%d:%d\n\r", i, jump_point_num[i]);
                        if(jump_point_num[i] > 9)
                        {
                                finish_line++;
                                finish_line_row = i;
                        }
                }
                if(finish_line >= 3 || valid)
                {
                        is_barn = 1;
                        //找右边界突变点
                        /*
                        for(int i = 27; i < IMAGE_HEIGHT - 1; i++)
                        {
                                //printf("%d:%d\n\r", i, _right_edge[i]);
                                if(_right_edge[i] - _right_edge[i + 1] < -6)
                                {
                                if(valid)
                                    set_last_line(0);
                                        barn_break_point = i;
                                        //printf("%d\n\r", barn_break_point);
                                        break;
                                }
                        }
                        if(barn_break_point > 20 && barn_break_point < 50 && valid)
                        {
                                int modify_point = barn_break_point + 40;
                                if(modify_point > IMAGE_HEIGHT - 1)
                                        modify_point = IMAGE_HEIGHT - 1;
                                float k = (float)(0 - (right_edge[barn_break_point])) / (modify_point - barn_break_point);
                                //修补突变点下方的左边界
                                for(int i = barn_break_point; i <= modify_point; i++)
                                {
                                        left_edge[i] = right_edge[barn_break_point] + k * (i - barn_break_point);
                                }
                                middle_line[IMAGE_HEIGHT - 1] = 40;
                                 if(barn_break_point <= 42)
                                {
                                        for(int i = IMAGE_HEIGHT - 2; i > barn_break_point; i--)
                                        {
                                                middle_line[i] = middle_line[i + 1] + (left_edge[i] - left_edge[i + 1]);
                                        }
                                }
                                else
                                {
                                        for(int i = IMAGE_HEIGHT - 2; i > barn_break_point; i--)
                                        {
                                                middle_line[i] = 80;
                                        }
                                }
                        }*/
                }
        }
        return is_barn;
}

// FORK 检测开始
#define max(a, b) a>b?a:b
int l_point_row = 0, r_point_row = 0;
int first_l_point, first_r_point;
float last_fork_end = 0;

int search_fork(void)
{
    if (all_lose != 0 && max(left_lose, right_lose) > 30)
        return 0;

    l_point_row = 0, r_point_row = 0;
    for (int i = IMAGE_HEIGHT - 10; i > get_last_line(); i--)
    {
        if (left_edge[i] < left_edge[i + 1] && left_edge[i] > 5)
        {
            l_point_row++;
            if (l_point_row == 1)
                first_l_point = i;
        }
        else
        {
            l_point_row = 0;
        }
        if (l_point_row == 4)
            break;
    }

    for (int i = IMAGE_HEIGHT - 10; i > get_last_line(); i--)
    {
        if (right_edge[i] > right_edge[i + 1] && right_edge[i] <= IMAGE_WIDTH - 5)
        {
            r_point_row++;
            if (r_point_row == 1)
                first_r_point = i;
        }
        else
        {
            r_point_row = 0;
        }
        if (r_point_row == 4)
            break;
    }

    if (l_point_row < 4 || r_point_row < 4 || abs(first_r_point - first_l_point) > 8)
        return 0;

    return 1;
}

/*****************************斑马线定义*************************************/
int line = 0;//纵坐标
int list = 0;//横坐标

int count_black;
int count_black_1;
int garage_count;
int white_black;
int black_white;
int region;
int garageout_flag = 0;
int js = 0;

void test_black()//黑色计数
{
    int i, j;
    count_black = 0;//左边黑色数
    count_black_1 = 0;
    for (i = IMAGE_WIDTH / 2; i > 5; i--)//从中间往左数黑色数
    {
        if (binary_image[35][i] == BLACK)//具体扫描行数自己调整
        {
            count_black++;
        }
    }
    for (j = IMAGE_WIDTH / 2; j < IMAGE_WIDTH - 5; j++)//从中间往右数黑色数
    {
        if (binary_image[35][j] == BLACK)//具体扫描行数自己调整
        {
            count_black_1++;//右边黑色数
        }
    }
}

/*****************************避障定义*************************************/
int l_diu = 0;
int r_diu = 0;

int block_in_process = 0;
int count_black_1;
int count_black;
int bzline = 0;
int bz_start = 30;
int zhidao_signal = 0;
int count_white;
int Road_Width[3];

int search_block(void)//避障
{
    Road_Width[0] = abs(right_edge[bz_start - 10] - left_edge[bz_start - 10]);//道路宽度扫描
    Road_Width[1] = abs(right_edge[bz_start] - left_edge[bz_start]);//道路宽度扫描
    Road_Width[2] = abs(right_edge[bz_start + 10] - left_edge[bz_start + 10]);
//    printf("%d,%d,%d\r\n", Road_Width[0], Road_Width[1], Road_Width[2]);

    if (abs(Road_Width[1] - Road_Width[0]) < 15 || abs(Road_Width[2] - Road_Width[1]) > 20)//通过两行间道路宽度的突变作为避障初判断
    {
        test_black();//扫描黑色数
//        printf("%d, %d, %d\r\n", count_black, count_black_1, count_black + count_black_1);
        if (count_black + count_black_1 > 32)//左右黑色数大于正常值
        {
            if ((right_lose + left_lose) < 33 && count_black > count_black_1)//左避障
            {
                block_in_process = 1;

            }
            if ((right_lose + left_lose) < 33 && count_black_1 > count_black)//右避障
            {
                block_in_process = 2;

            }
        }
    }
    return block_in_process;
}

void process_block_in_image(int dir)
{
    if (block_in_process == 1)//识别到左避障
    {
        for (int row = IMAGE_HEIGHT - 1; row > 0; row--)
        {
            middle_line[row] = right_edge[row] + 10;
//            middle_line[row] = middle_line[row] + 30;
//            left_edge[row] = left_edge[row] + 20;
        }
    }
    if (block_in_process == 2)//识别到右避障
    {
        for (int row = IMAGE_HEIGHT - 1; row > 0; row--)
        {
            middle_line[row] = middle_line[row] - 30;
//            right_edge[row] = right_edge[row] - 20;
        }
    }
}

int col_scan_all_white(int start_row)
{
    int j, i, ret = 0;
    for (i = 0; i < IMAGE_WIDTH; i++)
    {
        for (j = start_row; j < IMAGE_HEIGHT; j++)
        {
            if (binary_image[j][i] == WHITE)//计数值为 0 的点
            {
                break;
            }
        }
        if (j == IMAGE_HEIGHT)
            ret++;
        //printf("j=%d\n\r", j);
    }
    return ret;
}

int process_circle_flag = -1;

void analyze_image(void)
{
    variables_init();
    preprocess_image(OSTU);     //压缩图像、二值化图像
    search_border_line_and_Mid_line();
    calc_middle_line_curvity();
    calc_middleline_variance();

    search_break(55, get_last_line() < 20 ? 20 : get_last_line());

    barn_exist_flag = search_barn(0, 0);

    circle_forecast = search_circle();
    if (-1 != process_circle_flag)
    {
        process_circle_in_image(process_circle_flag);
    }
    if (0 != block_in_process)
    {
        process_block_in_image(block_in_process);
    }
    search_cross();

    calc_image_error();
    modify_err = get_image_error_single();
    complex_image();

    //send_binary_image();
//    calculate_road_width();
}

float get_image_error_single(void)//摄像头拟合中线
{
#define INDEX 1
    float mid_val = 0.0f;
    float result = 0.0f;
    float weight_sum = 0.0f;

    for (uint8 i = IMAGE_HEIGHT - 1; i > get_last_line(); i--)
    {
        result += image_row_weight[INDEX][i] * middle_line[i];
        weight_sum += image_row_weight[INDEX][i];
    }
    if (weight_sum != 0)
    {
        result = result / weight_sum;
    }
    else
    {
        result = MID_LINE_VAL;
    }
    mid_val = result - MID_LINE_VAL;
    return mid_val;
}

void set_process_circle_flag(int dir)
{
    process_circle_flag = dir;
    circle_image_process_complete = 0;
}

void clear_process_circle_flag(void)
{
    process_circle_flag = -1;
}

void clear_process_block_flag(void)
{
    block_in_process = 0;
}



int Circle_Exit_Judgement(void)
{
    int black_pixel_cnt = 0;
    for(int i = 0; i < 80; i++)
    {
        if(binary_image[60][i] == BLACK)
        {
            black_pixel_cnt++;
        }
    }
    if(black_pixel_cnt > 75)
    {
        gpio_set_level(C4, 1);
        return 1;

    }

    else return 0;
}
