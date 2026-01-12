/*
 * encoder.c
 *
 *  Created on: 2024\1\23
 *      Author: Misaka
 */
//#include "zf_common_headfile.h"
//#include "string.h"

#include "ec11.h"
unsigned char EC11_A_Now,EC11_B_Now,EC11_Key;
KEY_MSG key_msg = {0};
static  char    EC11_A_Last = 0;                        //EC11的A引脚上一次的状态
static  char    EC11_B_Last = 0;                        //EC11的B引脚上一次的状态

static   int    EC11_KEY_COUNT = 0;                     //EC11按键动作计数器
static   int    EC11_KEY_DoubleClick_Count = 0;         //EC11按键双击动作计数器
static   int    FLAG_EC11_KEY_ShotClick = 0;            //EC11按键短按动作标志
static   int    FLAG_EC11_KEY_LongClick = 0;            //EC11按键长按动作标志
static   int    FLAG_EC11_KEY_DoubleClick = 0;          //EC11按键双击动作标志

void EC11_Init(void)
{
    gpio_init(A8, GPI, 1, GPI_FLOATING_IN);
    gpio_init(D8, GPI, 1, GPI_FLOATING_IN);
    gpio_init(B12, GPI, 1, GPI_FLOATING_IN);
    EC11_A_Now = 1;
    EC11_B_Now = 1;
    EC11_Key = 1;

    //--------清除按键计数器和标志位--------//
    EC11_KEY_COUNT = 0;                     //EC11按键动作计数器
    EC11_KEY_DoubleClick_Count = 0;         //EC11按键双击动作计数器
    FLAG_EC11_KEY_ShotClick = 0;            //EC11按键短按动作标志
    FLAG_EC11_KEY_LongClick = 0;            //EC11按键长按动作标志
    FLAG_EC11_KEY_DoubleClick = 0;          //EC11按键双击动作标志
}

//char Encoder_Scan(void)
//{
//    char result = 0;
//    static uint8_t flag;
//    uint8_t turn;
//    EC11_A_Now = gpio_get_level(A8);
//    EC11_B_Now = gpio_get_level(D8);
//    EC11_Key = gpio_get_level(B12);
//    if((EC11_A_Now == 1) && (EC11_B_Now == 1))
//    {
//        flag = 1;
//    }
//    if(flag == 1)
//    {
//        if(EC11_A_Now != EC11_B_Now)
//        {
//            turn = gpio_get_level(A8);
//        }
//        if((EC11_A_Now == 0) && (EC11_B_Now == 0))
//        {
//            flag = 0;
//            if(turn)
//            {
//                result = 1;
//                key_msg.updated_flag = 1;
//            }
//            else
//            {
//                result = -1;
//                key_msg.updated_flag = 1;
//            }
//        }
//    }
//    if(EC11_Key == 0)
//    {
//        if(result == 1) {result = 2;key_msg.updated_flag = 1;}
//        else if(result == -1) {result = -2;key_msg.updated_flag = 1;}
//        else {result = 3;key_msg.updated_flag = 1;}
//    }
//    key_msg.key_id = result;
//    char temp[10];
//    sprintf(temp, "re=%d\n", result);
//    debug_send_buffer(temp,10);
//    return result;      //返回值的取值：   0：无动作； 1：顺时针； -1：逆时针； 2:按键按下并同时顺时针转动; -2:按键按下并同时逆时针转动; 3：按键按下无转动
//
//}

char EC11_Scan()
{

    char ScanResult = 0;    //返回编码器扫描结果，用于分析编码器的动作
                            //返回值的取值：   0：无动作；      1：正转；           -1：反转；
                            //                  2：只按下按键；    3：按着按键正转；   -3：按着按键反转
    EC11_A_Now = gpio_get_level(D8);
    EC11_B_Now = gpio_get_level(A8);
    EC11_Key = gpio_get_level(B12);
    if(EC11_A_Now != EC11_A_Last)   //以A为时钟，B为数据。正转时AB反相，反转时AB同相
    {
        if(EC11_A_Now == 0)
        {
            if(EC11_B_Now == 1)      //只需要采集A的上升沿或下降沿的任意一个状态，若A下降沿时B为1，正转
            {
                ScanResult = FORDWARD_ROTATION;     //正转
                key_msg.key_id = ScanResult;
                key_msg.updated_flag = 1;
            }
            else
            {
                ScanResult = REVERSE_ROTATION;      //反转
                key_msg.key_id = ScanResult;
                key_msg.updated_flag = 1;
            }
        }
        EC11_A_Last = EC11_A_Now;   //更新编码器上一个状态暂存变量
        EC11_B_Last = EC11_B_Now;   //更新编码器上一个状态暂存变量
    }
    if(EC11_Key == 0)
    {
        ScanResult = 2;
        while(!gpio_get_level(B12));
        key_msg.key_id = ScanResult;
        key_msg.updated_flag = 1;
    }

    return ScanResult;
}
