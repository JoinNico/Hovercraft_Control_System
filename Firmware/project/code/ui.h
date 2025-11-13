/*
 * ui.h
 *
 *  Created on: 2024/1/24
 *      Author: Misaka
 */

#ifndef UI_H_
#define UI_H_

#include "zf_common_headfile.h"

extern void UI_Init(void);
typedef enum
{
    E_HOME,
    E_LIST1,
    E_LIST2,
    E_LIST3,
    E_LIST4,
    E_LIST5,
    E_LIST6,
    E_LIST7,
    E_LIST8,
    E_UI_MAX, //界面最大值
} E_UI;

typedef enum
{
    E_NONE,           //运行状态机 无
    E_STATE_HOME,     //运行状态->主页运行
    E_STATE_SETTING,     //运行状态->设置运行
    E_STATE_SETTING_S,  //运行状态->子设置运行
    E_STATE_MUSIC,      //运行状态->音乐运行
    E_STATE_RUN_MUSIC,      //运行状态->跑向音乐
    E_STATE_RUN_SETTING,//运行状态->跑向设置
    E_STATE_RUN_HOME_FROM_SETTING,//运行状态->设置跑向主页
    E_STATE_RUN_SETTINGS_FROM_SETTING,//运行状态->设置跑向子设置
    E_STATE_RUN_SETTING_FROM_SETTINGS,//运行状态->子设置跑向设置
    E_STATE_RUN_HOME_FROM_MUSIC,//运行状态->音乐跑向主页
    E_STATE_RUN_LIST_FROM_MUSIC,//运行状态->音乐跑向列表
    E_STATE_RUN_MUSIC_FROM_LIST,//运行状态->音乐跑向列表
    E_STATE_ARRIVE, //运行状态->到达
}E_STATE;

typedef struct
{
    char key_id;
    char updated_flag;
    char key_detected;
}KEY_MSG;

typedef struct
{
    uint8_t index;//UI的索引
    void (*cb)(KEY_MSG*);//ui的执行函数
}UI_LIST;


typedef struct
{
    char* str;    //内容
    uint8_t height;     //单个字体高度
    uint8_t font;  //字体
} setting_LIST;

extern KEY_MSG key_msg;
extern UI_LIST ui_list[];
extern char ui_index;

#endif /* UI_H_ */
