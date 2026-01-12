#include "identify.h"

#define LENGTH_CTL      smartcar_param.length_control
#define LENGTH_TOTAL    smartcar_param.length_total

int road_type = BEND;
int last_road_type;
float circle_modify_err = 0;

int cross_state = 0;

int ele_lock = _BEND;


int length_circle1 ;
int length_circle2 ;
int length_barn ;

int length_block ;
int length_break;

int length_init(void)
{
    circle_times = smartcar_param.circle_num;
    smartcar_param.circle_num = smartcar_param.circle_num + smartcar_param.circle_dir[1];

    return 0;
}

int get_lock(void)
{
  return ele_lock;
}

void set_lock(int lock)
{
  ele_lock = lock;
//  alarm_times(1,400);
}

void clear_lock(void)
{
  ele_lock = _BEND;
//  alarm_times(3,100);
}

/*----------------------------图像检测圆环---------------------------------------------*/
int left_circle_forecast_flag = 0;
int right_circle_forecast_flag = 0;

#define LEFT_CIRCLE     0
#define RIGHT_CIRCLE    1

#define CIRCLE_NUM                      1
#define CIRCLE_DIR(circle_num)          smartcar_param.circle_dir[circle_num]
#define CIRCLE_FORECAST_DISTANCE        0.6
#define CIRCLE_ENTRY_DISTANCE           0.60 //0.2
#define IN_CIRCLE_DISTANCE              2.5f //2.4
#define CIRCLE_READY_EXIT_DISTANCE      0.5f
#define CIRCLE_EXIT_DISTANCE            0.00
#define CIRCLE_EXIT_ANGLE               -10

#define CIRCLE_DIS_CTRL                 20.0

int detect_circle_image(void)
{
//#define CIRCLE_NUM smartcar_param.circle_num
//#define CIRCLE_DIR(circle_num)  smartcar_param.circle_dir[circle_num]
//#define CIRCLE_ENTRY_DISTANCE(circle_num) smartcar_param.circle_entrydis[circle_num]// 0.60
//#define IN_CIRCLE_DISTANCE(circle_num) smartcar_param.circle_size[circle_num]//1.60
//#define CIRCLE_READY_EXIT_DISTANCE(circle_num)   smartcar_param.circle_rdy_exitdis[circle_num]//  0.60
//#define CIRCLE_EXIT_DISTANCE(circle_num) smartcar_param.circle_exitdis[circle_num] // 0.80
//#define CIRCLE_EXIT_ANGLE(circle_num)   smartcar_param.circle_exit_angle[circle_num] //-10

    static int circle_cnt = 0;
    static int circle_flag = 0;
    static float circle_dis_rec = 0;


    int circle_dir = 0;
    circle_dir = CIRCLE_DIR(circle_cnt);// 0 -->left    1-->right

    if(((get_lock() != _BEND && get_lock() != _CIRCLE)) /*|| (get_distance() < CIRCLE_DIS_CTRL)*/)
      return 0;
    switch(circle_flag)
    {
        case 0:
            if(0 == circle_forecast.circle_fore_dir && CIRCLE_DIR(circle_cnt) == 0 && left_circle_forecast_flag <= 0)
            {
                left_circle_forecast_flag--;
                if(left_circle_forecast_flag < -1)
                {
                    left_circle_forecast_flag = 1;
                    circle_dis_rec = get_distance();
                    circle_flag = CIRCLE_FORECAST;
                    set_lock(_CIRCLE);
                }
            }
            else if(left_circle_forecast_flag <= 0)
            {
                left_circle_forecast_flag = 0;
            }

            if(1 == circle_forecast.circle_fore_dir && CIRCLE_DIR(circle_cnt) == 1 && right_circle_forecast_flag <= 0)
            {
                right_circle_forecast_flag--;
                if(right_circle_forecast_flag < -1)
                {
                    right_circle_forecast_flag = 1;
                    circle_dis_rec = get_distance();
                    circle_flag = CIRCLE_FORECAST;
                    set_lock(_CIRCLE);
                }
            }
            else if(right_circle_forecast_flag <= 0)
            {
                right_circle_forecast_flag = 0;
            }
        break;

        case CIRCLE_FORECAST:
            set_process_circle_flag(circle_dir);
            if(get_distance() - circle_dis_rec > CIRCLE_FORECAST_DISTANCE
               && (left_circle_forecast_flag == 1 || right_circle_forecast_flag == 1))
            {
                circle_flag = CIRCLE_READY_ENTRY;
//                alarm_times(1, 50);

                left_circle_forecast_flag = 0;
                right_circle_forecast_flag = 0;
            }
        break;

        case CIRCLE_READY_ENTRY:
            if(((left_lose > 10) && right_lose < 5 && CIRCLE_DIR(circle_cnt) == 0)||
               ((right_lose > 10) && (left_lose < 5) && CIRCLE_DIR(circle_cnt) == 1))
            {
                circle_flag = CIRCLE_ENTRY;
//                alarm_times(1, 50);
                circle_dis_rec = get_distance();


                circle_entry_process = 1;
           }
        break;

        case CIRCLE_ENTRY:
            if (/*(get_distance() - circle_dis_rec) > CIRCLE_ENTRY_DISTANCE(circle_cnt) ||*/1 == circle_image_process_complete)
            {
                circle_flag = CIRCLE_IN;
//                alarm_times(1, 50);
                circle_dis_rec = get_distance();
                circle_entry_process = 0;
                in_circle_mending = 1;
           }
        break;

        case CIRCLE_IN:
            if (left_lose > 15 && (get_distance() - circle_dis_rec) >  IN_CIRCLE_DISTANCE)
            {
//                circle_flag = CIRCLE_READY_EXIT;
                circle_flag = CIRCLE_EXIT;
//                alarm_times(1, 50);
                circle_dis_rec = get_distance();

                in_circle_mending = 0;
                circle_exit_process = 1;
//                circle_exit_detect = 1;
            }
//            Circle_Exit_Judgement();

        break;

        case CIRCLE_READY_EXIT:
            if(/*(break_point[0] > 10 && break_point[0] < 35) && */(get_distance() - circle_dis_rec) > CIRCLE_READY_EXIT_DISTANCE)
            {
                circle_flag = CIRCLE_EXIT;
//                alarm_times(1, 50);
                circle_dis_rec = get_distance();

                circle_exit_detect = 0;
                circle_exit_process = 1;
            }
        break;

        case CIRCLE_EXIT:
            if( ((left_lose < 10)  && CIRCLE_DIR(circle_cnt) == 0)||
                ((right_lose < 10) && CIRCLE_DIR(circle_cnt) == 1) /*|| (get_distance() - circle_dis_rec) > CIRCLE_EXIT_DISTANCE*/)
            {
                circle_flag = 0;
                circle_cnt++;
                clear_lock();

                if(circle_cnt >= CIRCLE_NUM)
                    circle_cnt = 0;

                clear_process_circle_flag();
                circle_exit_process = 0;
            }

        break;

        default:

        break;
    }

    circle_modify_err = modify_err;
    if(CIRCLE_ENTRY == circle_flag)
    {
//        if(LEFT_CIRCLE == CIRCLE_DIR(circle_cnt))
//        {
//            if(circle_modify_err > -1.0)
//                circle_modify_err = -1.0;
//        }
//        else if(RIGHT_CIRCLE == CIRCLE_DIR(circle_cnt))
//        {
//            if(circle_modify_err < 1.0)
//                circle_modify_err = 1.0;
//        }
    }
    else if(CIRCLE_IN == circle_flag)
    {

    }
    else if(CIRCLE_READY_EXIT == circle_flag)
    {
      if(circle_dir == 0)
            circle_modify_err = CIRCLE_EXIT_ANGLE;
      else
            circle_modify_err = -CIRCLE_EXIT_ANGLE;
    }
    else if(CIRCLE_EXIT == circle_flag)
    {

    }

    return circle_flag;
}

/* ----------------------------------------检测入库---------------------------------------------- */
#define STOPLINE_DIS_CTRL 25.0
#define STOPLINE_IN_DISTANCE 1.0
int detect_stop(void)
{
    static int barn_flag = 0;
    static float barn_dis_rec = 0;
//    static float barn_dis = 0;
//    static int finish_line_flag = 0;


    if(get_distance() < ( STOPLINE_DIS_CTRL ))
       return 0;

    if(get_lock() != _BEND && get_lock() != _BARN )
       return 0;

    switch(barn_flag)
    {
            case 0:
                if( 1 == barn_exist_flag  ) //摄像头
                {
                    set_lock(_BARN);
                    barn_flag = FINISH;
                    barn_dis_rec = get_distance();
//                    finish_line_flag = 1;
                }
            break;
            case FINISH:
                if((get_distance() - barn_dis_rec) > STOPLINE_IN_DISTANCE)
                {
                    barn_flag = FINISH_STOP;
                }
            break;
            case FINISH_STOP:
            break;
            default:

            break;
    }

    return barn_flag;
}

/* ----------------------------------------检测路障---------------------------------------------- */
//#define IN_BLOCK_DISTANCE       0.7
//#define BLOCK_DIS_CTRL          1.5
//int detect_block(void)
//{
//    static int block_flag = 0;
//    static int block_num = 1;
//    static int block_pre_flag = 0;
//    static float block_pre_flag_point = 0;
////    static float block_dis = 0;
////    static int block_number = 0;
//
//    //如果等于任何其它非bend  则返回
//    if((get_lock() != _BEND && get_lock() != _BLOCK ) || get_distance() < BLOCK_DIS_CTRL/*||  motor_start_flag == 0 || block_number>=smartcar_param.road_block_number*/ )
//        return 0;
//    if(block_num != 0)
//    {
//        block_pre_flag = search_block();
//    }
//    if(block_pre_flag)
//    {
//        block_pre_flag = 0;
//        block_flag = BLOCK_IN;
//        block_num--;
//        block_pre_flag_point = get_distance();
//        set_lock(_BLOCK);
//    }
//    if(BLOCK_IN == block_flag )
//    {
////        printf("%.2f,%.2f,%.2f\r\n",get_distance(),block_pre_flag_point,get_distance() - block_pre_flag_point);
//        if(get_distance() - block_pre_flag_point > IN_BLOCK_DISTANCE )
//        {
//            block_flag = 0;
//            clear_process_block_flag();
//            clear_lock();
//        }
//    }
//
//    return block_flag;
//}

#define IN_BLOCK_DISTANCE       1.5
#define BLOCK_DIS_CTRL          0.5

int detect_block(void)
{
    static int block_flag = 0;
    static int block_pre_flag = 0;
    static float block_pre_flag_point = 0;
    static int block_number = 1;

    if(get_lock() != _BEND && get_lock() != _BLOCK )
        return 0;
    if(get_distance() > BLOCK_DIS_CTRL && block_number != 0)
    {
        block_pre_flag = 1;
        block_in_process = 1;
    }

    if(block_pre_flag)
    {
        block_pre_flag = 0;
        block_flag = BLOCK_IN;
        block_number = 0;
        block_pre_flag_point = get_distance();
        set_lock(_BLOCK);
    }
    if(BLOCK_IN == block_flag )
    {
        if(get_distance() - block_pre_flag_point > IN_BLOCK_DISTANCE )
        {
            block_flag = 0;
            clear_process_block_flag();
            clear_lock();
        }
    }

    return block_flag;
}
/* ----------------------------------------检测长直道--------------------------------------------- */
int detect_long_straight_road(void)
{
    if(get_lock() != _BEND/* && get_lock() != _CROSS*/)
        return 0;
//    if(left_circle_forecast_flag == 1 || right_circle_forecast_flag == 1)
//      return 0;
    int threshold = 16;
    if((get_last_line() < 40 && get_variance() < threshold))
        {
                return LONG_STRAIGHT;
        } else {
                return 0;
        }
}

/* ----------------------------------------检测十字---------------------------------------------- */
int detect_cross(void)
{
    if(get_lock() != _BEND && get_lock() != _CROSS)
      return 0;
//    static float trigger_position = 0;

//    if(cross_handle == 1 && cross_state == 0)
//    {
//        cross_state = 1;
//    }
//    else if(cross_handle == 0 && cross_state == 1)
//        cross_state = 0;
    if(is_cross == 1)
    {
        cross_state = CROSS;
        set_lock(_CROSS);
    }else{
        cross_state = 0;
        clear_lock();
    }

    return cross_state;
}
#define   SW_VALUE  smartcar_param.sw_value
void analyze_road(void)
{
    int ret;

    last_road_type = road_type;
    road_type = BEND;

//    if(0 != (ret = detect_out_bound())) // 出界
//    {
//        road_type = ret;
//        return;
//    }

    if(0 != (ret = detect_circle_image()))
    {
        road_type = ret;
        return;
    }

    if(0 != (ret = detect_stop()))
    {
        road_type = ret;
        return;
    }

    if(0 != (ret = detect_cross()))
    {
        road_type = ret;
        return;
    }

//    if(0 != (ret = detect_long_straight_road()))
//    {
//        road_type = ret;
//        return;
//    }

    if(0 != (ret = detect_block()))
    {
        road_type = ret;
        return;
    }

//    if((uint8)SW_VALUE & 0x08) // 8(坡道)4(出发/库)2(圆环)1(入库)
//    {
//        if(0 != (ret = detect_ramp()))// 8(坡道)4(出发/库)2(圆环)1(入库)
//        {
//            road_type = ret;
//            return;
//        }
//    }


//    if((uint8)SW_VALUE & 0x04) // 8(坡道)4(出发/库)2(圆环)1(入库)
//    {
//        if(0 != (ret = detect_start()))
//        {
//            road_type = ret;
//            return;
//        }
//    }


//    if((uint8)SW_VALUE & 0x02)// 8(路碍)4(出发/库)2(圆环)1(入库)
//    {
//        if(0 != (ret = detect_circle_image()))
//        {
//            road_type = ret;
//            return;
//        }
//
//    }


//
//    if((uint8)SW_VALUE & 0x01)//// 8(坡道)4(出发/库)2(圆环)1(入库)
//    {
//        if(0 != (ret = detect_stop()))
//        {
//            road_type = ret;
//            return;
//        }
//    }

}
