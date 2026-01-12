#include "imu.h"

IMU_TYPE imu;

/*******************************************************************************
* 函  数 ：float FindPos(float*a,int low,int high)
* 功  能 ：确定一个元素位序
* 参  数 ：a  数组首地址
*          low数组最小下标
*          high数组最大下标
* 返回值 ：返回元素的位序low
* 备  注 : 无
*******************************************************************************/
float FindPos(float*a,int low,int high)
{
    float val = a[low];                      //选定一个要确定值val确定位置
    while(low<high)
    {
        while(low<high && a[high]>=val)
            high--;                       //如果右边的数大于VAL下标往前移
        a[low] = a[high];             //当右边的值小于VAL则复值给A[low]

        while(low<high && a[low]<=val)
            low++;                        //如果左边的数小于VAL下标往后移
        a[high] = a[low];             //当左边的值大于VAL则复值给右边a[high]
    }
    a[low] = val;
    return low;
}

/*******************************************************************************
* 函  数 ：void QuiteSort(float* a,int low,int high)
* 功  能 ：快速排序
* 参  数 ：a  数组首地址
*          low数组最小下标
*          high数组最大下标
* 返回值 ：无
* 备  注 : 无
*******************************************************************************/
void QuiteSort(float* a,int low,int high)
{
    int pos;
    if(low<high)
    {
        pos = FindPos(a,low,high); //排序一个位置
        QuiteSort(a,low,pos-1);    //递归调用
        QuiteSort(a,pos+1,high);
    }
}

/********************************************************************************
* 函  数 ：void  SortAver_FilterXYZ(INT16_XYZ *acc,FLOAT_XYZ *Acc_filt,uint8_t n)
* 功  能 ：去最值平均值滤波三组数据
* 参  数 ：*acc 要滤波数据地址
*          *Acc_filt 滤波后数据地址
* 返回值 ：无
* 备  注 : 无
********************************************************************************/
void SortAver_FilterXYZ(IMU_TYPE *Acc_filt,uint8_t n)
{
    static float bufx[BUF_SIZ],bufy[BUF_SIZ],bufz[BUF_SIZ];
    static uint8_t cnt =0,flag = 1;
    float temp1=0,temp2=0,temp3=0;
    uint8_t i;
    bufx[cnt] = Acc_filt->a_raw[imu_x];
    bufy[cnt] = Acc_filt->a_raw[imu_y];
    bufz[cnt] = Acc_filt->a_raw[imu_z];
    cnt++;      //这个的位置必须在赋值语句后，否则bufx[0]不会被赋值
    if(cnt<n && flag)
        return;   //数组填不满不计算
    else
        flag = 0;

    QuiteSort(bufx,0,n-1);
    QuiteSort(bufy,0,n-1);
    QuiteSort(bufz,0,n-1);
    for(i=1;i<n-1;i++)
    {
        temp1 += bufx[i];
        temp2 += bufy[i];
        temp3 += bufz[i];
    }

    if(cnt>=n) cnt = 0;
    Acc_filt->a[imu_x]  = temp1/(n-2);
    Acc_filt->a[imu_y]  = temp2/(n-2);
    Acc_filt->a[imu_z]  = temp3/(n-2);
}

void get_atg_hardware(void)
{

//  imu.a_raw[imu_x] = icm20602_acc_x;
//  imu.a_raw[imu_y] = icm20602_acc_y;
//  imu.a_raw[imu_z] = icm20602_acc_z;

  imu.g_raw[imu_x] = icm20602_gyro_x;
  imu.g_raw[imu_y] = icm20602_gyro_y;
  imu.g_raw[imu_z] = icm20602_gyro_z;

}

/******************************************************************************
* 函  数：uint8_t MPU6050_OffSet(INT16_XYZ value,INT16_XYZ *offset,uint16_t sensivity)
* 功  能：MPU6050零偏校准
* 参  数：value：    MPU6050原始数据
*         offset：    校准后的零偏值
*         sensivity：加速度计的灵敏度
* 返回值：1校准完成 0校准未完成
* 备  注：无
*******************************************************************************/
uint8_t IMU_OffSet_Cal(IMU_TYPE* imu,uint16_t sensivity)
{
    static int32_t tempgx=0,tempgy=0,tempgz=0;
    static uint16_t cnt_a=0;//使用static修饰的局部变量，表明次变量具有静态存储周期，也就是说该函数执行完后不释放内存
    if(cnt_a==0)
    {
        imu->g_raw[imu_x] = 0;imu->g_raw[imu_x] = 0;imu->g_raw[imu_z] = 0;
        tempgx = 0;tempgy = 0;tempgz = 0;
        cnt_a = 1;
        sensivity = 0;
        imu->g_offset[imu_x] = 0;imu->g_offset[imu_y] = 0;imu->g_offset[imu_z] = 0;
    }
    tempgx += imu->g_raw[imu_x];
    tempgy += imu->g_raw[imu_x];
    tempgz += imu->g_raw[imu_z] - sensivity ;//加速度计校准 sensivity 等于 MPU6050初始化时设置的灵敏度值（8196LSB/g）;陀螺仪校准 sensivity = 0；
    if(cnt_a==1000)               //200个数值求平均
    {
        imu->g_offset[imu_x] = tempgx/cnt_a;
        imu->g_offset[imu_y] = tempgy/cnt_a;
        imu->g_offset[imu_z] = tempgz/cnt_a;
        cnt_a = 0;
        return 1;
    }
    cnt_a++;
    return 0;
}

#define GYRO_OFFSET 0x01 //第一位陀螺仪校准标志位
#define ACC_OFFSET 0x02  //第二位加速度校准标志位

uint8_t      SENSER_OFFSET_FLAG;                     //传感器校准标志位
/******************************************************************************
* 函  数：void MPU6050_DataProcess(void)
* 功  能：对MPU6050进行去零偏处理
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/
//void IMU_Offset(void)
//{
//    //加速度去零偏AD值
////    IMU_ACC_RAW.X = icm_acc_x - ACC_OFFSET_RAW.X;
////    IMU_ACC_RAW.Y = icm_acc_y - ACC_OFFSET_RAW.Y;
////    IMU_ACC_RAW.Z = icm_acc_z - ACC_OFFSET_RAW.Z;
//    //陀螺仪去零偏AD值
//    imu.g_raw[imu_x] -= imu.g_offset[imu_x];
//    imu.g_raw[imu_y] -= imu.g_offset[imu_y];
//    imu.g_raw[imu_z] -= imu.g_offset[imu_z];
//
//    if(GET_FLAG(GYRO_OFFSET)) //陀螺仪进行零偏校准
//    {
//        if(IMU_OffSet(&imu,0))
//        {
//
//            SENSER_FLAG_RESET(GYRO_OFFSET);
//            PID_WriteFlash(); //保存陀螺仪的零偏数据
//            GYRO_Offset_LED();
//            SENSER_FLAG_SET(ACC_OFFSET);//校准加速度

//           printf("GYRO_OFFSET_RAW Value :X=%d  Y=%d  Z=%d\n",GYRO_OFFSET_RAW.X,GYRO_OFFSET_RAW.Y,GYRO_OFFSET_RAW.Z);
//           printf("\n");
//        }
//    }
//    if(GET_FLAG(ACC_OFFSET)) //加速度计进行零偏校准
//    {
//        if(IMU_OffSet(&imu,8196))
//        {
//            SENSER_FLAG_RESET(ACC_OFFSET);
////            PID_WriteFlash(); //保存加速度计的零偏数据
////            ACC_Offset_LED();
////             printf("ACC_OFFSET_RAW Value X=%d  Y=%d  Z=%d\n",ACC_OFFSET_RAW.X,ACC_OFFSET_RAW.Y,ACC_OFFSET_RAW.Z);
////             printf("\n");
//        }
//    }
//}


/*********************************************************************************************************
* 函  数：void Prepare_Data(void)
* 功　能：对陀螺仪去零偏后的数据滤波及赋予物理意义，为姿态解算做准备
* 参  数：无
* 返回值：无
**********************************************************************************************************/
void Prepare_Data(void)
{
//    static uint8_t IIR_mode = 1;

//    MPU6050_Read();    //触发读取 ，立即返回
//    IMU_Offset();  //对MPU6050进行处理，减去零偏。如果没有计算零偏就计算零偏

//    SortAver_FilterXYZ(&imu,12);//对加速度原始数据进行去极值滑动窗口滤波

    //加速度AD值 转换成 米/平方秒
//    imu.a[imu_x] = (float)imu.a_raw[imu_x] * Acc_Gain * G;
//    imu.a[imu_y] = (float)imu.a_raw[imu_y] * Acc_Gain * G;
//    imu.a[imu_z] = (float)imu.a_raw[imu_z] * Acc_Gain * G;

    static float win_1[WIN_SIZE];
//    static float win_2[WIN_SIZE];
//    static float win_3[WIN_SIZE];
    //陀螺仪AD值 转换成 角度/秒
    imu.g[imu_x] = (float)imu.g_raw[imu_x] * Gyro_Gain;//-gyro_offsite.x;
    imu.g[imu_y] = (float)imu.g_raw[imu_y] * Gyro_Gain;//-gyro_offsite.y;
    imu.g[imu_z] = (float)imu.g_raw[imu_z] * Gyro_Gain;//-gyro_offsite.z;

    imu.g[imu_z] = SlidingFilter(win_1, imu.g[imu_z]);
//    imu.g[imu_x] = (float)imu.g_raw[imu_x] * Gyro_Gain-imu.g_offset[imu_x];
//    imu.g[imu_y] = (float)imu.g_raw[imu_y] * Gyro_Gain-imu.g_offset[imu_y];
//    imu.g[imu_z] = (float)imu.g_raw[imu_z] * Gyro_Gain-imu.g_offset[imu_z];

//    if(MY_ABS(imu.g[imu_z])<=0.06){
//        imu.g[imu_z] = 0;
//    }
}

/****************************************************************************************************
* 函  数：static float invSqrt(float x)
* 功　能: 快速计算 1/Sqrt(x)
* 参  数：要计算的值
* 返回值：计算的结果
* 备  注：比普通Sqrt()函数要快四倍 See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
*****************************************************************************************************/
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f375a86 - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*********************************************************************************************************
* 函  数：void IMUupdate(FLOAT_XYZ *Gyr_rad,FLOAT_XYZ *Acc_filt,FLOAT_ANGLE *Att_Angle)
* 功　能：获取姿态角
* 参  数：Gyr_rad  指向角速度的指针（注意单位必须是弧度）
*         Acc_filt  指向加速度的指针
*         Att_Angle 指向姿态角的指针
* 返回值：无
* 备  注：求解四元数和欧拉角都在此函数中完成
**********************************************************************************************************/
//Kp=Ki=0 就是完全相信陀螺仪
#define Kp 1.50f                         // proportional gain governs rate of convergence to accelerometer/magnetometer
                                         //比例增益控制加速度计，磁力计的收敛速率
#define Ki 0.005f                        // integral gain governs rate of convergence of gyroscope biases
                                         //积分增益控制陀螺偏差的收敛速度
#define halfT 0.005f                     // half the sample period
                                         //采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
float   accb[3],DCMgb[3][3];                  //方向余弦阵（将 惯性坐标系 转化为 机体坐标系）
void IMUupdate(IMU_TYPE *IMU_Data)
{
    uint8_t i;
    float matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };//初始化矩阵
    float ax = IMU_Data->a[imu_x],ay = IMU_Data->a[imu_y],az = IMU_Data->a[imu_z];
    float gx = IMU_Data->g[imu_x],gy = IMU_Data->g[imu_y],gz = IMU_Data->g[imu_z];
    float vx, vy, vz;
    float ex, ey, ez;
    float norm;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    if(ax*ay*az==0)
        return;

    //加速度计测量的重力向量(机体坐标系)
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //  printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",ax,ay,az);

    //陀螺仪积分估计重力向量(机体坐标系)
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;
    // printf("vx=%0.2f vy=%0.2f vz=%0.2f\r\n",vx,vy,vz);

    //测量的重力向量与估算的重力向量差积求出向量间的误差
    ex = (ay*vz - az*vy); //+ (my*wz - mz*wy);
    ey = (az*vx - ax*vz); //+ (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx); //+ (mx*wy - my*wx);

    //用上面求出误差进行积分
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    //将误差PI后补偿到陀螺仪
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

    //四元素的微分方程
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    //单位化四元数
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    //矩阵R 将惯性坐标系(n)转换到机体坐标系(b)
    matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;  // 11(前列后行)
    matrix[1] = 2.f * (q1q2 + q0q3);        // 12
    matrix[2] = 2.f * (q1q3 - q0q2);        // 13
    matrix[3] = 2.f * (q1q2 - q0q3);        // 21
    matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;  // 22
    matrix[5] = 2.f * (q2q3 + q0q1);        // 23
    matrix[6] = 2.f * (q1q3 + q0q2);        // 31
    matrix[7] = 2.f * (q2q3 - q0q1);        // 32
    matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;  // 33

    //四元数转换成欧拉角(Z->Y->X)
    IMU_Data->yaw += IMU_Data->g[imu_z] * RadtoDeg * 0.01f;
//  IMU_Data->yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* RadtoDeg; // yaw
//    IMU_Data->yaw =0.0004*IMU_Data->yaw -0.49;
//    IMU_Data->yaw = -atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1) * RadtoDeg;
    IMU_Data->rol = -asin(2.f * (q1q3 - q0q2)) * RadtoDeg;                            // roll(负号要注意)
    IMU_Data->pit = -atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3) * RadtoDeg ; // pitch
    for(i=0;i<9;i++)
    {
        *(&(DCMgb[0][0])+i) = matrix[i];
    }

    //失控保护 (调试时可注释掉)
    //Safety_Check();
}

void imu_updata(void)
{
//    icm20602_get_acc();
    icm20602_get_gyro();
    get_atg_hardware();
    Prepare_Data();
//    IMUupdate(&imu);

}
