#include "omni.h"
#include <math.h>
#include "driver/ledc.h"


#define MOTOR_MAX 255


// 車輪角度
#define WHEEL1_ANGLE (M_PI/3)      //右下
#define WHEEL2_ANGLE (2*M_PI/3)    //左下
#define WHEEL3_ANGLE (M_PI)        //正面


static void motor_write(int motor, int speed)
{
    /*
       ここを実際のモータドライバへ接続する

       speed
       +255 正転
       -255 逆転
          0 停止
    */


    printf(
        "Motor%d speed=%d\n",
        motor,
        speed
    );
}



void omni_init(void)
{
    // PWM初期化など
}



void omni_move(float vx,float vy)
{

    float wheel_speed[3];


    float angle[3]={
        WHEEL1_ANGLE,
        WHEEL2_ANGLE,
        WHEEL3_ANGLE
    };


    for(int i=0;i<3;i++)
    {

        /*
          オムニ逆運動学

          車輪速度 =
          -sinθ * vx
          +cosθ * vy
        */

        wheel_speed[i] =
            sin(angle[i])*vy
            +cos(angle[i])*vx;

    }



    // 最大値を255へ正規化

    float max=0;

    for(int i=0;i<3;i++)
    {
        if(fabs(wheel_speed[i]) > max)
            max=fabs(wheel_speed[i]);
    }


    if(max>1)
    {
        for(int i=0;i<3;i++)
            wheel_speed[i]/=max;
    }



    for(int i=0;i<3;i++)
    {
        motor_write(
            i+1,
            (int)(wheel_speed[i]*MOTOR_MAX)
        );
    }

}