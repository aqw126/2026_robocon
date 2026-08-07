#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


//==============================
// 車輪配置
//==============================

// 右下
#define THETA0 (M_PI/3.0f)

// 左下
#define THETA1 (2.0f*M_PI/3.0f)

// 前
#define THETA2 (M_PI)

const float theta[3] =
{
    THETA0,
    THETA1,
    THETA2
};

//==============================
// パラメータ
//==============================

// 車体中心→車輪
#define ROBOT_RADIUS 0.25f

// PWM最大値
#define MOTOR_MAX 100.0f

//--------------------------------------------------
// モータ出力
//--------------------------------------------------
void set_motor_speed(int motor, float speed)
{
    // speed:-100～100

    printf("Motor%d : %.1f\n", motor, speed);

    /*
    ここを自分のモータドライバに置き換える

    speed>0
      正転

    speed<0
      逆転

    fabs(speed)
      PWM
    */
}

//--------------------------------------------------
// オムニ速度計算
//--------------------------------------------------
void omni_drive(float vx, float vy, float omega)
{
    float wheel[3];

    for(int i=0;i<3;i++)
    {
        wheel[i] =
            cosf(theta[i]) * vx
            +sinf(theta[i]) * vy
            +ROBOT_RADIUS * omega;
    }

    //------------------------------------------------
    // 正規化
    //------------------------------------------------

    float max=fabsf(wheel[0]);

    for(int i=1;i<3;i++)
    {
        if(fabsf(wheel[i])>max)
            max=fabsf(wheel[i]);
    }

    if(max>1.0f)
    {
        for(int i=0;i<3;i++)
            wheel[i]/=max;
    }

    //------------------------------------------------
    // PWMへ変換
    //------------------------------------------------

    for(int i=0;i<3;i++)
    {
        set_motor_speed(i,wheel[i]*MOTOR_MAX);
    }
}

//--------------------------------------------------

void app_main(void)
{
    while(1)
    {
        //-----------------------------
        // 前進
        //-----------------------------
        printf("\nForward\n");
        omni_drive(0,1,0);
        vTaskDelay(pdMS_TO_TICKS(3000));

        //-----------------------------
        // 後退
        //-----------------------------
        printf("\nBackward\n");
        omni_drive(0,-1,0);
        vTaskDelay(pdMS_TO_TICKS(3000));

        //-----------------------------
        // 右移動
        //-----------------------------
        printf("\nRight\n");
        omni_drive(1,0,0);
        vTaskDelay(pdMS_TO_TICKS(3000));

        //-----------------------------
        // 左移動
        //-----------------------------
        printf("\nLeft\n");
        omni_drive(-1,0,0);
        vTaskDelay(pdMS_TO_TICKS(3000));

        //-----------------------------
        // 左回転
        //-----------------------------
        printf("\nRotate CCW\n");
        omni_drive(0,0,1);
        vTaskDelay(pdMS_TO_TICKS(3000));

        //-----------------------------
        // 右回転
        //-----------------------------
        printf("\nRotate CW\n");
        omni_drive(0,0,-1);
        vTaskDelay(pdMS_TO_TICKS(3000));

        //-----------------------------
        // 停止
        //-----------------------------
        printf("\nStop\n");
        omni_drive(0,0,0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}