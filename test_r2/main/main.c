#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

//==============================
// 車輪配置
//==============================

#define MOTOR1_IN 5
#define MOTOR1_SD 4
#define MOTOR2_IN 6
#define MOTOR2_SD 7
#define MOTOR3_IN 15 
#define MOTOR3_SD 16

#define PWM_FREQ 1000
#define PWM_RES LEDC_TIMER_10_BIT

#define MOTOR1_CH LEDC_CHANNEL_0
#define MOTOR2_CH LEDC_CHANNEL_1
#define MOTOR3_CH LEDC_CHANNEL_2

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

void motor_init()
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL<<MOTOR1_IN) |
            (1ULL<<MOTOR2_IN) |
            (1ULL<<MOTOR3_IN),
    };

    gpio_config(&io_conf);


    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&timer);


    ledc_channel_config_t ch[3] =
    {
        {
            .channel=MOTOR1_CH,
            .timer_sel=LEDC_TIMER_0,
            .gpio_num=MOTOR1_SD,
            .duty=0,
            .speed_mode=LEDC_LOW_SPEED_MODE
        },

        {
            .channel=MOTOR2_CH,
            .timer_sel=LEDC_TIMER_0,
            .gpio_num=MOTOR2_SD,
            .duty=0,
            .speed_mode=LEDC_LOW_SPEED_MODE
        },

        {
            .channel=MOTOR3_CH,
            .timer_sel=LEDC_TIMER_0,
            .gpio_num=MOTOR3_SD,
            .duty=0,
            .speed_mode=LEDC_LOW_SPEED_MODE
        }
    };


    for(int i=0;i<3;i++)
    {
        ledc_channel_config(&ch[i]);
    }
}

//--------------------------------------------------
// モータ出力
//--------------------------------------------------
void set_motor_speed(int motor, float speed)
{
    int in_pin;
    int channel;


    if(motor==0)
    {
        in_pin=MOTOR1_IN;
        channel=MOTOR1_CH;
    }
    else if(motor==1)
    {
        in_pin=MOTOR2_IN;
        channel=MOTOR2_CH;
    }
    else
    {
        in_pin=MOTOR3_IN;
        channel=MOTOR3_CH;
    }


    //方向
    if(speed >= 0)
    {
        gpio_set_level(in_pin,1);
    }
    else
    {
        gpio_set_level(in_pin,0);
        speed=-speed;
    }


    //PWM
    if(speed>100)
        speed=100;


    uint32_t duty =
        (uint32_t)(speed * 1023 /100);


    ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        channel,
        duty
    );

    ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        channel
    );
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
    motor_init();

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