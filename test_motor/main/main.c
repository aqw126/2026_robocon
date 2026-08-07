#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// モーターピン
#define MOTOR1_IN 4
#define MOTOR1_SD 5

#define MOTOR2_IN 6
#define MOTOR2_SD 7

#define MOTOR3_IN 15
#define MOTOR3_SD 16


// PWM設定
#define PWM_FREQ 1000
#define PWM_RES LEDC_TIMER_10_BIT

#define MOTOR1_CH LEDC_CHANNEL_0
#define MOTOR2_CH LEDC_CHANNEL_1
#define MOTOR3_CH LEDC_CHANNEL_2


// 正転速度
// 0〜1023
#define SPEED 500


void motor_pwm_init()
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&timer);


    ledc_channel_config_t channels[3] = {
        {
            .gpio_num = MOTOR1_IN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = MOTOR1_CH,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0
        },
        {
            .gpio_num = MOTOR2_IN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = MOTOR2_CH,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0
        },
        {
            .gpio_num = MOTOR3_IN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = MOTOR3_CH,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0
        }
    };


    for(int i=0;i<3;i++){
        ledc_channel_config(&channels[i]);
    }
}


void motor_enable()
{
    gpio_set_level(MOTOR1_SD, 1);
    gpio_set_level(MOTOR2_SD, 1);
    gpio_set_level(MOTOR3_SD, 1);
}


void motor_forward()
{
    ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        MOTOR1_CH,
        SPEED
    );

    ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        MOTOR1_CH
    );


    ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        MOTOR2_CH,
        SPEED
    );

    ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        MOTOR2_CH
    );


    ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        MOTOR3_CH,
        SPEED
    );

    ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        MOTOR3_CH
    );
}


void app_main(void)
{
    // SDピン設定
    gpio_config_t io_conf = {
        .pin_bit_mask =
            (1ULL<<MOTOR1_SD) |
            (1ULL<<MOTOR2_SD) |
            (1ULL<<MOTOR3_SD),

        .mode = GPIO_MODE_OUTPUT
    };

    gpio_config(&io_conf);


    motor_pwm_init();

    motor_enable();


    // 全車輪正転
    motor_forward();


    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}