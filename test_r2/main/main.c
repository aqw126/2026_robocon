#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "omni.h"


void app_main(void)
{
    omni_init();

    while(1)
    {
        // 前進
        omni_move(0.0, 1.0);

        vTaskDelay(pdMS_TO_TICKS(2000));


        // 後退
        omni_move(0.0, -1.0);

        vTaskDelay(pdMS_TO_TICKS(2000));


        // 右移動
        omni_move(1.0, 0.0);

        vTaskDelay(pdMS_TO_TICKS(2000));


        // 左移動
        omni_move(-1.0, 0.0);

        vTaskDelay(pdMS_TO_TICKS(2000));


        // 停止
        omni_move(0,0);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}